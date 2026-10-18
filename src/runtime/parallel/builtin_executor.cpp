#include "runtime/parallel/builtin_executor.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "runtime/parallel/parallel_state.hpp"
#include "runtime/cpu/cpu_topology.hpp"

namespace mv::runtime {
namespace {

std::size_t automatic_thread_count() noexcept {
  return cpu_topology().available_logical_count;
}

struct ExecutorConfiguration {
  std::mutex mutex;
  std::size_t requested_count = 0;
  bool started = false;
  std::size_t started_count = 0;
};

ExecutorConfiguration& configuration() {
  static ExecutorConfiguration config;
  return config;
}

std::size_t normalized_thread_count(std::size_t requested) noexcept {
  return requested == 0 ? automatic_thread_count() : requested;
}

std::size_t ceil_div(std::size_t value, std::size_t divisor) noexcept {
  return 1 + (value - 1) / divisor;
}

std::size_t scaled_workers(std::size_t workers, std::size_t factor) noexcept {
  const auto maximum = (std::numeric_limits<std::size_t>::max)();
  return workers > maximum / factor ? maximum : workers * factor;
}

// 提交调用拥有此节点直至全部已领取工作结束。queue_mutex_ 保护队列链接、available、active、exception。
// done 使用 completion_mutex，闲置 worker 不阻碍调用方唤醒。原子 cursor 无需该锁领取；
// worker 获取参与 token 前，队列锁已发布不可变字段。
struct TaskGroup {
  IndexRange range;
  ParallelBodyView body;
  std::size_t grain, units, minimum, available, active = 0;
  alignas(64) std::atomic<std::size_t> next{0};
  alignas(64) TaskGroup* previous = nullptr;
  TaskGroup* following = nullptr;
  bool queued = false, done = false;
  std::atomic<bool> cancelled{false};
  std::mutex completion_mutex;
  std::condition_variable completion;
  std::exception_ptr exception;
};

}  // namespace

void execute_builtin_inline(IndexRange range, std::size_t grain,
                            ParallelBodyView body) {
  if (range.begin >= range.end) return;
  if (!body.invoke) throw std::invalid_argument("parallel body must not be null");
  detail::ParallelDepthGuard depth;
  detail::record_batches_queued(1);
  try {
    const auto chunk = grain ? grain : range.end - range.begin;
    for (auto begin = range.begin; begin < range.end;) {
      const auto end = begin + (std::min)(chunk, range.end - begin);
      body.invoke(body.context, {begin, end});
      begin = end;
    }
  } catch (...) {
    detail::record_batches_run(1);
    throw;
  }
  detail::record_batches_run(1);
}

class BuiltinExecutor::Impl {
 public:
  explicit Impl(std::size_t thread_count) : thread_count_(thread_count) {
    workers_.reserve(thread_count_);
    try {
      for (std::size_t index = 0; index < thread_count_; ++index)
        workers_.emplace_back([this, index] { worker_loop(index); });
    } catch (...) {
      {
        std::lock_guard lock(queue_mutex_);
        stopping_ = true;
      }
      ready_.notify_all();
      for (auto& worker : workers_) worker.join();
      throw;
    }
  }

  ~Impl() {
    {
      std::lock_guard lock(queue_mutex_);
      stopping_ = true;
    }
    ready_.notify_all();
    for (auto& worker : workers_) worker.join();
  }

  void execute(IndexRange range, std::size_t grain, ParallelBodyView body) {
    if (range.begin >= range.end) return;
    if (!body.invoke) throw std::invalid_argument("parallel body must not be null");
    const auto length = range.end - range.begin;
    if (thread_count_ == 1 || length == 1 || grain >= length) {
      execute_builtin_inline(range, grain, body);
      return;
    }
    const auto units = grain ? ceil_div(length, grain) : length;
    TaskGroup group{range, body, grain, units,
                    ceil_div(units, scaled_workers(thread_count_, grain ? 16 : 32)),
                    (std::min)(units, thread_count_)};
    std::unique_lock lock(queue_mutex_);
    append(group);
    live_groups_.store(++group_count_, std::memory_order_release);
    // 短索引范围也可能包含昂贵回调；按可用逻辑块招募，不根据范围长度猜测成本。
    const auto wake = (std::min)(units, thread_count_);
    if (wake == thread_count_) ready_.notify_all();
    else for (std::size_t i = 0; i < wake; ++i) ready_.notify_one();
    const auto started = std::chrono::steady_clock::now();
    lock.unlock();
    std::unique_lock completion_lock(group.completion_mutex);
    group.completion.wait(completion_lock, [&] { return group.done; });
    completion_lock.unlock();
    detail::record_wait_ns(static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now() - started).count()));
    if (group.exception) std::rethrow_exception(group.exception);
  }

  [[nodiscard]] std::size_t thread_count() const noexcept { return thread_count_; }

 private:
  void append(TaskGroup& group) noexcept {
    group.previous = tail_;
    group.following = nullptr;
    if (tail_) tail_->following = &group;
    else head_ = &group;
    tail_ = &group;
    group.queued = true;
  }

  void remove(TaskGroup& group) noexcept {
    if (group.previous) group.previous->following = group.following;
    else head_ = group.following;
    if (group.following) group.following->previous = group.previous;
    else tail_ = group.previous;
    group.queued = false;
  }

  bool claim(TaskGroup& group, IndexRange& batch) noexcept {
    auto begin = group.next.load(std::memory_order_relaxed);
    const auto groups = (std::max)(std::size_t{1}, live_groups_.load(std::memory_order_acquire));
    const auto participants = (std::max)(std::size_t{1}, thread_count_ / groups);
    std::size_t take;
    do {
      if (begin >= group.units) return false;
      const auto remaining = group.units - begin;
      take = (std::min)(remaining, (std::max)(group.minimum,
          ceil_div(remaining, scaled_workers(participants, 8))));
    } while (!group.next.compare_exchange_weak(begin, begin + take,
                                               std::memory_order_relaxed));
    const auto unit = group.grain ? group.grain : 1;
    batch.begin = group.range.begin + begin * unit;
    const auto available = group.range.end - batch.begin;
    batch.end = take > available / unit ? group.range.end : batch.begin + take * unit;
    return true;
  }

  void worker_loop(std::size_t index) noexcept {
    detail::set_worker_index(index);
    TaskGroup* previous = nullptr;
    std::exception_ptr error;
    for (;;) {
      TaskGroup* group;
      IndexRange batch;
      bool recruit = false;
      {
        std::unique_lock lock(queue_mutex_);
        if (previous) {
          if (error) {
            if (!previous->exception) previous->exception = std::move(error);
            error = nullptr;
          }
          --previous->active;
          const bool exhausted = previous->cancelled.load(std::memory_order_acquire) ||
              previous->next.load(std::memory_order_relaxed) >= previous->units;
          if (exhausted) {
            if (previous->queued) remove(*previous);
            if (previous->active == 0) {
              std::lock_guard completion_lock(previous->completion_mutex);
              previous->done = true;
              live_groups_.store(--group_count_, std::memory_order_release);
              previous->completion.notify_one();
            }
          } else {
            ++previous->available;
            if (previous->queued) remove(*previous);
            append(*previous);
            // 归还未完成工作也属于就绪发布，须唤醒初次入队后闲置的 worker。
            recruit = true;
          }
          previous = nullptr;
        }
        for (;;) {
          ready_.wait(lock, [&] { return stopping_ || head_ != nullptr; });
          if (!head_ && stopping_) return;
          group = head_;
          // 先取得有效工作再计参与者，短组不等待全部唤醒线程空进空出。
          if (!group->cancelled.load(std::memory_order_acquire) && claim(*group, batch)) break;
          remove(*group);  // 活跃拥有者完成或取消工作组。
        }
        ++group->active;
        if (--group->available == 0 ||
            group->next.load(std::memory_order_relaxed) >= group->units) remove(*group);
        // 初次提交已通知逻辑 worker 预算，只有新归还工作需要再招募闲置 worker。
        recruit = recruit && head_;
      }
      if (recruit) ready_.notify_one();
      // 独占时不进入全局队列；竞争时每个已领取批次结束后返回就绪队列。
      // 批次大小按剩余工作和并发组自适应，不依赖时钟或回调时长猜测；已领取工作不被抢占。
      try {
        detail::ParallelDepthGuard depth;
        do {
          detail::record_batches_queued(1);
          try {
            const auto chunk = group->grain ? group->grain : batch.end - batch.begin;
            for (auto begin = batch.begin; begin < batch.end;) {
              if (group->cancelled.load(std::memory_order_acquire)) break;
              const auto end = begin + (std::min)(chunk, batch.end - begin);
              group->body.invoke(group->body.context, {begin, end});
              begin = end;
            }
          } catch (...) {
            detail::record_batches_run(1);
            throw;
          }
          detail::record_batches_run(1);
          if (live_groups_.load(std::memory_order_acquire) > 1) break;
        } while (!group->cancelled.load(std::memory_order_acquire) && claim(*group, batch));
      } catch (...) {
        error = std::current_exception();
        group->cancelled.store(true, std::memory_order_release);
      }
      previous = group;
    }
  }

  std::size_t thread_count_;
  std::mutex queue_mutex_;
  std::condition_variable ready_;
  std::size_t group_count_ = 0;
  alignas(64) std::atomic<std::size_t> live_groups_{0};
  TaskGroup* head_ = nullptr;
  TaskGroup* tail_ = nullptr;
  bool stopping_ = false;
  std::vector<std::thread> workers_;
};

BuiltinExecutor::BuiltinExecutor() {
  ExecutorConfiguration& config = configuration();
  std::size_t count = 0;
  {
    std::lock_guard lock(config.mutex);
    config.started = true;
    count = normalized_thread_count(config.requested_count);
    config.started_count = count;
  }
  try {
    impl_ = std::make_unique<Impl>(count);
  } catch (...) {
    std::lock_guard lock(config.mutex);
    config.started = false;
    config.started_count = 0;
    throw;
  }
}

BuiltinExecutor::~BuiltinExecutor() = default;

void BuiltinExecutor::execute(IndexRange range, std::size_t grain_size,
                              ParallelBodyView body) {
  impl_->execute(range, grain_size, body);
}

std::size_t BuiltinExecutor::max_parallelism() const noexcept {
  return impl_->thread_count();
}

Executor& builtin_executor() {
  static BuiltinExecutor executor;
  // 用户诊断前发布完整单例；sink 可递归查询/执行，但不能重入初始化器。
  static std::atomic<bool> announced{false};
  if (!announced.load(std::memory_order_acquire) &&
      !announced.exchange(true, std::memory_order_acq_rel))
    detail::emit_scheduler_diagnostic("executor_started");
  return executor;
}

std::size_t parallel_thread_count() noexcept {
  ExecutorConfiguration& config = configuration();
  std::lock_guard lock(config.mutex);
  return config.started ? config.started_count : normalized_thread_count(config.requested_count);
}

bool builtin_executor_started() noexcept {
  ExecutorConfiguration& config = configuration();
  std::lock_guard lock(config.mutex);
  return config.started;
}

std::size_t builtin_started_worker_count() noexcept {
  ExecutorConfiguration& config = configuration();
  std::lock_guard lock(config.mutex);
  return config.started ? config.started_count : 0;
}

void set_parallel_thread_count(std::size_t count) {
  ExecutorConfiguration& config = configuration();
  std::lock_guard lock(config.mutex);
  if (config.started &&
      normalized_thread_count(count) != config.started_count) {
    throw std::logic_error(
        "parallel thread count cannot change after executor startup");
  }
  config.requested_count = count;
}

}  // namespace mv::runtime
