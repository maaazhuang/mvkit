#include "runtime/execution/execution_context.hpp"

#include <algorithm>

namespace mv::runtime {
namespace {

std::shared_ptr<std::atomic<bool>> make_cancel_flag() {
  return std::make_shared<std::atomic<bool>>(false);
}

}  // namespace

ExecutionContext::time_point ExecutionContext::default_clock() {
  return clock_type::now();
}

ExecutionContext::ExecutionContext() : cancelled_(make_cancel_flag()) {}

ExecutionContext::ExecutionContext(time_point deadline)
    : ExecutionContext(deadline, &ExecutionContext::default_clock) {}

ExecutionContext::ExecutionContext(time_point deadline, ClockFn clock)
    : cancelled_(make_cancel_flag()),
      deadline_(deadline),
      has_deadline_(true),
      clock_(clock != nullptr ? clock : &ExecutionContext::default_clock) {}

ExecutionContext::ExecutionContext(
    std::shared_ptr<std::atomic<bool>> cancelled, time_point deadline,
    bool has_deadline, ClockFn clock)
    : cancelled_(cancelled ? std::move(cancelled) : make_cancel_flag()),
      deadline_(deadline),
      has_deadline_(has_deadline),
      clock_(clock != nullptr ? clock : &ExecutionContext::default_clock) {}

std::pmr::memory_resource* ExecutionContext::scratch_resource() noexcept {
  return scratch_.resource();
}

void ExecutionContext::reset_scratch() noexcept { scratch_.reset(); }

void ExecutionContext::request_cancel() noexcept {
  cancelled_->store(true, std::memory_order_release);
}

bool ExecutionContext::cancelled() const noexcept {
  return cancelled_->load(std::memory_order_acquire);
}

ExecutionContext::time_point ExecutionContext::now() const {
  return clock_ != nullptr ? clock_() : default_clock();
}

StopReason ExecutionContext::stop_reason() const {
  if (cancelled()) return StopReason::Cancelled;
  if (has_deadline_ && now() >= deadline_) return StopReason::DeadlineExceeded;
  return StopReason::None;
}

StopReason ExecutionContext::poll() const { return stop_reason(); }

std::unique_ptr<ExecutionContext> ExecutionContext::nest() const {
  return std::unique_ptr<ExecutionContext>(new ExecutionContext(
      cancelled_, deadline_, has_deadline_, clock_));
}

std::unique_ptr<ExecutionContext> ExecutionContext::nest(
    time_point child_deadline) const {
  time_point deadline = child_deadline;
  const bool has_deadline = true;
  if (has_deadline_) {
    deadline = (std::min)(deadline_, child_deadline);
  }
  return std::unique_ptr<ExecutionContext>(new ExecutionContext(
      cancelled_, deadline, has_deadline, clock_));
}

const CpuFeatures& ExecutionContext::cpu() const noexcept {
  return cpu_features();
}

const char* ExecutionContext::selected_isa() const noexcept {
  return cpu_features().selection().kernel;
}

}  // namespace mv::runtime
