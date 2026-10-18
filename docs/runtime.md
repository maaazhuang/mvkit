# 内存与并行运行时

runtime 位于私有实现层，不向 Public C/C++ 头暴露内存资源或执行器类型。

## 内存

长期图像、区域、轮廓与模型载荷通过持久资源分配；短期算法工作空间使用独立的 scratch 资源。两类内置池都基于 `std::pmr::memory_resource`，按块大小和对齐分档复用空闲块，减少重复向系统分配内存。

每个池提供 32 个线程本地分片和一个共享分片：本地分片减少常用小块分配的竞争，共享分片处理回退路径；资源拥有全部缓存，支持跨线程释放及线程退出后的回收。算子内 scratch arena 优先使用 4 KiB 内联缓冲，不足时从 scratch 资源获取空间；arena 生命周期结束时统一回收。

默认每池空闲预算为 128 MiB，全局空闲缓存预算为 256 MiB。预算针对缓存块，不是进程内存上限；同时存活的业务图像、点云、模型及其他分配可以超过该值。对象复制、可写分离和算子临时空间也会影响峰值内存。私有统计属于采样信息，不是应用级资源保证。

## 并行

多线程框架主要参考 OpenCV [`parallel_for_`](https://docs.opencv.org/4.x/d7/dff/tutorial_how_to_use_OpenCV_parallel_for_.html) 的区间分块与任务执行方式进行改造。MVKit 使用内置线程池复用 worker，将区间拆成任务块，由参与线程动态领取；不依赖 OpenCV 运行库。

默认串行，可通过构建选项启用内部并行。轻量任务、单分块和嵌套调用可走串行路径；只有接入该框架的算子分支参与并行。调用同步返回，工作线程异常回传调用方。

启用内置线程池：

```sh
cmake --preset linux -DMVKIT_ENABLE_PARALLEL=ON -DMVKIT_PARALLEL_BACKEND=builtin
```

应用中建议在首次并行操作前设置预算：

```cpp
mv::SetSystem("thread_num", 4);
```

不同业务线程可共享只读图像与模型；并发修改、清理或重新设置同一可变句柄时需要调用方同步。内部并行不替代业务对象的读写同步。避免外层业务线程与内部 worker 同时无限扩张。

## 后端配置

| 组合 | CMake 参数 |
|---|---|
| 内置池＋串行（默认） | `MVKIT_ENABLE_MEMORY_POOL=ON`、`MVKIT_MEMORY_BACKEND=builtin` |
| 系统分配＋串行 | `MVKIT_ENABLE_MEMORY_POOL=OFF`、`MVKIT_MEMORY_BACKEND=system` |
| 内置线程池 | `MVKIT_ENABLE_PARALLEL=ON`、`MVKIT_PARALLEL_BACKEND=builtin` |

内存和并行选项可组合。未实现的后端会在配置阶段报错；当前不提供 OpenMP/TBB/custom 后端。

## CPU 特性检测与派发

运行时首次使用时检测并缓存部署设备的能力。x86/x64 通过 CPUID 获取 SSE2、AVX、AVX2 等特性，并通过 OSXSAVE 和 XGETBV 检查操作系统是否支持保存 XMM/YMM 寄存器状态。

默认自动模式按以下条件派发：

1. 构建中包含 AVX2 内核，且 CPU 支持 AVX/AVX2、操作系统支持对应寄存器状态时，允许自动使用 AVX2 分支。
2. 条件不满足时，选择可用的 SSE2 或标量路径。
3. 具体算子再检查对应内核是否存在、输入布局及尺寸等条件；需要 FMA 的内核另行检查 FMA 支持。

AVX2 使用独立内核编译，不要求整个程序或部署设备统一开启 AVX2。`MVKIT_SIMD=auto` 为默认策略；`scalar`、`sse2`、`avx2` 可用于诊断，指定不可用的指令集会安全回退到标量路径。环境变量应在首次使用 runtime 前设置。
