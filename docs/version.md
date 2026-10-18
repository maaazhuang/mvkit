# mvkit 0.1.0

mvkit 的首个公开版本，提供面向工业机器视觉的 C11 和 C++14 接口，库实现使用 C++20。

- 图像处理、Region 与形态学、XLD 亚像素轮廓处理与拟合。
- Shape/NCC 模板匹配，相机标定与几何变换，双目、结构光及三维点云处理。
- 内置内存池、可选线程池，以及运行时 CPU 特性检测和 AVX2 派发。
- 五个独立功能 demo，分别演示 C++ 过程式与对象式调用；保留合并 demo 复现效果图。
- 默认中文 README，提供英文 README；自有代码采用 Apache-2.0，第三方保留原有许可。

共享库名称为 `mvkit` 和 `mvkitcpp`。Windows 对应 `mvkit.dll`、`mvkitcpp.dll`；Linux 使用带版本的 `libmvkit.so` 和 `libmvkitcpp.so`。下游 CMake 继续链接 `mvkit::C` 或 `mvkit::CXX`。

项目版本由根 CMake 统一设置为 `0.1.0`，`GetSystem("version")` 和 C 接口的对应查询返回 `0.1.0`。首个 GitHub Release 与标签均使用 `0.1.0`，在正式源码首次上传后创建。

当前处于开发阶段，不保证 API/ABI 兼容。建议 fork 或固定 commit/tag 使用；本版本不提供 Measure/Metrology 等测量相关接口。
