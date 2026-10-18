# C++ 接口使用

推荐使用 C++ 函数式接口和对象式接口，两种方式可组合使用，对象与元组均在离开作用域时自动释放。函数式接口的输出参数使用 `&output`，输入与输出按 iconic 输入→iconic 输出→控制输入→控制输出组织：

```cpp
#include <mvkit/cpp/mvkit.hpp>

mv::MvImage image, gray;
mv::ReadImage(&image, "examples/images/bracket.png");
mv::Rgb1ToGray(image, &gray);
mv::MvXLDCont edges;
mv::EdgesSubPix(gray, &edges, "canny", 1.2, 18, 40);
```

相同操作也可通过类的成员方法表达：

```cpp
mv::MvImage image;
image.ReadImage("examples/images/bracket.png");
mv::MvImage gray = image.Rgb1ToGray();
mv::MvXLDCont edges = gray.EdgesSubPix("canny", 1.2, 18, 40);
```

`MvImage`、`MvRegion`、`MvXLDCont` 等都可表示零个、一个或多个对象；图像通道数不等于对象个数。`CountObj()` 查询对象数，`SelectObj()` 使用从 1 开始的索引；`MvTuple` 的元素索引从 0 开始。多个 iconic 输入的广播和配对规则以对应头文件说明为准。

复制图像对象通常共享载荷；可写像素访问可能产生独立副本，并使先前借用的指针失效。不要在同一对象上并发写入。`MvTuple` 拥有其字符串、数值及控制句柄引用；把结果存入 Tuple 不需要手工管理底层 C Tuple。

形状和 NCC 模型可作为控制 Tuple 或对应 `MvShapeModel`/`MvNCCModel` 类使用。模型读写和通用序列化采用 MVKit 自有格式。`ClearHandle` 清除共享句柄内容，会使共享该句柄的所有别名失效，不同于销毁一个 Tuple 引用。

C++ 错误通过 `mv::MvException` 抛出；C 接口返回 `MvStatus` 并提供具体错误信息。应在应用边界捕获异常，按输入、参数和对象状态定位问题。C 接口主要供其他语言绑定及需要显式生命周期管理的应用使用；C++ 业务推荐使用函数式或对象式接口。

当前为开发阶段，公开头、签名及类型布局可能变化；固定版本后编译和部署全部调用方。接口说明以本仓库 `include/mvkit/` 为准，不使用完整开发版或 HALCON 的接口数量作为本版本契约。
