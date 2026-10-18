# 功能演示 Demo

每个独立 demo 对应一个功能，源文件中的 `procedural()` 和 `object()` 分别展示 C++ 过程式（函数式）与对象式调用。两种方式执行相同处理流程，可对照学习，也可在业务中组合使用。公共头 `demo_common.hpp` 为独立及合并 demo 提供图片读写、素材预处理、结果叠加和命令行辅助。

| 功能 | 源码 | 构建目标 | 演示内容 |
|---|---|---|---|
| 图像处理 | [image_processing.cpp](image_processing.cpp) | `mvkit_demo_image_processing` | 读取垫圈图像、灰度化、缩放，添加脉冲噪声后做中值滤波 |
| Region 与形态学 | [region_morphology.cpp](region_morphology.cpp) | `mvkit_demo_region_morphology` | 阈值、开运算、连通域、面积筛选和填洞 |
| XLD 边缘与筛选 | [xld_edges.cpp](xld_edges.cpp) | `mvkit_demo_xld_edges` | Canny 亚像素轮廓、长度与闭合性筛选、圆度筛选 |
| Shape 模板匹配 | [shape_matching.cpp](shape_matching.cpp) | `mvkit_demo_shape_matching` | 图像模板建模、旋转搜索、输出位姿和匹配轮廓 |
| NCC 模板匹配 | [ncc_matching.cpp](ncc_matching.cpp) | `mvkit_demo_ncc_matching` | 灰度模板建模、旋转搜索、输出位姿和模板边框 |
| 合并演示 | [industrial_demo.cpp](industrial_demo.cpp) | `mvkit_industrial_demo` | 完整执行以上处理链，输出中间结果、统计与五组三联图，供示例效果复现 |

输入图片位于 [images](images/)，使用随包 AI 生成的垫圈和支架图片。处理结果由 SDK 实际计算；滤波噪声和模板搜索图像从输入图确定性派生。示例按 480×320 缩放，并使用适合这两张图片的固定参数。

## 编译

在项目根目录运行，示例默认开启；仅构建库时可设置 `MVKIT_BUILD_EXAMPLES=OFF`。工具与依赖准备见[构建说明](../docs/build.md)。

Windows：

```powershell
./scripts/setup-windows.ps1
# 修改示例后可单独重新构建目标。
cmake --build --preset windows --target mvkit_demo_xld_edges --parallel
```

Ubuntu / WSL：

```sh
bash scripts/setup-ubuntu.sh --system
cmake --preset linux
cmake --build --preset linux --parallel
```

下列所有程序均使用公开 SDK，C++ 应用部分只需要 C++14。

## 运行

五个独立 demo 的命令格式：

```text
mvkit_demo_<功能> <图片目录> <已存在的输出目录> [procedural|object|both]
```

默认 `both`，分别执行两种调用方式。`procedural` 仅运行过程式代码，`object` 仅运行对象式代码。输出文件名包含功能和调用方式，便于对照；重复运行会覆盖对应结果。输出目录须预先创建，所有路径均相对于程序启动目录，也可使用绝对路径。

Windows，从项目根目录运行五个独立 demo：

```powershell
New-Item -ItemType Directory -Path build/demo-output -Force
$demos = 'image_processing','region_morphology','xld_edges','shape_matching','ncc_matching'
foreach ($demo in $demos) {
  & "./build/bin/Release/mvkit_demo_$demo.exe" ./examples/images ./build/demo-output both
  if ($LASTEXITCODE -ne 0) { throw "Demo failed: $demo" }
}
```

Ubuntu / WSL：

```sh
mkdir -p build/demo-output
for demo in image_processing region_morphology xld_edges shape_matching ncc_matching; do
  ./build/bin/mvkit_demo_$demo ./examples/images ./build/demo-output both || exit 1
done
```

只运行 XLD 的对象式演示：

```sh
./build/bin/mvkit_demo_xld_edges ./examples/images ./build/demo-output object
```

图像处理输出原图、加噪图和滤波图；Region 输出原图、区域边界和填洞结果；XLD 输出原图、全部有效边缘和筛选后的圆形轮廓。Shape/NCC 输出模板、搜索图、定位结果及 `*_pose.csv`，其中包含行、列、角度（弧度）和匹配分数。控制台同时显示区域/轮廓数量或匹配位姿。

合并 demo 使用相同的图片目录和输出目录，不接受调用方式参数：

```powershell
New-Item -ItemType Directory -Path build/combined-output -Force
./build/bin/Release/mvkit_industrial_demo.exe ./examples/images ./build/combined-output
```

```sh
mkdir -p build/combined-output
./build/bin/mvkit_industrial_demo ./examples/images ./build/combined-output
```

合并 demo 输出全部中间 PNG、`results.json` 以及 `image-processing.png`、`region-morphology.png`、`subpixel-edges.png`、`shape-matching.png`、`ncc-matching.png` 五幅无文字三联图。图像排版在 C++ 中完成，无需额外脚本或 Python 依赖。
