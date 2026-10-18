# 构建与安装

## 依赖与工具

构建库需要 C++20；公开 C/C++ 接口分别面向 C11/C++14。Linux 使用 CMake 3.20+ 和 Ninja；Windows 支持 VS2019/2022/2026，VS2026 需要 CMake 4.2+。默认构建 Release、x64（Windows），开启图片 demo。公开版本没有测试与基准构建选项。

私有数值实现使用随仓库提供的 Eigen 3.4.0。JPEG/PNG/TIFF/zlib 优先通过 `find_package` 查找系统开发库；缺失项再使用 `3rdparty/` 中的源码或固定哈希归档，最后从固定上游地址下载。版本与 SHA-256 以 [Dependencies.cmake](../cmake/Dependencies.cmake) 为准，不使用 vcpkg。

## Windows

```powershell
./scripts/setup-windows.ps1
```

脚本按 VS2026→VS2022→VS2019 查找编译器和 CMake，仅负责工具选择及参数传递。也可以从合适的 CMake 直接运行：

```powershell
cmake --preset windows
cmake --build --preset windows --parallel
```

VS2022/2019 分别使用 `windows-2022` / `windows-2019` 配置与构建预设。示例位于 `build/bin/Release/`。切换编译器或生成器时使用新的构建目录。

## Ubuntu 与 WSL

```sh
bash scripts/setup-ubuntu.sh --system
cmake --preset linux
cmake --build --preset linux --parallel
```

脚本只安装构建环境；`--system` 另安装 JPEG/PNG/TIFF 开发包。不带该选项时可使用源码回退。Linux 示例位于 `build/bin/`。

## 常用配置

| 选项 | 用途 |
|---|---|
| `MVKIT_BUILD_EXAMPLES=OFF` | 仅构建库 |
| `MVKIT_USE_SYSTEM_LIBS=OFF` | 使用固定源码依赖 |
| `MVKIT_OFFLINE=ON` | 禁止下载；必须已备齐系统库、源码或归档 |
| `MVKIT_USE_EIGEN=OFF` | 使用内置数值后端 |
| `MVKIT_WARNINGS_AS_ERRORS=ON` | 将编译警告视为错误 |
| `CMAKE_PREFIX_PATH` | 自定义系统依赖安装前缀 |

离线目录为 `3rdparty/jpeg`、`png`、`tiff`、`zlib`，归档可放 `3rdparty/downloads`。源码目录直接包含上游 `CMakeLists.txt`，版本须与固定归档一致，并保留完整许可。首次源码回退需要联网；后续可复用缓存。图像 demo 使用仓库内 AI 素材，不下载外部数据集。

## 安装与部署

```sh
cmake --install build --config Release --prefix out/mvkit
```

安装内容为公开头、`mvkit`/`mvkitcpp` 共享库、`lib/cmake/MVKit` 和 `share/MVKit` 中的 SDK 使用说明、接口/功能/runtime 文档及许可材料；启用 Eigen 时还包含对应源码快照。SDK 说明中的链接按安装目录生成。下游使用 `find_package(MVKit CONFIG REQUIRED)` 和 `mvkit::CXX`（推荐）或 `mvkit::C`。公开头采用 UTF-8，CMake 目标会自动向 MSVC 调用方传递 `/utf-8`；手工配置编译命令时也应使用 UTF-8 源码编码。

Windows 构建和 SDK 安装会解析共享库依赖，将找到的非系统运行 DLL 放入对应 `bin` 目录；依赖须能从构建产物目录、依赖前缀或 PATH 找到。Windows 仍需要匹配的 Microsoft C++ 运行库。实际分发的第三方许可须保留，系统库许可不会由 CMake 自动收集，组件清单见[第三方组件与许可](../README.md#第三方组件与许可)。
