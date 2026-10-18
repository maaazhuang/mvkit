# 发布脚本

| 脚本 | 用途 |
|---|---|
| setup-windows.ps1 | 自动选择 VS2019/2022/2026 与 CMake，配置并构建；支持离线及自定义依赖前缀 |
| setup-ubuntu.sh | 安装 Ubuntu 构建工具，可选系统 JPEG/PNG/TIFF 开发包 |

构建逻辑统一位于根目录 CMake，脚本不维护第二套实现。不提供开发侧审计、基准和测试脚本。功能 demo 的编译与运行见[示例说明](../examples/README.md)。
