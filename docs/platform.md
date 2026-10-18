# 开发与验证环境

2026-10-17 记录的主要开发环境：

| 项目 | 配置 |
|---|---|
| CPU | Intel Core i5-13600K，14 核、20 逻辑线程 |
| 内存 | 64 GiB |
| Windows | Windows 11 x64 |
| Windows 编译器 | Visual Studio 2026，MSVC 19.51 |
| Windows CMake | 4.3.1-msvc1 |
| Linux 环境 | WSL2，Ubuntu 24.04.4 LTS（`Ubuntu-24.04`） |
| WSL 内核 | 6.18.33.2-microsoft-standard-WSL2 |
| Linux 工具 | GCC 13.3.0、CMake 3.28.3、Ninja 1.11.1 |

WSL 是 Linux 构建与运行环境；这些记录不代表已经验证全部 Linux 发行版或物理 Linux 主机。VS2019/2022 对应构建预设保留，但不把预设存在视为本次已经运行验证。

公开版本不承诺稳定 API/ABI。建议 fork 或固定 commit/tag，在升级后重编译应用，并使用自身数据验证结果、内存占用和延迟。
