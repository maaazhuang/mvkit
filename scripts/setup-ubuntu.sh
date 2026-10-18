#!/usr/bin/env bash
set -euo pipefail
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != "--system" ) ]]; then
  echo "usage: $0 [--system]" >&2
  exit 2
fi
# 只准备环境；编译统一使用根目录的 CMake 预设。
sudo apt-get update
sudo apt-get install -y build-essential cmake ninja-build
if [[ "${1:-}" == "--system" ]]; then
  sudo apt-get install -y libjpeg-dev libpng-dev libtiff-dev
fi
