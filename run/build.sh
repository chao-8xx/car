#!/usr/bin/env bash
set -euo pipefail

# 从脚本位置定位工程，允许从任意当前目录调用。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"
TOOLCHAIN_DIR="${CROSSTOOL_DIR:-/opt/loongson-gnu-toolchain-13.2}"
JOBS="${JOBS:-12}"

rm -rf "${BUILD_DIR}"
cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" \
  -DCROSSTOOL_DIR="${TOOLCHAIN_DIR}"
cmake --build "${BUILD_DIR}" --parallel "${JOBS}"

echo "构建完成：${BUILD_DIR}/main"
