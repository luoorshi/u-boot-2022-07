#!/bin/bash

# 设置GCC交叉编译工具链环境变量
# 注意：这个脚本只在当前终端窗口生效，关闭窗口后环境变量会恢复

# 获取脚本所在目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 设置GCC工具链路径
export PATH="$SCRIPT_DIR/6.3.1/bin:$PATH"
export GCC_COLORS=auto

# 显示当前环境变量
echo "========================================="
echo "GCC交叉编译环境已设置"
echo "========================================="
echo "GCC工具链路径: $SCRIPT_DIR/6.3.1/bin"
echo "GCC_COLORS=auto"
echo ""
echo "测试GCC版本："
arm-linux-gnueabihf-gcc -v 2>&1 | head -n 5
echo ""
echo "环境变量已设置，可以开始编译u-boot"
echo "========================================="
