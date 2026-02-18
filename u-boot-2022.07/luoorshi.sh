#!/bin/bash

# 检查必需的编译工具
missing_tools=()

if ! command -v gcc &> /dev/null; then
    missing_tools+=("gcc")
fi

if ! command -v bison &> /dev/null; then
    missing_tools+=("bison")
fi

if ! command -v flex &> /dev/null; then
    missing_tools+=("flex")
fi

if [ ${#missing_tools[@]} -gt 0 ]; then
    echo "========================================="
    echo "警告：系统缺少必需的编译工具"
    echo "========================================="
    echo "缺少的工具: ${missing_tools[*]}"
    echo ""
    echo "请执行以下命令安装："
    echo "  sudo apt update"
    echo "  sudo apt install -y gcc build-essential bison flex"
    echo ""
    echo "或者安装完整的编译环境："
    echo "  sudo apt install -y git openssh-server"
    echo "  sudo apt install -y make make-guile"
    echo "  sudo apt install -y swig python-dev python3-dev bison flex python3-distutils"
    echo "  sudo apt-get install -y lsb-core"
    echo "  sudo apt-get install -y lib32stdc++6"
    echo "  sudo apt-get install -y libssl-dev"
    echo "  sudo apt-get install -y openssl"
    echo "========================================="
    echo ""
    echo "注意：编译可能会失败，请先安装缺少的工具"
    echo "========================================="
    echo ""
fi

# 检查是否已经设置了GCC环境变量
if [[ ":$PATH:" != *":6.3.1/bin:"* ]]; then
    echo "检测到未设置GCC交叉编译环境变量，正在设置..."
    source ../setup_gcc_env.sh
    echo ""
fi

make clean
# make nanopi_h3_defconfig ARCH=arm CROSS_COMPILE=arm-linux-
# make sunxi_h3_luoorshi_defconfig ARCH=arm CROSS_COMPILE=arm-linux-
# make orangepi_zero_plus2_h3_defconfig ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
make quark-luoorshi-h3_defconfig ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
# make V=1 ARCH=arm CROSS_COMPILE=arm-linux- -j4
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j4 2>&1 | tee build.log

