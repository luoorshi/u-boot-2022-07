make clean
# make nanopi_h3_defconfig ARCH=arm CROSS_COMPILE=arm-linux-
# make sunxi_h3_luoorshi_defconfig ARCH=arm CROSS_COMPILE=arm-linux-
make orangepi_zero_plus2_h3_defconfig ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf-
# make V=1 ARCH=arm CROSS_COMPILE=arm-linux- -j4
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -j4

