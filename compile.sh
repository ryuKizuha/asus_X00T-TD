#!/bin/7bash
export KBUILD_BUILD_USER=zRyu
export KBUILD_BUILD_HOST=ndeooo.xyz
# Compile plox
function compile() {
    make -j$(nproc) O=out ARCH=arm64 darkonah_defconfig
   time make -j$(nproc) ARCH=arm64 O=out \
                               CROSS_COMPILE=aarch64-linux-gnu- \
                               CROSS_COMPILE_ARM32=arm-linux-gnueabi-
}
compile
