#!/bin/sh
export ARCH=arm
export CROSS_COMPILE=arm-none-linux-gnueabi-

if [ "x$1" = "xconfig" ]; then
    make distclean
    make t66_config
else
    if [ -f u-boot.bin ]; then
        rm -f u-boot.bin
    fi

    if [ -f u-boot.imx ]; then
        rm -f u-boot.imx
    fi

    if [ -f u-boot.bin_noheader ]; then
        rm -f u-boot.bin_noheader
    fi
fi

make -j4

if [ -f u-boot.bin ]; then
    cp u-boot.bin u-boot.usb

    if [ -f u-boot.usb ]; then
        ls -l u-boot.usb
    else
        echo "no u-boot.usb found"
    fi

    if [ -f u-boot.imx ]; then
        ls -l u-boot.imx
    else
        echo "no u-boot.imx found"
    fi
else
    echo "Build fail...."
fi
