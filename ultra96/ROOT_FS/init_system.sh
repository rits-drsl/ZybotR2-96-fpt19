#!/bin/sh
SCRIPT_DIR=$(cd $(dirname $0); pwd)
DTO_DIR=/sys/kernel/config/device-tree/overlays

# カーネルモジュールのインストール
lsmod | grep "v4l2"
if [ $? = 0 ]; then
    rmmod v4l2
fi
cd $SCRIPT_DIR/driver/v4l2/build
make
insmod v4l2.ko

cd $SCRIPT_DIR/driver/usbserial
insmod usbserial.ko

cd $SCRIPT_DIR/driver/cp210x
insmod cp210x.ko

# bitstreamファイルのインストール
cp $SCRIPT_DIR/firmware/fpga.bin /lib/firmware

# device tree overlayの実行
## congigurarion fpga
mkdir -p $DTO_DIR/fpga
dtc -I dts -O dtb -o $SCRIPT_DIR/dts/fpga-load.dtb $SCRIPT_DIR/dts/fpga-load.dts
cp $SCRIPT_DIR/dts/fpga-load.dtb $DTO_DIR/fpga/dtbo

## gpio
mkdir -p $DTO_DIR/gpio
dtc -I dts -O dtb -o $SCRIPT_DIR/dts/gpio.dtb $SCRIPT_DIR/dts/gpio.dts
cp $SCRIPT_DIR/dts/gpio.dtb $DTO_DIR/gpio/dtbo

## preimproc
mkdir -p $DTO_DIR/preimproc
dtc -I dts -O dtb -o $SCRIPT_DIR/dts/preimproc.dtb $SCRIPT_DIR/dts/preimproc.dts
cp $SCRIPT_DIR/dts/preimproc.dtb $DTO_DIR/preimproc/dtbo

## v4l2 driver
mkdir -p $DTO_DIR/v4l2
dtc -I dts -O dtb -o $SCRIPT_DIR/dts/v4l2.dtb $SCRIPT_DIR/dts/v4l2.dts
cp $SCRIPT_DIR/dts/v4l2.dtb $DTO_DIR/v4l2/dtbo

# 共有ライブラリのbuild
cd $SCRIPT_DIR/lib
sh build_all.sh
