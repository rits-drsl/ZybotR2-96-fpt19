#!/bin/bash

export GIT_SSL_NO_VERIFY=1

SCRIPT_DIR=$(cd $(dirname $0); pwd)
mkdir $SCRIPT_DIR/tmpdir && cd $SCRIPT_DIR/tmpdir

# dependent package insatll
apt install -y build-essential \
    cmake git libgtk2.0-dev pkg-config libwebp-dev \
    libtiff-dev libjasper-dev ccache libglew-dev libpython2.7-dev \
    ffmpeg libavcodec-dev libavutil-dev libavformat-dev libswscale-dev libavdevice-dev \
    libdc1394-22-dev libraw1394-dev \
    libjpeg-dev libpng12-dev libtiff5-dev libopenexr-dev

apt install -y libusb-1.0-0-dev
git clone https://github.com/ktossell/libuvc.git
mkdir -p $SCRIPT_DIR/tmpdir/libuvc/build && cd $SCRIPT_DIR/tmpdir/libuvc/build
sed -i -e 's/libusb.h/libusb-1.0\/libusb.h/g' $SCRIPT_DIR/tmpdir/libuvc/include/libuvc/libuvc_internal.h
cmake $SCRIPT_DIR/tmpdir/libuvc
make -j4
make install
cd $SCRIPT_DIR/tmpdir

git clone https://github.com/stevenlovegrove/Pangolin.git
mkdir -p $SCRIPT_DIR/tmpdir/Pangolin/build && cd $SCRIPT_DIR/tmpdir/Pangolin/build
cmake $SCRIPT_DIR/tmpdir/Pangolin
cmake --build .
make install
rm -rf $SCRIPT_DIR/tmpdir

export GIT_SSL_NO_VERIFY=0
