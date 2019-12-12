#!/bin/bash

export GIT_SSL_NO_VERIFY=1

SCRIPT_DIR=$(cd $(dirname $0); pwd)
mkdir $SCRIPT_DIR/tmpdir && cd $SCRIPT_DIR/tmpdir

git clone https://github.com/opencv/opencv.git
cd $SCRIPT_DIR/tmpdir/opencv
git checkout 3.4
cd $SCRIPT_DIR/tmpdir
git clone https://github.com/opencv/opencv_contrib.git
cd $SCRIPT_DIR/tmpdir/opencv_contrib
git checkout 3.4

apt install -y build-essential
apt install -y cmake git libgtk2.0-dev pkg-config libwebp-dev  libtiff-dev libjasper-dev
apt install -y ccache

yes | wget http://ports.ubuntu.com/ubuntu-ports/pool/universe/c/checkinstall/checkinstall_1.6.2-3ubuntu1_armhf.deb
yes | dpkg -i checkinstall_1.6.2-3ubuntu1_armhf.deb
yes | rm checkinstall_1.6.2-3ubuntu1_armhf.deb

yes | wget http://ftp.debian.org/debian/pool/main/a/auto-apt/auto-apt_0.3.22_armhf.deb
yes | dpkg -i auto-apt_0.3.22_armhf.deb
yes | rm auto-apt_0.3.22_armhf.deb

# build about 3 hours on zynq-7000
mkdir $SCRIPT_DIR/tmpdir/opencv/build && cd $SCRIPT_DIR/tmpdir/opencv/build
cmake $SCRIPT_DIR/tmpdir/opencv
make -j4
make install
/bin/bash -c 'echo "/usr/local/lib" > /etc/ld.so.conf.d/opencv.conf'
ldconfig
rm -rf $SCRIPT_DIR/tmpdir
pkg-config --modversion opencv

export GIT_SSL_NO_VERIFY=0
