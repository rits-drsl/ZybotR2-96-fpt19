#!/bin/bash

export GIT_SSL_NO_VERIFY=1
apt purge -y cmake
SCRIPT_DIR=$(cd $(dirname $0); pwd)
mkdir $SCRIPT_DIR/tmpdir && cd $SCRIPT_DIR/tmpdir

wget https://github.com/Kitware/CMake/releases/download/v3.13.4/cmake-3.13.4.tar.gz
tar xvf cmake-3.13.4.tar.gz
cd $SCRIPT_DIR/tmpdir/cmake-3.13.4

./bootstrap
make -j4
make install
ln -s /opt/cmake-3.13.4-Linux-x86_64/bin/* /usr/bin
echo "export PATH=/usr/local/bin:\$PATH" >> ~/.bashrc
source .bashrc
cmake --version

rm -rf $SCRIPT_DIR/tmpdir

export GIT_SSL_NO_VERIFY=0
