#!/bin/bash

export GIT_SSL_NO_VERIFY=1

SCRIPT_DIR=$(cd $(dirname $0); pwd)
mkdir $SCRIPT_DIR/tmpdir && cd $SCRIPT_DIR/tmpdir

apt install -y build-essential \
    cmake git libgtk2.0-dev pkg-config libwebp-dev  libtiff-dev libjasper-dev ccache

wget http://bitbucket.org/eigen/eigen/get/3.3.7.tar.gz
tar -zxf 3.3.7.tar.gz
mkdir $SCRIPT_DIR/tmpdir/eigen-eigen-323c052e1731/build && cd $SCRIPT_DIR/tmpdir/eigen-eigen-323c052e1731/build
cmake $SCRIPT_DIR/tmpdir/eigen-eigen-323c052e1731
make -j4
make install
rm -rf $SCRIPT_DIR/tmpdir

export GIT_SSL_NO_VERIFY=0
