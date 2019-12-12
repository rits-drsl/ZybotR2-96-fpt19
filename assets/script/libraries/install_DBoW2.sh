#!/bin/bash

export GIT_SSL_NO_VERIFY=1

SCRIPT_DIR=$(cd $(dirname $0); pwd)

mkdir $SCRIPT_DIR/tmpdir && cd $SCRIPT_DIR/tmpdir
apt install -y build-essential \
    cmake git libgtk2.0-dev pkg-config libwebp-dev  libtiff-dev libjasper-dev \
    ccache \
    libboost-dev

git clone https://github.com/dorian3d/DBoW2.git
mkdir $SCRIPT_DIR/tmpdir/DBoW2/build && cd $SCRIPT_DIR/tmpdir/DBoW2/build
cmake $SCRIPT_DIR/tmpdir/DBoW2
make -j4
make install
rm -rf $SCRIPT_DIR/tmpdir

export GIT_SSL_NO_VERIFY=0
