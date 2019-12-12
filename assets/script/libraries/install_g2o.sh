#!/bin/bash

export GIT_SSL_NO_VERIFY=1

SCRIPT_DIR=$(cd $(dirname $0); pwd)
mkdir $SCRIPT_DIR/tmpdir && cd $SCRIPT_DIR/tmpdir

apt install -y build-essential \
    cmake git libgtk2.0-dev pkg-config libwebp-dev  libtiff-dev libjasper-dev ccache \
    libsuitesparse-dev - qtdeclarative5-dev - qt5-qmake - libqglviewer-dev \
    libeigen3-dev

git clone https://github.com/RainerKuemmerle/g2o.git
mkdir $SCRIPT_DIR/tmpdir/g2o/build && cd $SCRIPT_DIR/tmpdir/g2o/build
cmake $SCRIPT_DIR/tmpdir/g2o
make -j4
make install
rm -rf $SCRIPT_DIR/tmpdir

export GIT_SSL_NO_VERIFY=0
