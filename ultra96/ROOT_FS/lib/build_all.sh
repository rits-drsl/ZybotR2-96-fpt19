#!/bin/sh
SCRIPT_DIR=$(cd $(dirname $0); pwd)

# zynqpl
cd $SCRIPT_DIR/zynqpl
mkdir -p build && cd build
cmake .. && make -j4
ln -sfn $SCRIPT_DIR/zynqpl/build/libzynqpl.so $SCRIPT_DIR/libzynqpl.so
ln -sfn $SCRIPT_DIR/zynqpl/include $SCRIPT_DIR/include/zynqpl

# imploc
cd $SCRIPT_DIR/improc
mkdir -p build && cd build
cmake .. && make -j4
ln -sfn $SCRIPT_DIR/improc/build/libimproc.so $SCRIPT_DIR/libimproc.so
ln -sfn $SCRIPT_DIR/improc/include $SCRIPT_DIR/include/improc

# control
cd $SCRIPT_DIR/control
mkdir -p build && cd build
cmake .. && make -j4
ln -sfn $SCRIPT_DIR/control/build/libcontrol.so $SCRIPT_DIR/libcontrol.so
ln -sfn $SCRIPT_DIR/control/include $SCRIPT_DIR/include/control

# optor
cd $SCRIPT_DIR/optor
mkdir -p build && cd build
cmake .. && make -j4
ln -sfn $SCRIPT_DIR/optor/build/liboptor.so $SCRIPT_DIR/liboptor.so
ln -sfn $SCRIPT_DIR/optor/include $SCRIPT_DIR/include/optor

# planner
cd $SCRIPT_DIR/planner
mkdir -p build && cd build
cmake .. && make -j4
ln -sfn $SCRIPT_DIR/planner/build/libplanner.so $SCRIPT_DIR/libplanner.so
ln -sfn $SCRIPT_DIR/planner/include $SCRIPT_DIR/include/planner
