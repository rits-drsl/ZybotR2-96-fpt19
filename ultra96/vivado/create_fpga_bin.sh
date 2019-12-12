#!/bin/sh
SCRIPT_DIR=$(cd $(dirname $0); pwd)

TARGET_FILE_NAME=fpga.bin

TARGET_DIR=$SCRIPT_DIR/../ROOT_FS/firmware

BIT_FILE_DIR=$SCRIPT_DIR/prj/fad_design/fad_design.runs/impl_1

BIT_FILE_NAME=$(ls $BIT_FILE_DIR | grep .*.bit$)

# binファイル生成用のbifファイルを用意する
cat <<EOF > $BIT_FILE_DIR/fpga.bif
all:
{
    [destination_device = pl] $BIT_FILE_DIR/$BIT_FILE_NAME
}
EOF

# binファイルを生成
bootgen -image $BIT_FILE_DIR/fpga.bif -arch zynqmp -w -o $TARGET_DIR/$TARGET_FILE_NAME

# bifファイルを削除
rm $BIT_FILE_DIR/fpga.bif

# メッセージ出力
echo "[SUCCESS] create $TARGET_DIR/$TARGET_FILE_NAME"
