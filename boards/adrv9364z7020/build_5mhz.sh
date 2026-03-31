#!/bin/bash
# 5MHz mode FPGA build script

set -e

XILINX_DIR=/tools/Xilinx
BOARD_NAME=adrv9364z7020

echo "=========================================="
echo "Starting 5MHz FPGA build for $BOARD_NAME"
echo "Start time: $(date)"
echo "=========================================="

# Source Vivado environment
source $XILINX_DIR/Vivado/2022.2/settings64.sh

cd /home/sdr1/openwifi-hw/boards/$BOARD_NAME

# Step 1: Regenerate IP repo
echo ""
echo "[Step 1/2] Regenerating IP repo..."
echo ""
vivado -mode batch -source ../ip_repo_gen.tcl 2>&1 | tee ip_repo_build_5mhz.log

# Step 2: Build bitstream
echo ""
echo "[Step 2/2] Building bitstream..."
echo ""
vivado -mode batch -source ../openwifi.tcl 2>&1 | tee bitstream_build_5mhz.log

echo ""
echo "=========================================="
echo "Build completed!"
echo "End time: $(date)"
echo "=========================================="

# Check if bitstream was generated
if [ -f "openwifi_$BOARD_NAME/system_top.xsa" ]; then
    echo "SUCCESS: system_top.xsa generated"
    ls -la openwifi_$BOARD_NAME/system_top.xsa
else
    echo "ERROR: system_top.xsa not found!"
    exit 1
fi
