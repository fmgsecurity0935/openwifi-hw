#!/bin/bash
# 20MHz AP (OPP) FPGA build script
# Builds standard 20MHz bitstream with Opportunistic TX (OPP) logic in xpu.

set -e

XILINX_DIR=/tools/Xilinx
BOARD_NAME=adrv9364z7020
BOARD_DIR=/home/sdr1/openwifi-hw/boards/$BOARD_NAME
IP_DIR=/home/sdr1/openwifi-hw/ip
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_BIN="${BOARD_DIR}/system_top_20mhz_opp_v22_${TIMESTAMP}.bit.bin"

echo "=========================================="
echo "20MHz AP (OPP v22: dual-DFT LTF+L-SIG with clock-enable) FPGA Build"
echo "Board : $BOARD_NAME"
echo "Start : $(date)"
echo "=========================================="

# ── Step 0: Set board_def.v to 20MHz ──────────────────────────────────────
echo ""
echo "[Step 0] Setting SAMPLING_RATE_MHZ=20 in board_def.v..."
BOARD_DEF="$IP_DIR/board_def.v"
# Revert SAMPLING_RATE_MHZ from 5 to 20
sed -i 's/`define SAMPLING_RATE_MHZ.*5/`define SAMPLING_RATE_MHZ       20/' "$BOARD_DEF"
grep "SAMPLING_RATE_MHZ" "$BOARD_DEF"
echo "board_def.v updated."

# ── Step 1: Regenerate IP repo ─────────────────────────────────────────────
echo ""
echo "[Step 1/2] Regenerating IP repo..."
# CRITICAL: Delete existing ip_repo to force full re-synthesis of xpu IP
# Without this, Vivado reuses cached IP → source changes (e.g., opp_cooldown) silently ignored
# (This was confirmed root cause of v11 = v10 performance: ip_repo NOT deleted between builds)
if [ -d "${BOARD_DIR}/ip_repo" ]; then
    echo "Deleting stale ip_repo cache: ${BOARD_DIR}/ip_repo"
    rm -rf "${BOARD_DIR}/ip_repo"
    echo "ip_repo deleted."
fi
source $XILINX_DIR/Vivado/2022.2/settings64.sh
cd "$BOARD_DIR"
vivado -mode batch -source ../ip_repo_gen.tcl 2>&1 | tee "ip_repo_20mhz_opp_${TIMESTAMP}.log"

# ── Step 2: Build bitstream ────────────────────────────────────────────────
echo ""
echo "[Step 2/2] Building bitstream (clean project — no synthesis cache)..."
# Remove existing Vivado project to force full re-synthesis
# Without this, Vivado reuses cached synth_1 and misses IP source changes
PROJECT_DIR="${BOARD_DIR}/openwifi_${BOARD_NAME}"
if [ -d "$PROJECT_DIR" ]; then
    echo "Removing existing project: $PROJECT_DIR"
    rm -rf "$PROJECT_DIR"
fi
vivado -mode batch -source ../openwifi.tcl 2>&1 | tee "bitstream_20mhz_opp_${TIMESTAMP}.log"

echo ""
echo "=========================================="
echo "Build completed!"
echo "End time: $(date)"
echo "=========================================="

# ── Step 3: Convert .bit → .bit.bin via bootgen ───────────────────────────
# Vivado writes impl_1/system_top.bit (not system_top.bit.bin in project root)
BITFILE="${BOARD_DIR}/openwifi_${BOARD_NAME}/openwifi_${BOARD_NAME}.runs/impl_1/system_top.bit"
BOOTGEN="${XILINX_DIR}/Vivado/2022.2/bin/bootgen"
BIF_FILE="/tmp/opp_v22_build.bif"

if [ ! -f "$BITFILE" ]; then
    echo "ERROR: $BITFILE not found!"
    exit 1
fi

cat > "$BIF_FILE" <<EOF
all:
{
    $BITFILE
}
EOF

echo "Running bootgen to convert .bit → .bit.bin ..."
"$BOOTGEN" -image "$BIF_FILE" -arch zynq -o "$OUTPUT_BIN" -w on

if [ -f "$OUTPUT_BIN" ]; then
    echo "SUCCESS: bitstream saved to $OUTPUT_BIN"
    ls -lh "$OUTPUT_BIN"
else
    echo "ERROR: bootgen failed, $OUTPUT_BIN not created!"
    exit 1
fi
