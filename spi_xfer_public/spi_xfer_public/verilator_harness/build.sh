#!/bin/bash
# Build script for Verilator compilation of spi_xfer_top
# Usage: ./build.sh [verilator_path]
#
# The hidden configuration values SPEC_DFS_MIN / SPEC_HOLD_SS are compile-time
# constant input pins driven by sim_main.cpp (defaults 4 / 4).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DUT_DIR="${SCRIPT_DIR}/../dut"
OBJ_DIR="${SCRIPT_DIR}/obj_dir"

# Clean previous build
rm -rf "${OBJ_DIR}"
mkdir -p "${OBJ_DIR}"

VERILATOR=${1:-verilator}

echo "=== Building spi_xfer_public with Verilator ==="

${VERILATOR} \
    --cc \
    --exe \
    --public \
    --trace \
    -Wno-UNUSEDSIGNAL \
    -Wno-UNUSEDPARAM \
    -Wno-WIDTH \
    -Wno-LATCH \
    --top-module spi_xfer_top \
    -Mdir "${OBJ_DIR}" \
    "${SCRIPT_DIR}/sim_main.cpp" \
    "${DUT_DIR}/spi_sclkgen.v" \
    "${DUT_DIR}/spi_mstfsm.v" \
    "${DUT_DIR}/spi_shift.v" \
    "${DUT_DIR}/spi_fifo.v" \
    "${DUT_DIR}/spi_regfile.v" \
    "${DUT_DIR}/spi_xfer_top.v"

echo "=== Building Verilator library ==="
cd "${OBJ_DIR}"
make -j$(nproc) -f Vspi_xfer_top.mk Vspi_xfer_top

echo "=== Build complete ==="
echo "Executable: ${OBJ_DIR}/Vspi_xfer_top"
