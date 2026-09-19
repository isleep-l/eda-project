#!/bin/bash
# Build script for Verilator compilation of spi_m_top
# Usage: ./build.sh [verilator_path]
#
# The hidden configuration values SPEC_CFS_MIN / SPEC_HOLD_SS / SPEC_TXFTLR_DFLT
# are constant input pins driven by sim_main.cpp (defaults 8 / 4 / 1).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DUT_DIR="${SCRIPT_DIR}/../dut"
OBJ_DIR="${SCRIPT_DIR}/obj_dir"

# Clean previous build
rm -rf "${OBJ_DIR}"
mkdir -p "${OBJ_DIR}"

VERILATOR=${1:-verilator}

echo "=== Building spi_master_public with Verilator ==="

${VERILATOR} \
    --cc \
    --exe \
    --public \
    --trace \
    -Wno-UNUSEDSIGNAL \
    -Wno-UNUSEDPARAM \
    -Wno-WIDTH \
    -Wno-LATCH \
    -Wno-MULTIDRIVEN \
    --top-module spi_m_top \
    -Mdir "${OBJ_DIR}" \
    "${SCRIPT_DIR}/sim_main.cpp" \
    "${DUT_DIR}/spi_m_sclkgen.v" \
    "${DUT_DIR}/spi_m_mstfsm.v" \
    "${DUT_DIR}/spi_m_shift.v" \
    "${DUT_DIR}/spi_m_fifo.v" \
    "${DUT_DIR}/spi_m_intctl.v" \
    "${DUT_DIR}/spi_m_regfile.v" \
    "${DUT_DIR}/spi_m_top.v"

echo "=== Building Verilator library ==="
cd "${OBJ_DIR}"
make -j$(nproc) -f Vspi_m_top.mk Vspi_m_top

echo "=== Build complete ==="
echo "Executable: ${OBJ_DIR}/Vspi_m_top"
