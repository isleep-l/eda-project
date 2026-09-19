#!/bin/bash
# dma_xfer_public - Verilator build + Python binding (Linux/WSL).
# Builds dma_top (+ arbiter) and links sim_main.cpp to produce the runnable
# Vdma_top executable.
#
# The hidden configuration values ARB_PRIO / DONE_OK_MODES / XFER_HOLD_N are
# public Verilator module parameters baked in at BUILD time via -G. They are
# not exposed as runtime-assignable members (Verilator >=5.020 has no such
# binding), so sim_main.cpp does NOT set them.
#
# Defaults here are the PUBLIC PLACEHOLDERS (ascending priority, inert
# done_ok modes, hold=16) — identical out-of-box semantics to local_sim.py.
# The eval-side truth is injected only by setting the environment variables
# before calling build.sh (internal verification); no secret is committed.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DUT_DIR="${SCRIPT_DIR}/../dut"
OBJ_DIR="${SCRIPT_DIR}/obj_dir"

# Hidden config values; override via the environment when verifying.
# !!! Keep these as the public placeholders — never bake eval truth here.
ARB_PRIO=${ARB_PRIO:-0x30}
DONE_OK_MODES=${DONE_OK_MODES:-0}
XFER_HOLD_N=${XFER_HOLD_N:-16}

# Clean previous build
rm -rf "${OBJ_DIR}"
mkdir -p "${OBJ_DIR}"

VERILATOR=${1:-verilator}

echo "=== Building dma_xfer_public with Verilator ==="
echo "  ARB_PRIO=$ARB_PRIO  DONE_OK_MODES=$DONE_OK_MODES  XFER_HOLD_N=$XFER_HOLD_N"

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
    -Wno-BLKANDNBLK \
    -GARB_PRIO="${ARB_PRIO}" \
    -GDONE_OK_MODES="${DONE_OK_MODES}" \
    -GXFER_HOLD_N="${XFER_HOLD_N}" \
    --top-module dma_top \
    -Mdir "${OBJ_DIR}" \
    "${SCRIPT_DIR}/sim_main.cpp" \
    "${DUT_DIR}/dma_top.v" \
    "${DUT_DIR}/arbiter.v"

echo "=== Building Verilator library ==="
cd "${OBJ_DIR}"
make -j$(nproc) -f Vdma_top.mk Vdma_top

echo "=== Build complete ==="
echo "Executable: ${OBJ_DIR}/Vdma_top"
