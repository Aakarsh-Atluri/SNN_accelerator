#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK_DIR="${SCRIPT_DIR}/../sim"
mkdir -p "${WORK_DIR}"
cd "${WORK_DIR}"

echo "=== 1. Compiling Verilog RTL & Testbench with xvlog ==="
xvlog -i "${SCRIPT_DIR}/../rtl" \
      "${SCRIPT_DIR}/../rtl/weight_bram.v" \
      "${SCRIPT_DIR}/../rtl/cascaded_adder.v" \
      "${SCRIPT_DIR}/../rtl/lif_neuron.v" \
      "${SCRIPT_DIR}/../rtl/spike_counter.v" \
      "${SCRIPT_DIR}/../rtl/spike_unpacker.v" \
      "${SCRIPT_DIR}/../rtl/master_fsm.v" \
      "${SCRIPT_DIR}/../rtl/fifo_eth_controller.v" \
      "${SCRIPT_DIR}/../rtl/snn_eth_top.v" \
      "${SCRIPT_DIR}/tb_snn_eth_top.v"

echo "=== 2. Elaborating Snapshot with xelab ==="
xelab -debug typical tb_snn_eth_top -s tb_snn_eth_top_snap

echo "=== 3. Running Simulation with xsim ==="
xsim tb_snn_eth_top_snap -R
