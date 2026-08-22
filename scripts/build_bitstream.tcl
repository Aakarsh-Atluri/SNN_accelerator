# ==============================================================================
# Vivado Non-Project Batch Flow: Build Bitstream for snn_eth_top
# Target Board: Digilent Arty A7-100T (xc7a100tcsg324-1)
# ==============================================================================

set SCRIPT_DIR [file normalize [file dirname [info script]]]
set PROJ_DIR   [file normalize "${SCRIPT_DIR}/.."]
set BUILD_DIR  "${PROJ_DIR}/build"

file mkdir $BUILD_DIR
cd $BUILD_DIR

set_param general.maxThreads 8

puts "=================================================================="
puts "  STARTING SYNTHESIS: snn_eth_top (Ethernet SNN Accelerator)      "
puts "=================================================================="

# 1. Read Verilog Design Files
read_verilog "${PROJ_DIR}/rtl/weight_bram.v"
read_verilog "${PROJ_DIR}/rtl/cascaded_adder.v"
read_verilog "${PROJ_DIR}/rtl/lif_neuron.v"
read_verilog "${PROJ_DIR}/rtl/spike_counter.v"
read_verilog "${PROJ_DIR}/rtl/spike_unpacker.v"
read_verilog "${PROJ_DIR}/rtl/master_fsm.v"
read_verilog "${PROJ_DIR}/rtl/fifo_eth_controller.v"
read_verilog "${PROJ_DIR}/rtl/snn_eth_top.v"

# 2. Read Constraints
read_xdc "${PROJ_DIR}/constraints/arty_a7_100t_eth.xdc"

# 3. Synthesize Design
synth_design -top snn_eth_top -part xc7a100tcsg324-1 -flatten_hierarchy rebuilt
write_checkpoint -force post_synth.dcp
report_timing_summary -file post_synth_timing_summary.rpt
report_utilization -file post_synth_util.rpt

puts "=================================================================="
puts "  RUNNING IMPLEMENTATION & ROUTING                                "
puts "=================================================================="

# 4. Opt & Place Design
opt_design
place_design
report_clock_utilization -file post_place_clock_util.rpt

# 5. Route Design
route_design
write_checkpoint -force post_route.dcp
report_timing_summary -file post_route_timing_summary.rpt
report_utilization -file post_route_util.rpt

puts "=================================================================="
puts "  GENERATING BITSTREAM: snn_eth_top.bit                           "
puts "=================================================================="

# 6. Write Bitstream
write_bitstream -force snn_eth_top.bit

puts "=================================================================="
puts "  BUILD COMPLETED SUCCESSFULLY! Bitstream: ${BUILD_DIR}/snn_eth_top.bit"
puts "=================================================================="
