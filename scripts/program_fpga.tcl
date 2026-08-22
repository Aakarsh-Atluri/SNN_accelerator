# ==============================================================================
# Vivado Hardware Manager Script: Flash snn_eth_top.bit to Arty A7-100T
# ==============================================================================

set SCRIPT_DIR [file normalize [file dirname [info script]]]
set BIT_FILE   [file normalize "${SCRIPT_DIR}/../build/snn_eth_top.bit"]

if {![file exists $BIT_FILE]} {
    puts "ERROR: Bitstream file $BIT_FILE not found. Run synthesis/implementation first."
    exit 1
}

puts "Connecting to hardware target..."
open_hw_manager
connect_hw_server -allow_non_jtag
open_hw_target

set device [lindex [get_hw_devices xc7a100t_0] 0]
if {$device == ""} {
    puts "ERROR: Arty A7-100T (xc7a100t_0) not detected."
    exit 1
}

current_hw_device $device
set_property PROGRAM.FILE $BIT_FILE $device

puts "Programming FPGA with $BIT_FILE ..."
program_hw_devices $device
refresh_hw_device $device

puts "FPGA successfully programmed!"
close_hw_target
disconnect_hw_server
close_hw_manager
exit 0
