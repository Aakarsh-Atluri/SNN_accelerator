## =============================================================================
## Constraints for SNN Accelerator on Digilent Arty A7-100T (xc7a100tcsg324-1)
## =============================================================================

## Clock signal (100 MHz onboard oscillator)
set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports { clk }];
create_clock -add -name sys_clk_pin -period 10.000 -waveform {0 5} [get_ports { clk }];

## Reset button (RESET pushbutton / CPU_RESETN, active-LOW)
set_property -dict { PACKAGE_PIN C2    IOSTANDARD LVCMOS33 } [get_ports { rst_n }];

## USB-UART Interface (FTDI FT2232HQ Bridge)
## rx: FPGA input from PC (FTDI TXD -> FPGA RXD)
## tx: FPGA output to PC (FPGA TXD -> FTDI RXD)
set_property -dict { PACKAGE_PIN A9    IOSTANDARD LVCMOS33 } [get_ports { rx }];
set_property -dict { PACKAGE_PIN D10   IOSTANDARD LVCMOS33 } [get_ports { tx }];

## User LEDs
## result_led: LED 0 (LD0, Green) - 1 = Collision detected, 0 = Safe
## done_led:   LED 1 (LD1, Green) - 1 = Inference complete
set_property -dict { PACKAGE_PIN H5    IOSTANDARD LVCMOS33 } [get_ports { result_led }];
set_property -dict { PACKAGE_PIN J5    IOSTANDARD LVCMOS33 } [get_ports { done_led }];

## Configuration Voltage and SPI Flash Bitstream Properties
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
