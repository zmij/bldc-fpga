//Copyright (C)2014-2023 GOWIN Semiconductor Corporation.
//All rights reserved.
//File Title: Timing Constraints file
//GOWIN Version: 1.9.9 Beta-1
//Created Time: 2023-08-07 11:25:16
create_clock -name xtal -period 37.037 -waveform {0 18.518} [get_ports {sys_clk}]
create_clock -name apb_pclk -period 18.519 -waveform {0 9.259} [get_nets {apb_pclk}]
create_clock -name pwm_clk -period 10 -waveform {0 5} [get_nets {clk_100mhz_}]
