`include "gowin_empu/gowin_empu.v"
`include "gowin_pllvr/gowin_pllvr.v"

module top (
    input sys_clk,

    input  uart0_rxd,
    output uart0_txd,

    input  uart1_rxd,
    output uart1_txd,

    input reset_n
);
  wire clk_54mhz_;

  gowin_pllvr_x2 pll_clk_54mhz_ (
      .clkout(clk_54mhz_),  //output clkout
      .clkin(sys_clk)  //input clkin
  );


  gowin_empu_top empu_ (
      .sys_clk(clk_54mhz_),  //input sys_clk
      .uart0_rxd(uart0_rxd),  //input uart0_rxd
      .uart0_txd(uart0_txd),  //output uart0_txd
      .uart1_rxd(uart1_rxd),  //input uart1_rxd
      .uart1_txd(uart1_txd),  //output uart1_txd
      .reset_n(reset_n)  //input reset_n
  );
endmodule
