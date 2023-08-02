`include "gowin_empu/gowin_empu.v"
`include "gowin_pllvr/gowin_pllvr.v"
`include "bldc/apb2_bldc.sv"

module top (
    input sys_clk,

    input  uart0_rxd,
    output uart0_txd,

    input  uart1_rxd,
    output uart1_txd,

    input hall_states_t hall_values,
    output [5:0] pwm_out,
    output [3:0] probes,

    input reset_n
);
  wire clk_54mhz_;

  gowin_pllvr_x2 pll_clk_54mhz_ (
      .clkout(clk_54mhz_),  //output clkout
      .clkin(sys_clk)  //input clkin
  );

  gowin_pllvr_100mhz pll_clk_100mhz_ (
      .clkout(clk_100mhz_),  //output clkout
      .clkin(sys_clk)  //input clkin
  );

  // APB2 wires
  wire apb_pclk, apb_prst, apb_penable, apb_pwrite;
  wire [7:0] apb_paddr;
  wire [3:0] apb_pstrb;
  wire [2:0] apb_pprot;
  wire [31:0] apb_pwdata, apb_prdata1;

  wire apb_psel1, apb_pready1, apb_pslverr1;

  // EMPU device instantiation
  gowin_empu_top empu_ (
      .sys_clk  (clk_54mhz_),  //input sys_clk
      .uart0_rxd(uart0_rxd),   //input uart0_rxd
      .uart0_txd(uart0_txd),   //output uart0_txd
      .uart1_rxd(uart1_rxd),   //input uart1_rxd
      .uart1_txd(uart1_txd),   //output uart1_txd

      // Common APB2 wires
      .master_pclk(apb_pclk),  //output master_pclk
      .master_prst(apb_prst),  //output master_prst
      .master_penable(apb_penable),  //output master_penable
      .master_paddr(apb_paddr),  //output [7:0] master_paddr
      .master_pwrite(apb_pwrite),  //output master_pwrite
      .master_pwdata(apb_pwdata),  //output [31:0] master_pwdata
      .master_pstrb(apb_pstrb),  //output [3:0] master_pstrb
      .master_pprot(apb_pprot),  //output [2:0] master_pprot

      // APB2 peripheral 1
      .master_psel1(apb_psel1),  //output master_psel1
      .master_prdata1(apb_prdata1),  //input [31:0] master_prdata1
      .master_pready1(apb_pready1),  //input master_pready1
      .master_pslverr1(apb_pslverr1),  //input master_pslverr1

      .reset_n(reset_n)  //input reset_n
  );

  wire [1:0] dir;
  wire [5:0] phase_enable;

  apb2_bldc_perpheral #(
      .pole_pairs(11)
  ) bldc (
      .pclk(apb_pclk),
      .pwm_clk(clk_100mhz_),

      .preset_n(apb_prst),
      .penable(apb_penable),
      .pwrite(apb_pwrite),
      .paddr(apb_paddr),
      .pwdata(apb_pwdata),
      .pstrb(apb_pstrb),
      .pprot(apb_pprot),

      .psel(apb_psel1),
      .prdata(apb_prdata1),
      .pready(apb_pready1),
      .pslverr(apb_pslverr1),

      .hall_values(hall_values),
      .pwm_out(pwm_out),

      .detected_dir(dir)
  );

  assign probes = {apb_pwrite, apb_psel1, apb_pready1};

endmodule
