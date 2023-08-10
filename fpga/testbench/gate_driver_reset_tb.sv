`include "bldc/gate_driver_reset.sv"
`timescale 10ns / 10ns

module gate_driver_reset_tb;
  localparam tick = 2;
  reg   clk = 1;
  logic reset_n = 0;
  logic driver_enable = 0;
  logic slow_reset = 0;
  logic reset_start = 0;

  logic driver_enable_out;
  logic reset_done;

  always #1 clk = ~clk;

  initial begin
    $dumpfile("gate_driver_reset_tb");
    $dumpvars(0, gate_driver_reset_tb);
  end

  gate_driver_reset #(
      .clk_freq_hz(50_000_000),  // 20ns, #2 == 1 tick, 1µs = 50 ticks (#100)
      .fast_reset_us(1),  // 50 ticks
      .slow_reset_us(5)  // 250 ticks
  ) dut (
      .sys_clk(clk),
      .reset_n(reset_n),
      .driver_enable(driver_enable),
      .driver_enable_out(driver_enable_out),
      .slow_reset(slow_reset),
      .reset_start(reset_start),
      .reset_done(reset_done)
  );

  initial begin
    #tick;
    reset_n <= 1;
    #tick;
    assert (driver_enable_out == driver_enable)
    else $error("Out enable must be equal to in enable (low)");
    driver_enable <= 1;
    #tick;
    assert (driver_enable_out == driver_enable)
    else $error("Out enable must be equal to in enable (high)");

    // Fast reset
    reset_start <= 1;
    #tick;
    reset_start <= 0;
    #tick;

    assert (driver_enable_out == 0)
    else $error("Out enable must be low (resetting)");
    assert (reset_done == 0)
    else $error("Reset done must be low");

    #(tick * 49);
    assert (driver_enable_out == 0)
    else $error("Out enable must be low (resetting)");
    assert (reset_done == 0)
    else $error("Reset done must be low");
    #(tick);
    assert (driver_enable_out == driver_enable)
    else $error("Out enable must be equal to in enable (after reset)");
    assert (reset_done == 1)
    else $error("Reset done must be high");


    // Slow reset
    #tick;
    reset_start <= 1;
    slow_reset  <= 1;
    #tick;
    reset_start <= 0;
    #2;
    assert (driver_enable_out == 0)
    else $error("Out enable must be low (resetting)");
    assert (reset_done == 0)
    else $error("Reset done must be low");
    #(tick * 249);
    assert (driver_enable_out == 0)
    else $error("Out enable must be low (resetting)");
    assert (reset_done == 0)
    else $error("Reset done must be low");
    #tick;
    assert (driver_enable_out == driver_enable)
    else $error("Out enable must be equal to in enable (after reset)");
    assert (reset_done == 1)
    else $error("Reset done must be high");

    #tick;
    driver_enable <= 0;
    #tick;
    assert (driver_enable_out == driver_enable)
    else $error("Out enable must be equal to in enable (low)");
    $finish;
  end
endmodule
