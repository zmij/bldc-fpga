`include "bldc/window_counter.sv"

`timescale 10ns / 10ns

module window_counter_tb;
  localparam clk_freq_hz = 50_000_000;  // 20ns, #2 == 1 tick, 1µs = 50 ticks (#100)
  localparam tick = 2;
  localparam us = tick * 50;
  localparam ms = us * 1000;

  logic clk = 1;
  logic reset_n = 1;

  always #1 clk = ~clk;

  initial begin
    $dumpfile("window_counter_tb");
    $dumpvars(0, window_counter_tb);
  end

  localparam counter_width = 12;

  logic event_;
  logic [counter_width - 1:0] counter_;
  logic valid_;


  window_counter #(
      .clk_freq_hz(clk_freq_hz),
      .sample_time_ms(5),
      .sample_count(10)
  ) dut_ (
      .sys_clk(clk),

      .enable (event_),
      .counter(counter_),
      .valid  (valid_),

      .reset_n(reset_n)
  );

  task emit_event();
    begin
      event_ = 1;
      #tick;
      event_ = 0;
    end
  endtask

  initial begin
    event_  = 0;
    reset_n = 0;
    #tick;
    reset_n = 1;

    #tick;

    for (int i = 0; i < 50; ++i) begin
      emit_event();
      #(ms * 2);

      if (i < 24) begin
        assert (valid_ == 0)
        else $error("Valid signal unexpectedly went high at %d ms", i * 2);
      end else begin
        assert (valid_ == 1)
        else $error("Valid signal unexpectedly went low at %d ms", i * 2);
        assert (counter_ == 25)
        else $error("Invalid counter value %d at %d", counter_, i * 2);
      end

    end

    #ms;
    $finish;
  end
endmodule
