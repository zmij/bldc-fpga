`ifndef __BLDC_DEBOUNCE_SV__
`define __BLDC_DEBOUNCE_SV__

`timescale 100ns / 1ns

module debounce #(
    parameter counter_sz = 16
) (
    input clk,
    input signal_in,
    output reg signal_out,
    output signal_up,
    output signal_down,
    output signal_change
);
  typedef logic [counter_sz - 1:0] counter_t;
  reg [1:0] signal_synk_;
  counter_t cnt_ = 0;

  function counter_t truncate(input logic [counter_sz:0] in);
    return in[counter_sz-1:0];
  endfunction

  always @(posedge clk) begin
    signal_synk_[0] <= signal_in;
    signal_synk_[1] <= signal_synk_[0];
  end

  wire signal_idle_ = (signal_out == signal_synk_[1]);
  wire cnt_at_max_ = &cnt_;

  always @(posedge clk) begin
    if (signal_idle_) begin
      cnt_ <= 0;
    end else begin
      cnt_ <= truncate(cnt_ + 1);
      if (cnt_at_max_) begin
        signal_out <= signal_synk_[1];
      end
    end
  end

  assign signal_down = ~signal_idle_ & cnt_at_max_ & ~signal_out;
  assign signal_up = ~signal_idle_ & cnt_at_max_ & signal_out;
  assign signal_change = ~signal_idle_ & cnt_at_max_;

endmodule

module debounce_us #(
    parameter clk_freq_hz = 54_000_000,
    parameter debounce_time_us = 50
) (
    input clk,
    input signal_in,
    output reg signal_out,
    output signal_up,
    output signal_down,
    output signal_change
);
  localparam ticks_per_us = clk_freq_hz / 1_000_000;
  localparam counter_sz = $clog2(debounce_time_us * ticks_per_us);

  debounce #(
      .counter_sz(counter_sz)
  ) deb_ (
      .clk(clk),
      .signal_in(signal_in),
      .signal_out(signal_out),
      .signal_up(signal_up),
      .signal_down(signal_down),
      .signal_change(signal_change)
  );

endmodule

`endif  // __BLDC_DEBOUNCE_SV__
