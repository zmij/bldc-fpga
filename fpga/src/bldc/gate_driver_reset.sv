`ifndef __BLDC_DRIVER_RESET_SV__
`define __BLDC_DRIVER_RESET_SV__

/**
 * Module for enabling gate driver (like DRV8032) and resetting it by 
 * driving enable line low less than 10µs for fast reset and more than 
 * 20µs for full reset.
 * 
 * The module keeps the out signal equal to incoming. When the start_reset 
 * is set high and the incoming signal is high, the module sets the out line
 * low and starts a counter, after which it sets the outgouing signal to
 * incoming.
 */
module gate_driver_reset #(
    parameter clk_freq_hz   = 54_000_000,
    parameter fast_reset_us = 5,
    parameter slow_reset_us = 20
) (
    input sys_clk,
    input reset_n,
    input logic driver_enable,
    output logic driver_enable_out,
    input logic slow_reset,
    input logic reset_start,  // pull high to start reset
    output logic reset_done
);
  localparam ticks_per_us = clk_freq_hz / 1_000_000;
  localparam fast_reset_ticks = ticks_per_us * fast_reset_us - 1;
  localparam slow_reset_ticks = ticks_per_us * slow_reset_us - 1;
  localparam counter_width = $clog2(slow_reset_ticks) + 1;

  typedef logic [counter_width - 1:0] counter_t;

  counter_t max_;
  counter_t cnt_;

  function counter_t truncate(input logic [counter_width:0] in);
    return in[counter_width-1:0];
  endfunction

  typedef enum logic {
    state_normal,
    state_resetting
  } reset_state_t;

  reset_state_t state_;

  task normal_task();
    begin
      if (reset_start) begin
        state_ <= state_resetting;
        cnt_ <= 0;
        reset_done <= 0;
        if (slow_reset) max_ <= slow_reset_ticks;
        else max_ <= fast_reset_ticks;
      end
    end
  endtask

  task resetting_task();
    begin
      cnt_ <= truncate(cnt_ + 1);
      if (cnt_ == max_) begin
        state_ <= state_normal;
        reset_done <= 1;
      end
    end
  endtask

  always @(posedge sys_clk or negedge reset_n) begin
    if (reset_n == 0) begin
      state_ <= state_normal;
      cnt_ <= 0;
      reset_done <= 0;
    end else begin
      case (state_)
        state_normal: normal_task();
        state_resetting: resetting_task();
      endcase
    end
  end

  assign driver_enable_out = (state_ == state_resetting) ? 0 : driver_enable;

endmodule

`endif  // __BLDC_DRIVER_RESET_SV__
