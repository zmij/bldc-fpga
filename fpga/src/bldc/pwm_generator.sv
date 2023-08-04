`ifndef __BLDC_PWM_GENERATOR__
`define __BLDC_PWM_GENERATOR__

//----------------------------------------------------------------------------
// Module for PWM generator
//----------------------------------------------------------------------------
/**
 * @module pwm_generator
 * @brief Generates PWM signals based on the provided clock and frequency settings
 *
 * @param clock_freq_hz The frequency of the clock in Hz (default: 100_286_000)
 * @param pwm_freq_hz The desired PWM frequency in Hz (default: 100_000)
 * @param cnt_size The number of bits required to fit the counter (default: calculated based on clock and PWM frequency)
 * @param enable Input signal to enable/disable the PWM generator
 * @param pwm_clk Input clock signal for the PWM
 * @param duty_width Input signal specifying the width of the duty cycle
 * @param cycle_ticks Output signal indicating the total number of clock ticks in a single PWM cycle
 * @param pwm Output signal representing the PWM output
 */
module pwm_generator #(
    parameter clock_freq_hz = 100_286_000,
    parameter pwm_freq_hz = 100_000,
    parameter cnt_size = $clog2(
        clock_freq_hz / pwm_freq_hz
    ) + 1  // number of bits to fit the counter
) (
    input enable,
    input pwm_clk,
    input [cnt_size - 1:0] duty_width,
    output [cnt_size - 1:0] cycle_ticks,
    output reg pwm
);
  localparam cycle_width = clock_freq_hz / pwm_freq_hz;
  reg [cnt_size - 1:0] cnt;

  assign cycle_ticks = cycle_width;

  always @(posedge pwm_clk or negedge enable) begin
    if (enable == 0) begin
      pwm <= 1'b0;
      cnt <= 0;
    end else if (enable == 1) begin
      if (cnt == cycle_width - 1) cnt <= 0;
      else cnt <= cnt + 1;
      pwm <= (cnt < duty_width) ? 1 : 0;
    end
  end
endmodule

/**
 * @module pwm_channel
 * @brief Represents a single PWM channel
 *
 * @param enable Input signal to enable/disable the PWM channel
 * @param pwm_in Input PWM signal
 * @param pwm_out Output PWM signal
 */
module pwm_channel (
    input  logic enable,
    input  logic pwm_in,
    output logic pwm_out
);
  assign pwm_out = enable & pwm_in;
endmodule

/**
 * @module pwm_commutator
 * @brief Generates multiple PWM channels based on the provided input
 *
 * @param channel_count The number of PWM channels to generate (default: 1)
 * @param channel_enable Input signals to enable/disable individual PWM channels
 * @param pwm_in Input PWM signal to be distributed across channels
 * @param pwm_out Output PWM signals for each channel
 */
module pwm_commutator #(
    parameter channel_count = 1
) (
    input logic [channel_count - 1:0] channel_enable,
    input logic pwm_in,
    output logic [channel_count - 1:0] pwm_out
);
  genvar i;
  generate
    for (i = 0; i < channel_count; ++i) begin : pwm
      pwm_channel chan (
          .enable (channel_enable[i]),
          .pwm_in (pwm_in),
          .pwm_out(pwm_out[i])
      );
    end
  endgenerate
endmodule


`endif  // __BLDC_PWM_GENERATOR__
