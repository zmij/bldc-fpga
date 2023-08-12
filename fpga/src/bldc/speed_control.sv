`ifndef __BLDC_SPEED_CONTROL_SV__
`define __BLDC_SPEED_CONTROL_SV__

`include "bldc/types.sv"

module speed_control #(
    parameter clk_freq_hz = 27_000_000,  // The frequency of the clock in Hertz
    parameter counter_width = 32,  // The width of the internal counter
    parameter pwm_clk_freq_hz = 100_286_000,
    parameter pwm_freq_hz = 100_000,
    parameter pwm_counter_width = counter_width,
    parameter pole_pairs = 1,  // The number of pole pairs in the motor
    parameter rpm_counter_width = 12,
    parameter rpm_measurement_ms = 100,  // Period of RPM management in ms
    parameter max_rpm = 450  // Max RPM
) (
    input logic sys_clk,
    input logic enable,

    input logic [rpm_counter_width - 1:0] rpm,
    input rotation_direction_t current_direction,

    input logic [rpm_counter_width - 1:0] target_rpm,
    input rotation_direction_t target_direction,

    //@{
    /** @name Passtrhu when disabled */
    input logic [pwm_counter_width - 1:0] pwm_duty_in,
    input logic driver_enable_in,
    //@}

    output logic [pwm_counter_width - 1:0] pwm_duty_out,
    output logic driver_enable_out,

    input logic reset_n
);
  typedef logic [pwm_counter_width - 1:0] pwm_t;
  typedef logic signed [pwm_counter_width:0] signed_pwm_t;
  typedef logic [rpm_counter_width - 1:0] rpm_t;
  typedef logic signed [rpm_counter_width:0] signed_rpm_t;

  localparam pwm_t pwm_cycle_ticks = pwm_clk_freq_hz / pwm_freq_hz;
  localparam pwm_t max_pwm = pwm_cycle_ticks * 4 / 10;

  localparam KP = 10;
  localparam KI = 1;

  signed_rpm_t current_rpm_;
  signed_rpm_t error_, error_prev_, error_integral_, error_diff_;

  signed_pwm_t pwm_duty_change_;

  pwm_t calculated_pwm_;

  task automatic reset();
    calculated_pwm_ <= 0;
  endtask

  task automatic calculate_pwm();
    begin
      if (target_direction == DIR_NONE || target_direction == DIR_BRAKE) begin
        calculated_pwm_ <= 0;
      end else begin
      end
    end
  endtask

  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (!reset_n || !enable) begin
      reset();
    end else begin
      calculate_pwm();
    end
  end

  assign current_rpm_ = (current_direction == target_direction) ? rpm : -rpm;
  assign pwm_duty_out = enable ? calculated_pwm_ : pwm_duty_in;
  assign driver_enable_out = enable ? 1 : driver_enable_in;

endmodule

`endif  // __BLDC_SPEED_CONTROL_SV__
