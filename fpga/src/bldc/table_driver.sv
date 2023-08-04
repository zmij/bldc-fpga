`ifndef __BLDC_TABLE_DRIVER_SV__
`define __BLDC_TABLE_DRIVER_SV__

`include "bldc/types.sv"
`include "bldc/encoder.sv"
`include "bldc/table_commutator.sv"
`include "bldc/pwm_generator.sv"
`include "bldc/debounce.sv"

module table_bldc_driver #(
    parameter clk_freq_hz = 54_000_000,
    parameter pwm_clk_freq_hz = 100_286_000,
    parameter pwm_freq_hz = 100_000,
    parameter pole_pairs = 1,
    parameter counter_width = 32,
    parameter pwm_counter_width = $clog2(pwm_clk_freq_hz / pwm_freq_hz) + 1
) (
    input sys_clk,
    input pwm_clk,
    //@{
    /** @name Motor feedback interface */
    //@}
    input hall_states_t hall_values,  // TODO debounce values
    input fault_n,  // Driver fault, active low
    input overcurrent_n,  // Overcurrent warning, active low
    //@{
    /** @name Encoder interface */
    output rotation_direction_t detected_dir,
    output logic [counter_width - 1:0] encoder_counter,
    output logic [counter_width - 1:0] rotation_duration,
    output logic [counter_width - 1:0] rpm,
    output logic [2:0] sector,
    output hall_error,
    //@}
    //@{
    /** @name Motor control interface*/
    output logic [5:0] phase_enable,
    output logic [5:0] pwm_out,
    output gate_enable,
    //@}
    //@{
    /** @name Driver control interface */
    input enable,
    input rotation_direction_t direction,
    input logic [pwm_counter_width - 1:0] pwm_duty,
    //@}
    //@{
    /** @name Information interface */
    output logic [pwm_counter_width - 1:0] pwm_cycle_ticks,
    output logic [2:0] driver_state,
    //@}
    input reset_n
);
  typedef enum logic [2:0] {
    state_idle = 'd0,
    state_startup = 'd1,
    state_reset_gate = 'd2,
    state_run = 'd3,
    state_error = 'd4
  } driver_state_t;

  driver_state_t state_;
  rotation_direction_t desired_direction_;
  hall_states_t hall_values_debounced_;

  genvar i;
  generate
    for (i = 0; i < 3; ++i) begin : hall_debounce
      debounce_us #(
          .clk_freq_hz(clk_freq_hz),
          .debounce_time_us(50)
      ) db (
          .clk(sys_clk),
          .signal_in(hall_values[i]),
          .signal_out(hall_values_debounced_[i])
      );
    end
  endgenerate

  //--------------------------------------------------------------------------
  // Encoder instance
  //--------------------------------------------------------------------------
  three_phase_encoder #(
      .clk_freq_hz(clk_freq_hz),
      .pole_pairs(pole_pairs),
      .counter_width(counter_width)
  ) encoder_ (
      .clk(sys_clk),
      .reset_n(reset_n),
      .hall_values(hall_values_debounced_),
      .overall_counter(encoder_counter),
      .rotation_direction(detected_dir),
      .rotation_duration(rotation_duration),
      .rpm(rpm),
      .sector(sector)
  );

  //--------------------------------------------------------------------------
  // BLDC commutation table
  //--------------------------------------------------------------------------
  bldc_commutation_table comm_table_ (
      .clk(sys_clk),
      .dir(direction),
      .hall_values(hall_values_debounced_),
      .phase_enable(phase_enable),
      .error(hall_error)
  );
  //--------------------------------------------------------------------------
  // PWM generation
  //--------------------------------------------------------------------------
  wire pwm_;
  assign gate_enable = reset_n & enable & ~hall_error;

  pwm_generator #(
      .clock_freq_hz(pwm_clk_freq_hz),
      .pwm_freq_hz  (pwm_freq_hz)
  ) pwm_gen_ (
      .enable(gate_enable),
      .pwm_clk(pwm_clk),
      .duty_width(pwm_duty),
      .cycle_ticks(pwm_cycle_ticks),
      .pwm(pwm_)
  );

  pwm_commutator #(
      .channel_count(6)
  ) pwm_comm_ (
      .channel_enable(phase_enable),
      .pwm_in(pwm_),
      .pwm_out(pwm_out)
  );

  task reset();
    begin
      state_ <= state_idle;
      desired_direction_ <= DIR_NONE;
    end
  endtask


  //--------------------------------------------------------------------------
  // State machine transition handling
  //--------------------------------------------------------------------------
  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (reset_n == 0) begin
      reset();
    end else begin
      case (state_)
        state_idle: begin
          desired_direction_ <= direction;
          if ((direction != DIR_NONE) & enable) begin
            state_ <= state_startup;
          end
        end
        state_startup: begin
          if (~fault_n) begin
            state_ = state_reset_gate;
          end else begin
            state_ = state_run;
          end
        end
        state_reset_gate: begin
          if (fault_n) begin
            state_ = state_startup;
          end
        end
        state_run: begin
          if (~enable | (direction != desired_direction_)) begin
            state_ = state_idle;
          end else if (~fault_n) begin
            state_ = state_reset_gate;
          end
        end
      endcase
    end
  end

  assign driver_state = state_;

endmodule

`endif  //__BLDC_TABLE_DRIVER_SV__
