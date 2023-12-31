`ifndef __BLDC_TABLE_DRIVER_SV__
`define __BLDC_TABLE_DRIVER_SV__

`include "bldc/types.sv"
`include "bldc/encoder.sv"
`include "bldc/table_commutator.sv"
`include "bldc/pwm_generator.sv"
`include "bldc/debounce.sv"
`include "bldc/gate_driver_reset.sv"
`include "bldc/position_control.sv"

//`define GATE_RESET_STATES

module table_bldc_driver #(
    parameter clk_freq_hz = 54_000_000,
    parameter pwm_clk_freq_hz = 100_286_000,
    parameter pwm_freq_hz = 100_000,
    parameter rpm_measurement_ms = 100,  // Period of RPM management in ms
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
    input logic invert_phases,
    input fault_n,  // Driver fault, active low
    input overcurrent_n,  // Overcurrent warning, active low
    //@{
    /** @name Encoder interface */
    output rotation_direction_t detected_dir,
    output logic [counter_width - 1:0] encoder_counter,
    output logic [counter_width - 1:0] transitions_per_period,
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
    /** @name Position control interface */
    input pos_ctl_enable,
    input logic [counter_width - 1:0] target_position,
    output logic [pwm_counter_width - 1:0] pwm_duty_out,
    output rotation_direction_t dir_out,
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
    state_run = 'd2,
    state_error = 'd3,
    state_pos_ctl = 'd4,
    state_gate_reset_start = 'd5,
    state_gate_reset_wait = 'd6,
    state_gate_reset_done = 'd7
  } driver_state_t;

  driver_state_t state_;
  rotation_direction_t desired_direction_;
  hall_states_t hall_values_debounced_;

  genvar i;
  generate
    for (i = 0; i < 3; ++i) begin : hall_debounce
      debounce_us #(
          .clk_freq_hz(clk_freq_hz),
          .debounce_time_us(10)
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
      .rpm_measurement_ms(rpm_measurement_ms),
      .counter_width(counter_width)
  ) encoder_ (
      .clk(sys_clk),
      .reset_n(reset_n),
      .hall_values(hall_values_debounced_),
      .overall_counter(encoder_counter),
      .rotation_direction(detected_dir),
      .transitions_per_period(transitions_per_period),
      .rpm(rpm),
      .sector(sector)
  );

  //--------------------------------------------------------------------------
  // Position control
  //--------------------------------------------------------------------------

  position_control #(
      .clk_freq_hz(clk_freq_hz),
      .counter_width(counter_width),
      .pwm_clk_freq_hz(pwm_clk_freq_hz),
      .pwm_freq_hz(pwm_freq_hz),
      .pwm_counter_width(pwm_counter_width),
      .pole_pairs(pole_pairs),
      .rpm_measurement_ms(rpm_measurement_ms)
  ) pos_ctl_ (
      .sys_clk(sys_clk),

      .enable(pos_ctl_enable),
      .encoder_position(encoder_counter),
      .target_position(target_position),

      .pwm_duty_in(pwm_duty),
      .dir_in(desired_direction_),

      .pwm_duty_out(pwm_duty_out),
      .dir_out(dir_out),

      .reset_n(reset_n)
  );

  //--------------------------------------------------------------------------
  // BLDC commutation table
  //--------------------------------------------------------------------------
  bldc_commutation_table comm_table_ (
      .clk(sys_clk),
      .dir(dir_out),
      .hall_values(hall_values_debounced_),
      .invert_phases(invert_phases),
      .phase_enable(phase_enable),
      .error(hall_error)
  );

  //--------------------------------------------------------------------------
  // PWM generation
  //--------------------------------------------------------------------------
  wire  pwm_;
  logic pwm_enable_;

  pwm_generator #(
      .clock_freq_hz(pwm_clk_freq_hz),
      .pwm_freq_hz  (pwm_freq_hz)
  ) pwm_gen_ (
      .enable(pwm_enable_),
      .pwm_clk(pwm_clk),
      .duty_width(pwm_duty_out),
      .cycle_ticks(pwm_cycle_ticks),
      .pwm(pwm_)
  );

  pwm_commutator #(
      .channel_count(3)
  ) pwm_comm_hi_ (
      .channel_enable(phase_enable[5:3]),
      .pwm_in(pwm_),
      .pwm_out(pwm_out[5:3])
  );

  assign pwm_out[2:0] = phase_enable[2:0];

  //--------------------------------------------------------------------------
  // Gate enable/reset module
  //--------------------------------------------------------------------------
  logic reset_start_, reset_done_;
  gate_driver_reset #(
      .clk_freq_hz(clk_freq_hz)
  ) gate_reset_ (
      .sys_clk(sys_clk),
      .reset_n(reset_n),
      .driver_enable(enable & ~hall_error),
      .driver_enable_out(gate_enable),
      .slow_reset(1'b1),
      .reset_start(reset_start_),
      .reset_done(reset_done_)
  );

  //--------------------------------------------------------------------------
  // Tasks
  //--------------------------------------------------------------------------
  task reset();
    begin
      state_ <= state_idle;
      desired_direction_ <= DIR_NONE;
      reset_start_ <= 0;
    end
  endtask

  task idle_state_task();
    desired_direction_ <= direction;
    if (hall_error) begin
      state_ <= state_error;
    end else if (pos_ctl_enable) begin
      state_ <= state_pos_ctl;
    end else if ((desired_direction_ != DIR_NONE) & enable) begin
      state_ <= state_startup;
    end
  endtask

  task startup_state_task();
    begin
`ifdef GATE_RESET_STATES
      if (~fault_n) begin
        state_ = state_gate_reset_start;
      end else begin
        state_ = state_run;
      end
`else
      if (hall_error) begin
        state_ <= state_error;
      end else begin
        state_ = state_run;
      end
`endif
    end
  endtask

  task run_state_task();
    begin
`ifdef GATE_RESET_STATES
      if (hall_error) begin
        state_ <= state_error;
      end else if (~enable | (direction != desired_direction_)) begin
        state_ = state_idle;
      end else if (~fault_n) begin
        state_ = state_gate_reset_start;
      end
`else
      if (hall_error) begin
        state_ <= state_error;
      end else if (~enable | (direction != desired_direction_)) begin
        state_ = state_idle;
      end
`endif
    end
  endtask

  task error_state_task();
    begin
      if (~hall_error) begin
        state_ = state_idle;
      end
    end
  endtask

  task pos_ctl_task();
    begin
      if (hall_error) begin
        state_ <= state_error;
      end else if (~pos_ctl_enable) begin
        state_ <= state_idle;
      end
    end
  endtask

  task gate_reset_start_task();
    begin
      reset_start_ <= 1;
      state_ = state_gate_reset_wait;
    end
  endtask

  task gate_reset_wait_task();
    begin
      reset_start_ <= 0;
      if (reset_done_) begin
        state_ <= state_gate_reset_done;
      end
    end
  endtask

  task gate_reset_done_task();
    begin
      if (fault_n) begin
        state_ <= state_idle;
      end else begin
        // TODO calculate resets and go to error state
        state_ <= state_gate_reset_start;
      end
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
        state_idle: idle_state_task();
        state_startup: startup_state_task();
        state_run: run_state_task();
        state_error: error_state_task();
        state_pos_ctl: pos_ctl_task();

        state_gate_reset_start: gate_reset_start_task();
        state_gate_reset_wait:  gate_reset_wait_task();
        state_gate_reset_done:  gate_reset_done_task();
      endcase
    end
  end

  assign pwm_enable_  = (state_ != state_idle) ? 1 : 0;
  assign driver_state = state_;

endmodule

`endif  //__BLDC_TABLE_DRIVER_SV__
