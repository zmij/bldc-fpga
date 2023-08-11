`ifndef __BLDC_POSITION_CONTROL_SV__
`define __BLDC_POSITION_CONTROL_SV__

`include "bldc/types.sv"

module position_control #(
    parameter clk_freq_hz = 27_000_000,  // The frequency of the clock in Hertz
    parameter counter_width = 32,  // The width of the internal counter
    parameter pwm_clk_freq_hz = 100_286_000,
    parameter pwm_freq_hz = 100_000,
    parameter pwm_counter_width = counter_width,
    parameter pole_pairs = 1,  // The number of pole pairs in the motor
    parameter rpm_measurement_ms = 100  // Period of RPM management in ms
) (
    input logic sys_clk,

    input logic enable,
    input logic [counter_width-1:0] encoder_position,
    input logic [counter_width-1:0] target_position,

    input logic [pwm_counter_width-1:0] pwm_duty_in,
    input rotation_direction_t dir_in,

    output logic [pwm_counter_width-1:0] pwm_duty_out,
    output rotation_direction_t dir_out,

    input logic reset_n
);
  localparam counter_max = {(counter_width) {1'b1}};
  localparam half_counter = counter_max / 2;

  localparam pwm_cycle_ticks = pwm_clk_freq_hz / pwm_freq_hz;
  localparam pwm_max = pwm_cycle_ticks / 3;
  localparam pwm_min = pwm_cycle_ticks / 10;
  localparam pwm_range = pwm_max - pwm_min;
  localparam pwm_step = (pwm_max - pwm_min) / 4;

  localparam full_turn = pole_pairs * 6;
  localparam full_speed_distance = full_turn * 32;
  localparam min_speed_distance = full_turn * 2;
  localparam distance_range = full_speed_distance - min_speed_distance;

  typedef logic [pwm_counter_width-1:0] pwm_counter_t;
  typedef logic [counter_width-1:0] counter_t;

  typedef enum logic {
    state_idle = 0,
    state_run  = 1
  } state_t;

  state_t state_;

  pwm_counter_t calculated_pwm_;
  rotation_direction_t calculated_dir_;
  counter_t start_position_;
  counter_t distance_;
  counter_t travelled_;

  // Update state and remember starting position when enabled
  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (!reset_n) begin
      state_ <= state_idle;
      start_position_ <= 0;
    end else begin
      case (state_)
        state_idle: begin
          if (enable) begin
            start_position_ <= encoder_position;
            state_ <= state_run;
          end
        end
        state_run: begin
          if (~enable || target_position == encoder_position) begin
            state_ <= state_idle;
          end
        end
      endcase
    end
  end

  //--------------------------------------------------------------------------
  // Calculate PWM based on distance travelled or distance to target
  //--------------------------------------------------------------------------
  function pwm_counter_t truncate(input counter_t val);
    begin
      return val[pwm_counter_width-1:0];
    end
  endfunction

  function pwm_counter_t get_pwm(input counter_t distance);
    begin
      if (distance > full_speed_distance) return pwm_max;
      else if (distance > min_speed_distance)
        return truncate(pwm_min + distance * pwm_range / distance_range);
      else return pwm_min;
    end
  endfunction

  task automatic reset_pwm();
    begin
      calculated_pwm_ <= 0;
      calculated_dir_ <= DIR_NONE;
    end
  endtask

  task automatic calculate_pwm();
    begin
      if (target_position == encoder_position) begin
        calculated_pwm_ <= 0;
        calculated_dir_ <= DIR_BRAKE;
      end else begin
        if (target_position - encoder_position < half_counter) begin
          calculated_dir_ <= DIR_CW;
        end else begin
          calculated_dir_ <= DIR_CCW;
        end
        calculated_pwm_ <= get_pwm((distance_ < travelled_) ? distance_ : travelled_);
      end
    end
  endtask

  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (!reset_n) begin
      reset_pwm();
    end else if (enable) begin
      calculate_pwm();
    end
  end

  //--------------------------------------------------------------------------
  // Calculate distance to target
  //--------------------------------------------------------------------------
  task automatic calculate_distance();
    begin
      if (target_position - encoder_position < half_counter) begin
        distance_ <= target_position - encoder_position;
      end else begin
        distance_ <= encoder_position - target_position;
      end
    end
  endtask

  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (!reset_n) begin
      distance_ <= 0;
    end else begin
      calculate_distance();
    end
  end

  //--------------------------------------------------------------------------
  // Calculate PWM based on distance travelled or distance to target
  //--------------------------------------------------------------------------
  task automatic calculate_travelled();
    begin
      if (start_position_ - encoder_position < half_counter) begin
        travelled_ <= start_position_ - encoder_position;
      end else begin
        travelled_ <= encoder_position - start_position_;
      end
    end
  endtask

  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (!reset_n) begin
      travelled_ <= 0;
    end else if (state_ == state_run) begin
      calculate_travelled();
    end
  end

  assign pwm_duty_out = enable ? calculated_pwm_ : pwm_duty_in;
  assign dir_out = rotation_direction_t'(enable ? calculated_dir_ : dir_in);

endmodule

`endif  // __BLDC_POSITION_CONTROL_SV__
