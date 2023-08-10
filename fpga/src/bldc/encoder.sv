/**
 * @file three_phase_encoder.sv
 *
 * @brief Encoder for BLDC motor using HALL sensors and phase inputs to detect direction at once.
 */

`ifndef __BLDC_ENCODER_SV__
`define __BLDC_ENCODER_SV__

`include "bldc/types.sv"

//----------------------------------------------------------------------------
// Encoder for BLDC motor
// Uses HALL sensors to detect direction at once
//----------------------------------------------------------------------------
/**
 * @class three_phase_encoder
 * @brief Module for encoding the rotation of a BLDC motor using HALL sensors.
 * @details This module detects the rotation direction, overall counter, rotation duration, and current sector of the motor.
 * The rotation is encoded based on the HALL sensor values.
 */
module three_phase_encoder #(
    parameter clk_freq_hz = 27_000_000,  // The frequency of the clock in Hertz
    parameter counter_width = 32,  // The width of the internal counter
    parameter pole_pairs = 1,  // The number of pole pairs in the motor
    parameter idle_ticks = clk_freq_hz * 60 / (6 * pole_pairs) // Time to exceed for IDLE state in ticks
) (
    input logic clk,  // Clock signal
    input logic reset_n,  // Reset signal
    input hall_states_t hall_values,  // Hall sensor values
    output reg [counter_width-1:0] overall_counter, // Overall counter that increments or decrements based on rotation
    output reg [counter_width-1:0] rotation_duration,  // Duration of one rotation in ticks
    output [counter_width-1:0] rpm,
    output rotation_direction_t rotation_direction,  // Rotation direction
    output logic [2:0] sector  // Current sector
);
  localparam cycle_size = 6;
  localparam ticks_per_minute = clk_freq_hz * 60;

  typedef logic [counter_width-1:0] counter_t;

  hall_states_t prev_hall_values_;  // Previous hall sensor values
  counter_t cycle_counter_;  // Internal counter to keep track of ticks taking to make a cycle
  counter_t idle_counter_;  // Internal counter to keep track of ticks when nothin happens
  logic [2:0] transition_counter_;  // Counter for detecting change in direction
  logic direction_changed;  // Flag to indicate change in direction
  logic idle_;

  /**
   * @brief Task to update the rotation information based on the hall_values.
   * @details This task is called whenever the hall_values changes.
   */
  task automatic update_direction;
    begin
      if (hall_values != prev_hall_values_) begin
        if (prev_hall_values_.next() == hall_values) begin
          if (rotation_direction != DIR_CW) direction_changed = 1;
          else direction_changed = 0;
          rotation_direction <= DIR_CW;
        end else if (prev_hall_values_.prev() == hall_values) begin
          if (rotation_direction != DIR_CCW) direction_changed = 1;
          else direction_changed = 0;
          rotation_direction <= DIR_CCW;
        end
        prev_hall_values_ <= hall_values;
        idle_ <= 0;
      end else begin
        idle_ <= 1;
        if (idle_counter_ > idle_ticks) begin
          rotation_direction <= DIR_NONE;
        end
      end
    end
  endtask

  /**
   * @brief Task to update rotation duration
   */
  task update_duration();
    begin
      cycle_counter_ <= cycle_counter_ + 1;
      if (!idle_) begin
        if (direction_changed) begin
          transition_counter_ <= 0;
          cycle_counter_ <= 0;
          rotation_duration <= 0;
        end else begin
          if (transition_counter_ < cycle_size) begin
            transition_counter_ <= transition_counter_ + 1;
          end else begin
            rotation_duration <= cycle_counter_ * pole_pairs;
            cycle_counter_ <= 0;
            transition_counter_ <= 0;
          end
        end
      end else begin
        if (idle_counter_ > idle_ticks) begin
          transition_counter_ <= 0;
          rotation_duration   <= 0;
        end
      end
    end
  endtask

  /**
   * Update idle ticks counter 
   */
  task update_idle_counter();
    begin
      if (idle_) begin
        idle_counter_ <= idle_counter_ + 1;
      end else begin
        idle_counter_ <= 0;
      end
    end
  endtask

  /**
   * @brief Task to update conter value depending on direction
   */
  task update_counter();
    begin
      if (!idle_) begin
        if (rotation_direction == DIR_CW) begin
          overall_counter <= overall_counter + 1;
        end else if (rotation_direction == DIR_CCW) begin
          overall_counter <= overall_counter - 1;
        end
      end
    end
  endtask

  /**
   * @brief Always block to update the rotation when the clock edge occurs or reset signal changes.
   * @details This always block is sensitive to the positive edge of the clock signal and negative edge of the reset signal.
   */
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      idle_ <= 1;
      rotation_direction <= DIR_NONE;
    end else begin
      update_direction();
    end
  end

  /**
   * @brief Always block to update rotation duration
   */
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      cycle_counter_ <= 0;
      transition_counter_ <= 0;
      rotation_duration <= 0;
    end else begin
      update_duration();
    end
  end

  /**
   * @brief Always block to update the encoder counter
   */
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      overall_counter <= 0;
    end else begin
      update_counter();
    end
  end

  /**
   * @brief Always block to update the idle counter
   */
  always_ff @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
      idle_counter_ <= 0;
    end else begin
      update_idle_counter();
    end
  end

  /**
   * @brief Always block to update the current sector whenever the clock edge occurs.
   * @details This always block is sensitive to the positive edge of the clock signal.
   */
  always @(posedge clk) begin
    if (!reset_n) begin
      sector <= 3'b111;
    end else begin
      case (hall_values)
        HALL_AC: sector <= 'd0;
        HALL_A:  sector <= 'd1;
        HALL_AB: sector <= 'd2;
        HALL_B:  sector <= 'd3;
        HALL_BC: sector <= 'd4;
        HALL_C:  sector <= 'd5;
        default: sector <= 3'b111;
      endcase
    end
  end

  assign rpm = rotation_duration == 0 ? 0 : ticks_per_minute / rotation_duration;

endmodule


`endif  // __BLDC_ENCODDR_SV__
