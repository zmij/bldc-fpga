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
    output rotation_direction_t rotation_direction,  // Rotation direction
    output logic [2:0] sector  // Current sector
);

  hall_states_t prev_hall_values;  // Previous hall sensor values
  logic [counter_width-1:0] tick_counter;  // Internal counter to keep track of ticks
  logic [2:0] transition_counter;  // Counter for detecting change in direction
  logic direction_changed;  // Flag to indicate change in direction

  /**
   * @brief Task to update the rotation information based on the hall_values.
   * @details This task is called whenever the hall_values changes.
   */
  task automatic update_rotation;
    begin
      if (hall_values != prev_hall_values) begin
        if (prev_hall_values.next() == hall_values) begin
          if (rotation_direction != DIR_CW) direction_changed = 1;
          else direction_changed = 0;
          rotation_direction <= DIR_CW;
          overall_counter <= overall_counter + 1;
        end else if (prev_hall_values.prev() == hall_values) begin
          if (rotation_direction != DIR_CCW) direction_changed = 1;
          else direction_changed = 0;
          rotation_direction <= DIR_CCW;
          overall_counter <= overall_counter - 1;
        end

        prev_hall_values   <= hall_values;
        transition_counter <= direction_changed ? 0 : transition_counter + 1;
        if (transition_counter == 6) begin
          rotation_duration <= rotation_direction == DIR_CW ? tick_counter : -tick_counter;
          tick_counter <= 0;
          transition_counter <= 0;
        end else if (direction_changed) begin
          rotation_duration <= 0;
        end
      end else begin
        tick_counter <= tick_counter + 1;
        if (tick_counter > idle_ticks) begin
          rotation_direction <= DIR_NONE;
          rotation_duration  <= 0;
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
      tick_counter <= 0;
      transition_counter <= 0;
      overall_counter <= 0;
      rotation_direction <= DIR_NONE;
      rotation_duration <= 0;
    end else begin
      update_rotation();
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

endmodule


`endif  // __BLDC_ENCODDR_SV__
