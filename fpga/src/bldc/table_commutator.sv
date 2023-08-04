`ifndef __BLDC_TABLE_COMMUTATOR__
`define __BLDC_TABLE_COMMUTATOR__

`include "bldc/types.sv"

//----------------------------------------------------------------------------
// BLDC commutation table
// hall_values are an array of hall inputs {A, B, C}
// phase_enable is an array of output enable signals
// {hi_A, hi_B, hi_C, lo_A, lo_B, lo_C}
//----------------------------------------------------------------------------
/**
 * @brief This module is a commutation table for BLDC motor driver phase signals.
 *
 * This module is a commutation table that determines the high and low phase
 * signals for a BLDC motor driver based on the motor's rotation direction
 * and the current hall sensor values. It is designed for use in combinatorial logic.
 *
 * The commutation table implements the following rules:
 *  - For clockwise rotation, the commutation sequence is as follows:
 *    0: A high, B low
 *    1: A high, C low
 *    2: B high, C low
 *    3: B high, A low
 *    4: C high, A low
 *    5: C high, B low
 *
 *  - For counter-clockwise rotation, the sequence is as follows:
 *    0: B high, A low
 *    1: C high, A low
 *    2: C high, B low
 *    3: A high, B low
 *    4: A high, C low
 *    5: B high, C low
 *
 *  - In the case of no commutation (rotation direction is DIR_NONE), all phases are driven low.
 *
 *  - In the case of braking (rotation direction is DIR_BRAKE), all low phases are driven high and high phases are driven low.
 *
 *  - In case of invalid hall values or invalid rotation direction, all phases are driven low.
 *
 * @note The outputs of this module change as soon as any of the inputs change. The outputs should be stable no later than the current clock tick change.
 *
 * @param rotation_direction The direction of the motor's rotation. Should be one of the values defined in the rotation_direction_t enum.
 * @param hall_values The current values of the motor's Hall sensors. Should be one of the valid values defined in the hall_states_t enum.
 * @param phase_signals The 6-bit output signal, consisting of the following values (MSB to LSB): A High, B High, C High, A Low, B Low, C Low.
 *
 */
module bldc_commutation_table (
    input clk,
    input rotation_direction_t dir,
    input hall_states_t hall_values,
    output logic [5:0] phase_enable,
    output reg error
);
  task phases_off(input set_error);
    begin
      phase_enable <= {PHASE_OFF, PHASE_OFF};
      error <= set_error;
    end
  endtask

  task brake();
    begin
      phase_enable <= {PHASE_OFF, PHASE_ON};
      error <= 0;
    end
  endtask

  task enable_phases(input phase_states_t hi, input phase_states_t lo);
    begin
      phase_enable <= (dir == DIR_CW) ? {hi, lo} : {lo, hi};
      error <= 0;
    end
  endtask

  always @(posedge clk) begin
    if (dir == DIR_NONE) phases_off(.set_error(0));
    else if (dir == DIR_BRAKE) brake();
    else if (dir == DIR_CW || dir == DIR_CCW)
      case (hall_values)
        HALL_AC: enable_phases(.hi(PHASE_A), .lo(PHASE_B));  // Sector 0
        HALL_A:  enable_phases(.hi(PHASE_A), .lo(PHASE_C));  // Sector 1
        HALL_AB: enable_phases(.hi(PHASE_B), .lo(PHASE_C));  // Sector 2
        HALL_B:  enable_phases(.hi(PHASE_B), .lo(PHASE_A));  // Sector 3
        HALL_BC: enable_phases(.hi(PHASE_C), .lo(PHASE_A));  // Sector 4
        HALL_C:  enable_phases(.hi(PHASE_C), .lo(PHASE_B));  // Sector 5
        default: phases_off(.set_error(1));
      endcase
    else phases_off(.set_error(1));
  end
endmodule

`endif  // __BLDC_TABLE_COMMUTATOR__
