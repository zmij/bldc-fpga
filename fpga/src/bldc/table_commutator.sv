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
    output logic [5:0] phase_enable
);
  reg  [5:0] commutation_table[0:2][0:5];
  wire [1:0] table_number;

  always @(posedge clk) begin
    if (dir == DIR_NONE) phase_enable <= {PHASE_OFF, PHASE_OFF};
    else if (dir == DIR_BRAKE) phase_enable <= {PHASE_OFF, PHASE_ON};
    else if (dir == DIR_CW || dir == DIR_CCW)
      case (hall_values)
        HALL_AC:
        phase_enable <= (dir == DIR_CW) ? {PHASE_A, PHASE_B} : {PHASE_B, PHASE_A};  // Sector 0
        HALL_A:
        phase_enable <= (dir == DIR_CW) ? {PHASE_A, PHASE_C} : {PHASE_C, PHASE_A};  // Sector 1
        HALL_AB:
        phase_enable <= (dir == DIR_CW) ? {PHASE_B, PHASE_C} : {PHASE_C, PHASE_B};  // Sector 2
        HALL_B:
        phase_enable <= (dir == DIR_CW) ? {PHASE_B, PHASE_A} : {PHASE_A, PHASE_B};  // Sector 3
        HALL_BC:
        phase_enable <= (dir == DIR_CW) ? {PHASE_C, PHASE_A} : {PHASE_A, PHASE_C};  // Sector 4
        HALL_C:
        phase_enable <= (dir == DIR_CW) ? {PHASE_C, PHASE_B} : {PHASE_B, PHASE_C};  // Sector 3
        default: phase_enable <= {PHASE_OFF, PHASE_OFF};
      endcase
    else phase_enable <= {PHASE_OFF, PHASE_OFF};
  end
endmodule

`endif  // __BLDC_TABLE_COMMUTATOR__
