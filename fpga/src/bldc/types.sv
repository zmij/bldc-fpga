`ifndef __BLDC_TYPES_SV__
`define __BLDC_TYPES_SV__

typedef enum bit [2:0] {
  HALL_AC = 3'b101,
  HALL_A  = 3'b100,
  HALL_AB = 3'b110,
  HALL_B  = 3'b010,
  HALL_BC = 3'b011,
  HALL_C  = 3'b001
} hall_states_t;

typedef enum logic [1:0] {
  DIR_NONE  = 2'b00,
  DIR_CW    = 2'b01,
  DIR_CCW   = 2'b11,
  DIR_BRAKE = 2'b10
} rotation_direction_t;

typedef enum bit [2:0] {
  PHASE_OFF = 3'b000,
  PHASE_A_C = 3'b101,
  PHASE_A   = 3'b100,
  PHASE_A_B = 3'b110,
  PHASE_B   = 3'b010,
  PHASE_B_C = 3'b011,
  PHASE_C   = 3'b001,
  PHASE_ON  = 3'b111
} phase_states_t;


`endif  // __BLDC_TYPES_SV__
