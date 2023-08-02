/**

@file apb2_bldc_peripheral.sv
@brief APB2 BLDC Peripheral Module
This module is a peripheral device for a BLDC motor controller using the APB2 bus interface.
It provides an interface to control and monitor the BLDC motor.

*/
`ifndef __APB2_BLDC_SV__
`define __APB2_BLDC_SV__

`include "bldc/types.sv"
`include "bldc/encoder.sv"
`include "bldc/table_commutator.sv"
`include "bldc/pwm_generator.sv"

/**
 * @class apb2_bldc_perpheral
 *
 * @brief APB2 BLDC Peripheral Module
 *
 * This module provides an interface to control and monitor a BLDC motor using the APB2 bus.
 * It provides registers to configure the motor parameters and read the motor status.
 *
 * Registers:
 * status               0x00
 *  [2:0] hall_values    RO
 *  [5:3] sector         RO
 *  [7:6] detected_dir   RO
 *  [14:8] phase_enable  RO
 * counter              0x04
 *  [31:0] value         RO
 * rotation duration    0x08
 *  [31:0] value         RO
 * rmp                  0x0c
 *  [31:0] value         RO
 * control              0x10
 *  [0:0] enable         R/W
 *  [2:1] dir            R/W
 */
module apb2_bldc_perpheral #(
    // Data width for APB2 bus
    parameter data_width = 32,
    // Address width for APB2 bus
    parameter addr_width = 8,
    // APB2 clock frequency
    parameter clk_freq_hz = 54_000_000,
    // PWM clock frequency
    parameter pwm_clk_freq_hz = 100_286_000,
    // PWM output frequency
    parameter pwm_freq_hz = 100_000,
    // Number of BLDC pole pairs
    parameter pole_pairs = 1
) (
    /**
    @name APB2 Interface Signals
    @defgroup APB2_Interface APB2 Interface Signals
    @brief Signals used to interface with the APB2 bus
    @{ 
    */
    /**
    @brief APB2 clock signal
    This is the clock signal for the APB2 bus. 
    */
    input pclk,

    /**
    @brief APB2 enable signal
    This signal is used to enable the APB2 bus transactions.
    */
    input penable,
    /**
    @brief APB2 write enable signal
    This signal is used to indicate whether the APB2 transaction is a write transaction.
    */
    input pwrite,
    /**
    @brief APB2 address signal
    This signal represents the address of the APB2 transaction.
    The address implements register addresses, the access is on a 4-byte boundary.
    */
    input [addr_width - 1:0] paddr,
    /**
    @brief APB2 write data signal
    This signal represents the data to be written to the APB2 address.
    */
    input [data_width - 1:0] pwdata,
    /**
    @brief APB2 byte enable signal
    This signal is used to indicate which bytes of the APB2 write data are valid.
    In this module, it is ignored.
    */
    input [data_width/8 - 1:0] pstrb,
    /**
    @brief APB2 protection signal
    This signal is used to implement protection for the APB2 transactions.
    In this module, it is ignored.
    */
    input [2:0] pprot,

    /**
    @brief APB2 select signal
    This signal is used to select the APB2 peripheral.
    */
    input psel,
    /**
    @brief APB2 read data signal
    This signal represents the data read from the APB2 address.
    */
    output reg [data_width - 1:0] prdata,
    /**
    @brief APB2 data ready signal
    This signal is used to indicate that the APB2 data is ready to be read or the write data has been accounted for.
    */
    output reg pready,
    /**
    @brief APB2 error signal
    This signal is used to indicate an error in the APB2 transactions.
    */
    output reg pslverr,
    /** @} */  // end of APB2_Interface

    /**
    @brief Rotation detected by the encoder
    This signal represents the rotation direction detected by the encoder.
    The possible values are CW (clockwise) and CCW (counter-clockwise).
    */
    output rotation_direction_t detected_dir,
    /** @} */  // end of Encoder_Interface

    /**
    @name Hall Sensor Signals
    @defgroup Hall_Sensor Hall Sensor Signals
    @brief Signals used to read the hall sensor values
    @{
    */
    /**
    @brief Input hall values
    This signal represents the current values of the hall sensors connected to the BLDC motor.
    The values are represented as a 3-bit vector, where each bit represents the state of a single hall sensor.
    The order of the bits is {A, B, C}, where A is the first bit and C is the last bit.
    */
    input hall_states_t hall_values,
    /** @} */  // end of Hall_Sensor
    /**
    @name Motor Control Signals
    @defgroup Motor_Control Motor Control Signals
    @brief Signals used to control the BLDC motor
    @{
    */

    input pwm_clk,
    /**
    @brief Phase enable signal
    This signal is used to enable the phases of the BLDC motor.
    The signal is a 6-bit vector, where the first 3 bits represent the high side
    phase enable signals (AH, BH, CH), and the last 3 bits represent the low side
    phase enable signals (AL, BL, CL). 
    */
    output [5:0] phase_enable,
    output [5:0] pwm_out,
    /** @} */  // end of Motor_Control

    /**
    @name Reset Signal
    @defgroup Reset_Signal Reset Signal
    @brief Signal used to reset the module
    @{ 
    */
    /**
    @brief Reset signal
    This signal is used to reset the module.
    It has an active low polarity.
    */
    input preset_n  // reset signal, active low
    /** @} */  // end of Reset_Signal
);
  localparam counter_width = 32;
  localparam pwm_counter_width = $clog2(pwm_clk_freq_hz / pwm_freq_hz) + 1;

  // Register addresses
  localparam reg_status = 8'h00 * 4;
  localparam reg_enc_counter = 8'h01 * 4;
  localparam reg_enc_rot_duration = 8'h02 * 4;
  localparam reg_rpm = 8'h03 * 4;
  localparam reg_control = 8'h04 * 4;
  localparam reg_pwm_control = 8'h05 * 4;

  typedef enum logic [1:0] {
    idle_state,
    w_enable,
    r_enable
  } apb_state_t;

  apb_state_t apb_state_;

  reg enable_;
  rotation_direction_t dir_;
  logic [pwm_counter_width - 1:0] pwm_duty_;

  //--------------------------------------------------------------------------
  // Encoder instance
  //--------------------------------------------------------------------------
  wire [counter_width - 1:0] enc_counter_;
  wire [counter_width - 1:0] rot_duration_;
  wire [counter_width - 1:0] rpm_;
  wire [2:0] sector_;

  three_phase_encoder #(
      .clk_freq_hz(clk_freq_hz),
      .pole_pairs(pole_pairs),
      .counter_width(counter_width)
  ) enc_inst (
      .clk(pclk),
      .reset_n(preset_n),
      .hall_values(hall_values),
      .overall_counter(enc_counter_),
      .rotation_direction(detected_dir),
      .rotation_duration(rot_duration_),
      .rpm(rpm_),
      .sector(sector_)
  );

  //--------------------------------------------------------------------------
  // BLDC commutation table
  //--------------------------------------------------------------------------
  bldc_commutation_table comm_table_ (
      .clk(pclk),
      .dir(dir_),
      .hall_values(hall_values),
      .phase_enable(phase_enable)
  );

  //--------------------------------------------------------------------------
  // PWM generation
  //--------------------------------------------------------------------------
  wire pwm_;
  logic [pwm_counter_width - 1:0] pwm_cycle_ticks_;

  pwm_generator #(
      .clock_freq_hz(pwm_clk_freq_hz),
      .pwm_freq_hz  (pwm_freq_hz)
  ) pwm_gen_ (
      .enable(preset_n & enable_),
      .pwm_clk(pwm_clk),
      .duty_width(pwm_duty_),
      .cycle_ticks(pwm_cycle_ticks_),
      .pwm(pwm_)
  );

  pwm_commutator #(
      .channel_count(6)
  ) pwm_comm_ (
      .channel_enable(phase_enable),
      .pwm_in(pwm_),
      .pwm_out(pwm_out)
  );

  //--------------------------------------------------------------------------
  // Reset task
  //--------------------------------------------------------------------------
  task reset();
    begin
      apb_state_ <= idle_state;
      prdata <= {(data_width) {1'bz}};
      pready <= 1;
      pslverr <= 0;

      enable_ <= 0;
      dir_ <= DIR_NONE;
      pwm_duty_ <= 0;
    end
  endtask

  //--------------------------------------------------------------------------
  // Process idle state
  //--------------------------------------------------------------------------
  task process_ide();
    begin
      prdata  <= {(data_width) {1'bz}};
      pslverr <= 0;
      if (psel) begin
        pready <= 0;
        if (pwrite) apb_state_ <= w_enable;
        else apb_state_ <= r_enable;
      end
    end
  endtask

  //--------------------------------------------------------------------------
  // Read register tasks
  //--------------------------------------------------------------------------
  task read_registers();
    begin
      if (psel && !pwrite && penable) begin
        case (paddr)
          reg_status: read_status_register();
          reg_enc_counter: read_enc_counter();
          reg_enc_rot_duration: read_enc_rot_duration();
          reg_rpm: read_rpm();
          reg_control: read_control_register();
          reg_pwm_control: read_pwm_control_register();
          // Write requested address for now
          default: prdata[addr_width-1:0] <= paddr;
        endcase
        pready <= 1;
      end
      apb_state_ <= idle_state;
    end
  endtask

  task read_status_register();
    localparam reg_status_padding = {(data_width - 14) {1'b0}};
    begin
      prdata <= {reg_status_padding, phase_enable, detected_dir, sector_, hall_values};
    end
  endtask

  task read_enc_counter();
    begin
      prdata <= enc_counter_;
    end
  endtask

  task read_enc_rot_duration();
    begin
      prdata <= rot_duration_;
    end
  endtask

  task read_rpm();
    begin
      prdata <= rpm_;
    end
  endtask

  task read_control_register();
    localparam reg_control_padding = {(data_width - 3) {1'b0}};
    begin
      prdata <= {reg_control_padding, dir_, enable_};
    end
  endtask

  task read_pwm_control_register();
    localparam reg_pwm_padding = {(16 - pwm_counter_width) {1'b0}};
    begin
      prdata <= {reg_pwm_padding, pwm_cycle_ticks_, reg_pwm_padding, pwm_duty_};
    end
  endtask

  //--------------------------------------------------------------------------
  // Write register tasks
  //--------------------------------------------------------------------------
  task write_registers();
    begin
      if (psel && pwrite && penable) begin
        case (paddr)
          reg_control: write_control_register();
          reg_pwm_control: write_pwm_control_register();
          //default: pslverr <= 1;
        endcase
        pready <= 1;
      end
      apb_state_ <= idle_state;
    end
  endtask

  task write_control_register();
    begin
      enable_ = pwdata[0];
      dir_ = rotation_direction_t'(pwdata[2:1]);
    end
  endtask

  task write_pwm_control_register();
    begin
      pwm_duty_ <= pwdata[pwm_counter_width-1:0];
    end
  endtask

  always @(negedge preset_n or posedge pclk) begin
    if (preset_n == 0) begin
      reset();
    end else begin
      case (apb_state_)
        idle_state: process_ide();
        w_enable:   write_registers();
        r_enable:   read_registers();
      endcase
    end
  end

endmodule
/** @} */  // end of apb2_bldc_peripheral

`endif  // __APB2_BLDC_SB__
