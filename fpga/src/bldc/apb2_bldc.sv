/**

@file apb2_bldc_peripheral.sv
@brief APB2 BLDC Peripheral Module
This module is a peripheral device for a BLDC motor controller using the APB2 bus interface.
It provides an interface to control and monitor the BLDC motor.

*/
`ifndef __APB2_BLDC_SV__
`define __APB2_BLDC_SV__

`include "bldc/types.sv"
`include "bldc/table_driver.sv"
`include "bldc/position_control.sv"
`include "bldc/speed_control.sv"

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
 *  [2:0] hall_values    RO off 0
 *  [5:3] sector         RO off 3
 *  [7:6] detected_dir   RO off 6
 *  [14:8] phase_enable  RO off 8
 *  [14:14] hal_error    RO off 14
 *  [15:15] fault        RO
 *  [16:16] ocw          RO
 *  [19:17] driver_state RO // The size may increase
 *  [25:20] pole_pairs   RO
 * counter              0x04
 *  [31:0] value         RO
 * rotation duration    0x08
 *  [31:0] value         RO
 * rmp                  0x0c
 *  [31:0] value         RO
 * control              0x10
 *  [0:0] enable         R/W
 *  [2:1] dir            R/W
 *  [3:3] invert_phases  R/W
 * pwm control          0x14
 *  [15:0] pwm_duty      R/W
 *  [31:16] cycle_ticks  R/W
 * pos control          0x18
 *  [0:0] enable         R/W
 * target pos           0x1c
 *  [31:0] pos           R/W
 * speed control        0x20
 *  [0:0] enable         R/W
 *  [12:1] target rpm    R/W
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
    // Period of RPM management in ms
    parameter rpm_measurement_ms = 100,
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
    input fault_n,  // Driver fault, active low
    input overcurrent_n,  // Overcurrent warning, active low
    output gate_enable,  // Enable driver gates
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
  localparam rpm_counter_width = 12;
  localparam pwm_counter_width = $clog2(pwm_clk_freq_hz / pwm_freq_hz) + 1;

  // Register addresses
  localparam reg_status = 8'h00 * 4;
  localparam reg_enc_counter = 8'h01 * 4;
  localparam reg_enc_transitions_per_period = 8'h02 * 4;
  localparam reg_rpm = 8'h03 * 4;
  localparam reg_control = 8'h04 * 4;
  localparam reg_pwm_control = 8'h05 * 4;
  localparam reg_pos_control = 8'h06 * 4;
  localparam reg_tgt_pos = 8'h07 * 4;
  localparam reg_speed_control = 8'h08 * 4;

  typedef logic [pwm_counter_width - 1:0] pwm_t;
  typedef logic [counter_width - 1:0] position_t;
  typedef logic [rpm_counter_width - 1:0] rpm_t;

  logic [4:0] pole_pairs_;
  assign pole_pairs_ = pole_pairs;

  typedef enum logic [1:0] {
    idle_state,
    w_enable,
    r_enable
  } apb_state_t;

  apb_state_t apb_state_;

  logic manual_enable_;

  logic hall_error_;
  logic invert_phases_;

  pwm_t pwm_cycle_ticks_;

  rotation_direction_t manual_dir_;
  pwm_t user_pwm_duty_;


  position_t enc_position_;
  position_t transitions_per_period_;
  rpm_t rpm_ms_;
  rpm_t rpm_1s_;

  logic pos_ctl_enable_;
  position_t target_position_;
  rotation_direction_t pos_ctl_dir_;
  logic pos_driver_enable_;
  pwm_t pos_ctl_pwm_duty_;

  logic user_speed_ctl_enable_;
  rpm_t user_target_rpm_;
  rpm_t target_rpm_;
  logic speed_ctl_drv_enable_;
  pwm_t speed_pwm_duty_;

  wire [2:0] sector_;

  wire [2:0] driver_state_;

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
      .sys_clk(pclk),

      .enable(pos_ctl_enable_),
      .encoder_position(enc_position_),
      .target_position(target_position_),

      .pwm_duty_in(user_pwm_duty_),
      .dir_in(manual_dir_),
      .driver_enable_in(manual_enable_),

      .pwm_duty_out(pos_ctl_pwm_duty_),
      .dir_out(pos_ctl_dir_),
      .driver_enable_out(pos_driver_enable_),

      .reset_n(preset_n)
  );

  assign target_rpm_ = user_target_rpm_;

  //--------------------------------------------------------------------------
  // Speed control
  //--------------------------------------------------------------------------
  speed_control #(
      .clk_freq_hz(clk_freq_hz),
      .counter_width(counter_width),
      .pwm_clk_freq_hz(pwm_clk_freq_hz),
      .pwm_freq_hz(pwm_freq_hz),
      .pwm_counter_width(pwm_counter_width),
      .pole_pairs(pole_pairs),
      .rpm_counter_width(rpm_counter_width),
      .rpm_measurement_ms(rpm_measurement_ms)
  ) speed_ctl_ (
      .sys_clk(pclk),

      .enable(user_speed_ctl_enable_),

      .rpm(rpm_ms_),
      .current_direction(detected_dir),

      .target_rpm(user_target_rpm_),
      .target_direction(pos_ctl_dir_),

      .pwm_duty_in(pos_ctl_pwm_duty_),
      .driver_enable_in(pos_driver_enable_),

      .pwm_duty_out(speed_pwm_duty_),
      .driver_enable_out(speed_ctl_drv_enable_),

      .reset_n(preset_n)
  );

  //--------------------------------------------------------------------------
  // Table driver commutaion
  //--------------------------------------------------------------------------
  table_bldc_driver #(
      .clk_freq_hz(clk_freq_hz),
      .pwm_clk_freq_hz(pwm_clk_freq_hz),
      .pwm_freq_hz(pwm_freq_hz),
      .rpm_measurement_ms(rpm_measurement_ms),
      .pole_pairs(pole_pairs),
      .counter_width(counter_width),
      .rpm_counter_width(rpm_counter_width),
      .pwm_counter_width(pwm_counter_width)
  ) driver_ (
      .sys_clk(pclk),
      .pwm_clk(pwm_clk),

      .hall_values(hall_values),
      .invert_phases(invert_phases_),
      .fault_n(fault_n),
      .overcurrent_n(overcurrent_n),

      .detected_dir(detected_dir),
      .encoder_counter(enc_position_),
      .transitions_per_period(transitions_per_period_),
      .rpm(rpm_ms_),
      .rpm_1s(rpm_1s_),
      .sector(sector_),
      .hall_error(hall_error_),

      .phase_enable(phase_enable),
      .pwm_out(pwm_out),
      .gate_enable(gate_enable),

      .enable(speed_ctl_drv_enable_),
      .direction(pos_ctl_dir_),
      .pwm_duty(speed_pwm_duty_),

      .pwm_cycle_ticks(pwm_cycle_ticks_),
      .driver_state(driver_state_),

      .reset_n(preset_n)
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

      manual_enable_ <= 0;
      manual_dir_ <= DIR_NONE;
      user_pwm_duty_ <= 0;
      pos_ctl_enable_ <= 0;
      user_speed_ctl_enable_ <= 0;

      target_position_ <= 0;
      user_target_rpm_ <= 0;

      target_rpm_ <= 0;
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
          reg_enc_transitions_per_period: read_transitions_per_period();
          reg_rpm: read_rpm();
          reg_control: read_control_register();
          reg_pwm_control: read_pwm_control_register();
          reg_pos_control: read_pos_control_register();
          reg_tgt_pos: read_target_pos_register();
          reg_speed_control: read_speed_control_register();
          // Write requested address for now
          default: prdata[addr_width-1:0] <= paddr;
        endcase
        pready <= 1;
      end
      apb_state_ <= idle_state;
    end
  endtask

  task read_status_register();
    localparam reg_status_padding = {(data_width - 6) {1'b0}};
    begin
      prdata <= {
        reg_status_padding,
        pole_pairs_,
        driver_state_,
        ~overcurrent_n,
        ~fault_n,
        hall_error_,
        phase_enable,
        detected_dir,
        sector_,
        hall_values
      };
    end
  endtask

  task read_enc_counter();
    begin
      prdata <= enc_position_;
    end
  endtask

  task read_transitions_per_period();
    begin
      prdata <= transitions_per_period_;
    end
  endtask

  task read_rpm();
    localparam reg_rpm_padding = {(data_width / 2 - rpm_counter_width) {1'b0}};
    begin
      prdata <= {reg_rpm_padding, rpm_1s_, reg_rpm_padding, rpm_ms_};
    end
  endtask

  task read_control_register();
    localparam reg_control_padding = {(data_width - 4) {1'b0}};
    begin
      prdata <= {reg_control_padding, invert_phases_, pos_ctl_dir_, manual_enable_};
    end
  endtask

  task read_pwm_control_register();
    localparam reg_pwm_padding = {(16 - pwm_counter_width) {1'b0}};
    begin
      prdata <= {reg_pwm_padding, pwm_cycle_ticks_, reg_pwm_padding, pos_ctl_pwm_duty_};
    end
  endtask

  task read_pos_control_register();
    localparam reg_pos_control_padding = {(data_width - 1) {1'b0}};
    begin
      prdata <= {reg_pos_control_padding, pos_ctl_enable_};
    end
  endtask

  task read_target_pos_register();
    begin
      prdata <= target_position_;
    end
  endtask

  task read_speed_control_register();
    localparam reg_speed_control_padding = {(data_width - rpm_counter_width - 1) {1'b0}};
    begin
      prdata <= {reg_speed_control_padding, target_rpm_, user_speed_ctl_enable_};
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
          reg_pos_control: write_pos_control_register();
          reg_tgt_pos: write_target_pos_register();
          default: pslverr <= 1;
        endcase
        pready <= 1;
      end
      apb_state_ <= idle_state;
    end
  endtask

  task write_control_register();
    begin
      manual_enable_ = pwdata[0];
      if (manual_enable_) manual_dir_ = rotation_direction_t'(pwdata[2:1]);
      else manual_dir_ = DIR_NONE;
      invert_phases_ = pwdata[3];
    end
  endtask

  task write_pwm_control_register();
    begin
      user_pwm_duty_ <= pwdata[pwm_counter_width-1:0];
    end
  endtask

  task write_pos_control_register();
    begin
      pos_ctl_enable_ <= pwdata[0];
    end
  endtask

  task write_target_pos_register();
    begin
      target_position_ <= pwdata;
    end
  endtask

  task write_speed_control_register();
    begin
      user_speed_ctl_enable_ <= pwdata[0];
      user_target_rpm_ <= pwdata[rpm_counter_width:1];
    end
  endtask

  //--------------------------------------------------------------------------
  // Always block for handling APB requests
  //--------------------------------------------------------------------------
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
