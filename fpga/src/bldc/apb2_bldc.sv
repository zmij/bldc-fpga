/**

@file apb2_bldc_peripheral.sv
@brief APB2 BLDC Peripheral Module
This module is a peripheral device for a BLDC motor controller using the APB2 bus interface.
It provides an interface to control and monitor the BLDC motor.

*/
`ifndef __APB2_BLDC_SV__
`define __APB2_BLDC_SV__

`include "bldc/types.sv"

/**
 * @class apb2_bldc_perpheral
 *
 * @brief APB2 BLDC Peripheral Module
 *
 * This module provides an interface to control and monitor a BLDC motor using the APB2 bus.
 * It provides registers to configure the motor parameters and read the motor status.
*/
module apb2_bldc_perpheral #(
    // Data width for APB2 bus
    parameter data_width = 32,
    // Address width for APB2 bus
    parameter addr_width = 8,
    // APB2 clock frequency
    parameter clk_freq_hz = 54_000_000,
    // Encoder clock frequency
    parameter enc_freq_hz = 27_000_000,
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
    @name Encoder Interface Signals
    @defgroup Encoder_Interface Encoder Interface Signals
    @brief Signals used to interface with the encoder
    @{
    */
    /**
    @brief Input clock for the encoder
    This signal is the input clock for the encoder.
    */
    input encoder_clk,
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

    /**
    @brief Phase enable signal
    This signal is used to enable the phases of the BLDC motor.
    The signal is a 6-bit vector, where the first 3 bits represent the high side
    phase enable signals (AH, BH, CH), and the last 3 bits represent the low side
    phase enable signals (AL, BL, CL). 
    */
    output [5:0] phase_enable,
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

  typedef enum logic [1:0] {
    idle_state,
    r_enable,
    w_enable
  } apb_state_t;

  apb_state_t apb_state_;

  always @(negedge preset_n or posedge pclk) begin
    if (preset_n == 0) begin
      apb_state_ <= idle_state;
      prdata <= {(data_width) {1'b0}};
      pready <= 0;
      pslverr <= 0;
    end else begin
      case (apb_state_)
        idle_state: begin
          prdata  <= {(data_width) {1'bz}};
          pready  <= 0;
          pslverr <= 0;
          if (psel) begin
            if (pwrite) apb_state_ <= w_enable;
            else apb_state_ <= r_enable;
          end
        end
        r_enable: begin
          if (psel && !pwrite && penable) begin
            // Write requested address for now
            prdata[addr_width-1:0] <= paddr;
            pready <= 1;
            apb_state_ <= idle_state;
          end
        end
        w_enable: begin
          if (psel && !pwrite && penable) begin
            // Do nothing for now
            pready <= 1;
            apb_state_ <= idle_state;
          end
        end
      endcase
    end
  end

endmodule
/** @} */  // end of apb2_bldc_peripheral

`endif  // __APB2_BLDC_SB__
