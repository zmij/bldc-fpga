`include "bldc/position_control.sv"

`timescale 10ns / 10ns

module position_control_tb;
  localparam tick = 2;
  localparam clk_freq_hz = 50_000_000;  // 20ns, #2 == 1 tick, 1µs = 50 ticks (#100)
  localparam counter_width = 32;
  localparam pwm_counter_width = 16;
  localparam max_counter = {(counter_width) {1'b1}};
  localparam half_counter = max_counter / 2;

  typedef logic [counter_width-1:0] counter_t;
  typedef logic [pwm_counter_width-1:0] pwm_counter_t;

  logic clk = 1;
  logic reset_n = 0;

  always #1 clk = ~clk;

  initial begin
    $dumpfile("position_control_tb");
    $dumpvars(0, position_control_tb);
  end

  logic enable = 0;

  counter_t encoder_position, target_position;
  pwm_counter_t duty_in, duty_out;
  rotation_direction_t dir_in, dir_out;


  position_control #(
      .clk_freq_hz(clk_freq_hz),
      .counter_width(counter_width),
      .pwm_counter_width(pwm_counter_width)
  ) dut_ (
      .sys_clk(clk),

      .enable(enable),
      .encoder_position(encoder_position),
      .target_position(target_position),

      .pwm_duty_in(duty_in),
      .dir_in(dir_in),

      .pwm_duty_out(duty_out),
      .dir_out(dir_out),

      .reset_n(reset_n)
  );

  task display_diff();
    begin
      $display("target - encoder == %d, ", target_position - encoder_position);
      $display("diff > half_counter == %d", target_position - encoder_position > half_counter);
    end
  endtask

  initial begin
    encoder_position = 0;
    dir_in = DIR_NONE;
    duty_in = 0;

    #tick;
    reset_n = 1;
    enable = 1;

    target_position = 100;
    display_diff();

    #tick;

    assert (dir_out == DIR_CW)
    else $error("Expected direction is CW, current is %0d", dir_out);

    target_position = max_counter - 100;
    display_diff();

    #tick;

    assert (dir_out == DIR_CCW)
    else $error("Expected direction is CCW, current is %0d", dir_out);

    encoder_position = 1000;
    target_position  = 500;
    display_diff();

    #tick;

    assert (dir_out == DIR_CCW)
    else $error("Expected direction is CCW, current is %0d", dir_out);

    encoder_position = target_position;
    #tick;

    assert (dir_out == DIR_BRAKE)
    else $error("Expected direction is BRAKE, current is %0d", dir_out);


    $finish;
  end
endmodule
