//`include "bldc/gate_driver_reset.sv"
`include "bldc/types.sv"
`include "bldc/debounce.sv"
`timescale 10ns / 10ns

module debounce_hall_values_tb;
  localparam tick = 2;
  localparam clk_freq_hz = 50_000_000;  // 20ns, #2 == 1 tick, 1µs = 50 ticks (#100)
  reg   clk = 1;
  logic reset_n = 0;


  always #1 clk = ~clk;

  initial begin
    $dumpfile("debounce_hall_values_tb");
    $dumpvars(0, debounce_hall_values_tb);
  end

  logic hall_a, hall_b, hall_c, fault_n, ocw_n;
  logic hall_a_d, hall_b_d, hall_c_d;

  debounce_us #(
      .clk_freq_hz(clk_freq_hz),
      .debounce_time_us(50)
  ) deb_a (
      .clk(clk),
      .signal_in(hall_a),
      .signal_out(hall_a_d)
  );

  debounce_us #(
      .clk_freq_hz(clk_freq_hz),
      .debounce_time_us(50)
  ) deb_b (
      .clk(clk),
      .signal_in(hall_b),
      .signal_out(hall_b_d)
  );

  debounce_us #(
      .clk_freq_hz(clk_freq_hz),
      .debounce_time_us(50)
  ) deb_c (
      .clk(clk),
      .signal_in(hall_c),
      .signal_out(hall_c_d)
  );

  task read_line(input int fd);
    string line;
    begin
      $fscanf(fd, "%s", line);
      $sscanf(line, "%d;%d;%d;%d;%d", hall_a, hall_b, hall_c, fault_n, ocw_n);
      $display("Hall values: %0d%0d%0d Fault: %0d OCW: %d", hall_a, hall_b, hall_c, fault_n, ocw_n);
    end
  endtask

  initial begin
    int fd;
    fd = $fopen("./test_data/hall_values.csv", "r");
    if (fd) $display("File was opened successfully: %0d", fd);
    else begin
      $error("Failed to opent file%0d", fd);
      $finish;
    end

    while (!$feof(
        fd
    )) begin
      read_line(fd);
      #(tick * 2);  // signal is sampled with 25MHz
    end
    // for (int i = 0; i < 5; ++i) begin
    //   read_line(fd);
    //   #(tick * 2);  // signal is sampled with 25MHz
    // end

    $fclose(fd);
    $finish;
  end


endmodule
