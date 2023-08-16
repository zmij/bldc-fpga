`ifndef __WINDOW_COUNTER_SV__
`define __WINDOW_COUNTER_SV__

module window_counter #(
    parameter clk_freq_hz = 27_000_000,
    parameter max_possible_value = 4000,
    parameter sample_time_ms = 10,
    parameter sample_count = 1000 / sample_time_ms,
    parameter counter_width = $clog2(max_possible_value)
) (
    input logic sys_clk,

    input logic enable,
    output logic [counter_width - 1:0] counter,
    output logic valid,
    output logic updated,

    input logic reset_n
);
  localparam ticks_per_milli = clk_freq_hz / 1000;
  localparam sample_period = ticks_per_milli * sample_time_ms;

  localparam max_sample = max_possible_value / sample_count + 1;
  localparam sample_width = $clog2(max_sample);
  localparam index_width = $clog2(sample_count);
  localparam time_counter_width = $clog2(sample_period);

  typedef logic [sample_width - 1:0] sample_t;
  typedef logic [counter_width - 1:0] counter_t;
  typedef logic [index_width - 1:0] index_t;
  typedef logic [time_counter_width - 1:0] time_counter_t;

  time_counter_t period_counter_;
  index_t sample_index_;
  sample_t current_sample_;
  sample_t [0:sample_count - 1] samples_;
  logic next_sample;

  function index_t next_index(input index_t idx);
    logic [index_width:0] res;
    begin
      if (idx == sample_count - 1) begin
        return 0;
      end else begin
        res = idx + 1;
        return res[index_width-1:0];
      end
    end
  endfunction

  function sample_t truncate_sample(input logic [sample_width:0] val);
    return val[sample_width-1:0];
  endfunction

  function time_counter_t truncate_period(input logic [time_counter_width:0] val);
    return val[time_counter_width-1:0];
  endfunction

  task automatic reset_current();
    begin
      current_sample_ <= 0;
    end
  endtask

  task automatic update_current();
    begin
      if (next_sample) begin
        current_sample_ <= enable;
      end else begin
        if (enable) begin
          current_sample_ <= truncate_sample(current_sample_ + 1);
        end
      end
    end
  endtask

  // Update current sample
  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (!reset_n) begin
      reset_current();
    end else begin
      update_current();
    end
  end

  task automatic reset_samples();
    begin
      samples_ <= {(sample_width * sample_count) {1'b0}};
      sample_index_ <= 0;
    end
  endtask

  task automatic update_samples();
    begin
      if (next_sample) begin
        samples_[sample_index_] <= current_sample_;
        sample_index_ <= next_index(sample_index_);
      end
    end
  endtask

  // Update samples array
  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (!reset_n) begin
      reset_samples();
    end else begin
      update_samples();
    end
  end

  task automatic reset_counter();
    begin
      counter <= 0;
      valid   <= 0;
      updated <= 0;
    end
  endtask

  task automatic update_counter();
    begin
      if (next_sample) begin
        counter <= counter - samples_[sample_index_] + current_sample_;
        if (sample_index_ == sample_count - 1) begin
          // Full samples
          valid <= 1;
        end
        updated <= 1;
      end else begin
        updated <= 0;
      end
    end
  endtask

  // Update counter
  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (!reset_n) begin
      reset_counter();
    end else begin
      update_counter();
    end
  end

  // Count ticks
  always_ff @(posedge sys_clk or negedge reset_n) begin
    if (!reset_n) begin
      period_counter_ <= 0;
    end else begin
      if (period_counter_ == sample_period - 1) begin
        period_counter_ <= 0;
      end else begin
        period_counter_ <= truncate_period(period_counter_ + 1);
      end
    end
  end

  assign next_sample = (period_counter_ == sample_period - 1) ? 1 : 0;
endmodule

`endif  // __WINDOW_COUNTER_SV__
