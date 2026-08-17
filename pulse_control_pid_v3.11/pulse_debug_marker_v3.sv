`timescale 1ns / 1ps

// GPIO debug marker for end-to-end latency measurements.
// Starts on the first ADC threshold crossing in a group and ends when the group
// controller emits a result (correction applied or not).  It is intended for
// DIO2_P, leaving OUT2 free for the programmable DAC waveform.
module pulse_debug_marker_v3 #(
    parameter int MAX_MARKER_CYCLES = 1250000 // 10 ms at 125 MHz
)(
    input  logic clk_i,
    input  logic rstn_i,
    input  logic clear_state_i,
    input  logic marker_enable_i,
    input  logic first_crossing_valid_i,
    input  logic group_result_valid_i,
    output logic marker_o,
    output logic [31:0] last_marker_cycles_o,
    output logic timeout_seen_o
);

    logic [31:0] counter;
    logic active;

    always_ff @(posedge clk_i) begin
        if (!rstn_i || clear_state_i) begin
            marker_o              <= 1'b0;
            counter               <= 32'd0;
            active                <= 1'b0;
            last_marker_cycles_o  <= 32'd0;
            timeout_seen_o        <= 1'b0;
        end
        else begin
            if (!marker_enable_i) begin
                marker_o <= 1'b0;
                counter  <= 32'd0;
                active   <= 1'b0;
            end
            else begin
                if (first_crossing_valid_i && !active) begin
                    marker_o <= 1'b1;
                    counter  <= 32'd0;
                    active   <= 1'b1;
                end
                else if (active && group_result_valid_i) begin
                    marker_o <= 1'b0;
                    last_marker_cycles_o <= counter;
                    counter <= 32'd0;
                    active  <= 1'b0;
                end
                else if (active) begin
                    if (counter + 1'b1 >= MAX_MARKER_CYCLES) begin
                        marker_o <= 1'b0;
                        last_marker_cycles_o <= counter + 1'b1;
                        timeout_seen_o <= 1'b1;
                        counter <= 32'd0;
                        active  <= 1'b0;
                    end
                    else begin
                        counter <= counter + 1'b1;
                    end
                end
            end
        end
    end

endmodule
