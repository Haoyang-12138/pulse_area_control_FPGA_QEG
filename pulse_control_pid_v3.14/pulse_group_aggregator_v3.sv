`timescale 1ns / 1ps

// Collects every detected ADC pulse into a group, emits a per-pulse logging
// record, and computes the mean area of valid pulses at each group boundary.
// The mean division is implemented as a pipelined reciprocal multiply (1..32)
// rather than a large single-cycle divider.
module pulse_group_aggregator_v3 #(
    parameter int MAX_PULSES_PER_GROUP = 32
)(
    input  logic                 clk_i,
    input  logic                 rstn_i,
    input  logic                 clear_state_i,

    input  logic [63:0]          timestamp_i,

    input  logic                 boundary_event_i,
    input  logic [15:0]          boundary_delay_samples_i,
    // When asserted, the first boundary arms group 0 instead of closing an
    // implicit pre-trigger group.  Subsequent boundaries close the current
    // group and immediately start the next one.  This is used by the new
    // external-trigger source; legacy DAC2/manual sources retain old semantics.
    input  logic                 boundary_starts_group_i,
    input  logic                 measurement_busy_i,

    input  logic                 measurement_valid_i,
    input  logic                 invalid_pulse_valid_i,
    input  logic [7:0]           invalid_reason_i,
    input  logic signed [13:0]   bpre_i,
    input  logic signed [13:0]   bpost_i,
    input  logic signed [13:0]   threshold_i,
    input  logic signed [13:0]   peak_raw_i,
    input  logic [15:0]          peak_height_i,
    input  logic [31:0]          duration_samples_i,
    input  logic signed [31:0]   area_i,

    input  logic [7:0]           expected_pulses_per_group_i,
    input  logic [7:0]           min_valid_pulses_per_group_i,

    input  logic signed [15:0]   correction_active_i,
    input  logic signed [15:0]   effective_dac_high_i,

    // One record for every valid or invalid detected pulse.
    output logic                 pulse_record_valid_o,
    output logic [319:0]         pulse_record_o,

    // One summary for every completed group.
    output logic                 group_ready_valid_o,
    output logic [63:0]          group_timestamp_o,
    output logic [31:0]          group_id_o,
    output logic [7:0]           detected_pulses_o,
    output logic [7:0]           valid_pulses_o,
    output logic [7:0]           invalid_pulses_o,
    output logic                 expected_mismatch_o,
    output logic                 group_overflow_o,
    output logic                 boundary_overrun_o,
    output logic                 group_valid_for_feedback_o,
    output logic signed [31:0]   mean_area_o,

    output logic [31:0]          total_valid_measurements_o,
    output logic [31:0]          total_invalid_measurements_o,
    output logic [31:0]          total_groups_o
);

    logic [31:0] current_group_id;
    logic [15:0] current_pulse_id;
    logic [7:0] detected_count;
    logic [7:0] valid_count;
    logic [7:0] invalid_count;
    logic signed [47:0] area_sum;
    logic group_overflow_work;
    logic boundary_overrun_work;

    logic boundary_pending;
    logic [15:0] boundary_delay_counter;
    logic close_requested;
    logic group_active;

    logic [63:0] closed_timestamp;
    logic [31:0] closed_group_id;
    logic [7:0] closed_detected;
    logic [7:0] closed_valid;
    logic [7:0] closed_invalid;
    logic signed [47:0] closed_area_sum;
    logic closed_expected_mismatch;
    logic closed_group_overflow;
    logic closed_boundary_overrun;

    logic closed_pending;
    logic mean_recip_pending;
    logic mean_mul_pending;
    logic signed [73:0] mean_product_reg;
    logic [24:0] reciprocal_value;
    logic [24:0] reciprocal_reg;

    logic signed [15:0] bpre16;
    logic signed [15:0] bpost16;
    logic signed [15:0] threshold16;
    logic signed [15:0] peak16;

    function automatic logic [24:0] reciprocal_q24(input logic [7:0] n);
        begin
            case (n)
                8'd1: reciprocal_q24 = 25'd16777216;
                8'd2: reciprocal_q24 = 25'd8388608;
                8'd3: reciprocal_q24 = 25'd5592405;
                8'd4: reciprocal_q24 = 25'd4194304;
                8'd5: reciprocal_q24 = 25'd3355443;
                8'd6: reciprocal_q24 = 25'd2796203;
                8'd7: reciprocal_q24 = 25'd2396745;
                8'd8: reciprocal_q24 = 25'd2097152;
                8'd9: reciprocal_q24 = 25'd1864135;
                8'd10: reciprocal_q24 = 25'd1677722;
                8'd11: reciprocal_q24 = 25'd1525201;
                8'd12: reciprocal_q24 = 25'd1398101;
                8'd13: reciprocal_q24 = 25'd1290555;
                8'd14: reciprocal_q24 = 25'd1198373;
                8'd15: reciprocal_q24 = 25'd1118481;
                8'd16: reciprocal_q24 = 25'd1048576;
                8'd17: reciprocal_q24 = 25'd986895;
                8'd18: reciprocal_q24 = 25'd932068;
                8'd19: reciprocal_q24 = 25'd883011;
                8'd20: reciprocal_q24 = 25'd838861;
                8'd21: reciprocal_q24 = 25'd798915;
                8'd22: reciprocal_q24 = 25'd762601;
                8'd23: reciprocal_q24 = 25'd729444;
                8'd24: reciprocal_q24 = 25'd699051;
                8'd25: reciprocal_q24 = 25'd671089;
                8'd26: reciprocal_q24 = 25'd645278;
                8'd27: reciprocal_q24 = 25'd621378;
                8'd28: reciprocal_q24 = 25'd599186;
                8'd29: reciprocal_q24 = 25'd578525;
                8'd30: reciprocal_q24 = 25'd559241;
                8'd31: reciprocal_q24 = 25'd541201;
                8'd32: reciprocal_q24 = 25'd524288;
                default: reciprocal_q24 = 25'd0;
            endcase
        end
    endfunction

    assign bpre16     = {{2{bpre_i[13]}}, bpre_i};
    assign bpost16    = {{2{bpost_i[13]}}, bpost_i};
    assign threshold16= {{2{threshold_i[13]}}, threshold_i};
    assign peak16     = {{2{peak_raw_i[13]}}, peak_raw_i};
    assign reciprocal_value = reciprocal_q24(closed_valid);

    always_ff @(posedge clk_i) begin
        if (!rstn_i || clear_state_i) begin
            current_group_id              <= 32'd0;
            current_pulse_id              <= 16'd0;
            detected_count                <= 8'd0;
            valid_count                   <= 8'd0;
            invalid_count                 <= 8'd0;
            area_sum                      <= '0;
            group_overflow_work           <= 1'b0;
            boundary_overrun_work         <= 1'b0;

            boundary_pending              <= 1'b0;
            boundary_delay_counter        <= 16'd0;
            close_requested               <= 1'b0;
            group_active                  <= 1'b0;

            closed_pending                <= 1'b0;
            mean_recip_pending            <= 1'b0;
            mean_mul_pending              <= 1'b0;
            reciprocal_reg                <= '0;
            mean_product_reg              <= '0;

            pulse_record_valid_o          <= 1'b0;
            pulse_record_o                <= '0;

            group_ready_valid_o           <= 1'b0;
            group_timestamp_o             <= '0;
            group_id_o                    <= '0;
            detected_pulses_o             <= '0;
            valid_pulses_o                <= '0;
            invalid_pulses_o              <= '0;
            expected_mismatch_o           <= 1'b0;
            group_overflow_o              <= 1'b0;
            boundary_overrun_o            <= 1'b0;
            group_valid_for_feedback_o    <= 1'b0;
            mean_area_o                   <= '0;

            total_valid_measurements_o    <= '0;
            total_invalid_measurements_o  <= '0;
            total_groups_o                <= '0;
        end
        else begin
            pulse_record_valid_o <= 1'b0;
            group_ready_valid_o  <= 1'b0;

            // Schedule a group boundary.  A programmable ADC-clock delay is
            // useful when GPIO timing precedes the optical/photodiode signal.
            if (boundary_event_i) begin
                if (boundary_pending || close_requested)
                    boundary_overrun_work <= 1'b1;
                boundary_pending       <= 1'b1;
                boundary_delay_counter <= boundary_delay_samples_i;
            end

            if (boundary_pending) begin
                if (boundary_delay_counter == 16'd0) begin
                    boundary_pending <= 1'b0;
                    if (boundary_starts_group_i && !group_active) begin
                        // External-trigger semantics: the first delayed trigger
                        // starts group 0.  Any detector results that happened
                        // before this point are deliberately ignored.
                        group_active       <= 1'b1;
                        current_group_id   <= 32'd0;
                        current_pulse_id   <= 16'd0;
                        detected_count     <= 8'd0;
                        valid_count        <= 8'd0;
                        invalid_count      <= 8'd0;
                        area_sum           <= '0;
                        group_overflow_work<= 1'b0;
                        boundary_overrun_work <= 1'b0;
                    end
                    else begin
                        close_requested <= 1'b1;
                    end
                end
                else begin
                    boundary_delay_counter <= boundary_delay_counter - 1'b1;
                end
            end

            // Log every detected pulse only after the first external trigger
            // when start-on-first-boundary mode is active.  The ADC detector
            // itself remains continuously enabled so its rolling baseline is
            // not re-primed on every group.
            if ((measurement_valid_i || invalid_pulse_valid_i) &&
                (!boundary_starts_group_i || group_active)) begin
                pulse_record_valid_o <= 1'b1;
                pulse_record_o <= {
                    {peak_height_i, 16'd0},                                      // word9
                    {correction_active_i, effective_dac_high_i},                 // word8
                    (measurement_valid_i ? area_i : 32'sd0),                     // word7
                    duration_samples_i,                                           // word6
                    {threshold16, peak16},                                         // word5
                    {bpre16, bpost16},                                             // word4
                    {current_pulse_id, invalid_reason_i,
                     6'd0, invalid_pulse_valid_i, measurement_valid_i},            // word3
                    current_group_id,                                              // word2
                    timestamp_i[63:32],                                             // word1
                    timestamp_i[31:0]                                               // word0
                };

                current_pulse_id <= current_pulse_id + 1'b1;
                if (detected_count != 8'hFF)
                    detected_count <= detected_count + 1'b1;

                if ((detected_count + 1'b1) > MAX_PULSES_PER_GROUP)
                    group_overflow_work <= 1'b1;

                if (measurement_valid_i) begin
                    if (valid_count != 8'hFF)
                        valid_count <= valid_count + 1'b1;
                    area_sum <= area_sum + {{16{area_i[31]}}, area_i};
                    total_valid_measurements_o <= total_valid_measurements_o + 1'b1;
                end
                else begin
                    if (invalid_count != 8'hFF)
                        invalid_count <= invalid_count + 1'b1;
                    total_invalid_measurements_o <= total_invalid_measurements_o + 1'b1;
                end
            end

            // Never split a pulse across groups.  Also avoid closing on the
            // same clock that a valid/invalid pulse result is being emitted.
            if (close_requested && !measurement_busy_i &&
                !measurement_valid_i && !invalid_pulse_valid_i &&
                !closed_pending && !mean_recip_pending && !mean_mul_pending) begin
                closed_timestamp          <= timestamp_i;
                closed_group_id           <= current_group_id;
                closed_detected           <= detected_count;
                closed_valid              <= valid_count;
                closed_invalid            <= invalid_count;
                closed_area_sum           <= area_sum;
                closed_expected_mismatch  <=
                    (expected_pulses_per_group_i != 8'd0) &&
                    (detected_count != expected_pulses_per_group_i);
                closed_group_overflow     <= group_overflow_work;
                closed_boundary_overrun   <= boundary_overrun_work;
                closed_pending            <= 1'b1;
                close_requested           <= 1'b0;

                current_group_id          <= current_group_id + 1'b1;
                current_pulse_id          <= 16'd0;
                detected_count            <= 8'd0;
                valid_count               <= 8'd0;
                invalid_count             <= 8'd0;
                area_sum                  <= '0;
                group_overflow_work       <= 1'b0;
                boundary_overrun_work     <= 1'b0;
                total_groups_o            <= total_groups_o + 1'b1;
            end

            // Stage 1: register the reciprocal lookup.  The v3.7 routed design
            // drove closed_valid through the 1..32 reciprocal LUT network directly
            // into the DSP multiplier and missed 125 MHz by ~0.38 ns.
            if (closed_pending) begin
                closed_pending <= 1'b0;
                if (closed_valid != 8'd0 && closed_valid <= MAX_PULSES_PER_GROUP)
                    reciprocal_reg <= reciprocal_value;
                else
                    reciprocal_reg <= 25'd0;
                mean_recip_pending <= 1'b1;
            end

            // Stage 2: multiply the registered area sum by the registered reciprocal.
            if (mean_recip_pending) begin
                mean_recip_pending <= 1'b0;
                mean_product_reg <= $signed(closed_area_sum) * $signed({1'b0, reciprocal_reg});
                mean_mul_pending <= 1'b1;
            end

            // Stage 3: emit the completed group.  No variable divider is used.
            if (mean_mul_pending) begin
                mean_mul_pending           <= 1'b0;
                group_ready_valid_o        <= 1'b1;
                group_timestamp_o          <= closed_timestamp;
                group_id_o                 <= closed_group_id;
                detected_pulses_o          <= closed_detected;
                valid_pulses_o             <= closed_valid;
                invalid_pulses_o           <= closed_invalid;
                expected_mismatch_o        <= closed_expected_mismatch;
                group_overflow_o           <= closed_group_overflow;
                boundary_overrun_o         <= closed_boundary_overrun;
                group_valid_for_feedback_o <=
                    (closed_valid >= min_valid_pulses_per_group_i) &&
                    (closed_valid != 8'd0) &&
                    (closed_valid <= MAX_PULSES_PER_GROUP) &&
                    !closed_group_overflow;
                mean_area_o <= $signed(mean_product_reg >>> 24);
            end
        end
    end

endmodule
