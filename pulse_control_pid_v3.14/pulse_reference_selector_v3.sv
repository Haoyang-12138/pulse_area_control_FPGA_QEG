`timescale 1ns / 1ps

// PC06 reference-pulse selector / accumulator.
//
// This module is deliberately placed AFTER the already-verified ADC pulse
// detector/group pulse-record formatter and BEFORE the existing PID.  It does
// not alter pulse detection, baseline estimation, the Q1.31 PID arithmetic, or
// the DAC2 physical-output path.
//
// A delayed/qualified external trigger produces reference_start_i in the
// 125 MHz ADC domain.  On that same clock edge reference_window_active_o rises;
// red_pitaya_top drives this signal to DIO2_P as the scope calibration marker.
// The window remains HIGH until the configured target pulse set is complete.
//
// Pulse positions are counted from zero after reference start.  Selection is a
// contiguous interval:
//     target_start_index_i ... target_start_index_i + target_pulse_count_i - 1
// Every detected pulse advances the index, whether valid or invalid.  Therefore
// an invalid physical pulse can never cause a later pulse to be silently
// substituted into the PID reference set.
//
// All selected valid areas are accumulated in a signed 48-bit sum.  As soon as
// the last selected target pulse completes, reference_ready_valid_o is emitted
// and the existing PID can start immediately; no following external trigger is
// required to close the feedback reference.
//
// Sparse logging semantics:
//   snapshot_arm_i waits for the NEXT reference start, captures only the pulse
//   records that are actually selected for that PID reference, and auto-disarms
//   when that target set completes.  The record format remains 320 bits.  Word2
//   is rewritten to reference_id and word3 pulse_id is rewritten to the
//   reference-relative detected pulse index.
module pulse_reference_selector_v3 (
    input  logic                 clk_i,
    input  logic                 rstn_i,
    input  logic                 clear_state_i,

    input  logic [63:0]          timestamp_i,
    input  logic                 reference_start_i,
    input  logic                 measurement_busy_i,

    input  logic [4:0]           target_start_index_i, // 0..31
    input  logic [5:0]           target_pulse_count_i, // 1..32

    // Existing coherent pulse record from pulse_group_aggregator_v3.
    input  logic                 pulse_record_valid_i,
    input  logic [319:0]         pulse_record_i,

    input  logic                 snapshot_arm_i,

    // Marker/reference-window state.  DIO2_P should be driven from this signal
    // (optionally gated by the existing marker_enable control bit).
    output logic                 reference_window_active_o,
    output logic                 config_valid_o,

    // One completion event per reference window.  This is the new PID source.
    output logic                 reference_ready_valid_o,
    output logic [63:0]          reference_timestamp_o,
    output logic [31:0]          reference_id_o,
    output logic [7:0]           reference_detected_pulses_o,
    output logic [7:0]           reference_valid_pulses_o,
    output logic [7:0]           reference_invalid_pulses_o,
    output logic                 reference_valid_for_feedback_o,
    output logic signed [31:0]   reference_sum_area_o,
    output logic                 reference_sum_overflow_o,
    output logic                 reference_incomplete_o,

    // Selected target-pulse record stream.  selected_* is asserted every
    // reference; snapshot_* is asserted only for an armed sparse capture.
    output logic                 selected_pulse_record_valid_o,
    output logic [319:0]         selected_pulse_record_o,
    output logic                 snapshot_pulse_record_valid_o,
    output logic [319:0]         snapshot_pulse_record_o,
    output logic                 snapshot_done_o,

    // Live diagnostics.
    output logic [31:0]          active_reference_id_o,
    output logic [5:0]           active_selected_count_o
);

    logic [31:0] next_reference_id;
    logic [31:0] current_reference_id;
    logic [5:0]  detected_index;
    logic [5:0]  selected_count;
    logic [5:0]  selected_valid_count;
    logic [5:0]  selected_invalid_count;
    logic signed [47:0] area_sum;
    logic        drop_inflight_record;

    logic        snapshot_arm_pending;
    logic        snapshot_capture_active;

    wire pulse_is_valid   = pulse_record_i[96];
    wire pulse_is_invalid = pulse_record_i[97] | ~pulse_record_i[96];
    wire signed [31:0] pulse_area = $signed(pulse_record_i[255:224]);

    wire [6:0] target_end_exclusive =
        {2'b00, target_start_index_i} + {1'b0, target_pulse_count_i};

    assign config_valid_o =
        (target_pulse_count_i >= 6'd1) &&
        (target_pulse_count_i <= 6'd32) &&
        (target_end_exclusive <= 7'd32);

    wire current_index_selected =
        ({1'b0, detected_index} >= {2'b00, target_start_index_i}) &&
        ({1'b0, detected_index} < target_end_exclusive);

    wire signed [47:0] pulse_area_ext =
        {{16{pulse_area[31]}}, pulse_area};
    wire signed [47:0] sum_with_current =
        area_sum + (pulse_is_valid ? pulse_area_ext : 48'sd0);

    wire sum_with_current_overflows_s32 =
        (sum_with_current > 48'sd2147483647) ||
        (sum_with_current < -48'sd2147483648);

    // Rewrite only the identity fields of the already-coherent pulse record.
    // All measurement fields (baseline, threshold, duration, area, peak, DAC
    // state, timestamp) pass through byte-for-byte.
    logic [319:0] rewritten_record;
    always_comb begin
        rewritten_record = pulse_record_i;
        rewritten_record[95:64]   = current_reference_id;       // word2
        rewritten_record[127:112] = {10'd0, detected_index};    // word3 pulse_id
    end

    assign active_reference_id_o    = current_reference_id;
    assign active_selected_count_o  = selected_count;

    always_ff @(posedge clk_i) begin
        if (!rstn_i || clear_state_i) begin
            next_reference_id                <= 32'd0;
            current_reference_id             <= 32'd0;
            detected_index                   <= 6'd0;
            selected_count                   <= 6'd0;
            selected_valid_count             <= 6'd0;
            selected_invalid_count           <= 6'd0;
            area_sum                         <= 48'sd0;
            drop_inflight_record             <= 1'b0;

            snapshot_arm_pending             <= 1'b0;
            snapshot_capture_active          <= 1'b0;

            reference_window_active_o        <= 1'b0;
            reference_ready_valid_o          <= 1'b0;
            reference_timestamp_o            <= 64'd0;
            reference_id_o                   <= 32'd0;
            reference_detected_pulses_o      <= 8'd0;
            reference_valid_pulses_o         <= 8'd0;
            reference_invalid_pulses_o       <= 8'd0;
            reference_valid_for_feedback_o   <= 1'b0;
            reference_sum_area_o             <= 32'sd0;
            reference_sum_overflow_o         <= 1'b0;
            reference_incomplete_o           <= 1'b0;

            selected_pulse_record_valid_o    <= 1'b0;
            selected_pulse_record_o          <= 320'd0;
            snapshot_pulse_record_valid_o    <= 1'b0;
            snapshot_pulse_record_o          <= 320'd0;
            snapshot_done_o                  <= 1'b0;
        end
        else begin
            reference_ready_valid_o       <= 1'b0;
            reference_sum_overflow_o      <= 1'b0;
            reference_incomplete_o        <= 1'b0;
            selected_pulse_record_valid_o <= 1'b0;
            snapshot_pulse_record_valid_o <= 1'b0;
            snapshot_done_o               <= 1'b0;

            if (snapshot_arm_i)
                snapshot_arm_pending <= 1'b1;

            if (reference_start_i) begin
                // A new qualified reference arriving before the old target set
                // completes invalidates the old reference.  No PID update is
                // emitted for the incomplete set.
                if (reference_window_active_o) begin
                    reference_incomplete_o <= 1'b1;
                    if (snapshot_capture_active)
                        snapshot_done_o <= 1'b1;
                end

                if (config_valid_o) begin
                    current_reference_id      <= next_reference_id;
                    next_reference_id         <= next_reference_id + 32'd1;
                    detected_index            <= 6'd0;
                    selected_count            <= 6'd0;
                    selected_valid_count      <= 6'd0;
                    selected_invalid_count    <= 6'd0;
                    area_sum                  <= 48'sd0;
                    reference_window_active_o <= 1'b1;

                    // If a pulse was already in progress at REFERENCE_START,
                    // discard exactly its eventual record.  The next newly
                    // detected pulse becomes reference-relative pulse index 0.
                    drop_inflight_record <= measurement_busy_i;

                    snapshot_capture_active <=
                        snapshot_arm_pending | snapshot_arm_i;
                    if (snapshot_arm_pending | snapshot_arm_i)
                        snapshot_arm_pending <= 1'b0;
                end
                else begin
                    reference_window_active_o <= 1'b0;
                    snapshot_capture_active   <= 1'b0;
                    drop_inflight_record      <= 1'b0;
                end
            end
            else if (reference_window_active_o && pulse_record_valid_i) begin
                if (drop_inflight_record) begin
                    // This record belongs to a pulse whose start preceded the
                    // reference marker, so it must not receive index 0.
                    drop_inflight_record <= 1'b0;
                end
                else begin
                    if (current_index_selected) begin
                        selected_pulse_record_valid_o <= 1'b1;
                        selected_pulse_record_o       <= rewritten_record;

                        if (snapshot_capture_active) begin
                            snapshot_pulse_record_valid_o <= 1'b1;
                            snapshot_pulse_record_o       <= rewritten_record;
                        end

                        selected_count <= selected_count + 6'd1;
                        if (pulse_is_valid) begin
                            selected_valid_count <= selected_valid_count + 6'd1;
                            area_sum             <= sum_with_current;
                        end
                        else begin
                            selected_invalid_count <= selected_invalid_count + 6'd1;
                        end

                        // This is the final configured target pulse.  Close the
                        // reference immediately and launch the PID source event.
                        if ((selected_count + 6'd1) >= target_pulse_count_i) begin
                            reference_window_active_o      <= 1'b0;
                            reference_ready_valid_o        <= 1'b1;
                            reference_timestamp_o          <= timestamp_i;
                            reference_id_o                 <= current_reference_id;
                            reference_detected_pulses_o    <= {2'd0, target_pulse_count_i};
                            reference_valid_pulses_o       <=
                                {2'd0, selected_valid_count} +
                                (pulse_is_valid ? 8'd1 : 8'd0);
                            reference_invalid_pulses_o     <=
                                {2'd0, selected_invalid_count} +
                                (pulse_is_invalid ? 8'd1 : 8'd0);
                            reference_sum_overflow_o       <=
                                sum_with_current_overflows_s32;
                            reference_sum_area_o           <=
                                sum_with_current_overflows_s32
                                    ? 32'sd0
                                    : sum_with_current[31:0];
                            reference_valid_for_feedback_o <=
                                pulse_is_valid &&
                                (selected_invalid_count == 6'd0) &&
                                !sum_with_current_overflows_s32;

                            if (snapshot_capture_active) begin
                                snapshot_capture_active <= 1'b0;
                                snapshot_done_o         <= 1'b1;
                            end
                        end
                    end

                    // Every detected record advances physical pulse position,
                    // including invalid records and prefix pulses not selected.
                    if (detected_index != 6'd63)
                        detected_index <= detected_index + 6'd1;
                end
            end
        end
    end

endmodule
