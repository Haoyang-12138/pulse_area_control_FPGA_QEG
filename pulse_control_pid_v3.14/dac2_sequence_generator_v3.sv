`timescale 1ns / 1ps

// Runtime-programmable OUT2 sequence generator, rewritten for robustness.
//
// PC06 keeps the v3.12 DAC timing/functional fix; only the sequence table/count expands to 32 segments.
// v3.12 DAC timing/functional fix:
//   * Keeps the v3.11 removal of the two-bank distributed-RAM waveform table.
//* Adds a registered correction/effective-HIGH candidate stage so the PID
//     correction cannot feed the waveform DSP through clamp arithmetic in the
//   same 125 MHz cycle.
//   * Generates the requested level through a short registered DSP pipeline.
//   * Applies PID correction only at a DAC period boundary.
//   * Delays period_boundary_o through the same pipeline as the physical sample,
//     keeping group timing aligned with the actual OUT2 waveform.
//   * A one-segment 0% or 100% waveform has an explicit direct path to LOW/HIGH.
//     This gives a built-in DC hardware sanity test without another FPGA build.
//
// Internal level conversion:
//   counts = low + (effective_high-low) * level_q16 / 65536
// with level 0x0000 treated as exact LOW and 0xFFFF treated as exact HIGH.
module dac2_sequence_generator_v3 #(
    parameter int MAX_SEGMENTS = 32
)(
    input  logic                 clk_i,
    input  logic                 rstn_i,

    input  logic                 enable_i,
    input  logic                 apply_i,

    input  logic [31:0]          period_cycles_shadow_i,
    input  logic [5:0]           segment_count_shadow_i,
    input  logic signed [15:0]   low_counts_shadow_i,
    input  logic signed [15:0]   high_counts_shadow_i,
    input  logic [15:0]          segment_level_q16_shadow_i [0:MAX_SEGMENTS-1],
    input  logic [31:0]          segment_duration_cycles_shadow_i [0:MAX_SEGMENTS-1],

    input  logic signed [15:0]   correction_pending_i,
    input  logic signed [15:0]   high_min_counts_i,
    input  logic signed [15:0]   high_max_counts_i,

    output logic signed [13:0]   dac_out_o,
    output logic                 period_boundary_o,
    output logic                 config_valid_o,

    output logic signed [15:0]   correction_active_o,
    output logic signed [15:0]   effective_high_counts_o,
    output logic signed [15:0]   nominal_high_counts_o,
    output logic signed [15:0]   nominal_low_counts_o,
    output logic [31:0]          active_period_cycles_o,
    output logic [5:0]           active_segment_count_o,
    output logic [4:0]           active_segment_index_o,
    output logic [15:0]          active_segment_level_q16_o
);

    logic [31:0] active_period_cycles;
    logic [5:0]  active_segment_count;
    logic signed [15:0] active_low_counts;
    logic signed [15:0] active_high_counts;
    logic signed [15:0] effective_high_reg;
    logic signed [15:0] correction_candidate_reg;
    logic signed [15:0] effective_high_candidate_reg;
    logic config_valid_reg;

    logic [4:0]  segment_index;
    logic [31:0] segment_cycle_count;
    logic [31:0] current_duration;
    logic [15:0] current_level;
    logic        enable_d;
    logic        boundary_pending;

    // Stage 1: registered span/level.
    logic signed [16:0] span_s1;
    logic signed [15:0] low_s1;
    logic signed [15:0] high_s1;
    logic        [15:0] level_s1;
    logic               valid_s1;
    logic               boundary_s1;

    // Stage 2: registered multiply.
    logic signed [33:0] product_s2;
    logic signed [33:0] low_shift_s2;
    logic signed [15:0] low_s2;
    logic signed [15:0] high_s2;
    logic        [15:0] level_s2;
    logic               valid_s2;
    logic               boundary_s2;

    // Stage 3: registered final output sample.
    logic signed [15:0] sample_s3;
    logic               valid_s3;
    logic               boundary_s3;

    logic single_segment_low;
    logic single_segment_high;

    function automatic logic signed [15:0] clamp_high(
        input logic signed [15:0] nominal_high,
        input logic signed [15:0] correction,
        input logic signed [15:0] minimum,
        input logic signed [15:0] maximum
    );
        logic signed [16:0] wide;
        begin
            wide = $signed(nominal_high) + $signed(correction);
            if (wide > $signed(maximum))
                clamp_high = maximum;
            else if (wide < $signed(minimum))
                clamp_high = minimum;
            else
                clamp_high = wide[15:0];
        end
    endfunction

    function automatic logic signed [13:0] clamp_dac14(
        input logic signed [15:0] v
    );
        begin
            if ($signed(v) > 16'sd8191)
                clamp_dac14 = 14'sd8191;
            else if ($signed(v) < -16'sd8192)
                clamp_dac14 = -14'sd8192;
            else
                clamp_dac14 = v[13:0];
        end
    endfunction

    assign config_valid_o = config_valid_reg;

    assign effective_high_counts_o     = effective_high_reg;
    assign nominal_high_counts_o       = active_high_counts;
    assign nominal_low_counts_o        = active_low_counts;
    assign active_period_cycles_o      = active_period_cycles;
    assign active_segment_count_o      = active_segment_count;
    assign active_segment_index_o      = segment_index;
    assign active_segment_level_q16_o  = current_level;

    // Explicit one-segment DC bypass.  This is both useful experimentally and
    // a runtime diagnostic of the complete FPGA->DAC physical output path.
    assign single_segment_low =
        (active_segment_count == 6'd1) &&
        (segment_level_q16_shadow_i[0] == 16'h0000);

    assign single_segment_high =
        (active_segment_count == 6'd1) &&
        (segment_level_q16_shadow_i[0] == 16'hFFFF);

    always_comb begin
        if (!enable_i || !config_valid_reg) begin
            dac_out_o = 14'sd0;
        end
        else if (single_segment_low) begin
            dac_out_o = clamp_dac14(active_low_counts);
        end
        else if (single_segment_high) begin
            dac_out_o = clamp_dac14(effective_high_reg);
        end
        else if (valid_s3) begin
            dac_out_o = clamp_dac14(sample_s3);
        end
        else begin
            dac_out_o = 14'sd0;
        end
    end

    // For ordinary sequence mode, boundary is delayed exactly like the sample.
    // For the DC bypass this delay is immaterial but preserves one definition.
    assign period_boundary_o =
        enable_i && config_valid_reg && valid_s3 && boundary_s3;

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            active_period_cycles <= 32'd25000;
            active_segment_count <= 6'd0;
            active_low_counts    <= 16'sd0;
            active_high_counts   <= 16'sd0;
            effective_high_reg            <= 16'sd0;
            correction_candidate_reg        <= 16'sd0;
            effective_high_candidate_reg    <= 16'sd0;
            config_valid_reg                <= 1'b0;

            correction_active_o             <= 16'sd0;

            segment_index        <= 5'd0;
            segment_cycle_count  <= 32'd0;
            current_duration     <= 32'd1;
            current_level        <= 16'd0;
            enable_d             <= 1'b0;
            boundary_pending     <= 1'b0;

            span_s1              <= 17'sd0;
            low_s1               <= 16'sd0;
            high_s1              <= 16'sd0;
            level_s1             <= 16'd0;
            valid_s1             <= 1'b0;
            boundary_s1          <= 1'b0;

            product_s2           <= 34'sd0;
            low_shift_s2         <= 34'sd0;
            low_s2               <= 16'sd0;
            high_s2              <= 16'sd0;
            level_s2             <= 16'd0;
            valid_s2             <= 1'b0;
            boundary_s2          <= 1'b0;

            sample_s3            <= 16'sd0;
            valid_s3             <= 1'b0;
            boundary_s3          <= 1'b0;
        end
        else begin
            enable_d <= enable_i;

            // Precompute the next correction in its own register stage.
            // Period boundaries only copy already-registered candidate values.
            correction_candidate_reg <= correction_pending_i;
            effective_high_candidate_reg <= clamp_high(
                active_high_counts,
                correction_pending_i,
                high_min_counts_i,
                high_max_counts_i
            );

            // Configuration is accepted only while OUT2 is disabled.  Python
            // already follows disable -> write shadows -> APPLY -> enable.
            if (!enable_i && apply_i) begin
                active_period_cycles <= period_cycles_shadow_i;
                active_segment_count <= segment_count_shadow_i;
                active_low_counts    <= low_counts_shadow_i;
                active_high_counts   <= high_counts_shadow_i;

                // Seed nominal HIGH while disabled.  The candidate stage
                // recomputes the corrected HIGH before the next enable edge.
                correction_active_o <= 16'sd0;
                effective_high_reg   <= high_counts_shadow_i;

                config_valid_reg <=
                    (period_cycles_shadow_i >= 32'd2) &&
                    (segment_count_shadow_i >= 6'd1) &&
                    (segment_count_shadow_i <= MAX_SEGMENTS) &&
                    ($signed(low_counts_shadow_i)  >= -16'sd8192) &&
                    ($signed(low_counts_shadow_i)  <=  16'sd8191) &&
                    ($signed(high_counts_shadow_i) >= -16'sd8192) &&
                    ($signed(high_counts_shadow_i) <=  16'sd8191) &&
                    ($signed(high_min_counts_i) < $signed(high_max_counts_i));

                segment_index       <= 5'd0;
                segment_cycle_count <= 32'd0;
                current_duration    <= segment_duration_cycles_shadow_i[0];
                current_level       <= segment_level_q16_shadow_i[0];
                boundary_pending    <= 1'b0;

                valid_s1    <= 1'b0;
                valid_s2    <= 1'b0;
                valid_s3    <= 1'b0;
                boundary_s1 <= 1'b0;
                boundary_s2 <= 1'b0;
                boundary_s3 <= 1'b0;
                sample_s3   <= 16'sd0;
            end
            else if (!enable_i) begin
                // Keep the sequence parked at its first segment.  Pipeline valid
                // is cleared so re-enable cannot emit stale samples.
                segment_index       <= 5'd0;
                segment_cycle_count <= 32'd0;
                current_duration    <= segment_duration_cycles_shadow_i[0];
                current_level       <= segment_level_q16_shadow_i[0];
                boundary_pending    <= 1'b0;

                valid_s1    <= 1'b0;
                valid_s2    <= 1'b0;
                valid_s3    <= 1'b0;
                boundary_s1 <= 1'b0;
                boundary_s2 <= 1'b0;
                boundary_s3 <= 1'b0;
                sample_s3   <= 16'sd0;
            end
            else if (!enable_d) begin
                // Clean start after an enable edge.
                segment_index       <= 5'd0;
                segment_cycle_count <= 32'd0;
                current_duration    <= segment_duration_cycles_shadow_i[0];
                current_level       <= segment_level_q16_shadow_i[0];
                boundary_pending    <= 1'b0;

                correction_active_o <= correction_candidate_reg;
                effective_high_reg  <= effective_high_candidate_reg;

                valid_s1    <= 1'b0;
                valid_s2    <= 1'b0;
                valid_s3    <= 1'b0;
                boundary_s1 <= 1'b0;
                boundary_s2 <= 1'b0;
                boundary_s3 <= 1'b0;
                sample_s3   <= 16'sd0;
            end
            else if (config_valid_reg) begin
                // ---------------- arithmetic pipeline ----------------
                // Stage 3: exact endpoints avoid Q0.16 endpoint rounding.
                valid_s3    <= valid_s2;
                boundary_s3 <= boundary_s2;
                if (valid_s2) begin
                    if (level_s2 == 16'h0000)
                        sample_s3 <= low_s2;
                    else if (level_s2 == 16'hFFFF)
                        sample_s3 <= high_s2;
                    else
                        sample_s3 <=
                            $signed((low_shift_s2 + product_s2) >>> 16);
                end

                // Stage 2: one DSP multiply.
                valid_s2    <= valid_s1;
                boundary_s2 <= boundary_s1;
                low_s2      <= low_s1;
                high_s2     <= high_s1;
                level_s2    <= level_s1;
                product_s2  <=
                    $signed(span_s1) *
                    $signed({1'b0, level_s1});
                low_shift_s2 <=
                    ($signed({{18{low_s1[15]}}, low_s1}) <<< 16);

                // Stage 1: registered inputs to the DSP.
                valid_s1    <= 1'b1;
                boundary_s1 <= boundary_pending;
                low_s1      <= active_low_counts;
                high_s1     <= effective_high_reg;
                level_s1    <= current_level;
                span_s1     <=
                    $signed(effective_high_reg) -
                    $signed(active_low_counts);
                boundary_pending <= 1'b0;

                // ---------------- logical segment engine ----------------
                if (segment_cycle_count + 1'b1 >= current_duration) begin
                    segment_cycle_count <= 32'd0;

                    if (({1'b0, segment_index} + 6'd1) >= active_segment_count) begin
                        // The NEXT logical sample is the first sample of a new
                        // period.  Apply the latest PID correction there.
                        segment_index    <= 5'd0;
                        current_duration <= segment_duration_cycles_shadow_i[0];
                        current_level    <= segment_level_q16_shadow_i[0];
                        boundary_pending <= 1'b1;

                        correction_active_o <= correction_candidate_reg;
                        effective_high_reg  <= effective_high_candidate_reg;
                    end
                    else begin
                        segment_index <= segment_index + 5'd1;
                        current_duration <=
                            segment_duration_cycles_shadow_i[segment_index + 5'd1];
                        current_level <=
                            segment_level_q16_shadow_i[segment_index + 5'd1];
                    end
                end
                else begin
                    segment_cycle_count <= segment_cycle_count + 1'b1;
                end
            end
            else begin
                valid_s1    <= 1'b0;
                valid_s2    <= 1'b0;
                valid_s3    <= 1'b0;
                boundary_s1 <= 1'b0;
                boundary_s2 <= 1'b0;
                boundary_s3 <= 1'b0;
                sample_s3   <= 16'sd0;
            end
        end
    end

endmodule
