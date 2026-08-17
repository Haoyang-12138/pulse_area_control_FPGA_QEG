`timescale 1ns / 1ps

// Runtime-configurable pulse detector / area measurement for IN1.
// All arithmetic is deliberately split across several 125 MHz cycles.
// Area units are ADC-count * sample; the 8 ns sample interval is applied in Python.
module pulse_measurement_v3 #(
    parameter int ADC_WIDTH = 14,
    parameter int MAX_BASELINE_SAMPLES = 256
)(
    input  logic                               clk_i,
    input  logic                               rstn_i,
    input  logic                               clear_state_i,
    input  logic                               measurement_enable_i,

    input  logic signed [ADC_WIDTH-1:0]        adc_i,

    // Dynamic threshold = local baseline +/- target_height * fraction.
    input  logic [15:0]                        threshold_fraction_q16_i,
    input  logic [15:0]                        target_height_i,

    // Runtime measurement controls.  Baseline lengths must be powers of two.
    input  logic [8:0]                         pre_baseline_samples_i,
    input  logic [8:0]                         post_baseline_samples_i,
    input  logic [31:0]                        min_pulse_samples_i,
    input  logic [31:0]                        max_pulse_samples_i,
    input  logic [15:0]                        adc_saturation_limit_i,
    input  logic                               pulse_polarity_negative_i,

    output logic                               config_valid_o,
    output logic                               busy_o,

    output logic signed [ADC_WIDTH-1:0]        bpre_o,
    output logic signed [ADC_WIDTH-1:0]        bpost_o,
    output logic signed [ADC_WIDTH-1:0]        threshold_o,
    output logic signed [ADC_WIDTH-1:0]        threshold_live_o,
    output logic signed [ADC_WIDTH-1:0]        peak_raw_o,
    output logic [15:0]                        peak_height_o,
    output logic [31:0]                        pulse_duration_samples_o,
    output logic signed [31:0]                 measured_pulse_area_o,

    output logic                               first_crossing_valid_o,
    output logic                               falling_edge_valid_o,
    output logic                               measurement_valid_o,
    output logic                               invalid_pulse_valid_o,
    output logic [7:0]                         invalid_reason_o
);

    localparam int PTR_W = $clog2(MAX_BASELINE_SAMPLES);
    localparam int BASE_ACC_W = ADC_WIDTH + $clog2(MAX_BASELINE_SAMPLES) + 3;
    localparam int AREA_ACC_W = 48;

    localparam logic [7:0] INVALID_NONE          = 8'h00;
    localparam logic [7:0] INVALID_TOO_SHORT     = 8'h01;
    localparam logic [7:0] INVALID_TOO_LONG      = 8'h02;
    localparam logic [7:0] INVALID_ADC_SAT       = 8'h03;
    localparam logic [7:0] INVALID_POST_OVERLAP  = 8'h04;
    localparam logic [7:0] INVALID_AREA_OVERFLOW = 8'h05;
    localparam logic [7:0] INVALID_CONFIG        = 8'h07;

    localparam logic signed [ADC_WIDTH-1:0] ADC_MAX =
        {1'b0, {(ADC_WIDTH-1){1'b1}}};
    localparam logic signed [ADC_WIDTH-1:0] ADC_MIN =
        {1'b1, {(ADC_WIDTH-1){1'b0}}};

    typedef enum logic [3:0] {
        ST_IDLE,
        ST_PULSE,
        ST_POST,
        ST_AREA_MUL,
        ST_AREA_SUB,
        ST_AREA_OUTPUT,
        ST_RECOVER
    } state_t;

    state_t state;

    // Do not reset this array.  Resetting every entry forces thousands of
    // flip-flops and hundreds of unique write-enable/control sets.  With an
    // asynchronous read and synchronous single-address write, Vivado can map
    // this 256 x 14 rolling buffer into a small distributed RAM instead.
    (* ram_style = "distributed" *)
    logic signed [ADC_WIDTH-1:0] baseline_buffer [0:MAX_BASELINE_SAMPLES-1];
    logic [PTR_W-1:0]            baseline_ptr;
    logic [8:0]                  baseline_fill_count;
    logic signed [BASE_ACC_W-1:0] baseline_sum;
    logic signed [ADC_WIDTH-1:0] baseline_mean;
    logic                         baseline_ready;

    logic [3:0] pre_shift;
    logic [3:0] post_shift;

    logic [31:0] threshold_product_reg;
    logic [15:0] threshold_offset;
    logic signed [ADC_WIDTH+2:0] threshold_wide;
    logic signed [ADC_WIDTH-1:0] threshold_current;
    logic signed [ADC_WIDTH-1:0] threshold_registered;
    logic                         threshold_registered_valid;

    logic [1:0] above_count;
    logic [1:0] below_count;
    logic [1:0] post_pulse_side_count;
    logic [1:0] recover_clear_count;

    logic signed [ADC_WIDTH-1:0] rise_sample_0;
    logic signed [ADC_WIDTH-1:0] rise_sample_1;
    logic signed [ADC_WIDTH-1:0] below_sample_0;
    logic signed [ADC_WIDTH-1:0] below_sample_1;

    logic signed [ADC_WIDTH-1:0] peak_raw_work;
    logic signed [AREA_ACC_W-1:0] pulse_raw_sum;
    logic signed [AREA_ACC_W-1:0] pulse_raw_sum_final;
    logic [31:0] pulse_sample_count;
    logic [31:0] pulse_sample_count_final;
    logic [31:0] pulse_elapsed_count;
    logic pulse_saturation_seen;

    logic signed [AREA_ACC_W-1:0] post_sum;
    logic [8:0] post_count;
    logic signed [ADC_WIDTH-1:0] bpost_calc;

    logic signed [ADC_WIDTH:0] baseline_pair_reg;
    logic signed [AREA_ACC_W-1:0] raw_sum_area_reg;
    logic [31:0] pulse_count_area_reg;
    logic signed [AREA_ACC_W-1:0] baseline_product_reg;
    logic signed [AREA_ACC_W-1:0] area_full_reg;

    logic signed [AREA_ACC_W-1:0] adc_area_ext;
    logic signed [AREA_ACC_W-1:0] rise0_area_ext;
    logic signed [AREA_ACC_W-1:0] rise1_area_ext;
    logic signed [AREA_ACC_W-1:0] below0_area_ext;
    logic signed [AREA_ACC_W-1:0] below1_area_ext;

    logic signed [BASE_ACC_W-1:0] adc_base_ext;
    logic signed [BASE_ACC_W-1:0] old_base_ext;
    logic signed [ADC_WIDTH-1:0] old_baseline_sample;
    logic signed [BASE_ACC_W-1:0] baseline_sum_next;

    logic sample_on_pulse_side_live;
    logic sample_on_pulse_side_latched;
    logic adc_saturated;
    logic area_overflow;

    // Detect runtime configuration changes and re-prime the rolling baseline.
    logic [8:0] cfg_pre_d;
    logic [8:0] cfg_post_d;
    logic [15:0] cfg_height_d;
    logic [15:0] cfg_fraction_d;
    logic cfg_polarity_d;
    logic config_changed;

    function automatic logic is_pow2_1_to_256(input logic [8:0] v);
        begin
            case (v)
                9'd1, 9'd2, 9'd4, 9'd8, 9'd16, 9'd32,
                9'd64, 9'd128, 9'd256: is_pow2_1_to_256 = 1'b1;
                default: is_pow2_1_to_256 = 1'b0;
            endcase
        end
    endfunction

    function automatic logic [3:0] pow2_shift(input logic [8:0] v);
        begin
            case (v)
                9'd1:   pow2_shift = 4'd0;
                9'd2:   pow2_shift = 4'd1;
                9'd4:   pow2_shift = 4'd2;
                9'd8:   pow2_shift = 4'd3;
                9'd16:  pow2_shift = 4'd4;
                9'd32:  pow2_shift = 4'd5;
                9'd64:  pow2_shift = 4'd6;
                9'd128: pow2_shift = 4'd7;
                9'd256: pow2_shift = 4'd8;
                default: pow2_shift = 4'd0;
            endcase
        end
    endfunction

    assign pre_shift  = pow2_shift(pre_baseline_samples_i);
    assign post_shift = pow2_shift(post_baseline_samples_i);

    assign config_valid_o =
        is_pow2_1_to_256(pre_baseline_samples_i) &&
        is_pow2_1_to_256(post_baseline_samples_i) &&
        (threshold_fraction_q16_i != 16'd0) &&
        (target_height_i != 16'd0) &&
        (min_pulse_samples_i >= 32'd3) &&
        (max_pulse_samples_i >= min_pulse_samples_i) &&
        (adc_saturation_limit_i > 16'd0) &&
        (adc_saturation_limit_i <= ((1 << (ADC_WIDTH-1)) - 1));

    assign config_changed =
        (cfg_pre_d      != pre_baseline_samples_i) ||
        (cfg_post_d     != post_baseline_samples_i) ||
        (cfg_height_d   != target_height_i) ||
        (cfg_fraction_d != threshold_fraction_q16_i) ||
        (cfg_polarity_d != pulse_polarity_negative_i);

    assign busy_o = (state != ST_IDLE);

    assign old_baseline_sample = baseline_buffer[baseline_ptr];
    assign adc_base_ext = {{(BASE_ACC_W-ADC_WIDTH){adc_i[ADC_WIDTH-1]}}, adc_i};
    assign old_base_ext = {{(BASE_ACC_W-ADC_WIDTH){old_baseline_sample[ADC_WIDTH-1]}}, old_baseline_sample};

    assign adc_area_ext = {{(AREA_ACC_W-ADC_WIDTH){adc_i[ADC_WIDTH-1]}}, adc_i};
    assign rise0_area_ext = {{(AREA_ACC_W-ADC_WIDTH){rise_sample_0[ADC_WIDTH-1]}}, rise_sample_0};
    assign rise1_area_ext = {{(AREA_ACC_W-ADC_WIDTH){rise_sample_1[ADC_WIDTH-1]}}, rise_sample_1};
    assign below0_area_ext = {{(AREA_ACC_W-ADC_WIDTH){below_sample_0[ADC_WIDTH-1]}}, below_sample_0};
    assign below1_area_ext = {{(AREA_ACC_W-ADC_WIDTH){below_sample_1[ADC_WIDTH-1]}}, below_sample_1};

    assign threshold_offset = threshold_product_reg[31:16];

    always_comb begin
        if (pulse_polarity_negative_i)
            threshold_wide = $signed(baseline_mean) - $signed({1'b0, threshold_offset});
        else
            threshold_wide = $signed(baseline_mean) + $signed({1'b0, threshold_offset});

        if (threshold_wide > $signed(ADC_MAX))
            threshold_current = ADC_MAX;
        else if (threshold_wide < $signed(ADC_MIN))
            threshold_current = ADC_MIN;
        else
            threshold_current = threshold_wide[ADC_WIDTH-1:0];
    end

    // Register the rolling threshold before it is used by the pulse detector.
    // This deliberately breaks the former baseline_mean -> threshold arithmetic
    // -> pulse-side compare -> distributed-RAM write-enable path.  The detector
    // therefore uses a threshold derived from the immediately preceding rolling
    // baseline update (8 ns old at 125 MHz), which is negligible compared with
    // the configured baseline window and preserves continuous full-rate sampling.
    assign threshold_live_o = threshold_registered;

    assign sample_on_pulse_side_live = threshold_registered_valid &&
        (pulse_polarity_negative_i
            ? ($signed(adc_i) < $signed(threshold_registered))
            : ($signed(adc_i) > $signed(threshold_registered)));

    assign sample_on_pulse_side_latched = pulse_polarity_negative_i
        ? ($signed(adc_i) < $signed(threshold_o))
        : ($signed(adc_i) > $signed(threshold_o));

    // Saturation is based on an adjustable magnitude below full scale.
    assign adc_saturated =
        ($signed(adc_i) >= $signed({1'b0, adc_saturation_limit_i[ADC_WIDTH-2:0]})) ||
        ($signed(adc_i) <= -$signed({1'b0, adc_saturation_limit_i[ADC_WIDTH-2:0]}));

    assign area_overflow =
        area_full_reg[AREA_ACC_W-1:32] != {(AREA_ACC_W-32){area_full_reg[31]}};

    always_comb begin
        // By default use the current rolling baseline sum update.
        if (!baseline_ready)
            baseline_sum_next = baseline_sum + adc_base_ext;
        else
            baseline_sum_next = baseline_sum - old_base_ext + adc_base_ext;

        bpost_calc = (post_sum + adc_area_ext) >>> post_shift;
    end

    integer k;
    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            state                       <= ST_IDLE;
            cfg_pre_d                   <= 9'd0;
            cfg_post_d                  <= 9'd0;
            cfg_height_d                <= 16'd0;
            cfg_fraction_d              <= 16'd0;
            cfg_polarity_d              <= 1'b0;

            threshold_product_reg       <= 32'd0;
            threshold_registered        <= '0;
            threshold_registered_valid  <= 1'b0;
            baseline_ptr                <= '0;
            baseline_fill_count         <= '0;
            baseline_sum                <= '0;
            baseline_mean               <= '0;
            baseline_ready              <= 1'b0;

            above_count                 <= '0;
            below_count                 <= '0;
            post_pulse_side_count       <= '0;
            recover_clear_count         <= '0;

            rise_sample_0               <= '0;
            rise_sample_1               <= '0;
            below_sample_0              <= '0;
            below_sample_1              <= '0;

            peak_raw_work               <= '0;
            pulse_raw_sum               <= '0;
            pulse_raw_sum_final         <= '0;
            pulse_sample_count          <= '0;
            pulse_sample_count_final    <= '0;
            pulse_elapsed_count         <= '0;
            pulse_saturation_seen       <= 1'b0;

            post_sum                    <= '0;
            post_count                  <= '0;

            baseline_pair_reg           <= '0;
            raw_sum_area_reg            <= '0;
            pulse_count_area_reg        <= '0;
            baseline_product_reg        <= '0;
            area_full_reg               <= '0;

            bpre_o                      <= '0;
            bpost_o                     <= '0;
            threshold_o                 <= '0;
            peak_raw_o                  <= '0;
            peak_height_o               <= '0;
            pulse_duration_samples_o    <= '0;
            measured_pulse_area_o       <= '0;

            first_crossing_valid_o      <= 1'b0;
            falling_edge_valid_o        <= 1'b0;
            measurement_valid_o         <= 1'b0;
            invalid_pulse_valid_o       <= 1'b0;
            invalid_reason_o            <= INVALID_NONE;

        end
        else begin
            first_crossing_valid_o <= 1'b0;
            falling_edge_valid_o   <= 1'b0;
            measurement_valid_o    <= 1'b0;
            invalid_pulse_valid_o  <= 1'b0;

            // Config arithmetic is deliberately registered; it is not part of
            // the critical measurement/feedback combinational path.
            threshold_product_reg <= target_height_i * threshold_fraction_q16_i;

            // Pipeline the dynamic threshold.  Only assert valid after a rolling
            // baseline has been primed.  During a pulse the rolling baseline is
            // frozen, so the registered threshold naturally remains stable enough
            // for first-crossing detection; threshold_o is still latched at the
            // first crossing and used for the falling edge exactly as before.
            threshold_registered <= threshold_current;
            threshold_registered_valid <= baseline_ready;

            cfg_pre_d      <= pre_baseline_samples_i;
            cfg_post_d     <= post_baseline_samples_i;
            cfg_height_d   <= target_height_i;
            cfg_fraction_d <= threshold_fraction_q16_i;
            cfg_polarity_d <= pulse_polarity_negative_i;

            if (clear_state_i || config_changed || !measurement_enable_i) begin
                state                    <= ST_IDLE;
                baseline_ptr             <= '0;
                baseline_fill_count      <= '0;
                baseline_sum             <= '0;
                baseline_mean            <= '0;
                baseline_ready           <= 1'b0;
                threshold_registered_valid <= 1'b0;
                above_count              <= '0;
                below_count              <= '0;
                post_pulse_side_count    <= '0;
                recover_clear_count      <= '0;
                pulse_raw_sum            <= '0;
                pulse_sample_count       <= '0;
                pulse_elapsed_count      <= '0;
                pulse_saturation_seen    <= 1'b0;
                invalid_reason_o         <= INVALID_NONE;
            end
            else if (!config_valid_o) begin
                state <= ST_IDLE;
                invalid_reason_o <= INVALID_CONFIG;
            end
            else begin
                case (state)
                    ST_IDLE: begin
                        below_count <= '0;
                        post_pulse_side_count <= '0;
                        recover_clear_count <= '0;
                        pulse_saturation_seen <= 1'b0;

                        if (!baseline_ready) begin
                            baseline_buffer[baseline_ptr] <= adc_i;
                            baseline_sum <= baseline_sum + adc_base_ext;

                            if (baseline_fill_count + 1'b1 >= pre_baseline_samples_i) begin
                                baseline_fill_count <= pre_baseline_samples_i;
                                baseline_ready <= 1'b1;
                                baseline_mean <= (baseline_sum + adc_base_ext) >>> pre_shift;
                                baseline_ptr <= '0;
                            end
                            else begin
                                baseline_fill_count <= baseline_fill_count + 1'b1;
                                baseline_ptr <= baseline_ptr + 1'b1;
                            end
                        end
                        else if (!sample_on_pulse_side_live) begin
                            above_count <= '0;
                            baseline_buffer[baseline_ptr] <= adc_i;
                            baseline_sum <= baseline_sum_next;
                            baseline_mean <= baseline_sum_next >>> pre_shift;

                            if (baseline_ptr == pre_baseline_samples_i - 1'b1)
                                baseline_ptr <= '0;
                            else
                                baseline_ptr <= baseline_ptr + 1'b1;
                        end
                        else begin
                            // Fixed 3-sample edge confirmation preserves the
                            // behaviour of the previously timing-clean design.
                            if (above_count == 2'd0) begin
                                first_crossing_valid_o <= 1'b1;
                                rise_sample_0 <= adc_i;
                                peak_raw_work <= adc_i;
                                bpre_o <= baseline_mean;
                                threshold_o <= threshold_registered;
                                above_count <= 2'd1;
                                pulse_saturation_seen <= adc_saturated;
                            end
                            else if (above_count == 2'd1) begin
                                rise_sample_1 <= adc_i;
                                if ((!pulse_polarity_negative_i && $signed(adc_i) > $signed(peak_raw_work)) ||
                                    ( pulse_polarity_negative_i && $signed(adc_i) < $signed(peak_raw_work)))
                                    peak_raw_work <= adc_i;
                                above_count <= 2'd2;
                                pulse_saturation_seen <= pulse_saturation_seen || adc_saturated;
                            end
                            else begin
                                pulse_raw_sum <= rise0_area_ext + rise1_area_ext + adc_area_ext;
                                pulse_sample_count <= 32'd3;
                                pulse_elapsed_count <= 32'd3;
                                if ((!pulse_polarity_negative_i && $signed(adc_i) > $signed(peak_raw_work)) ||
                                    ( pulse_polarity_negative_i && $signed(adc_i) < $signed(peak_raw_work)))
                                    peak_raw_work <= adc_i;
                                pulse_saturation_seen <= pulse_saturation_seen || adc_saturated;
                                above_count <= '0;
                                state <= ST_PULSE;
                            end
                        end
                    end

                    ST_PULSE: begin
                        pulse_saturation_seen <= pulse_saturation_seen || adc_saturated;

                        if (!sample_on_pulse_side_latched && below_count == 2'd2) begin
                            pulse_raw_sum_final <= pulse_raw_sum - below0_area_ext - below1_area_ext;
                            pulse_sample_count_final <= pulse_sample_count - 32'd2;
                            pulse_duration_samples_o <= pulse_sample_count - 32'd2;
                            falling_edge_valid_o <= 1'b1;
                            below_count <= '0;

                            peak_raw_o <= peak_raw_work;
                            if (pulse_polarity_negative_i) begin
                                if ($signed(bpre_o) > $signed(peak_raw_work))
                                    peak_height_o <= $unsigned($signed(bpre_o) - $signed(peak_raw_work));
                                else
                                    peak_height_o <= 16'd0;
                            end
                            else begin
                                if ($signed(peak_raw_work) > $signed(bpre_o))
                                    peak_height_o <= $unsigned($signed(peak_raw_work) - $signed(bpre_o));
                                else
                                    peak_height_o <= 16'd0;
                            end

                            if (pulse_saturation_seen || adc_saturated) begin
                                invalid_pulse_valid_o <= 1'b1;
                                invalid_reason_o <= INVALID_ADC_SAT;
                                state <= ST_RECOVER;
                            end
                            else if ((pulse_sample_count - 32'd2) < min_pulse_samples_i) begin
                                invalid_pulse_valid_o <= 1'b1;
                                invalid_reason_o <= INVALID_TOO_SHORT;
                                state <= ST_RECOVER;
                            end
                            else begin
                                post_sum <= '0;
                                post_count <= '0;
                                post_pulse_side_count <= '0;
                                state <= ST_POST;
                            end
                        end
                        else if (pulse_elapsed_count >= max_pulse_samples_i) begin
                            pulse_duration_samples_o <= pulse_elapsed_count;
                            peak_raw_o <= peak_raw_work;
                            invalid_pulse_valid_o <= 1'b1;
                            invalid_reason_o <= INVALID_TOO_LONG;
                            state <= ST_RECOVER;
                        end
                        else begin
                            pulse_elapsed_count <= pulse_elapsed_count + 1'b1;
                            pulse_raw_sum <= pulse_raw_sum + adc_area_ext;
                            pulse_sample_count <= pulse_sample_count + 1'b1;

                            if ((!pulse_polarity_negative_i && $signed(adc_i) > $signed(peak_raw_work)) ||
                                ( pulse_polarity_negative_i && $signed(adc_i) < $signed(peak_raw_work)))
                                peak_raw_work <= adc_i;

                            if (sample_on_pulse_side_latched) begin
                                below_count <= '0;
                            end
                            else begin
                                case (below_count)
                                    2'd0: begin
                                        below_sample_0 <= adc_i;
                                        below_count <= 2'd1;
                                    end
                                    2'd1: begin
                                        below_sample_1 <= adc_i;
                                        below_count <= 2'd2;
                                    end
                                    default: below_count <= below_count;
                                endcase
                            end
                        end
                    end

                    ST_POST: begin
                        if (adc_saturated) begin
                            invalid_pulse_valid_o <= 1'b1;
                            invalid_reason_o <= INVALID_ADC_SAT;
                            state <= ST_RECOVER;
                        end
                        else if (sample_on_pulse_side_latched && post_pulse_side_count == 2'd2) begin
                            invalid_pulse_valid_o <= 1'b1;
                            invalid_reason_o <= INVALID_POST_OVERLAP;
                            state <= ST_RECOVER;
                        end
                        else begin
                            post_sum <= post_sum + adc_area_ext;

                            // Refresh the rolling baseline with post-pulse data.
                            baseline_buffer[baseline_ptr] <= adc_i;
                            baseline_sum <= baseline_sum_next;
                            baseline_mean <= baseline_sum_next >>> pre_shift;
                            if (baseline_ptr == pre_baseline_samples_i - 1'b1)
                                baseline_ptr <= '0;
                            else
                                baseline_ptr <= baseline_ptr + 1'b1;

                            if (sample_on_pulse_side_latched)
                                post_pulse_side_count <= post_pulse_side_count + 1'b1;
                            else
                                post_pulse_side_count <= '0;

                            if (post_count + 1'b1 >= post_baseline_samples_i) begin
                                bpost_o <= (post_sum + adc_area_ext) >>> post_shift;
                                baseline_pair_reg <=
                                    $signed({bpre_o[ADC_WIDTH-1], bpre_o}) +
                                    $signed({bpost_calc[ADC_WIDTH-1], bpost_calc});
                                raw_sum_area_reg <= pulse_raw_sum_final;
                                pulse_count_area_reg <= pulse_sample_count_final;
                                post_count <= '0;
                                state <= ST_AREA_MUL;
                            end
                            else begin
                                post_count <= post_count + 1'b1;
                            end
                        end
                    end

                    ST_AREA_MUL: begin
                        // Dedicated cycle for the wide multiply.  This is one of
                        // the key timing-closure lessons from the previous build.
                        baseline_product_reg <= $signed(baseline_pair_reg) * $signed({1'b0, pulse_count_area_reg});
                        state <= ST_AREA_SUB;
                    end

                    ST_AREA_SUB: begin
                        area_full_reg <= raw_sum_area_reg - ($signed(baseline_product_reg) >>> 1);
                        state <= ST_AREA_OUTPUT;
                    end

                    ST_AREA_OUTPUT: begin
                        if (area_overflow) begin
                            invalid_pulse_valid_o <= 1'b1;
                            invalid_reason_o <= INVALID_AREA_OVERFLOW;
                        end
                        else begin
                            // Report positive pulse area for either polarity.
                            measured_pulse_area_o <= pulse_polarity_negative_i
                                ? -$signed(area_full_reg[31:0])
                                :  $signed(area_full_reg[31:0]);
                            measurement_valid_o <= 1'b1;
                            invalid_reason_o <= INVALID_NONE;
                        end
                        state <= ST_IDLE;
                    end

                    ST_RECOVER: begin
                        above_count <= '0;
                        below_count <= '0;
                        if (sample_on_pulse_side_latched) begin
                            recover_clear_count <= '0;
                        end
                        else if (recover_clear_count == 2'd2) begin
                            recover_clear_count <= '0;
                            state <= ST_IDLE;
                        end
                        else begin
                            recover_clear_count <= recover_clear_count + 1'b1;
                        end
                    end

                    default: state <= ST_IDLE;
                endcase
            end
        end
    end

endmodule
