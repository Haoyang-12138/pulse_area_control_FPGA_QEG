`timescale 1ns / 1ps

// Group-mean pulse-area PID controller.
//
// Important timing rule: no attempt is made to perform the full PID + plant-gain
// conversion in one 125 MHz cycle.  Kp/Ki/Kd multiplication, integral/derivative update,
// area-to-DAC multiplication, anti-windup selection, and final clamps are
// explicitly pipelined across states.  This preserves the timing-closure lesson
// from the previously working design while keeping controller latency small.
module pulse_area_pid_controller_v3 (
    input  logic                 clk_i,
    input  logic                 rstn_i,
    input  logic                 clear_state_i,

    input  logic                 feedback_enable_i,
    input  logic                 hold_integrator_i,
    input  logic                 use_first_valid_group_as_target_i,

    input  logic signed [31:0]   target_area_i,
    input  logic [31:0]          deadband_absolute_i,
    input  logic [15:0]          deadband_fraction_q16_i,
    input  logic                 deadband_fraction_mode_i,

    // Kp and Kd retain the verified Q16.16 representation.
    // Ki is Q1.31 PER GROUP UPDATE so slow integral gains remain representable
    // at a 1 MHz group rate.  Its fractional contribution is accumulated before
    // conversion back to integer area units.
    // ki_q31_per_update_i = KI_PER_S * dt * 2^31.
    // kd_q16_per_update_i = KD / dt, so D = kd_per_update * (e[n]-e[n-1]).
    input  logic [31:0]          kp_q16_i,
    input  logic [31:0]          ki_q31_per_update_i,
    input  logic [31:0]          kd_q16_per_update_i,

    // Q8.24: DAC counts per (ADC-count * sample).
    input  logic [31:0]          area_to_dac_gain_q24_i,
    input  logic                 correction_sign_negative_i,

    input  logic [15:0]          max_dac_step_counts_i,
    input  logic signed [15:0]   dac_high_min_counts_i,
    input  logic signed [15:0]   dac_high_max_counts_i,
    input  logic signed [31:0]   integral_min_area_i,
    input  logic signed [31:0]   integral_max_area_i,
    input  logic                 integral_fraction_mode_i,
    input  logic [15:0]          integral_limit_fraction_q16_i,

    input  logic signed [15:0]   nominal_dac_high_counts_i,
    input  logic signed [15:0]   correction_active_i,

    input  logic                 group_ready_valid_i,
    input  logic [63:0]          group_timestamp_i,
    input  logic [31:0]          group_id_i,
    input  logic [7:0]           detected_pulses_i,
    input  logic [7:0]           valid_pulses_i,
    input  logic [7:0]           invalid_pulses_i,
    input  logic                 expected_mismatch_i,
    input  logic                 group_overflow_i,
    input  logic                 boundary_overrun_i,
    input  logic                 group_valid_for_feedback_i,
    input  logic signed [31:0]   mean_area_i,

    output logic                 config_valid_o,
    output logic                 target_latched_o,
    output logic signed [31:0]   target_area_active_o,

    output logic signed [31:0]   area_error_o,
    output logic signed [31:0]   p_term_area_o,
    output logic signed [31:0]   i_term_area_o,
    output logic signed [31:0]   d_term_area_o,

    output logic signed [15:0]   correction_pending_o,
    output logic                 feedback_update_valid_o,
    output logic [31:0]          update_count_o,

    output logic                 correction_high_sat_o,
    output logic                 correction_low_sat_o,
    output logic                 integral_high_sat_o,
    output logic                 integral_low_sat_o,
    output logic [31:0]          last_processing_latency_cycles_o,

    // A result pulse is emitted for every group, even if feedback is disabled,
    // the group is invalid, or the first valid group is only used as target.
    output logic                 result_valid_o,
    output logic [63:0]          result_timestamp_o,
    output logic [31:0]          result_group_id_o,
    output logic [7:0]           result_detected_pulses_o,
    output logic [7:0]           result_valid_pulses_o,
    output logic [7:0]           result_invalid_pulses_o,
    output logic                 result_expected_mismatch_o,
    output logic                 result_group_overflow_o,
    output logic                 result_boundary_overrun_o,
    output logic                 result_group_valid_for_feedback_o,
    output logic signed [31:0]   result_mean_area_o,
    output logic                 result_deadband_o,
    output logic                 result_feedback_applied_o
);

    typedef enum logic [4:0] {
        PI_IDLE,
        PI_DEADBAND,
        PI_DEADBAND_DECIDE,
        PI_MUL_GAINS,
        PI_INTEGRAL,
        PI_I_RANGE,
        PI_I_CLAMP,
        PI_I_ROUND,
        PI_SUM,
        PI_GAIN_MUL,
        PI_SELECT,
        PI_SIGN,
        PI_SAT,
        PI_AW_CHECK,
        PI_BOUND,
        PI_STEP,
        PI_OUTPUT
    } pi_state_t;

    pi_state_t state;

    // Saturate a signed wide correction to signed 18 bits.  18 bits are wider
    // than every legal correction bound (signed 17-bit), so saturation here
    // cannot hide a physical-bound violation.
    function automatic logic signed [17:0] sat_s18(input logic signed [41:0] x);
        begin
            if (!x[41] && (|x[41:17]))
                sat_s18 = 18'sh1FFFF;       // +131071
            else if (x[41] && !(&x[41:17]))
                sat_s18 = 18'sh20000;       // -131072
            else
                sat_s18 = x[17:0];
        end
    endfunction

    // Symmetric round-to-nearest from signed Q31 area to signed integer area.
    // Positive half-way values round away from zero; negative half-way values
    // also round away from zero.  This avoids the negative floor bias of a bare
    // arithmetic >>> 31 while requiring only a 32-bit increment, not a 64-bit
    // absolute-value/add path.
    function automatic logic signed [31:0] round_q31_to_s32(
        input logic signed [63:0] x
    );
        logic signed [32:0] floor_value;
        logic [30:0] fraction;
        logic increment;
        begin
            floor_value = $signed(x[63:31]);
            fraction = x[30:0];
            if (!x[63])
                increment = (fraction >= 31'h40000000);
            else
                increment = (fraction > 31'h40000000);
            round_q31_to_s32 = floor_value[31:0] +
                               (increment ? 32'sd1 : 32'sd0);
        end
    endfunction

    logic signed [31:0] target_active;
    logic target_latched;

    logic signed [31:0] error_work;
    logic [31:0] error_abs;
    // Registered absolute target.  This deliberately breaks the old
    // target_active -> 32-bit negate/carry-chain -> DSP path that violated
    // 125 MHz timing.  The target is slow configuration state, so caching its
    // magnitude does not add latency to the normal per-group PID pipeline.
    logic [31:0] target_abs_cached;
    logic [47:0] deadband_product;
    logic [47:0] integral_limit_product;
    logic [31:0] deadband_value;
    // Registered effective I limits.  v3.7 still timed the DSP output of
    // integral_limit_product through a 32/64-bit saturation comparator into
    // integral_candidate in one 8 ns cycle.  These registers break that path.
    logic signed [31:0] integral_min_bound_reg;
    logic signed [31:0] integral_max_bound_reg;
    logic deadband_work;
    logic target_capture_work;

    logic signed [63:0] p_product;
    logic signed [63:0] i_product_q31;
    logic signed [63:0] d_product;
    logic signed [31:0] p_area_work;
    logic signed [31:0] d_area_work;
    logic signed [31:0] previous_error;
    logic signed [31:0] derivative_delta;
    logic derivative_initialized;

    // Integral state is kept in Q31 area units.  This is the central numerical
    // fix: fractional error*Ki contributions are accumulated across groups and
    // are never arithmetic-shifted away before accumulation.
    logic signed [63:0] integral_accumulator_fp;
    logic signed [31:0] integral_accumulator_area;
    logic signed [64:0] integral_sum_wide_fp;
    logic signed [63:0] integral_sum_fp;
    logic integral_sum_overflow_high;
    logic integral_sum_overflow_low;
    logic signed [63:0] integral_candidate_fp;
    logic signed [31:0] integral_candidate_area;
    logic signed [31:0] integral_current_area;
    logic integral_candidate_high_sat;
    logic integral_candidate_low_sat;

    logic signed [32:0] candidate_control_area;
    logic signed [32:0] current_control_area;
    logic signed [65:0] candidate_gain_product;
    logic signed [65:0] current_gain_product;
    // The old controller applied correction sign after the Q24 right shift.
    // Keep that exact arithmetic order, but pipeline shift -> sign -> saturation
    // so no wide negate/compare chain is required in one 125 MHz cycle.
    logic signed [41:0] candidate_gain_shifted;
    logic signed [41:0] current_gain_shifted;
    logic signed [41:0] candidate_signed_shifted;
    logic signed [41:0] current_signed_shifted;
    // Corrections are ultimately bounded by differences of signed 16-bit DAC
    // levels, i.e. a signed 17-bit physical range.  Keep one guard bit and
    // saturate the wide plant-gain result to signed 18 bits before anti-windup
    // comparisons.  This removes the old 64-bit compare/carry chain.
    logic signed [17:0] candidate_correction_raw;
    logic signed [17:0] current_correction_raw;

    logic freeze_integrator;
    logic signed [17:0] selected_correction_raw;
    logic signed [63:0] selected_integral_fp;
    logic signed [31:0] selected_integral_area;

    logic signed [16:0] correction_min_bound;
    logic signed [16:0] correction_max_bound;
    logic signed [16:0] correction_clamped_bound;
    logic signed [16:0] correction_step_low;
    logic signed [16:0] correction_step_high;
    logic signed [16:0] correction_after_step;
    logic correction_high_sat_work;
    logic correction_low_sat_work;

    logic latency_busy;
    logic [31:0] latency_counter;

    // Latched group metadata while the arithmetic pipeline runs.
    logic [63:0] meta_timestamp;
    logic [31:0] meta_group_id;
    logic [7:0] meta_detected;
    logic [7:0] meta_valid;
    logic [7:0] meta_invalid;
    logic meta_expected_mismatch;
    logic meta_group_overflow;
    logic meta_boundary_overrun;
    logic meta_group_valid;
    logic signed [31:0] meta_mean;

    assign target_area_active_o = target_active;
    assign target_latched_o = target_latched;

    assign config_valid_o =
        (kp_q16_i[31] == 1'b0) &&
        (ki_q31_per_update_i[31] == 1'b0) &&
        (kd_q16_per_update_i[31] == 1'b0) &&
        (area_to_dac_gain_q24_i != 32'd0) &&
        ($signed(dac_high_min_counts_i) < $signed(dac_high_max_counts_i)) &&
        (integral_fraction_mode_i
            ? (integral_limit_fraction_q16_i != 16'd0)
            : ($signed(integral_min_area_i) < $signed(integral_max_area_i))) &&
        (max_dac_step_counts_i != 16'd0) &&
        (use_first_valid_group_as_target_i || ($signed(target_area_i) > 0));

    assign error_abs  = error_work[31] ? $unsigned(-error_work) : $unsigned(error_work);
    assign deadband_value = deadband_fraction_mode_i
        ? (deadband_product >> 16)
        : deadband_absolute_i;

    always_comb begin
        correction_min_bound = $signed(dac_high_min_counts_i) - $signed(nominal_dac_high_counts_i);
        correction_max_bound = $signed(dac_high_max_counts_i) - $signed(nominal_dac_high_counts_i);

        correction_step_low  = $signed(correction_active_i) - $signed({1'b0, max_dac_step_counts_i});
        correction_step_high = $signed(correction_active_i) + $signed({1'b0, max_dac_step_counts_i});
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            state                               <= PI_IDLE;
            target_active                       <= 32'sd0;
            target_latched                      <= 1'b0;
            target_abs_cached                   <= 32'd0;
            error_work                          <= '0;
            deadband_product                    <= '0;
            integral_limit_product               <= '0;
            integral_min_bound_reg               <= '0;
            integral_max_bound_reg               <= '0;
            deadband_work                       <= 1'b0;
            target_capture_work                  <= 1'b0;
            p_product                           <= '0;
            i_product_q31                       <= '0;
            d_product                           <= '0;
            p_area_work                         <= '0;
            d_area_work                         <= '0;
            previous_error                      <= '0;
            derivative_delta                    <= '0;
            derivative_initialized              <= 1'b0;
            integral_accumulator_fp             <= '0;
            integral_accumulator_area           <= '0;
            integral_sum_wide_fp                <= '0;
            integral_sum_fp                     <= '0;
            integral_sum_overflow_high          <= 1'b0;
            integral_sum_overflow_low           <= 1'b0;
            integral_candidate_fp               <= '0;
            integral_candidate_area             <= '0;
            integral_current_area               <= '0;
            integral_candidate_high_sat         <= 1'b0;
            integral_candidate_low_sat          <= 1'b0;
            candidate_control_area              <= '0;
            current_control_area                <= '0;
            candidate_gain_product              <= '0;
            current_gain_product                <= '0;
            candidate_gain_shifted              <= '0;
            current_gain_shifted                <= '0;
            candidate_signed_shifted            <= '0;
            current_signed_shifted              <= '0;
            candidate_correction_raw            <= '0;
            current_correction_raw              <= '0;
            freeze_integrator                   <= 1'b0;
            selected_correction_raw             <= '0;
            selected_integral_fp                <= '0;
            selected_integral_area              <= '0;
            correction_clamped_bound            <= '0;
            correction_after_step               <= '0;
            correction_high_sat_work             <= 1'b0;
            correction_low_sat_work              <= 1'b0;

            area_error_o                        <= '0;
            p_term_area_o                       <= '0;
            i_term_area_o                       <= '0;
            d_term_area_o                       <= '0;
            correction_pending_o                <= '0;
            feedback_update_valid_o             <= 1'b0;
            update_count_o                      <= '0;
            correction_high_sat_o               <= 1'b0;
            correction_low_sat_o                <= 1'b0;
            integral_high_sat_o                 <= 1'b0;
            integral_low_sat_o                  <= 1'b0;
            last_processing_latency_cycles_o    <= '0;
            latency_busy                        <= 1'b0;
            latency_counter                     <= '0;

            result_valid_o                      <= 1'b0;
            result_timestamp_o                  <= '0;
            result_group_id_o                   <= '0;
            result_detected_pulses_o            <= '0;
            result_valid_pulses_o               <= '0;
            result_invalid_pulses_o             <= '0;
            result_expected_mismatch_o          <= 1'b0;
            result_group_overflow_o             <= 1'b0;
            result_boundary_overrun_o           <= 1'b0;
            result_group_valid_for_feedback_o   <= 1'b0;
            result_mean_area_o                  <= '0;
            result_deadband_o                   <= 1'b0;
            result_feedback_applied_o           <= 1'b0;

            meta_timestamp                      <= '0;
            meta_group_id                       <= '0;
            meta_detected                       <= '0;
            meta_valid                          <= '0;
            meta_invalid                        <= '0;
            meta_expected_mismatch              <= 1'b0;
            meta_group_overflow                 <= 1'b0;
            meta_boundary_overrun               <= 1'b0;
            meta_group_valid                    <= 1'b0;
            meta_mean                           <= '0;
        end
        else begin
            result_valid_o          <= 1'b0;
            feedback_update_valid_o <= 1'b0;
            correction_high_sat_o   <= 1'b0;
            correction_low_sat_o    <= 1'b0;
            integral_high_sat_o     <= 1'b0;
            integral_low_sat_o      <= 1'b0;

            if (!use_first_valid_group_as_target_i) begin
                target_active     <= target_area_i;
                target_abs_cached <= target_area_i[31]
                    ? $unsigned(-$signed(target_area_i))
                    : $unsigned(target_area_i);
                target_latched    <= 1'b1;
            end

            if (latency_busy)
                latency_counter <= latency_counter + 1'b1;

            if (clear_state_i) begin
                state                    <= PI_IDLE;
                integral_accumulator_fp  <= '0;
                integral_accumulator_area<= '0;
                previous_error           <= '0;
                derivative_delta         <= '0;
                derivative_initialized   <= 1'b0;
                p_term_area_o            <= '0;
                i_term_area_o            <= '0;
                d_term_area_o            <= '0;
                area_error_o             <= '0;
                correction_pending_o     <= '0;
                update_count_o           <= '0;
                latency_busy             <= 1'b0;
                latency_counter          <= '0;
                last_processing_latency_cycles_o <= '0;
                if (use_first_valid_group_as_target_i) begin
                    target_active     <= 32'sd0;
                    target_abs_cached <= 32'd0;
                    target_latched    <= 1'b0;
                end
            end
            else begin
                if (!feedback_enable_i && state == PI_IDLE) begin
                    correction_pending_o <= 16'sd0;
                    integral_accumulator_fp <= 64'sd0;
                    integral_accumulator_area <= 32'sd0;
                    previous_error <= 32'sd0;
                    derivative_delta <= 32'sd0;
                    derivative_initialized <= 1'b0;
                    i_term_area_o <= 32'sd0;
                    p_term_area_o <= 32'sd0;
                    d_term_area_o <= 32'sd0;
                end

                case (state)
                    PI_IDLE: begin
                        if (group_ready_valid_i) begin
                            meta_timestamp         <= group_timestamp_i;
                            meta_group_id          <= group_id_i;
                            meta_detected          <= detected_pulses_i;
                            meta_valid             <= valid_pulses_i;
                            meta_invalid           <= invalid_pulses_i;
                            meta_expected_mismatch <= expected_mismatch_i;
                            meta_group_overflow    <= group_overflow_i;
                            meta_boundary_overrun  <= boundary_overrun_i;
                            meta_group_valid       <= group_valid_for_feedback_i;
                            meta_mean              <= mean_area_i;

                            latency_counter <= 32'd0;
                            latency_busy    <= 1'b1;
                            target_capture_work <= 1'b0;

                            if (use_first_valid_group_as_target_i &&
                                !target_latched && group_valid_for_feedback_i &&
                                ($signed(mean_area_i) > 0)) begin
                                target_active     <= mean_area_i;
                                target_abs_cached <= mean_area_i[31]
                                    ? $unsigned(-$signed(mean_area_i))
                                    : $unsigned(mean_area_i);
                                target_latched    <= 1'b1;
                                error_work     <= 32'sd0;
                                area_error_o   <= 32'sd0;
                                p_term_area_o  <= 32'sd0;
                                i_term_area_o  <= integral_accumulator_area;
                                d_term_area_o  <= 32'sd0;
                                derivative_delta <= 32'sd0;
                                deadband_work       <= 1'b0;
                                target_capture_work <= 1'b1;
                                state               <= PI_OUTPUT;
                            end
                            else begin
                                error_work <= $signed(mean_area_i) - $signed(target_active);
                                area_error_o <= $signed(mean_area_i) - $signed(target_active);
                                // Use the registered target magnitude so a DSP input is
                                // not preceded by a 32-bit absolute-value carry chain.
                                deadband_product <= target_abs_cached * deadband_fraction_q16_i;
                                integral_limit_product <= target_abs_cached * integral_limit_fraction_q16_i;

                                if (!group_valid_for_feedback_i || !feedback_enable_i ||
                                    !config_valid_o || !target_latched) begin
                                    p_term_area_o <= 32'sd0;
                                    i_term_area_o <= integral_accumulator_area;
                                    d_term_area_o <= 32'sd0;
                                    derivative_delta <= 32'sd0;
                                    deadband_work <= 1'b0;
                                    state <= PI_OUTPUT;
                                end
                                else begin
                                    state <= PI_DEADBAND;
                                end
                            end
                        end
                    end

                    PI_DEADBAND: begin
                        // Cache the effective I clamp limits well before the
                        // integral clamp stage.  integral_limit_product was
                        // registered when the group arrived, so this separates
                        // its DSP output from the later saturation carry chain.
                        if (integral_fraction_mode_i) begin
                            integral_max_bound_reg <= $signed(integral_limit_product >> 16);
                            integral_min_bound_reg <= -$signed(integral_limit_product >> 16);
                        end
                        else begin
                            integral_max_bound_reg <= integral_max_area_i;
                            integral_min_bound_reg <= integral_min_area_i;
                        end

                        // Derivative is taken on the measured area error.  The first
                        // valid group after reset/invalid data deliberately has D=0
                        // so a stale error cannot create a derivative kick.
                        derivative_delta <= derivative_initialized
                            ? ($signed(error_work) - $signed(previous_error))
                            : 32'sd0;

                        // Always write the visible P/I/D registers in this stage.
                        // This prevents the deadband comparison from becoming a
                        // high-fanout clock-enable path into d_term_area_o/p_term_area_o.
                        // For a non-deadband group these temporary zero/current-I
                        // values are overwritten by PI_INTEGRAL before PI_OUTPUT, so
                        // the externally reported result is numerically unchanged.
                        p_term_area_o <= 32'sd0;
                        i_term_area_o <= integral_accumulator_area;
                        d_term_area_o <= 32'sd0;

                        // Register the deadband decision and use it only on the
                        // following clock.  The v3.9 routed worst path was
                        // error_work -> deadband compare -> d_term_area_o CE.
                        // This stage boundary leaves the compare driving only one
                        // ordinary data register rather than state/output controls.
                        deadband_work <= (error_abs <= deadband_value);
                        state <= PI_DEADBAND_DECIDE;
                    end

                    PI_DEADBAND_DECIDE: begin
                        if (deadband_work) begin
                            state <= PI_OUTPUT;
                        end
                        else begin
                            state <= PI_MUL_GAINS;
                        end
                    end

                    PI_MUL_GAINS: begin
                        // Three independent registered multiplies.  They are kept
                        // separate from the later sum and plant-gain multiply so the
                        // 125 MHz path does not become a one-cycle PID arithmetic chain.
                        p_product <= $signed(error_work) * $signed({1'b0, kp_q16_i[30:0]});
                        i_product_q31 <= $signed(error_work) * $signed({1'b0, ki_q31_per_update_i[30:0]});
                        d_product <= $signed(derivative_delta) * $signed({1'b0, kd_q16_per_update_i[30:0]});
                        state <= PI_INTEGRAL;
                    end

                    PI_INTEGRAL: begin
                        p_area_work <= $signed(p_product >>> 16);
                        d_area_work <= $signed(d_product >>> 16);
                        p_term_area_o <= $signed(p_product >>> 16);
                        d_term_area_o <= $signed(d_product >>> 16);

                        // Q31 integral stage 1: accumulate the FULL fractional
                        // product.  No >>>31 occurs here.  The extra guard bit
                        // catches overflow before the 64-bit clamp stage.
                        integral_sum_wide_fp <=
                            $signed({integral_accumulator_fp[63], integral_accumulator_fp}) +
                            $signed({i_product_q31[63], i_product_q31});

                        integral_candidate_high_sat <= 1'b0;
                        integral_candidate_low_sat  <= 1'b0;
                        state <= PI_I_RANGE;
                    end

                    PI_I_RANGE: begin
                        // Keep the expensive clamp compare at 64 bits, matching the
                        // old timing-closure width.  First detect only whether the
                        // guarded 65-bit sum lies outside signed-64 range.
                        integral_sum_overflow_high <=
                            (!integral_sum_wide_fp[64] && integral_sum_wide_fp[63]);
                        integral_sum_overflow_low <=
                            ( integral_sum_wide_fp[64] && !integral_sum_wide_fp[63]);
                        integral_sum_fp <= integral_sum_wide_fp[63:0];
                        state <= PI_I_CLAMP;
                    end

                    PI_I_CLAMP: begin
                        // Clamp in the same physical area units as before, but shift
                        // the registered integer limits into the internal Q31 domain.
                        if (hold_integrator_i) begin
                            integral_candidate_fp <= integral_accumulator_fp;
                        end
                        else if (integral_sum_overflow_high ||
                                 ($signed(integral_sum_fp) >
                                  $signed({integral_max_bound_reg[31],
                                           integral_max_bound_reg, 31'd0}))) begin
                            integral_candidate_fp <=
                                $signed({integral_max_bound_reg[31],
                                         integral_max_bound_reg, 31'd0});
                            integral_candidate_high_sat <= 1'b1;
                        end
                        else if (integral_sum_overflow_low ||
                                 ($signed(integral_sum_fp) <
                                  $signed({integral_min_bound_reg[31],
                                           integral_min_bound_reg, 31'd0}))) begin
                            integral_candidate_fp <=
                                $signed({integral_min_bound_reg[31],
                                         integral_min_bound_reg, 31'd0});
                            integral_candidate_low_sat <= 1'b1;
                        end
                        else begin
                            integral_candidate_fp <= integral_sum_fp;
                        end
                        state <= PI_I_ROUND;
                    end

                    PI_I_ROUND: begin
                        // Convert to integer area only at the P+I+D interface.
                        // Internal I state remains Q31 even when the visible term is 0.
                        integral_candidate_area <= round_q31_to_s32(integral_candidate_fp);
                        integral_current_area   <= integral_accumulator_area;
                        state <= PI_SUM;
                    end

                    PI_SUM: begin
                        candidate_control_area <= $signed(p_area_work) + $signed(d_area_work) +
                                                  $signed(integral_candidate_area);
                        current_control_area   <= $signed(p_area_work) + $signed(d_area_work) +
                                                  $signed(integral_current_area);
                        state <= PI_GAIN_MUL;
                    end

                    PI_GAIN_MUL: begin
                        // Plant calibration remains a registered DSP multiply and is
                        // intentionally separated from all following conversion logic.
                        candidate_gain_product <= $signed(candidate_control_area) *
                                                  $signed({1'b0, area_to_dac_gain_q24_i});
                        current_gain_product   <= $signed(current_control_area) *
                                                  $signed({1'b0, area_to_dac_gain_q24_i});
                        state <= PI_SELECT;
                    end

                    PI_SELECT: begin
                        // Preserve the legacy arithmetic order exactly: first perform
                        // the signed Q24 arithmetic right shift, then apply the
                        // correction sign in a later stage.
                        candidate_gain_shifted <= $signed(candidate_gain_product >>> 24);
                        current_gain_shifted   <= $signed(current_gain_product >>> 24);
                        state <= PI_SIGN;
                    end

                    PI_SIGN: begin
                        candidate_signed_shifted <= correction_sign_negative_i
                            ? -$signed(candidate_gain_shifted)
                            :  $signed(candidate_gain_shifted);
                        current_signed_shifted <= correction_sign_negative_i
                            ? -$signed(current_gain_shifted)
                            :  $signed(current_gain_shifted);
                        state <= PI_SAT;
                    end

                    PI_SAT: begin
                        // Every legal DAC correction bound fits in signed 17 bits.
                        // Saturating the registered signed correction to 18 bits keeps
                        // one guard bit and cannot hide a physical-bound violation.
                        candidate_correction_raw <= sat_s18(candidate_signed_shifted);
                        current_correction_raw   <= sat_s18(current_signed_shifted);
                        state <= PI_AW_CHECK;
                    end

                    PI_AW_CHECK: begin
                        // Compare only registered 18-bit corrections against the
                        // signed 17-bit physical bounds.  This replaces the v3.7
                        // 64-bit candidate_gain_product -> freeze_integrator path.
                        freeze_integrator <=
                            ($signed(candidate_correction_raw) >
                             $signed({correction_max_bound[16], correction_max_bound})) ||
                            ($signed(candidate_correction_raw) <
                             $signed({correction_min_bound[16], correction_min_bound}));
                        state <= PI_BOUND;
                    end

                    PI_BOUND: begin
                        selected_correction_raw <= freeze_integrator
                            ? current_correction_raw
                            : candidate_correction_raw;
                        selected_integral_fp <= freeze_integrator
                            ? integral_accumulator_fp
                            : integral_candidate_fp;
                        selected_integral_area <= freeze_integrator
                            ? integral_current_area
                            : integral_candidate_area;

                        correction_high_sat_work <= 1'b0;
                        correction_low_sat_work  <= 1'b0;

                        if ($signed(freeze_integrator ? current_correction_raw : candidate_correction_raw) >
                            $signed({correction_max_bound[16], correction_max_bound})) begin
                            correction_clamped_bound <= correction_max_bound;
                            correction_high_sat_work <= 1'b1;
                        end
                        else if ($signed(freeze_integrator ? current_correction_raw : candidate_correction_raw) <
                                 $signed({correction_min_bound[16], correction_min_bound})) begin
                            correction_clamped_bound <= correction_min_bound;
                            correction_low_sat_work <= 1'b1;
                        end
                        else begin
                            // Inside the physical bounds the 18-bit value is
                            // guaranteed to fit the signed 17-bit destination.
                            correction_clamped_bound <=
                                (freeze_integrator ? current_correction_raw[16:0]
                                                   : candidate_correction_raw[16:0]);
                        end
                        state <= PI_STEP;
                    end

                    PI_STEP: begin
                        if ($signed(correction_clamped_bound) > $signed(correction_step_high))
                            correction_after_step <= correction_step_high;
                        else if ($signed(correction_clamped_bound) < $signed(correction_step_low))
                            correction_after_step <= correction_step_low;
                        else
                            correction_after_step <= correction_clamped_bound;
                        state <= PI_OUTPUT;
                    end

                    PI_OUTPUT: begin
                        if (meta_group_valid && feedback_enable_i && config_valid_o &&
                            target_latched && !deadband_work && !target_capture_work) begin
                            correction_pending_o <= correction_after_step[15:0];
                            integral_accumulator_fp <= selected_integral_fp;
                            integral_accumulator_area <= selected_integral_area;
                            i_term_area_o <= selected_integral_area;
                            integral_high_sat_o <= integral_candidate_high_sat;
                            integral_low_sat_o  <= integral_candidate_low_sat;
                            correction_high_sat_o <= correction_high_sat_work;
                            correction_low_sat_o  <= correction_low_sat_work;
                            feedback_update_valid_o <= 1'b1;
                            update_count_o <= update_count_o + 1'b1;
                            result_feedback_applied_o <= 1'b1;
                        end
                        else begin
                            result_feedback_applied_o <= 1'b0;
                        end

                        // Keep derivative history aligned with consecutive valid
                        // groups.  Invalid groups break the history so the next valid
                        // sample starts with D=0 instead of using the wrong dt.
                        if (target_capture_work) begin
                            previous_error <= 32'sd0;
                            derivative_initialized <= 1'b1;
                        end
                        else if (meta_group_valid && target_latched) begin
                            previous_error <= error_work;
                            derivative_initialized <= 1'b1;
                        end
                        else if (!meta_group_valid) begin
                            derivative_initialized <= 1'b0;
                            d_term_area_o <= 32'sd0;
                        end

                        result_valid_o                    <= 1'b1;
                        result_timestamp_o                <= meta_timestamp;
                        result_group_id_o                 <= meta_group_id;
                        result_detected_pulses_o          <= meta_detected;
                        result_valid_pulses_o             <= meta_valid;
                        result_invalid_pulses_o           <= meta_invalid;
                        result_expected_mismatch_o        <= meta_expected_mismatch;
                        result_group_overflow_o           <= meta_group_overflow;
                        result_boundary_overrun_o         <= meta_boundary_overrun;
                        result_group_valid_for_feedback_o <= meta_group_valid;
                        result_mean_area_o                <= meta_mean;
                        result_deadband_o                 <= deadband_work;

                        // latency_counter starts at zero on the group-accept edge.
                        // Add one here so the reported value equals the actual number
                        // of 125 MHz clock periods from group acceptance to result.
                        last_processing_latency_cycles_o <= latency_counter + 1'b1;
                        latency_busy <= 1'b0;
                        state <= PI_IDLE;
                    end

                    default: state <= PI_IDLE;
                endcase
            end
        end
    end

endmodule
