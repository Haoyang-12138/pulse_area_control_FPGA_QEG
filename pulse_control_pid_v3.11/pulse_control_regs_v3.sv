`timescale 1ns / 1ps

// Unified runtime register map for GPIO timing, external-triggered ADC pulse
// grouping, PID feedback, OUT2 sequencing, FIFO logging, and debug status.
// Base address remains sys[7] (0x40700000 in the existing Red Pitaya project).
module pulse_control_regs_v3 #(
    parameter int MAX_GPIO_PULSES = 16,
    parameter int MAX_DAC_SEGMENTS = 16
)(
    input  logic                 clk_i,
    input  logic                 rstn_i,

    input  logic [31:0]          sys_addr_i,
    input  logic [31:0]          sys_wdata_i,
    input  logic                 sys_wen_i,
    input  logic                 sys_ren_i,
    output logic [31:0]          sys_rdata_o,
    output logic                 sys_err_o,
    output logic                 sys_ack_o,

    // Control / one-shot commands.
    output logic                 measurement_enable_o,
    output logic                 feedback_enable_o,
    output logic                 hold_integrator_o,
    output logic                 marker_enable_o,
    output logic                 gpio_enable_o,
    output logic                 dac2_enable_o,
    output logic                 use_first_valid_group_as_target_o,
    output logic                 pulse_polarity_negative_o,
    output logic                 fifo_continuous_log_enable_o,

    output logic                 clear_state_o,
    output logic                 clear_flags_o,
    output logic                 clear_fifos_o,
    output logic                 dac2_apply_o,
    output logic                 manual_group_boundary_o,
    output logic                 pulse_fifo_pop_o,
    output logic                 group_fifo_pop_o,
    output logic                 pulse_snapshot_arm_o,
    output logic                 gpio_apply_toggle_o,

    // ADC measurement configuration.
    output logic [15:0]          threshold_fraction_q16_o,
    output logic [15:0]          target_height_o,
    output logic [8:0]           pre_baseline_samples_o,
    output logic [8:0]           post_baseline_samples_o,
    output logic [31:0]          min_pulse_samples_o,
    output logic [31:0]          max_pulse_samples_o,
    output logic [15:0]          adc_saturation_limit_o,

    // Grouping configuration.
    output logic [7:0]           expected_pulses_per_group_o,
    output logic [7:0]           min_valid_pulses_per_group_o,
    output logic [1:0]           group_source_o, // 0 external DIO0_P, 1 DAC2, 2 manual
    // External-trigger delay in 200 MHz cycles (5 ns/cycle).  This replaces
    // the old 125 MHz group-boundary delay while keeping address 0x03C.
    output logic [31:0]          external_trigger_delay_cycles_o,

    // PI / calibration configuration.
    output logic signed [31:0]   target_area_o,
    output logic [31:0]          deadband_absolute_o,
    output logic [15:0]          deadband_fraction_q16_o,
    output logic                 deadband_fraction_mode_o,
    output logic [31:0]          kp_q16_o,
    output logic [31:0]          ki_q31_per_update_o,
    output logic [31:0]          kd_q16_per_update_o,
    output logic [31:0]          area_to_dac_gain_q24_o,
    output logic                 correction_sign_negative_o,
    output logic [15:0]          max_dac_step_counts_o,
    output logic signed [15:0]   dac_high_min_counts_o,
    output logic signed [15:0]   dac_high_max_counts_o,
    output logic signed [31:0]   integral_min_area_o,
    output logic signed [31:0]   integral_max_area_o,
    output logic [15:0]          integral_limit_fraction_q16_o,
    output logic                 integral_fraction_mode_o,

    // GPIO shadow configuration (200 MHz domain captures on APPLY toggle).
    output logic [31:0]          gpio_period_cycles_shadow_o,
    output logic [4:0]           gpio_pulse_count_shadow_o,
    output logic [31:0]          gpio_pulse_start_cycles_shadow_o [0:MAX_GPIO_PULSES-1],
    output logic [31:0]          gpio_pulse_end_cycles_shadow_o   [0:MAX_GPIO_PULSES-1],

    // OUT2 shadow configuration.
    output logic [31:0]          dac2_period_cycles_shadow_o,
    output logic [4:0]           dac2_segment_count_shadow_o,
    output logic signed [15:0]   dac2_low_counts_shadow_o,
    output logic signed [15:0]   dac2_high_counts_shadow_o,
    output logic [15:0]          dac2_segment_level_q16_shadow_o [0:MAX_DAC_SEGMENTS-1],
    output logic [31:0]          dac2_segment_duration_cycles_shadow_o [0:MAX_DAC_SEGMENTS-1],

    // Status / monitoring inputs.
    input  logic                 measurement_config_valid_i,
    input  logic                 controller_config_valid_i,
    input  logic                 gpio_config_valid_i,
    input  logic                 dac2_config_valid_i,
    input  logic                 target_latched_i,

    input  logic signed [13:0]   bpre_i,
    input  logic signed [13:0]   bpost_i,
    input  logic signed [13:0]   threshold_i,
    input  logic signed [13:0]   peak_raw_i,
    input  logic [15:0]          peak_height_i,
    input  logic [31:0]          pulse_duration_samples_i,
    input  logic signed [31:0]   measured_area_i,
    input  logic [7:0]           last_invalid_reason_i,
    input  logic                 invalid_pulse_valid_i,

    input  logic [63:0]          timestamp_i,
    input  logic [31:0]          total_valid_measurements_i,
    input  logic [31:0]          total_invalid_measurements_i,
    input  logic [31:0]          total_groups_i,

    input  logic signed [31:0]   target_area_active_i,
    input  logic signed [31:0]   area_error_i,
    input  logic signed [31:0]   p_term_area_i,
    input  logic signed [31:0]   i_term_area_i,
    input  logic signed [31:0]   d_term_area_i,
    input  logic signed [15:0]   correction_pending_i,
    input  logic signed [15:0]   correction_active_i,
    input  logic signed [15:0]   effective_dac_high_i,
    input  logic [31:0]          update_count_i,
    input  logic [31:0]          processing_latency_cycles_i,
    input  logic                 correction_high_sat_i,
    input  logic                 correction_low_sat_i,
    input  logic                 integral_high_sat_i,
    input  logic                 integral_low_sat_i,

    input  logic                 group_result_valid_i,
    input  logic [31:0]          last_group_id_i,
    input  logic [7:0]           last_group_detected_i,
    input  logic [7:0]           last_group_valid_i,
    input  logic [7:0]           last_group_invalid_i,
    input  logic signed [31:0]   last_group_mean_area_i,
    input  logic                 last_group_expected_mismatch_i,
    input  logic                 last_group_overflow_i,
    input  logic                 last_boundary_overrun_i,

    input  logic [31:0]          marker_cycles_i,
    input  logic                 marker_timeout_i,

    input  logic [31:0]          gpio_active_period_cycles_i,
    input  logic [4:0]           gpio_active_pulse_count_i,
    input  logic [31:0]          dac2_active_period_cycles_i,
    input  logic [4:0]           dac2_active_segment_count_i,

    input  logic [31:0]          external_trigger_count_i,
    input  logic [31:0]          external_trigger_active_delay_cycles_i,
    input  logic                 external_trigger_overrun_i,

    // FIFO status and head records.  Reading does NOT pop; write COMMAND bits.
    input  logic [15:0]          pulse_fifo_level_i,
    input  logic                 pulse_fifo_full_i,
    input  logic [31:0]          pulse_fifo_overflow_count_i,
    input  logic [319:0]         pulse_fifo_head_i,

    input  logic [15:0]          group_fifo_level_i,
    input  logic                 group_fifo_full_i,
    input  logic [31:0]          group_fifo_overflow_count_i,
    input  logic [415:0]         group_fifo_head_i
);

    localparam logic [31:0] VERSION = 32'h50433035; // "PC05"

    logic sys_en;
    logic [31:0] status_word;

    logic invalid_seen;
    logic correction_sat_seen;
    logic integral_sat_seen;
    logic marker_timeout_seen;
    logic group_mismatch_seen;
    logic group_overflow_seen;
    logic boundary_overrun_seen;
    logic external_trigger_overrun_seen;

    integer i;

    assign sys_en = sys_wen_i | sys_ren_i;

    assign status_word = {
        7'd0,
        external_trigger_overrun_seen,   // 24
        (correction_active_i != 16'sd0), // 23
        boundary_overrun_seen,           // 22
        group_overflow_seen,             // 21
        group_mismatch_seen,             // 20
        marker_timeout_seen,             // 19
        integral_sat_seen,               // 18
        correction_sat_seen,             // 17
        invalid_seen,                    // 16
        (group_fifo_overflow_count_i != 0), // 15
        (pulse_fifo_overflow_count_i != 0), // 14
        group_fifo_full_i,               // 13
        pulse_fifo_full_i,               // 12
        (group_fifo_level_i != 0),        // 11
        (pulse_fifo_level_i != 0),        // 10
        target_latched_i,                // 9
        dac2_config_valid_i,             // 8
        gpio_config_valid_i,             // 7
        controller_config_valid_i,       // 6
        measurement_config_valid_i,      // 5
        marker_enable_o,                 // 4
        dac2_enable_o,                   // 3
        gpio_enable_o,                   // 2
        feedback_enable_o,               // 1
        measurement_enable_o             // 0
    };

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            measurement_enable_o <= 1'b0;
            feedback_enable_o    <= 1'b0;
            hold_integrator_o    <= 1'b0;
            marker_enable_o      <= 1'b1;
            gpio_enable_o        <= 1'b0;
            dac2_enable_o        <= 1'b0;
            use_first_valid_group_as_target_o <= 1'b1;
            pulse_polarity_negative_o <= 1'b0;
            fifo_continuous_log_enable_o <= 1'b0;

            clear_state_o          <= 1'b0;
            clear_flags_o          <= 1'b0;
            clear_fifos_o          <= 1'b0;
            dac2_apply_o           <= 1'b0;
            manual_group_boundary_o<= 1'b0;
            pulse_fifo_pop_o       <= 1'b0;
            group_fifo_pop_o       <= 1'b0;
            pulse_snapshot_arm_o   <= 1'b0;
            gpio_apply_toggle_o    <= 1'b0;

            threshold_fraction_q16_o <= 16'h4000; // 25%
            target_height_o          <= 16'd819;
            pre_baseline_samples_o   <= 9'd32;
            post_baseline_samples_o  <= 9'd32;
            min_pulse_samples_o      <= 32'd3;
            max_pulse_samples_o      <= 32'd16384;
            adc_saturation_limit_o   <= 16'd8191;

            expected_pulses_per_group_o <= 8'd0;
            min_valid_pulses_per_group_o <= 8'd1;
            group_source_o                    <= 2'd0;
            external_trigger_delay_cycles_o    <= 32'd0;

            target_area_o                 <= 32'sd0;
            deadband_absolute_o           <= 32'd0;
            deadband_fraction_q16_o       <= 16'd328; // about 0.005
            deadband_fraction_mode_o      <= 1'b1;
            kp_q16_o                      <= 32'd13107; // 0.20
            ki_q31_per_update_o           <= 32'd0;
            kd_q16_per_update_o           <= 32'd0;     // D disabled by default
            area_to_dac_gain_q24_o        <= 32'd0;
            correction_sign_negative_o    <= 1'b1;
            max_dac_step_counts_o         <= 16'd164; // about 20 mV at 8192 cnt/V
            dac_high_min_counts_o         <= 16'sd82;
            dac_high_max_counts_o         <= 16'sd8191;
            integral_min_area_o           <= -32'sd1073741824;
            integral_max_area_o           <=  32'sd1073741824;
            integral_limit_fraction_q16_o <= 16'h4000; // 0.25 of target area
            integral_fraction_mode_o      <= 1'b1;

            // Old verified 100 us? no: 2000 cycles @ 200 MHz = 10 us.
            gpio_period_cycles_shadow_o <= 32'd2000;
            gpio_pulse_count_shadow_o   <= 5'd3;

            dac2_period_cycles_shadow_o <= 32'd25000; // 5 kHz @ 125 MHz
            dac2_segment_count_shadow_o <= 5'd4;
            dac2_low_counts_shadow_o    <= 16'sd0;
            dac2_high_counts_shadow_o   <= 16'sd8191;

            invalid_seen          <= 1'b0;
            correction_sat_seen   <= 1'b0;
            integral_sat_seen     <= 1'b0;
            marker_timeout_seen   <= 1'b0;
            group_mismatch_seen   <= 1'b0;
            group_overflow_seen   <= 1'b0;
            boundary_overrun_seen <= 1'b0;
            external_trigger_overrun_seen <= 1'b0;

            for (i = 0; i < MAX_GPIO_PULSES; i = i + 1) begin
                gpio_pulse_start_cycles_shadow_o[i] <= 32'd0;
                gpio_pulse_end_cycles_shadow_o[i]   <= 32'd0;
            end
            gpio_pulse_start_cycles_shadow_o[0] <= 32'd0;
            gpio_pulse_end_cycles_shadow_o[0]   <= 32'd2;
            gpio_pulse_start_cycles_shadow_o[1] <= 32'd6;
            gpio_pulse_end_cycles_shadow_o[1]   <= 32'd8;
            gpio_pulse_start_cycles_shadow_o[2] <= 32'd28;
            gpio_pulse_end_cycles_shadow_o[2]   <= 32'd1028;

            for (i = 0; i < MAX_DAC_SEGMENTS; i = i + 1) begin
                dac2_segment_level_q16_shadow_o[i] <= 16'd0;
                dac2_segment_duration_cycles_shadow_o[i] <= 32'd1;
            end
            dac2_segment_level_q16_shadow_o[0] <= 16'hFFFF;
            dac2_segment_duration_cycles_shadow_o[0] <= 32'd1250;
            dac2_segment_level_q16_shadow_o[1] <= 16'd0;
            dac2_segment_duration_cycles_shadow_o[1] <= 32'd2500;
            dac2_segment_level_q16_shadow_o[2] <= 16'hFFFF;
            dac2_segment_duration_cycles_shadow_o[2] <= 32'd2500;
            dac2_segment_level_q16_shadow_o[3] <= 16'd0;
            dac2_segment_duration_cycles_shadow_o[3] <= 32'd18750;
        end
        else begin
            clear_state_o           <= 1'b0;
            clear_flags_o           <= 1'b0;
            clear_fifos_o           <= 1'b0;
            dac2_apply_o            <= 1'b0;
            manual_group_boundary_o <= 1'b0;
            pulse_fifo_pop_o        <= 1'b0;
            group_fifo_pop_o        <= 1'b0;
            pulse_snapshot_arm_o    <= 1'b0;

            if (invalid_pulse_valid_i)
                invalid_seen <= 1'b1;
            if (correction_high_sat_i || correction_low_sat_i)
                correction_sat_seen <= 1'b1;
            if (integral_high_sat_i || integral_low_sat_i)
                integral_sat_seen <= 1'b1;
            if (marker_timeout_i)
                marker_timeout_seen <= 1'b1;
            if (group_result_valid_i && last_group_expected_mismatch_i)
                group_mismatch_seen <= 1'b1;
            if (group_result_valid_i && last_group_overflow_i)
                group_overflow_seen <= 1'b1;
            if (group_result_valid_i && last_boundary_overrun_i)
                boundary_overrun_seen <= 1'b1;
            if (external_trigger_overrun_i)
                external_trigger_overrun_seen <= 1'b1;

            if (sys_wen_i) begin
                // Window tables occupy compact address ranges and are handled
                // before the scalar register case.
                if ((sys_addr_i[19:0] >= 20'h00100) &&
                    (sys_addr_i[19:0] <  20'h00180)) begin
                    if (sys_addr_i[2] == 1'b0)
                        gpio_pulse_start_cycles_shadow_o[sys_addr_i[6:3]] <= sys_wdata_i;
                    else
                        gpio_pulse_end_cycles_shadow_o[sys_addr_i[6:3]] <= sys_wdata_i;
                end
                else if ((sys_addr_i[19:0] >= 20'h00200) &&
                         (sys_addr_i[19:0] <  20'h00280)) begin
                    if (sys_addr_i[2] == 1'b0)
                        dac2_segment_level_q16_shadow_o[sys_addr_i[6:3]] <= sys_wdata_i[15:0];
                    else
                        dac2_segment_duration_cycles_shadow_o[sys_addr_i[6:3]] <= sys_wdata_i;
                end
                else begin
                    case (sys_addr_i[19:0])
                        20'h00000: begin
                            measurement_enable_o <= sys_wdata_i[0];
                            feedback_enable_o    <= sys_wdata_i[1];
                            hold_integrator_o    <= sys_wdata_i[2];
                            marker_enable_o      <= sys_wdata_i[3];
                            gpio_enable_o        <= sys_wdata_i[4];
                            dac2_enable_o        <= sys_wdata_i[5];
                            use_first_valid_group_as_target_o <= sys_wdata_i[6];
                            pulse_polarity_negative_o <= sys_wdata_i[7];
                            fifo_continuous_log_enable_o <= sys_wdata_i[8];
                        end

                        // COMMAND write-one bits:
                        // 0 clear state, 1 clear sticky flags, 2 GPIO APPLY,
                        // 3 DAC2 APPLY, 4 clear FIFOs, 5 pop pulse FIFO,
                        // 6 pop group FIFO, 7 manual group boundary,
                        // 8 arm exactly one pulse snapshot.
                        20'h00004: begin
                            clear_state_o           <= sys_wdata_i[0];
                            clear_flags_o           <= sys_wdata_i[1];
                            if (sys_wdata_i[2])
                                gpio_apply_toggle_o <= ~gpio_apply_toggle_o;
                            dac2_apply_o            <= sys_wdata_i[3];
                            clear_fifos_o           <= sys_wdata_i[4];
                            pulse_fifo_pop_o        <= sys_wdata_i[5];
                            group_fifo_pop_o        <= sys_wdata_i[6];
                            manual_group_boundary_o <= sys_wdata_i[7];
                            pulse_snapshot_arm_o     <= sys_wdata_i[8];
                        end

                        20'h00010: threshold_fraction_q16_o <= sys_wdata_i[15:0];
                        20'h00014: target_height_o <= sys_wdata_i[15:0];
                        20'h00018: pre_baseline_samples_o <= sys_wdata_i[8:0];
                        20'h0001C: post_baseline_samples_o <= sys_wdata_i[8:0];
                        20'h00020: min_pulse_samples_o <= sys_wdata_i;
                        20'h00024: max_pulse_samples_o <= sys_wdata_i;
                        20'h00028: adc_saturation_limit_o <= sys_wdata_i[15:0];
                        20'h00030: expected_pulses_per_group_o <= sys_wdata_i[7:0];
                        20'h00034: min_valid_pulses_per_group_o <= sys_wdata_i[7:0];
                        20'h00038: group_source_o <= sys_wdata_i[1:0];
                        20'h0003C: external_trigger_delay_cycles_o <= sys_wdata_i;

                        20'h00040: target_area_o <= sys_wdata_i;
                        20'h00044: deadband_absolute_o <= sys_wdata_i;
                        20'h00048: deadband_fraction_q16_o <= sys_wdata_i[15:0];
                        20'h0004C: deadband_fraction_mode_o <= sys_wdata_i[0];
                        20'h00050: kp_q16_o <= sys_wdata_i;
                        20'h00054: ki_q31_per_update_o <= sys_wdata_i;
                        20'h00058: area_to_dac_gain_q24_o <= sys_wdata_i;
                        20'h0005C: correction_sign_negative_o <= sys_wdata_i[0];
                        20'h00060: max_dac_step_counts_o <= sys_wdata_i[15:0];
                        20'h00064: dac_high_min_counts_o <= sys_wdata_i[15:0];
                        20'h00068: dac_high_max_counts_o <= sys_wdata_i[15:0];
                        20'h0006C: integral_min_area_o <= sys_wdata_i;
                        20'h00070: integral_max_area_o <= sys_wdata_i;
                        20'h00074: integral_limit_fraction_q16_o <= sys_wdata_i[15:0];
                        20'h00078: integral_fraction_mode_o <= sys_wdata_i[0];
                        20'h0007C: kd_q16_per_update_o <= sys_wdata_i;

                        20'h00080: gpio_period_cycles_shadow_o <= sys_wdata_i;
                        20'h00084: gpio_pulse_count_shadow_o <= sys_wdata_i[4:0];

                        20'h00180: dac2_period_cycles_shadow_o <= sys_wdata_i;
                        20'h00184: dac2_segment_count_shadow_o <= sys_wdata_i[4:0];
                        20'h00188: dac2_low_counts_shadow_o <= sys_wdata_i[15:0];
                        20'h0018C: dac2_high_counts_shadow_o <= sys_wdata_i[15:0];
                        default: begin end
                    endcase
                end
            end

            if (clear_flags_o) begin
                invalid_seen          <= 1'b0;
                correction_sat_seen   <= 1'b0;
                integral_sat_seen     <= 1'b0;
                marker_timeout_seen   <= 1'b0;
                group_mismatch_seen   <= 1'b0;
                group_overflow_seen   <= 1'b0;
                boundary_overrun_seen <= 1'b0;
                external_trigger_overrun_seen <= 1'b0;
            end
        end
    end

    always_ff @(posedge clk_i) begin
        if (!rstn_i) begin
            sys_rdata_o <= 32'd0;
            sys_err_o   <= 1'b0;
            sys_ack_o   <= 1'b0;
        end
        else begin
            sys_err_o <= 1'b0;
            sys_ack_o <= sys_en;

            if ((sys_addr_i[19:0] >= 20'h00100) &&
                (sys_addr_i[19:0] <  20'h00180)) begin
                if (sys_addr_i[2] == 1'b0)
                    sys_rdata_o <= gpio_pulse_start_cycles_shadow_o[sys_addr_i[6:3]];
                else
                    sys_rdata_o <= gpio_pulse_end_cycles_shadow_o[sys_addr_i[6:3]];
            end
            else if ((sys_addr_i[19:0] >= 20'h00200) &&
                     (sys_addr_i[19:0] <  20'h00280)) begin
                if (sys_addr_i[2] == 1'b0)
                    sys_rdata_o <= {16'd0, dac2_segment_level_q16_shadow_o[sys_addr_i[6:3]]};
                else
                    sys_rdata_o <= dac2_segment_duration_cycles_shadow_o[sys_addr_i[6:3]];
            end
            else if ((sys_addr_i[19:0] >= 20'h00400) &&
                     (sys_addr_i[19:0] <= 20'h00424)) begin
                case (sys_addr_i[5:2])
                    4'd0: sys_rdata_o <= pulse_fifo_head_i[31:0];
                    4'd1: sys_rdata_o <= pulse_fifo_head_i[63:32];
                    4'd2: sys_rdata_o <= pulse_fifo_head_i[95:64];
                    4'd3: sys_rdata_o <= pulse_fifo_head_i[127:96];
                    4'd4: sys_rdata_o <= pulse_fifo_head_i[159:128];
                    4'd5: sys_rdata_o <= pulse_fifo_head_i[191:160];
                    4'd6: sys_rdata_o <= pulse_fifo_head_i[223:192];
                    4'd7: sys_rdata_o <= pulse_fifo_head_i[255:224];
                    4'd8: sys_rdata_o <= pulse_fifo_head_i[287:256];
                    4'd9: sys_rdata_o <= pulse_fifo_head_i[319:288];
                    default: sys_rdata_o <= 32'd0;
                endcase
            end
            else if ((sys_addr_i[19:0] >= 20'h00440) &&
                     (sys_addr_i[19:0] <= 20'h00470)) begin
                case ((sys_addr_i[19:0] - 20'h00440) >> 2)
                    0: sys_rdata_o <= group_fifo_head_i[31:0];
                    1: sys_rdata_o <= group_fifo_head_i[63:32];
                    2: sys_rdata_o <= group_fifo_head_i[95:64];
                    3: sys_rdata_o <= group_fifo_head_i[127:96];
                    4: sys_rdata_o <= group_fifo_head_i[159:128];
                    5: sys_rdata_o <= group_fifo_head_i[191:160];
                    6: sys_rdata_o <= group_fifo_head_i[223:192];
                    7: sys_rdata_o <= group_fifo_head_i[255:224];
                    8: sys_rdata_o <= group_fifo_head_i[287:256];
                    9: sys_rdata_o <= group_fifo_head_i[319:288];
                    10: sys_rdata_o <= group_fifo_head_i[351:320];
                    11: sys_rdata_o <= group_fifo_head_i[383:352];
                    12: sys_rdata_o <= group_fifo_head_i[415:384];
                    default: sys_rdata_o <= 32'd0;
                endcase
            end
            else begin
                case (sys_addr_i[19:0])
                    20'h00000: sys_rdata_o <= {
                        23'd0,
                        fifo_continuous_log_enable_o,
                        pulse_polarity_negative_o,
                        use_first_valid_group_as_target_o,
                        dac2_enable_o,
                        gpio_enable_o,
                        marker_enable_o,
                        hold_integrator_o,
                        feedback_enable_o,
                        measurement_enable_o};
                    20'h00004: sys_rdata_o <= 32'd0;
                    20'h00008: sys_rdata_o <= status_word;
                    20'h0000C: sys_rdata_o <= VERSION;

                    20'h00010: sys_rdata_o <= {16'd0, threshold_fraction_q16_o};
                    20'h00014: sys_rdata_o <= {16'd0, target_height_o};
                    20'h00018: sys_rdata_o <= {23'd0, pre_baseline_samples_o};
                    20'h0001C: sys_rdata_o <= {23'd0, post_baseline_samples_o};
                    20'h00020: sys_rdata_o <= min_pulse_samples_o;
                    20'h00024: sys_rdata_o <= max_pulse_samples_o;
                    20'h00028: sys_rdata_o <= {16'd0, adc_saturation_limit_o};
                    20'h00030: sys_rdata_o <= {24'd0, expected_pulses_per_group_o};
                    20'h00034: sys_rdata_o <= {24'd0, min_valid_pulses_per_group_o};
                    20'h00038: sys_rdata_o <= {30'd0, group_source_o};
                    20'h0003C: sys_rdata_o <= external_trigger_delay_cycles_o;

                    20'h00040: sys_rdata_o <= target_area_o;
                    20'h00044: sys_rdata_o <= deadband_absolute_o;
                    20'h00048: sys_rdata_o <= {16'd0, deadband_fraction_q16_o};
                    20'h0004C: sys_rdata_o <= {31'd0, deadband_fraction_mode_o};
                    20'h00050: sys_rdata_o <= kp_q16_o;
                    20'h00054: sys_rdata_o <= ki_q31_per_update_o;
                    20'h00058: sys_rdata_o <= area_to_dac_gain_q24_o;
                    20'h0005C: sys_rdata_o <= {31'd0, correction_sign_negative_o};
                    20'h00060: sys_rdata_o <= {16'd0, max_dac_step_counts_o};
                    20'h00064: sys_rdata_o <= {{16{dac_high_min_counts_o[15]}}, dac_high_min_counts_o};
                    20'h00068: sys_rdata_o <= {{16{dac_high_max_counts_o[15]}}, dac_high_max_counts_o};
                    20'h0006C: sys_rdata_o <= integral_min_area_o;
                    20'h00070: sys_rdata_o <= integral_max_area_o;
                    20'h00074: sys_rdata_o <= {16'd0, integral_limit_fraction_q16_o};
                    20'h00078: sys_rdata_o <= {31'd0, integral_fraction_mode_o};
                    20'h0007C: sys_rdata_o <= kd_q16_per_update_o;

                    20'h00080: sys_rdata_o <= gpio_period_cycles_shadow_o;
                    20'h00084: sys_rdata_o <= {27'd0, gpio_pulse_count_shadow_o};
                    20'h00180: sys_rdata_o <= dac2_period_cycles_shadow_o;
                    20'h00184: sys_rdata_o <= {27'd0, dac2_segment_count_shadow_o};
                    20'h00188: sys_rdata_o <= {{16{dac2_low_counts_shadow_o[15]}}, dac2_low_counts_shadow_o};
                    20'h0018C: sys_rdata_o <= {{16{dac2_high_counts_shadow_o[15]}}, dac2_high_counts_shadow_o};

                    // Live monitoring block.
                    20'h00300: sys_rdata_o <= {{18{bpre_i[13]}}, bpre_i};
                    20'h00304: sys_rdata_o <= {{18{bpost_i[13]}}, bpost_i};
                    20'h00308: sys_rdata_o <= {{18{threshold_i[13]}}, threshold_i};
                    20'h0030C: sys_rdata_o <= {{18{peak_raw_i[13]}}, peak_raw_i};
                    20'h00310: sys_rdata_o <= {16'd0, peak_height_i};
                    20'h00314: sys_rdata_o <= pulse_duration_samples_i;
                    20'h00318: sys_rdata_o <= measured_area_i;
                    20'h0031C: sys_rdata_o <= total_valid_measurements_i;
                    20'h00320: sys_rdata_o <= total_invalid_measurements_i;
                    20'h00324: sys_rdata_o <= {24'd0, last_invalid_reason_i};
                    20'h00328: sys_rdata_o <= timestamp_i[31:0];
                    20'h0032C: sys_rdata_o <= timestamp_i[63:32];
                    20'h00330: sys_rdata_o <= total_groups_i;
                    20'h00334: sys_rdata_o <= target_area_active_i;
                    20'h00338: sys_rdata_o <= area_error_i;
                    20'h0033C: sys_rdata_o <= p_term_area_i;
                    20'h00340: sys_rdata_o <= i_term_area_i;
                    20'h00344: sys_rdata_o <= {{16{correction_pending_i[15]}}, correction_pending_i};
                    20'h00348: sys_rdata_o <= {{16{correction_active_i[15]}}, correction_active_i};
                    20'h0034C: sys_rdata_o <= {{16{effective_dac_high_i[15]}}, effective_dac_high_i};
                    20'h00350: sys_rdata_o <= update_count_i;
                    20'h00354: sys_rdata_o <= processing_latency_cycles_i;
                    20'h00358: sys_rdata_o <= {group_fifo_level_i, pulse_fifo_level_i};
                    20'h0035C: sys_rdata_o <= pulse_fifo_overflow_count_i;
                    20'h00360: sys_rdata_o <= group_fifo_overflow_count_i;
                    20'h00364: sys_rdata_o <= marker_cycles_i;
                    20'h00368: sys_rdata_o <= last_group_id_i;
                    20'h0036C: sys_rdata_o <= {8'd0, last_group_invalid_i, last_group_valid_i, last_group_detected_i};
                    20'h00370: sys_rdata_o <= last_group_mean_area_i;
                    20'h00374: sys_rdata_o <= gpio_active_period_cycles_i;
                    20'h00378: sys_rdata_o <= {27'd0, gpio_active_pulse_count_i};
                    20'h0037C: sys_rdata_o <= dac2_active_period_cycles_i;
                    20'h00380: sys_rdata_o <= {27'd0, dac2_active_segment_count_i};
                    20'h00384: sys_rdata_o <= d_term_area_i;
                    20'h00388: sys_rdata_o <= external_trigger_count_i;
                    20'h0038C: sys_rdata_o <= external_trigger_active_delay_cycles_i;
                    default: sys_rdata_o <= 32'd0;
                endcase
            end
        end
    end

endmodule
