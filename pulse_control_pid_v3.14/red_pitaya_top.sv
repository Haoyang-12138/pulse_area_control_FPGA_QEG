////////////////////////////////////////////////////////////////////////////////
// Red Pitaya TOP module. It connects external pins and PS part with
// other application modules.
// Authors: Matej Oblak, Iztok Jeras
// (c) Red Pitaya  http://www.redpitaya.com
////////////////////////////////////////////////////////////////////////////////

/**
 * GENERAL DESCRIPTION:
 *
 * Top module connects PS part with rest of Red Pitaya applications.  
 *
 *                   /-------\      
 *   PS DDR <------> |  PS   |      AXI <-> custom bus
 *   PS MIO <------> |   /   | <------------+
 *   PS CLK -------> |  ARM  |              |
 *                   \-------/              |
 *                                          |
 *                            /-------\     |
 *                         -> | SCOPE | <---+
 *                         |  \-------/     |
 *                         |                |
 *            /--------\   |   /-----\      |
 *   ADC ---> |        | --+-> |     |      |
 *            | ANALOG |       | PID | <----+
 *   DAC <--- |        | <---- |     |      |
 *            \--------/   ^   \-----/      |
 *                         |                |
 *                         |  /-------\     |
 *                         -- |  ASG  | <---+ 
 *                            \-------/     |
 *                                          |
 *             /--------\                   |
 *    RX ----> |        |                   |
 *   SATA      | DAISY  | <-----------------+
 *    TX <---- |        | 
 *             \--------/ 
 *               |    |
 *               |    |
 *               (FREE)
 *
 * Inside analog module, ADC data is translated from unsigned neg-slope into
 * two's complement. Similar is done on DAC data.
 *
 * Scope module stores data from ADC into RAM, arbitrary signal generator (ASG)
 * sends data from RAM to DAC. MIMO PID uses ADC ADC as input and DAC as its output.
 *
 * Daisy chain connects with other boards with fast serial link. Data which is
 * send and received is at the moment undefined. This is left for the user.
 */

module red_pitaya_top #(
  // identification
  bit [0:5*32-1] GITH = '0,
  // module numbers
  parameter MNA = 2,  // number of acquisition modules
  parameter MNG = 2,  // number of generator   modules
  parameter ADW_125 = 14,
  parameter ADW_122 = 16,
  parameter DWE_Z20 = 11,
  parameter DWE_Z10 = 8,
  parameter DDW     = 14,
`ifdef Z20_122
  parameter ADW=ADW_122,
  parameter ADC_DW=ADW_122,
`else
  parameter ADW=ADW_125,
  parameter ADC_DW=ADW_125,
`endif
`ifdef Z20_xx
  parameter DWE=DWE_Z20
`else
  parameter DWE=DWE_Z10
`endif


)(
  // PS connections
  inout  logic [54-1:0] FIXED_IO_mio     ,
  inout  logic          FIXED_IO_ps_clk  ,
  inout  logic          FIXED_IO_ps_porb ,
  inout  logic          FIXED_IO_ps_srstb,
  inout  logic          FIXED_IO_ddr_vrn ,
  inout  logic          FIXED_IO_ddr_vrp ,
  // DDR
  inout  logic [15-1:0] DDR_addr   ,
  inout  logic [ 3-1:0] DDR_ba     ,
  inout  logic          DDR_cas_n  ,
  inout  logic          DDR_ck_n   ,
  inout  logic          DDR_ck_p   ,
  inout  logic          DDR_cke    ,
  inout  logic          DDR_cs_n   ,
  inout  logic [ 4-1:0] DDR_dm     ,
  inout  logic [32-1:0] DDR_dq     ,
  inout  logic [ 4-1:0] DDR_dqs_n  ,
  inout  logic [ 4-1:0] DDR_dqs_p  ,
  inout  logic          DDR_odt    ,
  inout  logic          DDR_ras_n  ,
  inout  logic          DDR_reset_n,
  inout  logic          DDR_we_n   ,

  // Red Pitaya periphery

  // ADC
  input  logic [MNA-1:0] [16-1:0] adc_dat_i,  // ADC data
  input  logic           [ 2-1:0] adc_clk_i,  // ADC clock {p,n}
  output logic           [ 2-1:0] adc_clk_o,  // optional ADC clock source (unused) [0] = p; [1] = n
  output logic                    adc_cdcs_o, // ADC clock duty cycle stabilizer
  // DAC
  output logic [ 14-1:0] dac_dat_o  ,  // DAC combined data
  output logic           dac_wrt_o  ,  // DAC write
  output logic           dac_sel_o  ,  // DAC channel select
  output logic           dac_clk_o  ,  // DAC clock
  output logic           dac_rst_o  ,  // DAC reset
  // PWM DAC
  output logic [  4-1:0] dac_pwm_o  ,  // 1-bit PWM DAC
  // XADC
  input  logic [  5-1:0] vinp_i     ,  // voltages p
  input  logic [  5-1:0] vinn_i     ,  // voltages n
  // Expansion connector
  inout  logic [DWE-1:0] exp_p_io  ,
  inout  logic [DWE-1:0] exp_n_io  ,
  // SATA connector
  output logic [  2-1:0] daisy_p_o  ,  // line 1 is clock capable
  output logic [  2-1:0] daisy_n_o  ,
  input  logic [  2-1:0] daisy_p_i  ,  // line 1 is clock capable
  input  logic [  2-1:0] daisy_n_i  ,

  `ifdef Z20_G2
  // Additional E3 connector
  output logic [  4-1:0] exp_e3p_o  ,  // line 3 is clock capable (SRCC)
  output logic [  4-1:0] exp_e3n_o  ,
  input  logic [  4-1:0] exp_e3p_i  ,  // line 3 is clock capable (MRCC)
  input  logic [  4-1:0] exp_e3n_i  ,

  input  logic           s1_orient_i ,
  input  logic           s1_link_i   ,
  `endif
  // LED
  output  logic [  8-1:0] led_o
);

////////////////////////////////////////////////////////////////////////////////
// local signals
////////////////////////////////////////////////////////////////////////////////

// GPIO input data width
localparam int unsigned GDW = DWE;
localparam RST_MAX = 64;
logic [4-1:0] fclk ; //[0]-125MHz, [1]-250MHz, [2]-50MHz, [3]-200MHz
logic [4-1:0] frstn;

logic [16-1:0] par_dat;

logic          daisy_trig;
logic [ 3-1:0] daisy_mode;
logic          trig_ext;
logic          trig_output_sel;
logic          trig_asg_out;
logic [ 4-1:0] trig_ext_asg01;



// PLL signals
logic                 adc_clk_in;
logic                 pll_adc_clk;
logic                 pll_dac_clk_1x;
logic                 pll_dac_clk_2x;
logic                 pll_dac_clk_2p;
logic                 pll_ser_clk;
logic                 pll_pwm_clk;
logic                 pll_locked;
logic                 pll_locked_r;
logic                 fpll_locked_r,fpll_locked_r2,fpll_locked_r3;

logic   [16-1:0]      rst_cnt = 'h0;
logic                 rst_after_locked;
logic                 rstn_pll;

// fast serial signals
logic                 ser_clk ;
// PWM clock and reset
logic                 pwm_clk ;
logic                 pwm_rstn;

// ADC clock/reset
logic                 adc_clk;
logic                 adc_rstn;
logic                 adc_clk_daisy;
logic                 scope_trigo;

//CAN
logic                 CAN0_rx, CAN0_tx;
logic                 CAN1_rx, CAN1_tx;
logic                 can_on;


// stream bus type
localparam type SBA_T = logic signed [ADW-1:0];  // acquire
localparam type SBG_T = logic signed [ 14-1:0];  // generate

SBA_T [MNA-1:0]          adc_dat;

// DAC signals
logic                    dac_clk_1x;
logic                    dac_clk_2x;
logic                    dac_clk_2p;
logic                    dac_axi_clk;
logic                    dac_rst;
logic                    dac_axi_rstn;

logic        [14-1:0] dac_dat_a, dac_dat_b;
logic        [14-1:0] dac_a    , dac_b    ;
logic signed [15-1:0] dac_a_sum, dac_b_sum;

// ============================================================
// Pulse-control v2 signals
// ============================================================
localparam int PC_MAX_GPIO_PULSES = 16;
localparam int PC_MAX_DAC_SEGMENTS = 32;
localparam int PC_PULSE_FIFO_DEPTH = 1024;
localparam int PC_GROUP_FIFO_DEPTH = 256;

// Runtime control.
logic pc_measurement_enable;
logic pc_feedback_enable;
logic pc_hold_integrator;
logic pc_marker_enable;
logic pc_gpio_enable;
logic pc_dac2_enable;
logic pc_use_first_target;
logic pc_pulse_polarity_negative;

// FIFO logging is deliberately decoupled from real-time measurement/group/PID.
logic pc_fifo_continuous_log_enable;
logic pc_pulse_snapshot_arm;
logic pc_pulse_fifo_push;
logic [319:0] pc_pulse_fifo_write_data;
logic pc_group_fifo_push;

logic pc_clear_state;
logic pc_clear_flags;
logic pc_clear_fifos;
logic pc_dac2_apply;
logic pc_manual_group_boundary;
logic pc_pulse_fifo_pop;
logic pc_group_fifo_pop;
logic pc_gpio_apply_toggle;

// Measurement configuration.
logic [15:0] pc_threshold_fraction_q16;
logic [15:0] pc_target_height;
logic [8:0]  pc_pre_baseline_samples;
logic [8:0]  pc_post_baseline_samples;
logic [31:0] pc_min_pulse_samples;
logic [31:0] pc_max_pulse_samples;
logic [15:0] pc_adc_saturation_limit;

// Grouping configuration.
logic [7:0]  pc_expected_pulses_per_group;
logic [7:0]  pc_min_valid_pulses_per_group;
logic [1:0]  pc_group_source;
logic [31:0] pc_external_trigger_delay_cycles;
logic [15:0] pc_external_trigger_min_high_points;

// Reference/PID pulse selection, relative to REFERENCE_START.
logic [4:0]  pc_reference_target_start_index;
logic [5:0]  pc_reference_target_pulse_count;

// PI / plant-calibration configuration.
logic signed [31:0] pc_target_area;
logic [31:0] pc_deadband_absolute;
logic [15:0] pc_deadband_fraction_q16;
logic pc_deadband_fraction_mode;
logic [31:0] pc_kp_q16;
logic [31:0] pc_ki_q31_per_update;
logic [31:0] pc_kd_q16_per_update;
logic [31:0] pc_area_to_dac_gain_q24;
logic pc_correction_sign_negative;
logic [15:0] pc_max_dac_step_counts;
logic signed [15:0] pc_dac_high_min_counts;
logic signed [15:0] pc_dac_high_max_counts;
logic signed [31:0] pc_integral_min_area;
logic signed [31:0] pc_integral_max_area;
logic [15:0] pc_integral_limit_fraction_q16;
logic pc_integral_fraction_mode;

// GPIO shadow configuration, written at 125 MHz and captured at 200 MHz.
logic [31:0] pc_gpio_period_cycles_shadow;
logic [4:0]  pc_gpio_pulse_count_shadow;
logic [31:0] pc_gpio_pulse_start_shadow [0:PC_MAX_GPIO_PULSES-1];
logic [31:0] pc_gpio_pulse_end_shadow   [0:PC_MAX_GPIO_PULSES-1];

// OUT2 shadow configuration.
logic [31:0] pc_dac2_period_cycles_shadow;
logic [5:0]  pc_dac2_segment_count_shadow;
logic signed [15:0] pc_dac2_low_counts_shadow;
logic signed [15:0] pc_dac2_high_counts_shadow;
logic [15:0] pc_dac2_segment_level_shadow [0:PC_MAX_DAC_SEGMENTS-1];
logic [31:0] pc_dac2_segment_duration_shadow [0:PC_MAX_DAC_SEGMENTS-1];

// GPIO generator.  Its period no longer crosses into the ADC domain for
// grouping; experimental groups are normally driven by external DIO0_P.
logic pc_gpio_signal;
logic pc_gpio_config_valid;
logic [31:0] pc_gpio_active_period_cycles;
logic [4:0]  pc_gpio_active_pulse_count;
logic [31:0] pc_gpio_cycle_counter;

// GPIO active-configuration readback crosses 200 MHz -> 125 MHz only for
// software/status readback.  The source values are held stable between APPLY
// events; two-stage sampling removes the direct asynchronous feed into the
// ADC-domain register mux.  Python already polls until the synchronized
// readback matches the requested configuration.
(* ASYNC_REG = "TRUE" *) logic [31:0] pc_gpio_active_period_sync1;
(* ASYNC_REG = "TRUE" *) logic [31:0] pc_gpio_active_period_sync2;
(* ASYNC_REG = "TRUE" *) logic [4:0]  pc_gpio_active_pulse_count_sync1;
(* ASYNC_REG = "TRUE" *) logic [4:0]  pc_gpio_active_pulse_count_sync2;
(* ASYNC_REG = "TRUE" *) logic [1:0]  pc_gpio_config_valid_sync;

// External-trigger enable is first registered in the 125 MHz source domain.
// This prevents combinational logic (measurement_enable && group_source)
// immediately before the 200 MHz synchronizer, which Vivado correctly flags
// as CDC-10.
logic pc_external_trigger_enable_adc;

// External DIO0_P trigger: asynchronous input -> 200 MHz synchronizer/delay
// -> toggle-event CDC into the 125 MHz ADC/control domain.
logic pc_ext_trigger_event_toggle_200;
logic pc_ext_trigger_overrun_toggle_200;
logic [31:0] pc_ext_trigger_active_delay_200;
logic [15:0] pc_ext_trigger_active_min_high_200;
(* ASYNC_REG = "TRUE" *) logic [1:0] pc_ext_trigger_event_sync;
(* ASYNC_REG = "TRUE" *) logic [1:0] pc_ext_trigger_overrun_sync;
logic pc_ext_trigger_event_seen;
logic pc_ext_trigger_overrun_seen_toggle;
logic pc_ext_trigger_boundary_pulse;
logic pc_ext_trigger_overrun_pulse;
logic [31:0] pc_ext_trigger_count;
(* ASYNC_REG = "TRUE" *) logic [31:0] pc_ext_trigger_delay_sync1;
(* ASYNC_REG = "TRUE" *) logic [31:0] pc_ext_trigger_delay_sync2;
(* ASYNC_REG = "TRUE" *) logic [15:0] pc_ext_trigger_min_high_sync1;
(* ASYNC_REG = "TRUE" *) logic [15:0] pc_ext_trigger_min_high_sync2;

// ADC measurement outputs.
logic pc_measurement_config_valid;
logic pc_measurement_busy;
logic signed [13:0] pc_bpre;
logic signed [13:0] pc_bpost;
logic signed [13:0] pc_threshold;
logic signed [13:0] pc_threshold_live;
logic signed [13:0] pc_peak_raw;
logic [15:0] pc_peak_height;
logic [31:0] pc_pulse_duration_samples;
logic signed [31:0] pc_measured_area;
logic pc_first_crossing_valid;
logic pc_falling_edge_valid;
logic pc_measurement_valid;
logic pc_invalid_pulse_valid;
logic [7:0] pc_invalid_reason;

// Hardware timestamp.
logic [63:0] pc_timestamp;

// OUT2 generator / feedback application.
logic signed [13:0] pc_dac2_out;
logic pc_dac2_period_boundary;
logic pc_dac2_config_valid;
logic signed [15:0] pc_correction_active;
logic signed [15:0] pc_effective_dac_high;
logic signed [15:0] pc_nominal_dac_high;
logic signed [15:0] pc_nominal_dac_low;
logic [31:0] pc_dac2_active_period_cycles;
logic [5:0]  pc_dac2_active_segment_count;
logic [4:0]  pc_dac2_active_segment_index;
logic [15:0] pc_dac2_active_segment_level;

// Selected group boundary.
logic pc_group_boundary_selected;
logic pc_group_boundary_starts_group;

// Group aggregator outputs.
logic pc_pulse_record_valid;
logic [319:0] pc_pulse_record;
logic pc_group_ready_valid;
logic [63:0] pc_group_timestamp;
logic [31:0] pc_group_id;
logic [7:0] pc_group_detected;
logic [7:0] pc_group_valid;
logic [7:0] pc_group_invalid;
logic pc_group_expected_mismatch;
logic pc_group_overflow;
logic pc_boundary_overrun;
logic pc_group_valid_for_feedback;
logic signed [31:0] pc_group_mean_area;
logic [31:0] pc_total_valid_measurements;
logic [31:0] pc_total_invalid_measurements;
logic [31:0] pc_total_groups;

// PC06 reference selector.  This is the ONLY source used by PID feedback.
// The existing group aggregator is retained as the coherent pulse-record
// formatter and for legacy/full-group diagnostics.
logic pc_reference_config_valid;
logic pc_reference_window_active;
logic pc_reference_ready_valid;
logic [63:0] pc_reference_timestamp;
logic [31:0] pc_reference_id;
logic [7:0] pc_reference_detected;
logic [7:0] pc_reference_valid;
logic [7:0] pc_reference_invalid;
logic pc_reference_valid_for_feedback;
logic signed [31:0] pc_reference_sum_area;
logic pc_reference_sum_overflow;
logic pc_reference_incomplete;
logic pc_reference_selected_record_valid;
logic [319:0] pc_reference_selected_record;
logic pc_reference_snapshot_record_valid;
logic [319:0] pc_reference_snapshot_record;
logic pc_reference_snapshot_done;
logic [31:0] pc_active_reference_id;
logic [5:0] pc_active_reference_selected_count;
logic pc_reference_marker;

// PI controller outputs and group-result metadata.
logic pc_controller_config_valid;
logic pc_target_latched;
logic signed [31:0] pc_target_area_active;
logic signed [31:0] pc_area_error;
logic signed [31:0] pc_p_term_area;
logic signed [31:0] pc_i_term_area;
logic signed [31:0] pc_d_term_area;
logic signed [15:0] pc_correction_pending;
logic pc_feedback_update_valid;
logic [31:0] pc_update_count;
logic pc_correction_high_sat;
logic pc_correction_low_sat;
logic pc_integral_high_sat;
logic pc_integral_low_sat;
logic [31:0] pc_processing_latency_cycles;
logic pc_group_result_valid;
logic [63:0] pc_result_timestamp;
logic [31:0] pc_result_group_id;
logic [7:0] pc_result_detected;
logic [7:0] pc_result_valid_pulses;
logic [7:0] pc_result_invalid_pulses;
logic pc_result_expected_mismatch;
logic pc_result_group_overflow;
logic pc_result_boundary_overrun;
logic pc_result_group_valid_for_feedback;
logic signed [31:0] pc_result_mean_area;
logic pc_result_deadband;
logic pc_result_feedback_applied;

// Debug marker now routed to DIO2_P, not OUT2.
logic pc_debug_marker;
logic [31:0] pc_last_marker_cycles;
logic pc_marker_timeout;

// FIFO signals.
logic pc_pulse_fifo_full;
logic pc_pulse_fifo_empty;
logic [10:0] pc_pulse_fifo_level;
logic [31:0] pc_pulse_fifo_overflow_count;
logic [319:0] pc_pulse_fifo_head;

logic pc_group_fifo_full;
logic pc_group_fifo_empty;
logic [8:0] pc_group_fifo_level;
logic [31:0] pc_group_fifo_overflow_count;
logic [415:0] pc_group_fifo_head;
logic [415:0] pc_group_record;

// ASG
SBG_T [2-1:0]            asg_dat;

// configuration
logic [2-1:0]            digital_loop;

// system bus
sys_bus_if   ps_sys      (.clk (fclk[0]), .rstn (frstn[0]));
sys_bus_if   sys [8-1:0] (.clk (adc_clk), .rstn (adc_rstn));

// GPIO interface
gpio_if #(.DW (3*GDW)) gpio ();

// AXI masters
axi_sys_if axi0_sys (.clk(adc_clk    ), .rstn(adc_rstn    ));
axi_sys_if axi1_sys (.clk(adc_clk    ), .rstn(adc_rstn    ));
axi_sys_if axi2_sys (.clk(dac_axi_clk), .rstn(dac_axi_rstn));
axi_sys_if axi3_sys (.clk(dac_axi_clk), .rstn(dac_axi_rstn));
////////////////////////////////////////////////////////////////////////////////
// PLL (clock and reset)
////////////////////////////////////////////////////////////////////////////////

// diferential clock input
IBUFDS i_clk (.I (adc_clk_i[1]), .IB (adc_clk_i[0]), .O (adc_clk_in));  // differential clock input

assign rstn_pll = frstn[0] & ~(!fpll_locked_r2 && fpll_locked_r3);
red_pitaya_pll pll (
  // inputs
  .clk         (adc_clk_in),  // clock
  .rstn        (rstn_pll  ),  // reset - active low
  // output clocks
  .clk_adc     (pll_adc_clk   ),  // ADC clock
  .clk_dac_1x  (pll_dac_clk_1x),  // DAC clock 125MHz
  .clk_dac_2x  (pll_dac_clk_2x),  // DAC clock 250MHz
  .clk_dac_2p  (pll_dac_clk_2p),  // DAC clock 250MHz -45DGR
  .clk_ser     (pll_ser_clk   ),  // fast serial clock
  .clk_pdm     (pll_pwm_clk   ),  // PWM clock
  // status outputs
  .pll_locked  (pll_locked    )
);

BUFG bufg_adc_clk     (.O (adc_clk    ), .I (pll_adc_clk   ));
BUFG bufg_dac_clk_1x  (.O (dac_clk_1x ), .I (pll_dac_clk_1x));
BUFG bufg_dac_clk_2x  (.O (dac_clk_2x ), .I (pll_dac_clk_2x));
BUFG bufg_dac_axi_clk (.O (dac_axi_clk), .I (pll_dac_clk_2x));

BUFG bufg_dac_clk_2p (.O (dac_clk_2p), .I (pll_dac_clk_2p));
BUFG bufg_ser_clk    (.O (ser_clk   ), .I (pll_ser_clk   ));
BUFG bufg_pwm_clk    (.O (pwm_clk   ), .I (pll_pwm_clk   ));

always @(posedge fclk[0]) begin
  fpll_locked_r   <= pll_locked;
  fpll_locked_r2  <= fpll_locked_r;
  fpll_locked_r3  <= fpll_locked_r2;
end

always @(posedge adc_clk) begin
  pll_locked_r      <= pll_locked;
  if ((pll_locked && !pll_locked_r) || rst_cnt > 0) begin // some clk cycles after rising edge of pll_locked
    if (rst_cnt < RST_MAX)
      rst_cnt <= rst_cnt + 1;
    else 
      rst_cnt <= 'h0;
  end else begin
    if (~pll_locked) begin
      rst_cnt <= 'h0;
    end
  end
end

assign rst_after_locked = |rst_cnt;
// ADC reset (active low)
always @(posedge adc_clk)
adc_rstn     <=  frstn[0] & ~rst_after_locked;

// DAC reset (active high)
always @(posedge dac_clk_1x)
dac_rst      <= ~frstn[0] |  rst_after_locked;

// DAC AXI reset (active low)
always @(posedge dac_axi_clk)
dac_axi_rstn <=  frstn[0] & ~rst_after_locked;

// PWM reset (active low)
always @(posedge pwm_clk)
pwm_rstn     <=  frstn[0] & ~rst_after_locked;

////////////////////////////////////////////////////////////////////////////////
//  Connections to PS
////////////////////////////////////////////////////////////////////////////////

wire scope_irq;
wire [1:0] scope_irq_ch;

red_pitaya_ps ps (
  .FIXED_IO_mio       (  FIXED_IO_mio                ),
  .FIXED_IO_ps_clk    (  FIXED_IO_ps_clk             ),
  .FIXED_IO_ps_porb   (  FIXED_IO_ps_porb            ),
  .FIXED_IO_ps_srstb  (  FIXED_IO_ps_srstb           ),
  .FIXED_IO_ddr_vrn   (  FIXED_IO_ddr_vrn            ),
  .FIXED_IO_ddr_vrp   (  FIXED_IO_ddr_vrp            ),
  // DDR
  .DDR_addr      (DDR_addr    ),
  .DDR_ba        (DDR_ba      ),
  .DDR_cas_n     (DDR_cas_n   ),
  .DDR_ck_n      (DDR_ck_n    ),
  .DDR_ck_p      (DDR_ck_p    ),
  .DDR_cke       (DDR_cke     ),
  .DDR_cs_n      (DDR_cs_n    ),
  .DDR_dm        (DDR_dm      ),
  .DDR_dq        (DDR_dq      ),
  .DDR_dqs_n     (DDR_dqs_n   ),
  .DDR_dqs_p     (DDR_dqs_p   ),
  .DDR_odt       (DDR_odt     ),
  .DDR_ras_n     (DDR_ras_n   ),
  .DDR_reset_n   (DDR_reset_n ),
  .DDR_we_n      (DDR_we_n    ),
  // system signals
  .fclk_clk_o    (fclk        ),
  .fclk_rstn_o   (frstn       ),
  // ADC analog inputs
  .vinp_i        (vinp_i      ),
  .vinn_i        (vinn_i      ),
  .scope_irq_i   (scope_irq   ),
  .scope_irq_ch1_i(scope_irq_ch[0]),
  .scope_irq_ch2_i(scope_irq_ch[1]),
  .scope_irq_ch3_i(1'b0),
  .scope_irq_ch4_i(1'b0),
  // CAN0
  .CAN0_rx       (CAN0_rx     ),
  .CAN0_tx       (CAN0_tx     ),
  // CAN1
  .CAN1_rx       (CAN1_rx     ),
  .CAN1_tx       (CAN1_tx     ),
  // GPIO
  .gpio          (gpio),
  // system read/write channel
  .bus           (ps_sys      ),
  // AXI masters

  .axi0_sys      (axi0_sys    ),
  .axi1_sys      (axi1_sys    ),
  .axi2_sys      (axi2_sys    ),
  .axi3_sys      (axi3_sys    )
);
////////////////////////////////////////////////////////////////////////////////
// system bus decoder & multiplexer (it breaks memory addresses into 8 regions)
////////////////////////////////////////////////////////////////////////////////

sys_bus_interconnect #(
  .SN (8),
  .SW (20)
) sys_bus_interconnect (
  .pll_locked_i(pll_locked),
  .bus_m (ps_sys),
  .bus_s (sys)
);


`ifndef SCOPE_ONLY

assign daisy_trig = |par_dat;
assign trig_ext   = gpio.i[GDW] & ~(daisy_mode[0] & daisy_trig);
////////////////////////////////////////////////////////////////////////////////
// Analog mixed signals (PDM analog outputs)
////////////////////////////////////////////////////////////////////////////////

logic [4-1:0] [8-1:0] pdm_cfg;

red_pitaya_ams i_ams (
  // power test
  .clk_i           (adc_clk ),  // clock
  .rstn_i          (adc_rstn),  // reset - active low
  // PWM configuration
  .dac_a_o         (pdm_cfg[0]),
  .dac_b_o         (pdm_cfg[1]),
  .dac_c_o         (pdm_cfg[2]),
  .dac_d_o         (pdm_cfg[3]),
  // System bus
  .sys_addr        (sys[4].addr ),
  .sys_wdata       (sys[4].wdata),
  .sys_wen         (sys[4].wen  ),
  .sys_ren         (sys[4].ren  ),
  .sys_rdata       (sys[4].rdata),
  .sys_err         (sys[4].err  ),
  .sys_ack         (sys[4].ack  )
);

red_pitaya_pdm pdm (
  // system signals
  .clk   (adc_clk ),
  .rstn  (adc_rstn),
  // configuration
  .cfg   (pdm_cfg),
  .ena      (1'b1),
  .rng      (8'd255),
  // PWM outputs
  .pdm (dac_pwm_o)
);

////////////////////////////////////////////////////////////////////////////////
// ADC IO
////////////////////////////////////////////////////////////////////////////////

ODDR i_adc_clk_p ( .Q(adc_clk_o[0]), .D1(1'b1), .D2(1'b0), .C(adc_clk_daisy), .CE(1'b1), .R(1'b0), .S(1'b0));
ODDR i_adc_clk_n ( .Q(adc_clk_o[1]), .D1(1'b0), .D2(1'b1), .C(adc_clk_daisy), .CE(1'b1), .R(1'b0), .S(1'b0));

assign adc_cdcs_o = 1'b1 ;

logic [2-1:0] [ADW-1:0] adc_dat_raw;

// IO block registers should be used here
// lowest 2 bits reserved for 16bit ADC

assign adc_dat_raw[0] = adc_dat_i[0][16-1 -: ADW];
assign adc_dat_raw[1] = adc_dat_i[1][16-1 -: ADW];

// transform into 2's complement (negative slope)
always @(posedge adc_clk) begin
  adc_dat[0] <= digital_loop[0] ? dac_a : {adc_dat_raw[0][ADW-1], ~adc_dat_raw[0][ADW-2:0]};
  adc_dat[1] <= digital_loop[0] ? dac_b : {adc_dat_raw[1][ADW-1], ~adc_dat_raw[1][ADW-2:0]};
end
// ============================================================
// Pulse-control v2: GPIO timing, IN1 measurement, grouping, PI, OUT2, logging
// ============================================================

// Free-running 125 MHz hardware timestamp.  Do not reset this on a software
// clear-state command so CSV time remains monotonic for long acquisitions.
always_ff @(posedge adc_clk) begin
    if (!adc_rstn)
        pc_timestamp <= 64'd0;
    else
        pc_timestamp <= pc_timestamp + 1'b1;
end

// ------------------------ 200 MHz GPIO generator ------------------------
gpio_sequence_generator_v3 #(
    .MAX_PULSES (PC_MAX_GPIO_PULSES)
) i_gpio_sequence_generator_v3 (
    .clk_200mhz_i                         (fclk[3]),
    .rstn_i                               (frstn[3]),
    .enable_async_i                       (pc_gpio_enable),
    .apply_toggle_async_i                 (pc_gpio_apply_toggle),
    .period_cycles_shadow_async_i         (pc_gpio_period_cycles_shadow),
    .pulse_count_shadow_async_i           (pc_gpio_pulse_count_shadow),
    .pulse_start_cycles_shadow_async_i    (pc_gpio_pulse_start_shadow),
    .pulse_end_cycles_shadow_async_i      (pc_gpio_pulse_end_shadow),
    .gpio_out_o                           (pc_gpio_signal),
    .period_toggle_o                      (),
    .config_valid_o                       (pc_gpio_config_valid),
    .active_period_cycles_o               (pc_gpio_active_period_cycles),
    .active_pulse_count_o                 (pc_gpio_active_pulse_count),
    .cycle_counter_o                      (pc_gpio_cycle_counter)
);

// Register the external-trigger enable condition in its source clock domain,
// and safely sample the GPIO generator's active readback into the ADC domain.
// No control decision depends on the synchronized GPIO readback values.
always_ff @(posedge adc_clk) begin
    if (!adc_rstn) begin
        pc_external_trigger_enable_adc      <= 1'b0;
        pc_gpio_active_period_sync1         <= 32'd0;
        pc_gpio_active_period_sync2         <= 32'd0;
        pc_gpio_active_pulse_count_sync1    <= 5'd0;
        pc_gpio_active_pulse_count_sync2    <= 5'd0;
        pc_gpio_config_valid_sync           <= 2'b00;
    end
    else begin
        pc_external_trigger_enable_adc <=
            pc_measurement_enable && (pc_group_source == 2'd0);

        pc_gpio_active_period_sync1      <= pc_gpio_active_period_cycles;
        pc_gpio_active_period_sync2      <= pc_gpio_active_period_sync1;
        pc_gpio_active_pulse_count_sync1 <= pc_gpio_active_pulse_count;
        pc_gpio_active_pulse_count_sync2 <= pc_gpio_active_pulse_count_sync1;
        pc_gpio_config_valid_sync        <=
            {pc_gpio_config_valid_sync[0], pc_gpio_config_valid};
    end
end

// -------------------- External DIO0_P group trigger ---------------------
// DIO0_P is sampled and delayed at 200 MHz (5 ns resolution).  Only a toggle
// event crosses back into the 125 MHz ADC domain; the old internal GPIO-period
// CDC is intentionally no longer used for group timing.
external_group_trigger_v3 i_external_group_trigger_v3 (
    .clk_200mhz_i                    (fclk[3]),
    .rstn_i                          (frstn[3]),
    .enable_async_i                  (pc_external_trigger_enable_adc),
    .trigger_async_i                 (exp_p_in[0]),
    .delay_cycles_shadow_async_i     (pc_external_trigger_delay_cycles),
    .min_high_points_shadow_async_i  (pc_external_trigger_min_high_points),
    .event_toggle_o                  (pc_ext_trigger_event_toggle_200),
    .overrun_toggle_o                (pc_ext_trigger_overrun_toggle_200),
    .active_delay_cycles_o           (pc_ext_trigger_active_delay_200),
    .active_min_high_points_o        (pc_ext_trigger_active_min_high_200)
);

// Safe event-toggle CDC into the ADC/control clock domain.
// The active delay is slow configuration readback; it is double-sampled and
// software waits for it to settle while GROUP_SOURCE=manual.
always_ff @(posedge adc_clk) begin
    if (!adc_rstn) begin
        pc_ext_trigger_event_sync         <= 2'b00;
        pc_ext_trigger_overrun_sync       <= 2'b00;
        pc_ext_trigger_event_seen         <= 1'b0;
        pc_ext_trigger_overrun_seen_toggle<= 1'b0;
        pc_ext_trigger_boundary_pulse     <= 1'b0;
        pc_ext_trigger_overrun_pulse      <= 1'b0;
        pc_ext_trigger_count              <= 32'd0;
        pc_ext_trigger_delay_sync1        <= 32'd0;
        pc_ext_trigger_delay_sync2        <= 32'd0;
        pc_ext_trigger_min_high_sync1     <= 16'd0;
        pc_ext_trigger_min_high_sync2     <= 16'd0;
    end
    else begin
        pc_ext_trigger_event_sync <=
            {pc_ext_trigger_event_sync[0], pc_ext_trigger_event_toggle_200};
        pc_ext_trigger_overrun_sync <=
            {pc_ext_trigger_overrun_sync[0], pc_ext_trigger_overrun_toggle_200};
        pc_ext_trigger_delay_sync1 <= pc_ext_trigger_active_delay_200;
        pc_ext_trigger_delay_sync2 <= pc_ext_trigger_delay_sync1;
        pc_ext_trigger_min_high_sync1 <= pc_ext_trigger_active_min_high_200;
        pc_ext_trigger_min_high_sync2 <= pc_ext_trigger_min_high_sync1;

        pc_ext_trigger_boundary_pulse <=
            pc_ext_trigger_event_sync[1] ^ pc_ext_trigger_event_seen;
        pc_ext_trigger_overrun_pulse <=
            pc_ext_trigger_overrun_sync[1] ^ pc_ext_trigger_overrun_seen_toggle;

        if (pc_ext_trigger_event_sync[1] ^ pc_ext_trigger_event_seen) begin
            pc_ext_trigger_event_seen <= pc_ext_trigger_event_sync[1];
            if (!pc_clear_state)
                pc_ext_trigger_count <= pc_ext_trigger_count + 1'b1;
        end
        if (pc_clear_state)
            pc_ext_trigger_count <= 32'd0;
        if (pc_ext_trigger_overrun_sync[1] ^ pc_ext_trigger_overrun_seen_toggle)
            pc_ext_trigger_overrun_seen_toggle <= pc_ext_trigger_overrun_sync[1];
    end
end

// -------------------------- IN1 measurement -----------------------------
pulse_measurement_v3 #(
    .ADC_WIDTH                (14),
    .MAX_BASELINE_SAMPLES     (256)
) i_pulse_measurement_v3 (
    .clk_i                    (adc_clk),
    .rstn_i                   (adc_rstn),
    .clear_state_i            (pc_clear_state),
    .measurement_enable_i     (pc_measurement_enable),
    .adc_i                    (adc_dat[0]),
    .threshold_fraction_q16_i (pc_threshold_fraction_q16),
    .target_height_i          (pc_target_height),
    .pre_baseline_samples_i   (pc_pre_baseline_samples),
    .post_baseline_samples_i  (pc_post_baseline_samples),
    .min_pulse_samples_i      (pc_min_pulse_samples),
    .max_pulse_samples_i      (pc_max_pulse_samples),
    .adc_saturation_limit_i   (pc_adc_saturation_limit),
    .pulse_polarity_negative_i(pc_pulse_polarity_negative),
    .config_valid_o           (pc_measurement_config_valid),
    .busy_o                   (pc_measurement_busy),
    .bpre_o                   (pc_bpre),
    .bpost_o                  (pc_bpost),
    .threshold_o              (pc_threshold),
    .threshold_live_o         (pc_threshold_live),
    .peak_raw_o               (pc_peak_raw),
    .peak_height_o            (pc_peak_height),
    .pulse_duration_samples_o (pc_pulse_duration_samples),
    .measured_pulse_area_o    (pc_measured_area),
    .first_crossing_valid_o   (pc_first_crossing_valid),
    .falling_edge_valid_o     (pc_falling_edge_valid),
    .measurement_valid_o      (pc_measurement_valid),
    .invalid_pulse_valid_o    (pc_invalid_pulse_valid),
    .invalid_reason_o         (pc_invalid_reason)
);

// -------------------------- OUT2 sequence -------------------------------
dac2_sequence_generator_v3 #(
    .MAX_SEGMENTS (PC_MAX_DAC_SEGMENTS)
) i_dac2_sequence_generator_v3 (
    .clk_i                               (adc_clk),
    .rstn_i                              (adc_rstn),
    .enable_i                            (pc_dac2_enable),
    .apply_i                             (pc_dac2_apply),
    .period_cycles_shadow_i              (pc_dac2_period_cycles_shadow),
    .segment_count_shadow_i              (pc_dac2_segment_count_shadow),
    .low_counts_shadow_i                 (pc_dac2_low_counts_shadow),
    .high_counts_shadow_i                (pc_dac2_high_counts_shadow),
    .segment_level_q16_shadow_i          (pc_dac2_segment_level_shadow),
    .segment_duration_cycles_shadow_i    (pc_dac2_segment_duration_shadow),
    .correction_pending_i                (pc_correction_pending),
    .high_min_counts_i                   (pc_dac_high_min_counts),
    .high_max_counts_i                   (pc_dac_high_max_counts),
    .dac_out_o                           (pc_dac2_out),
    .period_boundary_o                   (pc_dac2_period_boundary),
    .config_valid_o                      (pc_dac2_config_valid),
    .correction_active_o                 (pc_correction_active),
    .effective_high_counts_o             (pc_effective_dac_high),
    .nominal_high_counts_o               (pc_nominal_dac_high),
    .nominal_low_counts_o                (pc_nominal_dac_low),
    .active_period_cycles_o              (pc_dac2_active_period_cycles),
    .active_segment_count_o              (pc_dac2_active_segment_count),
    .active_segment_index_o              (pc_dac2_active_segment_index),
    .active_segment_level_q16_o          (pc_dac2_active_segment_level)
);

// Select what defines one feedback/acquisition group.
//   0: delayed external DIO0_P trigger (normal experiment)
//   1: DAC2 period boundary (debug/fallback, same 125 MHz domain)
//   2: manual command (safe idle)
// External-trigger mode uses start-on-first-boundary semantics so no implicit
// pre-trigger/empty group is emitted.
always_comb begin
    pc_group_boundary_starts_group = 1'b0;
    case (pc_group_source)
        2'd0: begin
            pc_group_boundary_selected = pc_ext_trigger_boundary_pulse;
            pc_group_boundary_starts_group = 1'b1;
        end
        2'd1: pc_group_boundary_selected = pc_dac2_period_boundary;
        2'd2: pc_group_boundary_selected = pc_manual_group_boundary;
        default: begin
            pc_group_boundary_selected = pc_ext_trigger_boundary_pulse;
            pc_group_boundary_starts_group = 1'b1;
        end
    endcase
end

// ------------------------ Per-group aggregation -------------------------
pulse_group_aggregator_v3 #(
    .MAX_PULSES_PER_GROUP (32)
) i_pulse_group_aggregator_v3 (
    .clk_i                          (adc_clk),
    .rstn_i                         (adc_rstn),
    .clear_state_i                  (pc_clear_state),
    .timestamp_i                    (pc_timestamp),
    .boundary_event_i               (pc_group_boundary_selected),
    // External trigger delay is already applied at 200 MHz.  Keep the legacy
    // aggregator delay hard-zero to avoid a second, confusing 8 ns delay.
    .boundary_delay_samples_i       (16'd0),
    .boundary_starts_group_i        (pc_group_boundary_starts_group),
    .measurement_busy_i             (pc_measurement_busy),
    .measurement_valid_i            (pc_measurement_valid),
    .invalid_pulse_valid_i          (pc_invalid_pulse_valid),
    .invalid_reason_i               (pc_invalid_reason),
    .bpre_i                         (pc_bpre),
    .bpost_i                        (pc_bpost),
    .threshold_i                    (pc_threshold),
    .peak_raw_i                     (pc_peak_raw),
    .peak_height_i                  (pc_peak_height),
    .duration_samples_i             (pc_pulse_duration_samples),
    .area_i                         (pc_measured_area),
    .expected_pulses_per_group_i    (pc_expected_pulses_per_group),
    .min_valid_pulses_per_group_i   (pc_min_valid_pulses_per_group),
    .correction_active_i            (pc_correction_active),
    .effective_dac_high_i           (pc_effective_dac_high),
    .pulse_record_valid_o           (pc_pulse_record_valid),
    .pulse_record_o                 (pc_pulse_record),
    .group_ready_valid_o            (pc_group_ready_valid),
    .group_timestamp_o              (pc_group_timestamp),
    .group_id_o                     (pc_group_id),
    .detected_pulses_o              (pc_group_detected),
    .valid_pulses_o                 (pc_group_valid),
    .invalid_pulses_o               (pc_group_invalid),
    .expected_mismatch_o            (pc_group_expected_mismatch),
    .group_overflow_o               (pc_group_overflow),
    .boundary_overrun_o             (pc_boundary_overrun),
    .group_valid_for_feedback_o     (pc_group_valid_for_feedback),
    .mean_area_o                    (pc_group_mean_area),
    .total_valid_measurements_o     (pc_total_valid_measurements),
    .total_invalid_measurements_o   (pc_total_invalid_measurements),
    .total_groups_o                 (pc_total_groups)
);

// ---------------------- PC06 reference selector ------------------------
// REFERENCE_START is the selected group-boundary pulse.  In normal external
// mode this is exactly the qualified + programmed-delay DIO0 event after the
// existing 200 -> 125 MHz toggle CDC.  DIO2_P is driven from the selector's
// reference_window_active output, so its rising edge is the same ADC-domain
// edge that resets pulse indexing for the PID reference.
pulse_reference_selector_v3 i_pulse_reference_selector_v3 (
    .clk_i                          (adc_clk),
    .rstn_i                         (adc_rstn),
    .clear_state_i                  (pc_clear_state),
    .timestamp_i                    (pc_timestamp),
    .reference_start_i              (pc_group_boundary_selected),
    .measurement_busy_i             (pc_measurement_busy),
    .target_start_index_i           (pc_reference_target_start_index),
    .target_pulse_count_i           (pc_reference_target_pulse_count),
    .pulse_record_valid_i           (pc_pulse_record_valid),
    .pulse_record_i                 (pc_pulse_record),
    .snapshot_arm_i                 (pc_pulse_snapshot_arm),
    .reference_window_active_o      (pc_reference_window_active),
    .config_valid_o                 (pc_reference_config_valid),
    .reference_ready_valid_o        (pc_reference_ready_valid),
    .reference_timestamp_o          (pc_reference_timestamp),
    .reference_id_o                 (pc_reference_id),
    .reference_detected_pulses_o    (pc_reference_detected),
    .reference_valid_pulses_o       (pc_reference_valid),
    .reference_invalid_pulses_o     (pc_reference_invalid),
    .reference_valid_for_feedback_o (pc_reference_valid_for_feedback),
    .reference_sum_area_o           (pc_reference_sum_area),
    .reference_sum_overflow_o       (pc_reference_sum_overflow),
    .reference_incomplete_o         (pc_reference_incomplete),
    .selected_pulse_record_valid_o  (pc_reference_selected_record_valid),
    .selected_pulse_record_o        (pc_reference_selected_record),
    .snapshot_pulse_record_valid_o  (pc_reference_snapshot_record_valid),
    .snapshot_pulse_record_o        (pc_reference_snapshot_record),
    .snapshot_done_o                (pc_reference_snapshot_done),
    .active_reference_id_o          (pc_active_reference_id),
    .active_selected_count_o        (pc_active_reference_selected_count)
);

assign pc_reference_marker = pc_reference_window_active;

// -------------------------- Pipelined PI --------------------------------
pulse_area_pid_controller_v3 i_pulse_area_pid_controller_v3 (
    .clk_i                              (adc_clk),
    .rstn_i                             (adc_rstn),
    .clear_state_i                      (pc_clear_state),
    .feedback_enable_i                  (pc_feedback_enable),
    .hold_integrator_i                  (pc_hold_integrator),
    .use_first_valid_group_as_target_i  (pc_use_first_target),
    .target_area_i                      (pc_target_area),
    .deadband_absolute_i                (pc_deadband_absolute),
    .deadband_fraction_q16_i            (pc_deadband_fraction_q16),
    .deadband_fraction_mode_i           (pc_deadband_fraction_mode),
    .kp_q16_i                           (pc_kp_q16),
    .ki_q31_per_update_i                (pc_ki_q31_per_update),
    .kd_q16_per_update_i                (pc_kd_q16_per_update),
    .area_to_dac_gain_q24_i             (pc_area_to_dac_gain_q24),
    .correction_sign_negative_i         (pc_correction_sign_negative),
    .max_dac_step_counts_i              (pc_max_dac_step_counts),
    .dac_high_min_counts_i              (pc_dac_high_min_counts),
    .dac_high_max_counts_i              (pc_dac_high_max_counts),
    .integral_min_area_i                (pc_integral_min_area),
    .integral_max_area_i                (pc_integral_max_area),
    .integral_fraction_mode_i           (pc_integral_fraction_mode),
    .integral_limit_fraction_q16_i      (pc_integral_limit_fraction_q16),
    .nominal_dac_high_counts_i          (pc_nominal_dac_high),
    .correction_active_i                (pc_correction_active),
    .group_ready_valid_i                (pc_reference_ready_valid),
    .group_timestamp_i                  (pc_reference_timestamp),
    .group_id_i                         (pc_reference_id),
    .detected_pulses_i                  (pc_reference_detected),
    .valid_pulses_i                     (pc_reference_valid),
    .invalid_pulses_i                   (pc_reference_invalid),
    .expected_mismatch_i                (1'b0),
    .group_overflow_i                   (pc_reference_sum_overflow),
    .boundary_overrun_i                 (1'b0),
    .group_valid_for_feedback_i         (pc_reference_valid_for_feedback),
    .mean_area_i                        (pc_reference_sum_area),
    .config_valid_o                     (pc_controller_config_valid),
    .target_latched_o                   (pc_target_latched),
    .target_area_active_o               (pc_target_area_active),
    .area_error_o                       (pc_area_error),
    .p_term_area_o                      (pc_p_term_area),
    .i_term_area_o                      (pc_i_term_area),
    .d_term_area_o                      (pc_d_term_area),
    .correction_pending_o               (pc_correction_pending),
    .feedback_update_valid_o            (pc_feedback_update_valid),
    .update_count_o                     (pc_update_count),
    .correction_high_sat_o              (pc_correction_high_sat),
    .correction_low_sat_o               (pc_correction_low_sat),
    .integral_high_sat_o                (pc_integral_high_sat),
    .integral_low_sat_o                 (pc_integral_low_sat),
    .last_processing_latency_cycles_o   (pc_processing_latency_cycles),
    .result_valid_o                     (pc_group_result_valid),
    .result_timestamp_o                 (pc_result_timestamp),
    .result_group_id_o                  (pc_result_group_id),
    .result_detected_pulses_o           (pc_result_detected),
    .result_valid_pulses_o              (pc_result_valid_pulses),
    .result_invalid_pulses_o            (pc_result_invalid_pulses),
    .result_expected_mismatch_o         (pc_result_expected_mismatch),
    .result_group_overflow_o            (pc_result_group_overflow),
    .result_boundary_overrun_o          (pc_result_boundary_overrun),
    .result_group_valid_for_feedback_o  (pc_result_group_valid_for_feedback),
    .result_mean_area_o                 (pc_result_mean_area),
    .result_deadband_o                  (pc_result_deadband),
    .result_feedback_applied_o          (pc_result_feedback_applied)
);

// -------------------------- Debug marker --------------------------------
pulse_debug_marker_v3 i_pulse_debug_marker_v3 (
    .clk_i                    (adc_clk),
    .rstn_i                   (adc_rstn),
    .clear_state_i            (pc_clear_state),
    .marker_enable_i          (pc_marker_enable),
    .first_crossing_valid_i   (pc_first_crossing_valid),
    .group_result_valid_i     (pc_group_result_valid),
    .marker_o                 (pc_debug_marker),
    .last_marker_cycles_o     (pc_last_marker_cycles),
    .timeout_seen_o           (pc_marker_timeout)
);

// ----------------------------- FIFOs ------------------------------------
// Real-time detector/reference/PID logic always keeps running.
//
// PC06 pulse FIFO semantics:
//   continuous=1 : log every target pulse actually selected for PID
//   continuous=0 : after COMMAND bit8, log ALL selected target pulses from
//                  exactly the next reference; selector auto-disarms.
// Linux never participates in the intra-reference pulse timing.
assign pc_pulse_fifo_push =
    pc_fifo_continuous_log_enable
        ? pc_reference_selected_record_valid
        : pc_reference_snapshot_record_valid;

assign pc_pulse_fifo_write_data =
    pc_fifo_continuous_log_enable
        ? pc_reference_selected_record
        : pc_reference_snapshot_record;

// The existing group-result FIFO is retained for optional continuous controller
// diagnostics.  Since the PID input is now the reference sum, these result
// records describe one completed PID reference, despite the legacy signal names.
assign pc_group_fifo_push =
    pc_group_result_valid && pc_fifo_continuous_log_enable;

pulse_sync_fifo_v3 #(
    .WIDTH (320),
    .DEPTH (PC_PULSE_FIFO_DEPTH)
) i_pulse_fifo_v3 (
    .clk_i              (adc_clk),
    .rstn_i             (adc_rstn),
    .clear_i            (pc_clear_fifos),
    .push_i             (pc_pulse_fifo_push),
    .push_data_i        (pc_pulse_fifo_write_data),
    .full_o             (pc_pulse_fifo_full),
    .pop_i              (pc_pulse_fifo_pop),
    .head_data_o        (pc_pulse_fifo_head),
    .empty_o            (pc_pulse_fifo_empty),
    .level_o            (pc_pulse_fifo_level),
    .overflow_count_o   (pc_pulse_fifo_overflow_count)
);

// group FIFO word map, word0 at [31:0]:
// 0/1 timestamp, 2 group id, 3 counts+flags, 4 mean, 5 target, 6 error,
// 7 P-area, 8 I-area, 9 D-area, 10 pending+active correction,
// 11 nominal+effective DAC high, 12 controller processing latency.
assign pc_group_record = {
    pc_processing_latency_cycles,
    {pc_nominal_dac_high, pc_effective_dac_high},
    {pc_correction_pending, pc_correction_active},
    pc_d_term_area,
    pc_i_term_area,
    pc_p_term_area,
    pc_area_error,
    pc_target_area_active,
    pc_result_mean_area,
    {{2{1'b0}}, pc_result_feedback_applied, pc_result_deadband,
     pc_result_boundary_overrun, pc_result_group_overflow,
     pc_result_expected_mismatch, pc_result_group_valid_for_feedback,
     pc_result_invalid_pulses, pc_result_valid_pulses, pc_result_detected},
    pc_result_group_id,
    pc_result_timestamp[63:32],
    pc_result_timestamp[31:0]
};

pulse_sync_fifo_v3 #(
    .WIDTH (416),
    .DEPTH (PC_GROUP_FIFO_DEPTH)
) i_group_fifo_v3 (
    .clk_i              (adc_clk),
    .rstn_i             (adc_rstn),
    .clear_i            (pc_clear_fifos),
    .push_i             (pc_group_fifo_push),
    .push_data_i        (pc_group_record),
    .full_o             (pc_group_fifo_full),
    .pop_i              (pc_group_fifo_pop),
    .head_data_o        (pc_group_fifo_head),
    .empty_o            (pc_group_fifo_empty),
    .level_o            (pc_group_fifo_level),
    .overflow_count_o   (pc_group_fifo_overflow_count)
);

////////////////////////////////////////////////////////////////////////////////
// DAC IO
////////////////////////////////////////////////////////////////////////////////

// OUT1 is intentionally held at zero in this pulse-control bitstream.
// OUT2 is exclusively owned by the runtime-programmable DAC2 sequence generator.
assign dac_a_sum = 15'sd0;
assign dac_b_sum = $signed({pc_dac2_out[13], pc_dac2_out});

// saturation
assign dac_a = (^dac_a_sum[15-1:15-2]) ? {dac_a_sum[15-1], {13{~dac_a_sum[15-1]}}} : dac_a_sum[14-1:0];
assign dac_b = (^dac_b_sum[15-1:15-2]) ? {dac_b_sum[15-1], {13{~dac_b_sum[15-1]}}} : dac_b_sum[14-1:0];

// output registers + signed to unsigned (also to negative slope)
always @(posedge dac_clk_1x)
begin // Loopback is for demonstration only. We avoid constraining for timing optimizations.
  dac_dat_a <= digital_loop[1] ? {adc_dat[0][ADW-1], ~adc_dat[0][ADW-2 -: 13]} : {dac_a[14-1], ~dac_a[14-2:0]};
  dac_dat_b <= digital_loop[1] ? {adc_dat[1][ADW-1], ~adc_dat[1][ADW-2 -: 13]} : {dac_b[14-1], ~dac_b[14-2:0]};
end

// DDR outputs
ODDR oddr_dac_clk          (.Q(dac_clk_o), .D1(1'b0     ), .D2(1'b1     ), .C(dac_clk_2p), .CE(1'b1), .R(1'b0   ), .S(1'b0));
ODDR oddr_dac_wrt          (.Q(dac_wrt_o), .D1(1'b0     ), .D2(1'b1     ), .C(dac_clk_2x), .CE(1'b1), .R(1'b0   ), .S(1'b0));
ODDR oddr_dac_sel          (.Q(dac_sel_o), .D1(1'b1     ), .D2(1'b0     ), .C(dac_clk_1x), .CE(1'b1), .R(dac_rst), .S(1'b0));
ODDR oddr_dac_rst          (.Q(dac_rst_o), .D1(dac_rst  ), .D2(dac_rst  ), .C(dac_clk_1x), .CE(1'b1), .R(1'b0   ), .S(1'b0));
ODDR oddr_dac_dat [14-1:0] (.Q(dac_dat_o), .D1(dac_dat_b), .D2(dac_dat_a), .C(dac_clk_1x), .CE(1'b1), .R(dac_rst), .S(1'b0));

////////////////////////////////////////////////////////////////////////////////
//  House Keeping
////////////////////////////////////////////////////////////////////////////////

logic [DWE-1: 0] exp_p_in ,  exp_n_in ;
logic [DWE-1: 0] exp_p_in_nontrigger;
logic [DWE-1: 0] exp_p_out,  exp_n_out;
logic [DWE-1: 0] exp_p_dir,  exp_n_dir;
logic [DWE-1: 0] exp_p_otr,  exp_n_otr;
logic [DWE-1: 0] exp_p_dtr,  exp_n_dtr;
logic [DWE-1: 0] exp_p_alt,  exp_n_alt;
logic [DWE-1: 0] exp_p_altr, exp_n_altr;
logic [DWE-1: 0] exp_p_altd, exp_n_altd;

red_pitaya_hk #(.DWE(DWE)) i_hk (
  // system signals
  .clk_i           (adc_clk    ),  // clock
  .rstn_i          (adc_rstn   ),  // reset - active low
  .fclk_i          (fclk[0]    ),  // clock
  .frstn_i         (frstn[0]   ),  // reset - active low

  // LED
  .led_o           (  led_o    ),  // LED output
  // global configuration
  .digital_loop    (digital_loop),
  .daisy_mode_o    (daisy_mode),
  // Expansion connector
  .exp_p_dat_i     (exp_p_in_nontrigger),  // DIO0_P reserved for external trigger
  .exp_p_dat_o     (exp_p_out),  // output data
  .exp_p_dir_o     (exp_p_dir),  // 1-output enable
  .exp_n_dat_i     (exp_n_in ),
  .exp_n_dat_o     (exp_n_out),
  .exp_n_dir_o     (exp_n_dir),
  .can_on_o        (can_on   ),
   // System bus
  .sys_addr        (sys[0].addr ),
  .sys_wdata       (sys[0].wdata),
  .sys_wen         (sys[0].wen  ),
  .sys_ren         (sys[0].ren  ),
  .sys_rdata       (sys[0].rdata),
  .sys_err         (sys[0].err  ),
  .sys_ack         (sys[0].ack  )
);


////////////////////////////////////////////////////////////////////////////////
// LED
////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
// GPIO
////////////////////////////////////////////////////////////////////////////////

assign trig_output_sel = daisy_mode[2] ? trig_asg_out : scope_trigo;

// Reserve DIO0_P as the dedicated qualified external-trigger input, DIO1_P for
// the existing 200 MHz programmable GPIO sequence, and DIO2_P for the PC06
// reference-window marker.  Do not change these pin roles.
// exp_p_altd bit0=0 below forces DIO0_P to input regardless of housekeeping.
assign exp_p_alt  = {{(DWE-3){1'b0}}, 3'b111};
assign exp_n_alt  = {{DWE-8{1'b0}},  can_on,  can_on, 5'h0, daisy_mode[1]  };

assign exp_p_altr = {{(DWE-3){1'b0}}, pc_reference_marker, pc_gpio_signal, 1'b0};
assign exp_n_altr = {{DWE-8{1'b0}}, CAN0_tx, CAN1_tx, 5'h0, trig_output_sel};

assign exp_p_altd = {{(DWE-3){1'b0}}, 3'b110};
assign exp_n_altd = {{DWE-8{1'b0}},   1'b1,   1'b1, 5'h0, 1'b1};

genvar GM;
generate
for(GM = 0 ; GM < DWE ; GM = GM + 1) begin : gpios
  assign exp_p_otr[GM] = exp_p_alt[GM] ? exp_p_altr[GM] : exp_p_out[GM];
  assign exp_n_otr[GM] = exp_n_alt[GM] ? exp_n_altr[GM] : exp_n_out[GM];

  assign exp_p_dtr[GM] = exp_p_alt[GM] ? exp_p_altd[GM] : exp_p_dir[GM];
  assign exp_n_dtr[GM] = exp_n_alt[GM] ? exp_n_altd[GM] : exp_n_dir[GM];
end
endgenerate

IOBUF i_iobufp [DWE-1:0] (.O(exp_p_in), .IO(exp_p_io), .I(exp_p_otr), .T(~exp_p_dtr) );
IOBUF i_iobufn [DWE-1:0] (.O(exp_n_in), .IO(exp_n_io), .I(exp_n_otr), .T(~exp_n_dtr) );

// DIO0_P is a dedicated external-trigger input in this bitstream.  Do not
// feed its asynchronous level into the stock housekeeping/general-GPIO input
// paths; those consumers see bit0 held low.  The raw exp_p_in[0] goes only to
// external_group_trigger_v3 above.
assign exp_p_in_nontrigger = {exp_p_in[DWE-1:1], 1'b0};

assign gpio.i[2*GDW-1:  GDW] = exp_p_in_nontrigger[GDW-1:0];
assign gpio.i[3*GDW-1:2*GDW] = exp_n_in[GDW-1:0];

assign CAN0_rx = can_on & exp_p_in[7];
assign CAN1_rx = can_on & exp_p_in[6];

////////////////////////////////////////////////////////////////////////////////
//  Stock oscilloscope removed in pulse-control v3.10
////////////////////////////////////////////////////////////////////////////////
//
// The custom pulse-control design measures IN1 continuously in
// pulse_measurement_v3 and logs per-pulse/per-group metrics through the PC06
// register/FIFO interface.  The stock Red Pitaya scope was therefore unused by
// the current Python control path, but its decimator/configuration logic still
// contributed real 125 MHz timing failures after the custom logic had nearly
// closed timing.  Remove it from this dedicated bitstream, exactly as the stock
// ASG was removed earlier.
//
// Consequence: stock SCPI oscilloscope/raw-buffer acquisition is not available
// while this pulse-control bitstream is loaded.  IN1 custom pulse measurement,
// FIFO logging, GPIO generation, OUT2 generation and PID feedback are unaffected.

assign scope_trigo    = 1'b0;
assign scope_irq      = 1'b0;
assign scope_irq_ch   = 2'b00;
assign trig_ext_asg01 = 4'b0000;

sys_bus_stub sys_bus_stub_1 (sys[1]);

// Tie off the unused HP0/HP1 AXI request-side interfaces presented to the PS.
assign axi0_sys.waddr  = '0;
assign axi0_sys.wdata  = '0;
assign axi0_sys.wsel   = '0;
assign axi0_sys.wsize  = '0;
assign axi0_sys.wvalid = 1'b0;
assign axi0_sys.wlen   = '0;
assign axi0_sys.wfixed = 1'b0;
assign axi0_sys.raddr  = '0;
assign axi0_sys.rsel   = '0;
assign axi0_sys.rsize  = '0;
assign axi0_sys.rvalid = 1'b0;
assign axi0_sys.rlen   = '0;
assign axi0_sys.rfixed = 1'b0;
assign axi0_sys.rrdys  = 1'b0;

assign axi1_sys.waddr  = '0;
assign axi1_sys.wdata  = '0;
assign axi1_sys.wsel   = '0;
assign axi1_sys.wsize  = '0;
assign axi1_sys.wvalid = 1'b0;
assign axi1_sys.wlen   = '0;
assign axi1_sys.wfixed = 1'b0;
assign axi1_sys.raddr  = '0;
assign axi1_sys.rsel   = '0;
assign axi1_sys.rsize  = '0;
assign axi1_sys.rvalid = 1'b0;
assign axi1_sys.rlen   = '0;
assign axi1_sys.rfixed = 1'b0;
assign axi1_sys.rrdys  = 1'b0;

////////////////////////////////////////////////////////////////////////////////
//  Stock DAC arbitrary signal generator removed in pulse-control v3.4
////////////////////////////////////////////////////////////////////////////////
//
// OUT2 is owned by dac2_sequence_generator_v3.  The stock ASG channel B was
// still instantiating a 250 MHz AXI/FIFO path even though its DAC output was
// unused; routed timing showed that path as the complete pll_dac_clk_2x worst
// path.  For this pulse-control bitstream we do not use the stock ASG at all.
//
// Consequence: OUT1 is held at zero in this bitstream.  If stock OUT1 ASG is
// required later, add back a channel-A-only generator rather than the full
// two-channel ASG.
assign asg_dat[0] = '0;
assign asg_dat[1] = '0;
assign trig_asg_out = 1'b0;

sys_bus_stub sys_bus_stub_2 (sys[2]);

// Tie off the unused HP2/HP3 AXI request-side interfaces presented to the PS.
// The PS wrapper still supplies response-side signals, but no transactions are
// requested from PL.
assign axi2_sys.waddr  = '0;
assign axi2_sys.wdata  = '0;
assign axi2_sys.wsel   = '0;
assign axi2_sys.wsize  = '0;
assign axi2_sys.wvalid = 1'b0;
assign axi2_sys.wlen   = '0;
assign axi2_sys.wfixed = 1'b0;
assign axi2_sys.raddr  = '0;
assign axi2_sys.rsel   = '0;
assign axi2_sys.rsize  = '0;
assign axi2_sys.rvalid = 1'b0;
assign axi2_sys.rlen   = '0;
assign axi2_sys.rfixed = 1'b0;
assign axi2_sys.rrdys  = 1'b0;

assign axi3_sys.waddr  = '0;
assign axi3_sys.wdata  = '0;
assign axi3_sys.wsel   = '0;
assign axi3_sys.wsize  = '0;
assign axi3_sys.wvalid = 1'b0;
assign axi3_sys.wlen   = '0;
assign axi3_sys.wfixed = 1'b0;
assign axi3_sys.raddr  = '0;
assign axi3_sys.rsel   = '0;
assign axi3_sys.rsize  = '0;
assign axi3_sys.rvalid = 1'b0;
assign axi3_sys.rlen   = '0;
assign axi3_sys.rfixed = 1'b0;
assign axi3_sys.rrdys  = 1'b0;

////////////////////////////////////////////////////////////////////////////////
// Stock Red Pitaya PID disabled for this bitstream
////////////////////////////////////////////////////////////////////////////////

// Keep the original system-bus slot valid without instantiating a second PID
// controller that could interfere with the custom pulse-area feedback loop.
sys_bus_stub sys_bus_stub_3 (sys[3]);

////////////////////////////////////////////////////////////////////////////////
// Daisy test code
////////////////////////////////////////////////////////////////////////////////

wire daisy_rx_rdy ;
wire dly_clk = fclk[3]; // 200MHz clock from PS - used for IDELAY (optionaly)
wire [16-1:0] par_dati = daisy_mode[0] ? {16{trig_output_sel}} : 16'h1234;
wire          par_dvi  = daisy_mode[0] ? 1'b0 : daisy_rx_rdy;

red_pitaya_daisy i_daisy (
   // SATA connector
  .daisy_p_o       (  daisy_p_o                  ),  // line 1 is clock capable
  .daisy_n_o       (  daisy_n_o                  ),
  .daisy_p_i       (  daisy_p_i                  ),  // line 1 is clock capable
  .daisy_n_i       (  daisy_n_i                  ),
   // Data
  .ser_clk_i       (  ser_clk                    ),  // high speed serial
  .dly_clk_i       (  dly_clk                    ),  // delay clock
   // TX
  .par_clk_i       (  adc_clk                    ),  // data paralel clock
  .par_rstn_i      (  adc_rstn                   ),  // reset - active low
  .par_rdy_o       (  daisy_rx_rdy               ),
  .par_dv_i        (  par_dvi                    ),
  .par_dat_i       (  par_dati                   ),
   // RX
  .par_clk_o       ( adc_clk_daisy               ),
  .par_rstn_o      (                             ),
  .par_dv_o        (                             ),
  .par_dat_o       ( par_dat                     ),

  .sync_mode_i     (  daisy_mode[0]              ),
  .debug_o         (/*led_o*/                    ),
   // System bus
  .sys_clk_i       (  adc_clk                    ),  // clock
  .sys_rstn_i      (  adc_rstn                   ),  // reset - active low
  .sys_addr_i      (  sys[5].addr                ),
  .sys_sel_i       (                             ),
  .sys_wdata_i     (  sys[5].wdata               ),
  .sys_wen_i       (  sys[5].wen                 ),
  .sys_ren_i       (  sys[5].ren                 ),
  .sys_rdata_o     (  sys[5].rdata               ),
  .sys_err_o       (  sys[5].err                 ),
  .sys_ack_o       (  sys[5].ack                 )
);

  `ifdef Z20_G2
  // DIO11 is TX clock
  // DIO12 is RX clock
  // exp_e3x_o={DIO11, DIO13, DIO15, DIO17}
  // exp_e3x_i={DIO12, DIO14, DIO16, DIO18}
red_pitaya_daisy  #(
  .IO_STD("LVDS_25"),
  .N_DATS(3)
) i_serlines_add
(
   // SATA connector
  .daisy_p_o       (  exp_e3p_o                  ),  // line 3 is clock capable (SRCC)
  .daisy_n_o       (  exp_e3n_o                  ),
  .daisy_p_i       (  exp_e3p_i                  ),  // line 3 is clock capable (MRCC)
  .daisy_n_i       (  exp_e3n_i                  ),
   // Data
  .ser_clk_i       (  ser_clk                    ),  // high speed serial
  .dly_clk_i       (  dly_clk                    ),  // delay clock
   // TX
  .par_clk_i       (  adc_clk                    ),  // data paralel clock
  .par_rstn_i      (  adc_rstn                   ),  // reset - active low
  //.par_rdy_o       (  daisy_rx_rdy               ),
  //.par_dv_i        (  par_dvi                    ),
  //.par_dat_i       (  par_dati                   ),
   // RX
  //.par_clk_o       ( adc_clk_daisy               ),
  //.par_rstn_o      (                             ),
  //.par_dv_o        (                             ),
  //.par_dat_o       ( par_dat                     ),

  .sync_mode_i     (  1'b0                       ),
  //.debug_o         (/*led_o*/                    ),
   // System bus
  .sys_clk_i       (  adc_clk                    ),  // clock
  .sys_rstn_i      (  adc_rstn                   ),  // reset - active low
  .sys_addr_i      (  sys[6].addr                ),
  .sys_sel_i       (                             ),
  .sys_wdata_i     (  sys[6].wdata               ),
  .sys_wen_i       (  sys[6].wen                 ),
  .sys_ren_i       (  sys[6].ren                 ),
  .sys_rdata_o     (  sys[6].rdata               ),
  .sys_err_o       (  sys[6].err                 ),
  .sys_ack_o       (  sys[6].ack                 )
);
  `else
  sys_bus_stub sys_bus_stub_6 (sys[6]);
  `endif
  pulse_control_regs_v3 #(
    .MAX_GPIO_PULSES  (PC_MAX_GPIO_PULSES),
    .MAX_DAC_SEGMENTS (PC_MAX_DAC_SEGMENTS)
  ) i_pulse_control_regs_v3 (
    .clk_i                              (adc_clk),
    .rstn_i                             (adc_rstn),
    .sys_addr_i                         (sys[7].addr),
    .sys_wdata_i                        (sys[7].wdata),
    .sys_wen_i                          (sys[7].wen),
    .sys_ren_i                          (sys[7].ren),
    .sys_rdata_o                        (sys[7].rdata),
    .sys_err_o                          (sys[7].err),
    .sys_ack_o                          (sys[7].ack),

    .measurement_enable_o               (pc_measurement_enable),
    .feedback_enable_o                  (pc_feedback_enable),
    .hold_integrator_o                  (pc_hold_integrator),
    .marker_enable_o                    (pc_marker_enable),
    .gpio_enable_o                      (pc_gpio_enable),
    .dac2_enable_o                      (pc_dac2_enable),
    .use_first_valid_group_as_target_o  (pc_use_first_target),
    .pulse_polarity_negative_o          (pc_pulse_polarity_negative),
    .fifo_continuous_log_enable_o         (pc_fifo_continuous_log_enable),
    .clear_state_o                      (pc_clear_state),
    .clear_flags_o                      (pc_clear_flags),
    .clear_fifos_o                      (pc_clear_fifos),
    .dac2_apply_o                       (pc_dac2_apply),
    .manual_group_boundary_o            (pc_manual_group_boundary),
    .pulse_fifo_pop_o                   (pc_pulse_fifo_pop),
    .group_fifo_pop_o                   (pc_group_fifo_pop),
    .pulse_snapshot_arm_o                 (pc_pulse_snapshot_arm),
    .gpio_apply_toggle_o                (pc_gpio_apply_toggle),

    .threshold_fraction_q16_o           (pc_threshold_fraction_q16),
    .target_height_o                    (pc_target_height),
    .pre_baseline_samples_o             (pc_pre_baseline_samples),
    .post_baseline_samples_o            (pc_post_baseline_samples),
    .min_pulse_samples_o                (pc_min_pulse_samples),
    .max_pulse_samples_o                (pc_max_pulse_samples),
    .adc_saturation_limit_o             (pc_adc_saturation_limit),
    .expected_pulses_per_group_o        (pc_expected_pulses_per_group),
    .min_valid_pulses_per_group_o       (pc_min_valid_pulses_per_group),
    .group_source_o                     (pc_group_source),
    .external_trigger_delay_cycles_o    (pc_external_trigger_delay_cycles),
    .external_trigger_min_high_points_o (pc_external_trigger_min_high_points),
    .reference_target_start_index_o     (pc_reference_target_start_index),
    .reference_target_pulse_count_o     (pc_reference_target_pulse_count),

    .target_area_o                      (pc_target_area),
    .deadband_absolute_o                (pc_deadband_absolute),
    .deadband_fraction_q16_o            (pc_deadband_fraction_q16),
    .deadband_fraction_mode_o           (pc_deadband_fraction_mode),
    .kp_q16_o                           (pc_kp_q16),
    .ki_q31_per_update_o                (pc_ki_q31_per_update),
    .kd_q16_per_update_o                (pc_kd_q16_per_update),
    .area_to_dac_gain_q24_o             (pc_area_to_dac_gain_q24),
    .correction_sign_negative_o         (pc_correction_sign_negative),
    .max_dac_step_counts_o              (pc_max_dac_step_counts),
    .dac_high_min_counts_o              (pc_dac_high_min_counts),
    .dac_high_max_counts_o              (pc_dac_high_max_counts),
    .integral_min_area_o                (pc_integral_min_area),
    .integral_max_area_o                (pc_integral_max_area),
    .integral_limit_fraction_q16_o      (pc_integral_limit_fraction_q16),
    .integral_fraction_mode_o           (pc_integral_fraction_mode),

    .gpio_period_cycles_shadow_o        (pc_gpio_period_cycles_shadow),
    .gpio_pulse_count_shadow_o          (pc_gpio_pulse_count_shadow),
    .gpio_pulse_start_cycles_shadow_o   (pc_gpio_pulse_start_shadow),
    .gpio_pulse_end_cycles_shadow_o     (pc_gpio_pulse_end_shadow),

    .dac2_period_cycles_shadow_o        (pc_dac2_period_cycles_shadow),
    .dac2_segment_count_shadow_o        (pc_dac2_segment_count_shadow),
    .dac2_low_counts_shadow_o           (pc_dac2_low_counts_shadow),
    .dac2_high_counts_shadow_o          (pc_dac2_high_counts_shadow),
    .dac2_segment_level_q16_shadow_o    (pc_dac2_segment_level_shadow),
    .dac2_segment_duration_cycles_shadow_o(pc_dac2_segment_duration_shadow),

    .measurement_config_valid_i         (pc_measurement_config_valid),
    .controller_config_valid_i          (pc_controller_config_valid),
    .gpio_config_valid_i                (pc_gpio_config_valid_sync[1]),
    .dac2_config_valid_i                (pc_dac2_config_valid),
    .target_latched_i                   (pc_target_latched),
    .bpre_i                             (pc_bpre),
    .bpost_i                            (pc_bpost),
    .threshold_i                        (pc_threshold),
    .peak_raw_i                         (pc_peak_raw),
    .peak_height_i                      (pc_peak_height),
    .pulse_duration_samples_i           (pc_pulse_duration_samples),
    .measured_area_i                    (pc_measured_area),
    .last_invalid_reason_i              (pc_invalid_reason),
    .invalid_pulse_valid_i              (pc_invalid_pulse_valid),
    .timestamp_i                        (pc_timestamp),
    .total_valid_measurements_i         (pc_total_valid_measurements),
    .total_invalid_measurements_i       (pc_total_invalid_measurements),
    .total_groups_i                     (pc_total_groups),
    .target_area_active_i               (pc_target_area_active),
    .area_error_i                       (pc_area_error),
    .p_term_area_i                      (pc_p_term_area),
    .i_term_area_i                      (pc_i_term_area),
    .d_term_area_i                      (pc_d_term_area),
    .correction_pending_i               (pc_correction_pending),
    .correction_active_i                (pc_correction_active),
    .effective_dac_high_i               (pc_effective_dac_high),
    .update_count_i                     (pc_update_count),
    .processing_latency_cycles_i        (pc_processing_latency_cycles),
    .correction_high_sat_i              (pc_correction_high_sat),
    .correction_low_sat_i               (pc_correction_low_sat),
    .integral_high_sat_i                (pc_integral_high_sat),
    .integral_low_sat_i                 (pc_integral_low_sat),

    .group_result_valid_i               (pc_group_result_valid),
    .last_group_id_i                    (pc_result_group_id),
    .last_group_detected_i              (pc_result_detected),
    .last_group_valid_i                 (pc_result_valid_pulses),
    .last_group_invalid_i               (pc_result_invalid_pulses),
    .last_group_mean_area_i             (pc_result_mean_area),
    .last_group_expected_mismatch_i     (pc_result_expected_mismatch),
    .last_group_overflow_i              (pc_result_group_overflow),
    .last_boundary_overrun_i            (pc_result_boundary_overrun),
    .marker_cycles_i                    (pc_last_marker_cycles),
    .marker_timeout_i                   (pc_marker_timeout),
    .gpio_active_period_cycles_i        (pc_gpio_active_period_sync2),
    .gpio_active_pulse_count_i          (pc_gpio_active_pulse_count_sync2),
    .dac2_active_period_cycles_i        (pc_dac2_active_period_cycles),
    .dac2_active_segment_count_i        (pc_dac2_active_segment_count),
    .external_trigger_count_i           (pc_ext_trigger_count),
    .external_trigger_active_delay_cycles_i(pc_ext_trigger_delay_sync2),
    .external_trigger_active_min_high_points_i(pc_ext_trigger_min_high_sync2),
    .external_trigger_overrun_i         (pc_ext_trigger_overrun_pulse),

    .reference_window_active_i          (pc_reference_window_active),
    .last_reference_id_i                (pc_reference_id),
    .last_reference_sum_area_i          (pc_reference_sum_area),
    .last_reference_selected_count_i    (pc_reference_detected[5:0]),
    .last_reference_valid_i             (pc_reference_valid_for_feedback),
    .reference_incomplete_i             (pc_reference_incomplete),
    .reference_sum_overflow_i           (pc_reference_sum_overflow),

    .pulse_fifo_level_i                 ({5'd0, pc_pulse_fifo_level}),
    .pulse_fifo_full_i                  (pc_pulse_fifo_full),
    .pulse_fifo_overflow_count_i        (pc_pulse_fifo_overflow_count),
    .pulse_fifo_head_i                  (pc_pulse_fifo_head),
    .group_fifo_level_i                 ({7'd0, pc_group_fifo_level}),
    .group_fifo_full_i                  (pc_group_fifo_full),
    .group_fifo_overflow_count_i        (pc_group_fifo_overflow_count),
    .group_fifo_head_i                  (pc_group_fifo_head)
  );

`else
IOBUF i_iobuf (.O(trig_ext), .IO(exp_p_io[0]), .I(1'b0), .T(1'b1) );

logic [2-1:0] [ADW-1:0] adc_dat_raw;

always @(posedge adc_clk) begin
  adc_dat_raw[0] <= adc_dat_i[0][16-1 -: ADW];
  adc_dat_raw[1] <= adc_dat_i[1][16-1 -: ADW];

  adc_dat[0] <= {adc_dat_raw[0][ADW-1], ~adc_dat_raw[0][ADW-2:0]};
  adc_dat[1] <= {adc_dat_raw[1][ADW-1], ~adc_dat_raw[1][ADW-2:0]};
end

//red_pitaya_hk #(.DWE(DWE)) i_hk (
  //// system signals
  //.clk_i           (adc_clk    ),  // clock
  //.rstn_i          (adc_rstn   ),  // reset - active low
  //.fclk_i          (fclk[0]    ),  // clock
  //.frstn_i         (frstn[0]   ),  // reset - active low
  ////// LED
  ////.led_o           (  led_o     ),  // LED output
  ////// global configuration
  ////.digital_loop    (digital_loop),
  ////.daisy_mode_o    (daisy_mode),
  ////// Expansion connector
  //// .exp_p_dat_i     (exp_p_in ),  // input data
  //// .exp_p_dat_o     (exp_p_out),  // output data
  //// .exp_p_dir_o     (exp_p_dir),  // 1-output enable
  //// .exp_n_dat_i     (exp_n_in ),
  //// .exp_n_dat_o     (exp_n_out),
  //// .exp_n_dir_o     (exp_n_dir),
  //// .can_on_o        (can_on   ),
  ////// System bus
  //.sys_addr        (sys[0].addr ),
  //.sys_wdata       (sys[0].wdata),
  //.sys_wen         (sys[0].wen  ),
  //.sys_ren         (sys[0].ren  ),
  //.sys_rdata       (sys[0].rdata),
  //.sys_err         (sys[0].err  ),
  //.sys_ack         (sys[0].ack  )
//);

//red_pitaya_scope i_scope (
  //// ADC
  //.adc_a_i       (adc_dat[0]  ),  // CH 1
  //.adc_b_i       (adc_dat[1]  ),  // CH 2
  //.adc_clk_i     (adc_clk     ),  // clock
  //.adc_rstn_i    (adc_rstn    ),  // reset - active low
  //.trig_ext_i    (trig_ext    ),  // external trigger
  //.trig_asg_i    (1'b0        ),  // ASG trigger
  //.trig_ext_asg_o(trig_ext_asg01),
  //.trig_ext_asg_i(trig_ext_asg01),
  ////.daisy_trig_o  (scope_trigo ),
  //// AXI0 master                 // AXI1 master
  //.axi0_waddr_o  (axi0_sys.waddr ),  .axi1_waddr_o  (axi1_sys.waddr ),
  //.axi0_wdata_o  (axi0_sys.wdata ),  .axi1_wdata_o  (axi1_sys.wdata ),
  //.axi0_wsel_o   (axi0_sys.wsel  ),  .axi1_wsel_o   (axi1_sys.wsel  ),
  //.axi0_wvalid_o (axi0_sys.wvalid),  .axi1_wvalid_o (axi1_sys.wvalid),
  //.axi0_wlen_o   (axi0_sys.wlen  ),  .axi1_wlen_o   (axi1_sys.wlen  ),
  //.axi0_wfixed_o (axi0_sys.wfixed),  .axi1_wfixed_o (axi1_sys.wfixed),
  //.axi0_werr_i   (axi0_sys.werr  ),  .axi1_werr_i   (axi1_sys.werr  ),
  //.axi0_wrdy_i   (axi0_sys.wrdy  ),  .axi1_wrdy_i   (axi1_sys.wrdy  ),
  //// System bus
  //.sys_addr      (sys[1].addr ),
  //.sys_wdata     (sys[1].wdata),
  //.sys_wen       (sys[1].wen  ),
  //.sys_ren       (sys[1].ren  ),
  //.sys_rdata     (sys[1].rdata),
  //.sys_err       (sys[1].err  ),
  //.sys_ack       (sys[1].ack  )
//);

assign dac_dat_a = 14'h0;
assign dac_dat_b = 14'h0;

// DDR outputs
ODDR oddr_dac_clk          (.Q(dac_clk_o), .D1(1'b0     ), .D2(1'b1     ), .C(dac_clk_2p), .CE(1'b1), .R(1'b0   ), .S(1'b0));
ODDR oddr_dac_wrt          (.Q(dac_wrt_o), .D1(1'b0     ), .D2(1'b1     ), .C(dac_clk_2x), .CE(1'b1), .R(1'b0   ), .S(1'b0));
ODDR oddr_dac_sel          (.Q(dac_sel_o), .D1(1'b1     ), .D2(1'b0     ), .C(dac_clk_1x), .CE(1'b1), .R(dac_rst), .S(1'b0));
ODDR oddr_dac_rst          (.Q(dac_rst_o), .D1(dac_rst  ), .D2(dac_rst  ), .C(dac_clk_1x), .CE(1'b1), .R(1'b0   ), .S(1'b0));
ODDR oddr_dac_dat [14-1:0] (.Q(dac_dat_o), .D1(dac_dat_b), .D2(dac_dat_a), .C(dac_clk_1x), .CE(1'b1), .R(dac_rst), .S(1'b0));

ODDR i_adc_clk_p ( .Q(adc_clk_o[0]), .D1(1'b1), .D2(1'b0), .C(1'b0), .CE(1'b1), .R(1'b0), .S(1'b0));
ODDR i_adc_clk_n ( .Q(adc_clk_o[1]), .D1(1'b0), .D2(1'b1), .C(1'b0), .CE(1'b1), .R(1'b0), .S(1'b0));

logic rxs_clk, rxs_dat;
IBUFDS #(.IOSTANDARD ("DIFF_HSTL_I_18")) i_IBUFGDS_clk
(
  .I  ( daisy_p_i[1]  ),
  .IB ( daisy_n_i[1]  ),
  .O  ( rxs_clk     )
);

IBUFDS #(.DIFF_TERM ("FALSE"), .IOSTANDARD ("DIFF_HSTL_I_18")) i_IBUFDS_dat
(
  .I  ( daisy_p_i[0]  ),
  .IB ( daisy_n_i[0]  ),
  .O  ( rxs_dat       )
);

OBUFDS #(.IOSTANDARD ("DIFF_HSTL_I_18"), .SLEW ("FAST")) i_OBUF_clk
(
  .O  ( daisy_p_o[1]  ),
  .OB ( daisy_n_o[1]  ),
  .I  ( 1'b0       )
);

OBUFDS #(.IOSTANDARD ("DIFF_HSTL_I_18"), .SLEW ("FAST")) i_OBUF_dat
(
  .O  ( daisy_p_o[0]  ),
  .OB ( daisy_n_o[0]  ),
  .I  ( 1'b0          )
);


assign adc_cdcs_o = 1'b1 ;
assign dac_pwm_o  = 1'b0;
generate
for (genvar i=2; i<7; i++) begin: for_sys2
  sys_bus_stub sys_bus_stub_2_5 (sys[i]);
end: for_sys2
endgenerate

`endif
endmodule: red_pitaya_top
