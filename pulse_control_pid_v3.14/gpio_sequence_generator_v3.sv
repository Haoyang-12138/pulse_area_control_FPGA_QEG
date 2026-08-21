`timescale 1ns / 1ps

// Programmable GPIO pulse-window generator.
// Clock: fclk[3] = 200 MHz (5 ns resolution).
//
// v3.9 timing-closure architecture:
//   * Python contract remains: disable -> write stable shadow table -> APPLY -> enable.
//   * On APPLY while disabled, the complete 16-window table is captured locally in
//     the 200 MHz domain and end-exclusive values are converted once to end-1.
//   * The run-time path keeps only current_start/current_endm1 plus a registered
//     next_index.  It does NOT destructively shift a 16 x 64-bit queue every pulse.
//   * Runtime window selection therefore sees a local, static table and a 5-bit
//     registered selector.  This avoids both the old cross-module shadow-table
//     routing path and the v3.8 high-fanout queue-shift/reset path.
module gpio_sequence_generator_v3 #(
    parameter int MAX_PULSES = 16
)(
    input  logic                 clk_200mhz_i,
    input  logic                 rstn_i,

    input  logic                 enable_async_i,
    input  logic                 apply_toggle_async_i,
    input  logic [31:0]          period_cycles_shadow_async_i,
    input  logic [4:0]           pulse_count_shadow_async_i,
    input  logic [31:0]          pulse_start_cycles_shadow_async_i [0:MAX_PULSES-1],
    input  logic [31:0]          pulse_end_cycles_shadow_async_i   [0:MAX_PULSES-1],

    output logic                 gpio_out_o,
    output logic                 period_toggle_o,
    output logic                 config_valid_o,
    output logic [31:0]          active_period_cycles_o,
    output logic [4:0]           active_pulse_count_o,
    output logic [31:0]          cycle_counter_o
);

    (* ASYNC_REG = "TRUE" *) logic enable_sync1, enable_sync2;
    (* ASYNC_REG = "TRUE" *) logic [3:0] apply_sync;

    logic apply_seen;
    logic apply_pending;
    logic apply_event;

    logic [31:0] active_period;
    logic [31:0] active_last_cycle;
    logic [4:0]  active_pulse_count;
    logic        active_config_valid;

    logic [31:0] cycle_counter;
    logic [4:0]  pulse_index;
    (* max_fanout = 16 *) logic [4:0] next_index;

    logic [31:0] current_start;
    logic [31:0] current_endm1;

    // Stable local copy of the configured windows.  These registers are only
    // written on APPLY while the output is disabled, then remain unchanged while
    // the sequencer is running.  The expensive end-1 arithmetic is also done only
    // at configuration time, not on the 200 MHz pulse-advance path.
    logic [31:0] local_start  [0:MAX_PULSES-1];
    logic [31:0] local_endm1  [0:MAX_PULSES-1];

    integer k;

    assign apply_event = apply_sync[3] ^ apply_seen;

    assign active_period_cycles_o = active_period;
    assign active_pulse_count_o   = active_pulse_count;
    assign cycle_counter_o        = cycle_counter;

    // Configuration validity is captured only on APPLY while disabled.
    // Keeping it registered removes the old 200 MHz active_period ->
    // config-valid comparator -> widespread reset/set control path.
    assign config_valid_o = active_config_valid;

    // Fast output path: current-window registers only.
    always_comb begin
        gpio_out_o = 1'b0;
        if (enable_sync2 && config_valid_o &&
            (pulse_index < active_pulse_count)) begin
            if ((cycle_counter >= current_start) &&
                (cycle_counter <= current_endm1))
                gpio_out_o = 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // 1. CDC synchronizers + configuration capture.
    //
    // Keep this separate from the run-time counter/window state.  In the old
    // monolithic always_ff Vivado was able to factor window-advance logic into
    // the cycle_counter reset/control cone.  At 200 MHz that produced the
    // observed cycle_counter -> current_endm1/next_index -> cycle_counter/R
    // critical path.  Splitting the state machines makes those control cones
    // structurally independent without changing the external behaviour.
    // ------------------------------------------------------------------
    always_ff @(posedge clk_200mhz_i) begin
        if (!rstn_i) begin
            enable_sync1        <= 1'b0;
            enable_sync2        <= 1'b0;
            apply_sync          <= 4'd0;
            apply_seen          <= 1'b0;
            apply_pending       <= 1'b0;

            active_period       <= 32'd2000;
            active_last_cycle   <= 32'd1999;
            active_pulse_count  <= 5'd0;
            active_config_valid <= 1'b0;

            for (k = 0; k < MAX_PULSES; k = k + 1) begin
                local_start[k] <= 32'd0;
                local_endm1[k] <= 32'd0;
            end
        end
        else begin
            enable_sync1 <= enable_async_i;
            enable_sync2 <= enable_sync1;
            apply_sync   <= {apply_sync[2:0], apply_toggle_async_i};

            if (apply_event) begin
                apply_seen    <= apply_sync[3];
                apply_pending <= 1'b1;
            end

            // Configuration is accepted only while disabled.  Python holds the
            // complete bundled shadow table stable around APPLY.
            if (!enable_sync2 && (apply_pending || apply_event)) begin
                active_period      <= period_cycles_shadow_async_i;
                active_last_cycle  <=
                    (period_cycles_shadow_async_i >= 32'd1)
                    ? (period_cycles_shadow_async_i - 1'b1)
                    : 32'd0;
                active_pulse_count <= pulse_count_shadow_async_i;
                active_config_valid <=
                    (period_cycles_shadow_async_i >= 32'd2) &&
                    (pulse_count_shadow_async_i <= MAX_PULSES);

                for (k = 0; k < MAX_PULSES; k = k + 1) begin
                    local_start[k] <= pulse_start_cycles_shadow_async_i[k];
                    local_endm1[k] <=
                        (pulse_end_cycles_shadow_async_i[k] >= 32'd1)
                        ? (pulse_end_cycles_shadow_async_i[k] - 1'b1)
                        : 32'd0;
                end

                apply_pending <= 1'b0;
            end
        end
    end

    // ------------------------------------------------------------------
    // 2. Period counter.
    //
    // This block intentionally depends only on enable/config and
    // active_last_cycle.  It has no dependency on current_endm1, pulse_index,
    // next_index or the local pulse table.
    // ------------------------------------------------------------------
    always_ff @(posedge clk_200mhz_i) begin
        if (!rstn_i) begin
            cycle_counter   <= 32'd0;
            period_toggle_o <= 1'b0;
        end
        else if (!enable_sync2 || !active_config_valid) begin
            cycle_counter <= 32'd0;
        end
        else if (cycle_counter == active_last_cycle) begin
            cycle_counter   <= 32'd0;
            period_toggle_o <= ~period_toggle_o;
        end
        else begin
            cycle_counter <= cycle_counter + 1'b1;
        end
    end

    // ------------------------------------------------------------------
    // 3. Pulse-window state.
    //
    // Window selection can still use the 32-bit end comparison and local
    // table mux, but that logic can no longer enter the period-counter state
    // update cone.
    // ------------------------------------------------------------------
    always_ff @(posedge clk_200mhz_i) begin
        if (!rstn_i) begin
            pulse_index   <= 5'd0;
            next_index    <= 5'd1;
            current_start <= 32'd0;
            current_endm1 <= 32'd0;
        end
        else if (!enable_sync2) begin
            pulse_index <= 5'd0;
            next_index  <= 5'd1;

            // Use the shadow entry directly on the APPLY clock because the
            // local table is captured by the same edge.
            if (apply_pending || apply_event) begin
                current_start <= pulse_start_cycles_shadow_async_i[0];
                current_endm1 <=
                    (pulse_end_cycles_shadow_async_i[0] >= 32'd1)
                    ? (pulse_end_cycles_shadow_async_i[0] - 1'b1)
                    : 32'd0;
            end
        end
        else if (!active_config_valid) begin
            pulse_index <= 5'd0;
            next_index  <= 5'd1;
        end
        else if (cycle_counter == active_last_cycle) begin
            // Start the next period from local entry 0.
            pulse_index   <= 5'd0;
            next_index    <= 5'd1;
            current_start <= local_start[0];
            current_endm1 <= local_endm1[0];
        end
        else if ((pulse_index < active_pulse_count) &&
                 (cycle_counter == current_endm1) &&
                 (next_index < active_pulse_count)) begin
            current_start <= local_start[next_index];
            current_endm1 <= local_endm1[next_index];
            pulse_index   <= next_index;
            next_index    <= next_index + 1'b1;
        end
    end

endmodule
