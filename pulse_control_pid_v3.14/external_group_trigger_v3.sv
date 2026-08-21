`timescale 1ns / 1ps

// PC06 external reference-trigger front-end — timing-fix 1.
//
// DIO0_P is asynchronous. It is synchronized into fclk[3] = 200 MHz and
// must remain HIGH for min_high_points_shadow_async_i consecutive 200 MHz
// samples before it is accepted. 1 point = 5 ns. Short HIGH pulses are
// ignored. A long HIGH level produces exactly one accepted event and must
// return LOW before the front-end rearms.
//
// Timing-fix rationale:
// The first PC06 implementation compared an incrementing 16-bit counter with
// (programmable_threshold - 1) on every 200 MHz cycle. Post-route timing showed
// this programmable compare/reset path as the dominant 200 MHz failure.
//
// This revision removes that dynamic threshold compare from the realtime
// qualification loop:
//   1) the held-static threshold is normalized and pre-decremented in two
//      registered configuration stages;
//   2) while DIO0_P is LOW, a countdown register is preloaded;
//   3) while DIO0_P is HIGH, the realtime path is only countdown decrement +
//      zero-terminal detection;
//   4) reaching zero accepts exactly one trigger and disarms until LOW.
//
// Qualification semantics are unchanged:
//   min_high_points = N => acceptance on the Nth synchronized HIGH sample.
//   min_high_points = 0 is treated as 1.
//
// The accepted event remains registered before entering the existing 32-bit
// programmable-delay scheduler. The delayed event is exported as a toggle for
// the existing 200 -> 125 MHz CDC in red_pitaya_top.
//
// Configuration originates in the 125 MHz AXI register domain and must remain
// static while the external source is active. Software should configure while
// the source is manual/idle and wait for active readback before enabling the
// external trigger source.
module external_group_trigger_v3 (
    input  logic         clk_200mhz_i,
    input  logic         rstn_i,

    input  logic         enable_async_i,
    input  logic         trigger_async_i,
    input  logic [31:0]  delay_cycles_shadow_async_i,
    input  logic [15:0]  min_high_points_shadow_async_i,

    output logic         event_toggle_o,
    output logic         overrun_toggle_o,
    output logic [31:0]  active_delay_cycles_o,
    output logic [15:0]  active_min_high_points_o
);

    (* ASYNC_REG = "TRUE" *) logic enable_sync1;
    (* ASYNC_REG = "TRUE" *) logic enable_sync2;
    (* ASYNC_REG = "TRUE" *) logic trigger_sync1;
    (* ASYNC_REG = "TRUE" *) logic trigger_sync2;

    // Slow held-static configuration buses. These retain the PC05/PC06 CDC
    // contract: software changes them only while the external source is idle.
    (* ASYNC_REG = "TRUE" *) logic [31:0] delay_sync1;
    (* ASYNC_REG = "TRUE" *) logic [31:0] delay_sync2;
    (* ASYNC_REG = "TRUE" *) logic [15:0] min_high_sync1;
    (* ASYNC_REG = "TRUE" *) logic [15:0] min_high_sync2;

    // Configuration-only threshold pipeline. Neither stage lies in the
    // realtime HIGH-width counter feedback path.
    logic [15:0] min_high_normalized_q;
    logic [15:0] min_high_minus_one_q;

    // Realtime width qualifier.
    logic [15:0] remaining_high_count;
    logic        armed_for_high;
    logic        qualified_trigger_pulse;

    // Existing delayed-event scheduler.
    logic        delay_pending;
    logic [31:0] delay_counter;

    assign active_delay_cycles_o = delay_sync2;

    // Input/configuration synchronization only.
    always_ff @(posedge clk_200mhz_i) begin
        if (!rstn_i) begin
            enable_sync1   <= 1'b0;
            enable_sync2   <= 1'b0;
            trigger_sync1  <= 1'b0;
            trigger_sync2  <= 1'b0;
            delay_sync1    <= 32'd0;
            delay_sync2    <= 32'd0;
            min_high_sync1 <= 16'd1;
            min_high_sync2 <= 16'd1;
        end
        else begin
            enable_sync1  <= enable_async_i;
            enable_sync2  <= enable_sync1;
            trigger_sync1 <= trigger_async_i;
            trigger_sync2 <= trigger_sync1;

            delay_sync1    <= delay_cycles_shadow_async_i;
            delay_sync2    <= delay_sync1;
            min_high_sync1 <= min_high_points_shadow_async_i;
            min_high_sync2 <= min_high_sync1;
        end
    end

    // Preprocess the programmable minimum-HIGH threshold away from the
    // 200 MHz realtime counter loop. Two explicit registered stages are used:
    //   stage A: normalize 0 -> 1
    //   stage B: compute N-1 and publish matching active readback
    // Configuration latency is irrelevant because software waits for active
    // readback before enabling external triggering.
    always_ff @(posedge clk_200mhz_i) begin
        if (!rstn_i) begin
            min_high_normalized_q    <= 16'd1;
            min_high_minus_one_q     <= 16'd0;
            active_min_high_points_o <= 16'd1;
        end
        else begin
            min_high_normalized_q <= (min_high_sync2 == 16'd0)
                                   ? 16'd1
                                   : min_high_sync2;

            min_high_minus_one_q     <= min_high_normalized_q - 16'd1;
            active_min_high_points_o <= min_high_normalized_q;
        end
    end

    // Width qualification.
    //
    // LOW rearms the detector and preloads the already-registered countdown.
    // Requiring LOW before the first accepted HIGH after enable preserves the
    // previous safety property: enabling while DIO0_P is already HIGH cannot
    // fabricate an accepted trigger.
    //
    // For N points, LOW preloads N-1. The first synchronized HIGH sample sees
    // N-1; each HIGH decrements. The Nth HIGH sample sees zero and accepts.
    // qualified_trigger_pulse is a registered one-cycle 200 MHz pulse.
    always_ff @(posedge clk_200mhz_i) begin
        if (!rstn_i) begin
            remaining_high_count    <= 16'd0;
            armed_for_high          <= 1'b0;
            qualified_trigger_pulse <= 1'b0;
        end
        else begin
            qualified_trigger_pulse <= 1'b0;

            if (!enable_sync2) begin
                remaining_high_count <= min_high_minus_one_q;
                armed_for_high       <= 1'b0;
            end
            else if (!trigger_sync2) begin
                remaining_high_count <= min_high_minus_one_q;
                armed_for_high       <= 1'b1;
            end
            else if (armed_for_high) begin
                if (remaining_high_count == 16'd0) begin
                    qualified_trigger_pulse <= 1'b1;
                    armed_for_high          <= 1'b0;
                end
                else begin
                    remaining_high_count <= remaining_high_count - 16'd1;
                end
            end
        end
    end

    // Delayed-event scheduler — intentionally unchanged from the prior PC06
    // candidate except for comments. The qualifier above is a registered source,
    // so width qualification remains isolated from this 32-bit delay path.
    always_ff @(posedge clk_200mhz_i) begin
        if (!rstn_i) begin
            delay_pending    <= 1'b0;
            delay_counter    <= 32'd0;
            event_toggle_o   <= 1'b0;
            overrun_toggle_o <= 1'b0;
        end
        else begin
            if (!enable_sync2) begin
                delay_pending <= 1'b0;
                delay_counter <= 32'd0;
            end
            else begin
                if (qualified_trigger_pulse) begin
                    if (delay_pending) begin
                        // Preserve the already-scheduled event and report the
                        // overlapping accepted trigger through the old toggle.
                        overrun_toggle_o <= ~overrun_toggle_o;
                    end
                    else if (delay_sync2 == 32'd0) begin
                        event_toggle_o <= ~event_toggle_o;
                    end
                    else begin
                        // N=1 means one 200 MHz clock after this registered
                        // qualified-trigger pulse reaches the scheduler.
                        delay_pending <= 1'b1;
                        delay_counter <= delay_sync2 - 32'd1;
                    end
                end

                if (delay_pending) begin
                    if (delay_counter == 32'd0) begin
                        delay_pending  <= 1'b0;
                        event_toggle_o <= ~event_toggle_o;
                    end
                    else begin
                        delay_counter <= delay_counter - 32'd1;
                    end
                end
            end
        end
    end

endmodule
