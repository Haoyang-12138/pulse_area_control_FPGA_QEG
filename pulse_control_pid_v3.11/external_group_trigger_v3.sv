`timescale 1ns / 1ps

// External group-trigger front-end.
//
// DIO0_P is asynchronous to the FPGA fabric.  The input is first synchronized
// into fclk[3] = 200 MHz, then rising edges are delayed by a programmable
// number of 200 MHz cycles (5 ns/cycle).  A toggle carries the delayed event
// safely into the 125 MHz ADC/control domain in red_pitaya_top.
//
// The delay configuration originates in the 125 MHz AXI register domain.  It
// is continuously double-synchronized here.  Software must only change it
// while GROUP_SOURCE=manual (the supplied Python driver enforces this), then
// wait for the active-delay readback to match before enabling external groups.
// This makes the multi-bit value effectively static while triggers are active.
//
// One delayed trigger may be pending at a time.  A new rising edge arriving
// before the previous delay expires is dropped and toggles overrun_toggle_o.
// For a 1 MHz external trigger, software therefore requires delay < 1 us.
module external_group_trigger_v3 (
    input  logic         clk_200mhz_i,
    input  logic         rstn_i,

    input  logic         enable_async_i,
    input  logic         trigger_async_i,
    input  logic [31:0]  delay_cycles_shadow_async_i,

    output logic         event_toggle_o,
    output logic         overrun_toggle_o,
    output logic [31:0]  active_delay_cycles_o
);

    (* ASYNC_REG = "TRUE" *) logic enable_sync1;
    (* ASYNC_REG = "TRUE" *) logic enable_sync2;
    (* ASYNC_REG = "TRUE" *) logic trigger_sync1;
    (* ASYNC_REG = "TRUE" *) logic trigger_sync2;
    logic trigger_sync2_d;

    // Slow configuration bus.  Software holds it stable while external-group
    // acquisition is disabled, so two-stage capture is sufficient and avoids
    // adding a second control bus/handshake to the already verified AXI path.
    (* ASYNC_REG = "TRUE" *) logic [31:0] delay_sync1;
    (* ASYNC_REG = "TRUE" *) logic [31:0] delay_sync2;

    logic delay_pending;
    logic [31:0] delay_counter;
    logic trigger_rise;

    assign trigger_rise = trigger_sync2 & ~trigger_sync2_d;
    assign active_delay_cycles_o = delay_sync2;

    always_ff @(posedge clk_200mhz_i) begin
        if (!rstn_i) begin
            enable_sync1     <= 1'b0;
            enable_sync2     <= 1'b0;
            trigger_sync1    <= 1'b0;
            trigger_sync2    <= 1'b0;
            trigger_sync2_d  <= 1'b0;
            delay_sync1      <= 32'd0;
            delay_sync2      <= 32'd0;
            delay_pending    <= 1'b0;
            delay_counter    <= 32'd0;
            event_toggle_o   <= 1'b0;
            overrun_toggle_o <= 1'b0;
        end
        else begin
            enable_sync1    <= enable_async_i;
            enable_sync2    <= enable_sync1;
            trigger_sync1   <= trigger_async_i;
            trigger_sync2   <= trigger_sync1;
            trigger_sync2_d <= trigger_sync2;

            delay_sync1 <= delay_cycles_shadow_async_i;
            delay_sync2 <= delay_sync1;

            if (!enable_sync2) begin
                // Disarm while the acquisition source is idle/manual.  Keep
                // synchronizing the physical input above so re-enabling does
                // not manufacture a false rising edge from a trigger already HIGH.
                delay_pending <= 1'b0;
                delay_counter <= 32'd0;
            end
            else if (trigger_rise) begin
                if (delay_pending) begin
                    // Do not corrupt the already scheduled event.  Record the
                    // dropped edge instead; software exposes this as a sticky
                    // status flag in the 125 MHz domain.
                    overrun_toggle_o <= ~overrun_toggle_o;
                end
                else if (delay_sync2 == 32'd0) begin
                    event_toggle_o <= ~event_toggle_o;
                end
                else begin
                    // N=1 means one 200 MHz clock after the synchronized edge.
                    delay_pending <= 1'b1;
                    delay_counter <= delay_sync2 - 1'b1;
                end
            end

            if (enable_sync2 && delay_pending) begin
                if (delay_counter == 32'd0) begin
                    delay_pending  <= 1'b0;
                    event_toggle_o <= ~event_toggle_o;
                end
                else begin
                    delay_counter <= delay_counter - 1'b1;
                end
            end
        end
    end

endmodule
