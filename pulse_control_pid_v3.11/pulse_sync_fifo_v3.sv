`timescale 1ns / 1ps

// Single-clock synchronous logging FIFO using the Xilinx XPM FIFO macro.
//
// Why XPM instead of behavioral RAM inference:
//   The pulse/group records are very wide (hundreds of bits).  Some Vivado
//   inference patterns can map such FIFOs into LUTRAM even with ram_style.
//   FIFO_MEMORY_TYPE="block" makes the implementation intent explicit and
//   keeps these logging buffers out of distributed RAM.
//
// READ_MODE="fwft" means dout always presents the current FIFO head, matching
// the register-map/software interface used by pulse_control_regs_v3.
//
// DEPTH must be >= 16 for xpm_fifo_sync and should be a power of two.
module pulse_sync_fifo_v3 #(
    parameter int WIDTH = 32,
    parameter int DEPTH = 1024
)(
    input  logic                         clk_i,
    input  logic                         rstn_i,
    input  logic                         clear_i,

    input  logic                         push_i,
    input  logic [WIDTH-1:0]             push_data_i,
    output logic                         full_o,

    input  logic                         pop_i,
    output logic [WIDTH-1:0]             head_data_o,
    output logic                         empty_o,

    output logic [$clog2(DEPTH+1)-1:0]   level_o,
    output logic [31:0]                  overflow_count_o
);

    localparam int LEVEL_W = $clog2(DEPTH + 1);

    logic fifo_rst;
    logic full_int;
    logic empty_int;
    logic [WIDTH-1:0] dout_int;
    logic push_accept;
    logic pop_accept;
    logic [LEVEL_W-1:0] level;

    // XPM reset is active high.  clear_i deliberately performs a logical FIFO
    // flush as well as clearing our software-visible count/overflow state.
    assign fifo_rst = (!rstn_i) || clear_i;

    assign push_accept = push_i && !full_int;
    assign pop_accept  = pop_i  && !empty_int;

    assign full_o      = full_int;
    assign empty_o     = empty_int;
    assign head_data_o = dout_int;
    assign level_o     = level;

`ifndef SYNTHESIS
    initial begin
        if (DEPTH < 16)
            $error("pulse_sync_fifo_v3: DEPTH must be >= 16 for XPM FIFO");
        if ((DEPTH & (DEPTH - 1)) != 0)
            $error("pulse_sync_fifo_v3: DEPTH should be a power of two");
    end
`endif

    // Keep only the small level/overflow bookkeeping in ordinary registers.
    // The wide payload storage itself lives inside block RAM through XPM.
    always_ff @(posedge clk_i) begin
        if (!rstn_i || clear_i) begin
            level            <= '0;
            overflow_count_o <= 32'd0;
        end
        else begin
            case ({push_accept, pop_accept})
                2'b10: level <= level + 1'b1;
                2'b01: level <= level - 1'b1;
                default: level <= level;
            endcase

            if (push_i && full_int)
                overflow_count_o <= overflow_count_o + 1'b1;
        end
    end

    // Xilinx Parameterized Macro.  FIFO_MEMORY_TYPE="block" is the critical
    // setting: Vivado must implement the data store using RAMB18/RAMB36 rather
    // than LUTRAM.
    xpm_fifo_sync #(
        .DOUT_RESET_VALUE    ("0"),
        .ECC_MODE            ("no_ecc"),
        .FIFO_MEMORY_TYPE    ("block"),
        .FIFO_READ_LATENCY   (0),
        .FIFO_WRITE_DEPTH    (DEPTH),
        .FULL_RESET_VALUE    (0),
        .PROG_EMPTY_THRESH   (10),
        .PROG_FULL_THRESH    (10),
        .RD_DATA_COUNT_WIDTH (1),
        .READ_DATA_WIDTH     (WIDTH),
        .READ_MODE           ("fwft"),
        .SIM_ASSERT_CHK      (0),
        .USE_ADV_FEATURES    ("0000"),
        .WR_DATA_COUNT_WIDTH (1),
        .WRITE_DATA_WIDTH    (WIDTH)
    ) i_xpm_fifo_sync (
        .almost_empty   (),
        .almost_full    (),
        .data_valid     (),
        .dbiterr        (),
        .dout           (dout_int),
        .empty          (empty_int),
        .full           (full_int),
        .overflow       (),
        .prog_empty     (),
        .prog_full      (),
        .rd_data_count  (),
        .rd_rst_busy    (),
        .sbiterr        (),
        .underflow      (),
        .wr_ack         (),
        .wr_data_count  (),
        .wr_rst_busy    (),

        .din            (push_data_i),
        .injectdbiterr  (1'b0),
        .injectsbiterr  (1'b0),
        .rd_en          (pop_accept),
        .rst            (fifo_rst),
        .sleep          (1'b0),
        .wr_clk         (clk_i),
        .wr_en          (push_accept)
    );

endmodule
