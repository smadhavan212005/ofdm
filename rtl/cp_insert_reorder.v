// ----------------------------------------------------------------------------
// cp_insert_reorder : bit-reversal reorder + cyclic-prefix insertion
//
//   Ping-pong buffer (2 banks x N complex words).
//   Write side : takes one symbol in BIT-REVERSED order, stores sample j at address
//                bitrev(j), so the bank ends up in natural time order.
//   Read side  : emits  y[N-CP .. N-1], y[0 .. N-1]   = N+CP beats per symbol,
//                AXI-Stream with tlast on the final beat of each symbol.
//   Input is stalled (i_ready=0) only when both banks are full, i.e. when the sink
//   (or the N/(N+CP) rate difference) cannot keep up.
// ----------------------------------------------------------------------------
`timescale 1ns/1ps
module cp_insert_reorder #(
    parameter N  = 64,
    parameter DW = 16,
    parameter CP = 16                 // cyclic prefix length, 0..N
)(
    input  wire                 clk,
    input  wire                 rst_n,
    // from IFFT core (bit-reversed order)
    input  wire                 i_valid,
    output wire                 i_ready,
    input  wire [2*DW-1:0]      i_data,           // {im, re}
    // AXI-Stream master
    output reg  [2*DW-1:0]      m_axis_tdata,     // {im, re}
    output reg                  m_axis_tvalid,
    input  wire                 m_axis_tready,
    output reg                  m_axis_tlast
);
    localparam LOGN = $clog2(N);
    localparam TOT  = N + CP;
    localparam RW   = $clog2(TOT + 1);

    function [LOGN-1:0] brev;
        input [LOGN-1:0] a;
        integer b;
        begin
            for (b = 0; b < LOGN; b = b + 1) brev[b] = a[LOGN-1-b];
        end
    endfunction

    reg [2*DW-1:0] mem [0:2*N-1];                 // {bank, addr}

    // bank-full flags as toggle pairs (single writer each -> no multi-driver issue)
    reg [1:0] wtog, rtog;
    wire [1:0] full = wtog ^ rtog;

    // ---------------- write side ----------------
    reg [LOGN-1:0] wcnt;
    reg            wbank;
    assign i_ready = ~full[wbank];
    wire   wr_en   = i_valid & i_ready;

    always @(posedge clk) begin
        if (wr_en) mem[{wbank, brev(wcnt)}] <= i_data;
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            wcnt <= 0; wbank <= 1'b0; wtog <= 2'b00;
        end else if (wr_en) begin
            if (wcnt == N - 1) begin
                wcnt        <= 0;
                wtog[wbank] <= ~wtog[wbank];
                wbank       <= ~wbank;
            end else begin
                wcnt <= wcnt + 1'b1;
            end
        end
    end

    // ---------------- read side ----------------
    reg [RW-1:0] ridx;
    reg          rbank;
    wire         adv      = ~m_axis_tvalid | m_axis_tready;
    wire [RW-1:0] raddr_w = (ridx < CP) ? (N - CP + ridx) : (ridx - CP);
    wire         rd_issue = adv & full[rbank];

    always @(posedge clk) begin
        if (rd_issue) m_axis_tdata <= mem[{rbank, raddr_w[LOGN-1:0]}];
    end

    always @(posedge clk) begin
        if (!rst_n) begin
            ridx <= 0; rbank <= 1'b0; rtog <= 2'b00;
            m_axis_tvalid <= 1'b0; m_axis_tlast <= 1'b0;
        end else if (adv) begin
            m_axis_tvalid <= full[rbank];
            m_axis_tlast  <= full[rbank] & (ridx == TOT - 1);
            if (full[rbank]) begin
                if (ridx == TOT - 1) begin
                    ridx        <= 0;
                    rtog[rbank] <= ~rtog[rbank];
                    rbank       <= ~rbank;
                end else begin
                    ridx <= ridx + 1'b1;
                end
            end
        end
    end
endmodule
