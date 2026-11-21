// ----------------------------------------------------------------------------
// mac_pe : one processing element of the output-stationary systolic array
//
//   acc    <= acc + a_in * b_in      when v_in (a valid operand pair is present)
//   a_out  <= a_in, b_out <= b_in    forwarded one hop (right / down) every clock
//   v_out  <= v_in                   valid token travels with the A operand
//   clr    synchronous: acc <= 0, v_out <= 0 (start of a new tile)
//   ce     clock enable (tied 1 when the clock itself is gated)
//
//   Operands are signed DW bits (INT8 by default), accumulator signed AW bits
//   (wraps on overflow, matching the MATLAB model). Requires AW >= 2*DW.
// ----------------------------------------------------------------------------
`timescale 1ns/1ps
module mac_pe #(
    parameter DW = 8,
    parameter AW = 32
)(
    input  wire                  clk,
    input  wire                  ce,
    input  wire                  clr,
    input  wire                  v_in,
    input  wire signed [DW-1:0]  a_in,
    input  wire signed [DW-1:0]  b_in,
    output reg                   v_out,
    output reg  signed [DW-1:0]  a_out,
    output reg  signed [DW-1:0]  b_out,
    output reg  signed [AW-1:0]  acc
);
    initial begin v_out = 0; a_out = 0; b_out = 0; acc = 0; end

    always @(posedge clk) if (ce) begin
        a_out <= a_in;
        b_out <= b_in;
        if (clr) begin
            acc   <= {AW{1'b0}};
            v_out <= 1'b0;                 // flush any stale valid token left from a previous tile
        end else begin
            v_out <= v_in;
            if (v_in) acc <= acc + a_in * b_in;
        end
    end
endmodule
