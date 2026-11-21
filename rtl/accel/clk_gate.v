// ----------------------------------------------------------------------------
// clk_gate : latch-based integrated clock gate (ICG), behavioural model
//
//   The enable is captured by a latch that is transparent while clk is LOW, so
//   `en` may change at any time during the low phase and gclk never glitches.
//     gclk = clk & latched_en
//   test_en forces the clock on (scan / DFT override).
//
//   Technology note: ASIC flows should map this to the library ICG cell; FPGA flows
//   should replace it with the vendor primitive (Xilinx BUFGCE / Intel ALTCLKCTRL)
//   or, for small designs, just use a clock-enable on the flops (CLK_GATE=0 in
//   systolic_mm does exactly that).
// ----------------------------------------------------------------------------
`timescale 1ns/1ps
module clk_gate (
    input  wire clk,
    input  wire en,
    input  wire test_en,
    output wire gclk
);
    reg en_l;
    initial en_l = 1'b0;
    always @(clk or en or test_en)
        if (!clk) en_l = en | test_en;      // transparent-low latch
    assign gclk = clk & en_l;
endmodule
