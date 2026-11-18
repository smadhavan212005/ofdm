// Self-checking testbench for ofdm_ifft_tx.
// Reads stim.hex / expected.hex (made by model/gen_vectors.m), writes rtl_out.hex.
//
// STALL_MODE 0 : source always valid, sink always ready (max throughput)
//            1 : random source gaps (incl. mid-symbol) + random sink back-pressure
//            2 : mode 1 + long idle gaps between symbols (forces auto-flush)
//                + occasional long sink stalls (forces both ping-pong banks full)
// TOL_LSB      : allowed |RTL - golden| per I/Q component (0 = bit exact)
`timescale 1ns/1ps
module tb_ofdm_ifft_tx;
    parameter N          = 64;
    parameter DW         = 16;
    parameter TW         = 16;
    parameter CP         = 16;
    parameter BPS        = 0;
    parameter NSYM       = 8;
    parameter SCALE_MASK = -1;
    parameter FLUSH_IDLE = 8;
    parameter STALL_MODE = 0;
    parameter TOL_LSB    = 0;
    parameter QAM_PEAK   = (1 << (DW - 2));

    localparam S_W   = (BPS == 0) ? 2*DW : BPS;
    localparam TOT_I = NSYM * N;
    localparam TOT_O = NSYM * (N + CP);

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg  [S_W-1:0]   s_data  = 0;
    reg              s_valid = 0;
    wire             s_ready;
    wire [2*DW-1:0]  m_data;
    wire             m_valid, m_last;
    reg              m_ready = 0;

    ofdm_ifft_tx #(.N(N), .DW(DW), .TW(TW), .CP(CP), .BPS(BPS), .QAM_PEAK(QAM_PEAK),
                   .SCALE_MASK(SCALE_MASK), .FLUSH_IDLE(FLUSH_IDLE), .TW_FILE("twiddle.mem")) dut (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tdata(s_data), .s_axis_tvalid(s_valid), .s_axis_tready(s_ready), .s_axis_tlast(1'b0),
        .m_axis_tdata(m_data), .m_axis_tvalid(m_valid), .m_axis_tready(m_ready), .m_axis_tlast(m_last));

    reg [63:0] stim [0:TOT_I-1];
    reg [63:0] expd [0:TOT_O-1];
    initial begin
        $readmemh("stim.hex", stim);
        $readmemh("expected.hex", expd);
    end

    // ---------------------------------------------------------------- source
    integer seed = 12345;
    integer idx = 0;           // next input sample to offer
    integer gap = 0;           // remaining idle cycles before next offer
    integer sent = 0;
    always @(posedge clk) begin
        if (!rst_n) begin
            s_valid <= 0; idx <= 0; gap <= 0; sent <= 0;
        end else begin
            if (s_valid && s_ready) begin
                s_valid <= 0;
                sent    <= sent + 1;
            end
            if (!s_valid || s_ready) begin
                if (gap > 0) begin
                    gap <= gap - 1;
                end else if (idx < TOT_I) begin
                    // decide whether to insert a gap *before* this sample
                    if (STALL_MODE >= 1 && ($random(seed) & 3) == 0 && !(idx != 0 && idx % N == 0 && STALL_MODE == 2 && ($random(seed) & 1))) begin
                        gap <= ($random(seed) & 3);
                        s_valid <= 0;
                    end else if (STALL_MODE == 2 && idx != 0 && (idx % N) == 0 && sent == idx && ($random(seed) & 1)) begin
                        // long idle at symbol boundary: longer than FLUSH_IDLE, sometimes longer than a flush
                        gap <= FLUSH_IDLE + 2 + (($random(seed) & 3) * (N / 2));
                        s_valid <= 0;
                    end else begin
                        s_valid <= 1;
                        s_data  <= stim[idx][S_W-1:0];
                        idx     <= idx + 1;
                    end
                end
            end
        end
    end

    // ---------------------------------------------------------------- sink
    integer stall_cnt = 0;
    always @(posedge clk) begin
        if (!rst_n) begin
            m_ready <= 0; stall_cnt <= 0;
        end else begin
            case (STALL_MODE)
                0: m_ready <= 1;
                1: m_ready <= (($random(seed) & 7) < 5);
                default: begin
                    if (stall_cnt > 0) begin
                        stall_cnt <= stall_cnt - 1;
                        m_ready   <= 0;
                    end else if (($random(seed) & 127) == 0) begin
                        stall_cnt <= 20 + ($random(seed) & 63);
                        m_ready   <= 0;
                    end else begin
                        m_ready <= (($random(seed) & 7) < 5);
                    end
                end
            endcase
        end
    end

    // ---------------------------------------------------------------- monitor / checker
    integer n_out = 0, errors = 0, maxerr = 0, last_errs = 0;
    integer f_out;
    integer e_re, e_im, ae;
    reg signed [DW-1:0] g_re, g_im, r_re, r_im;
    integer pos_in_sym;
    initial f_out = $fopen("rtl_out.hex", "w");

    always @(posedge clk) begin
        if (rst_n && m_valid && m_ready) begin
            $fwrite(f_out, "%016X\n", {{(64-2*DW){1'b0}}, m_data});
            if (n_out < TOT_O) begin
                g_re = expd[n_out][DW-1:0];       g_im = expd[n_out][2*DW-1:DW];
                r_re = m_data[DW-1:0];            r_im = m_data[2*DW-1:DW];
                e_re = r_re - g_re;               e_im = r_im - g_im;
                ae = (e_re < 0 ? -e_re : e_re); if (ae > maxerr) maxerr = ae;
                ae = (e_im < 0 ? -e_im : e_im); if (ae > maxerr) maxerr = ae;
                if ((e_re < -TOL_LSB || e_re > TOL_LSB || e_im < -TOL_LSB || e_im > TOL_LSB)) begin
                    errors = errors + 1;
                    if (errors <= 10)
                        $display("MISMATCH out#%0d (sym %0d, beat %0d): rtl=(%0d,%0d) exp=(%0d,%0d)",
                                 n_out, n_out/(N+CP), n_out%(N+CP), r_re, r_im, g_re, g_im);
                end
                pos_in_sym = n_out % (N + CP);
                if (m_last !== (pos_in_sym == N + CP - 1)) begin
                    errors = errors + 1;
                    if (errors <= 10) $display("TLAST error at out#%0d (beat %0d)", n_out, pos_in_sym);
                end
            end else begin
                errors = errors + 1;
                if (errors <= 10) $display("EXTRA output beat #%0d", n_out);
            end
            n_out = n_out + 1;
        end
    end

    // ---------------------------------------------------------------- sequencing
    integer cyc = 0, max_cyc;
    always @(posedge clk) cyc <= cyc + 1;
    integer t_first = -1, t_last = -1;
    always @(posedge clk) if (rst_n && m_valid && m_ready) begin
        if (t_first < 0) t_first = cyc;
        t_last = cyc;
    end

    initial begin
        max_cyc = (NSYM * (N + CP) + 4 * N + 64) * ((STALL_MODE == 0) ? 3 : 40);
        repeat (5) @(posedge clk);
        rst_n <= 1;
        // wait for all outputs or timeout
        while (n_out < TOT_O && cyc < max_cyc) @(posedge clk);
        // make sure nothing extra shows up
        repeat (4 * N + 2 * K_LAT) @(posedge clk);
        $display("-----------------------------------------------------------");
        $display("N=%0d DW=%0d TW=%0d CP=%0d BPS=%0d NSYM=%0d STALL_MODE=%0d FLUSH_IDLE=%0d",
                 N, DW, TW, CP, BPS, NSYM, STALL_MODE, FLUSH_IDLE);
        $display("outputs: %0d / %0d, max |err| = %0d LSB (tol %0d), errors = %0d", n_out, TOT_O, maxerr, TOL_LSB, errors);
        if (STALL_MODE == 0 && n_out == TOT_O)
            $display("first output @cycle %0d, last @cycle %0d  (%0d cycles for %0d beats)", t_first, t_last, t_last - t_first + 1, TOT_O);
        if (errors == 0 && n_out == TOT_O) $display("TB PASS");
        else                               $display("TB FAIL");
        $fclose(f_out);
        $finish;
    end
    localparam K_LAT = N + 2*$clog2(N) - 2;
endmodule
