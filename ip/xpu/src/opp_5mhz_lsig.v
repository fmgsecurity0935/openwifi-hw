// opp_5mhz_lsig.v — Minimal 5MHz L-SIG decoder for OPP packet-end prediction
// v15f: 8-bit DFT precision, Q-path removed, running twiddle phase counter
//
// Changes from v15e:
//   - DFT samples/twiddle: 16-bit → 8-bit (Q7 fixed-point)
//   - DFT accumulator: 24-bit → 16-bit
//   - cmult_q removed entirely (BPSK only needs Re sign)
//   - dft_sum_q removed
//   - Twiddle index: 2× 6x6 multiply → running 6-bit phase counter
//   - Net savings: ~200-400 LUT, 2 DSP, ~80 FF

module opp_5mhz_lsig (
    input  wire        clk,           // 100MHz system clock
    input  wire        rst_n,
    input  wire [15:0] iq_i,
    input  wire [15:0] iq_q,
    input  wire        iq_valid,      // 20MHz strobe
    output reg         lsig_valid,
    output reg  [3:0]  lsig_rate,
    output reg  [11:0] lsig_length,
    output reg         pkt_end_pulse,
    output reg  [15:0] pkt_remaining_us,
    input  wire        tsf_pulse_1M,
    // v17 debug outputs
    output wire [3:0]  debug_fsm,
    output wire [5:0]  debug_stf_plateau,
    output wire [15:0] debug_stf_power_hi,
    output wire        debug_lsig_valid_sticky,
    input  wire [31:0] stf_power_th_reg   // configurable STF threshold
);

// ============================================================================
// Section 1: CIC 4:1 Decimator (20MHz → 5MHz) — full 16-bit precision
// ============================================================================
reg signed [17:0] acc_i, acc_q;
reg [1:0]  dec_cnt;
reg signed [15:0] dec_i, dec_q;
reg        dec_valid;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        acc_i <= 0; acc_q <= 0;
        dec_cnt   <= 0;
        dec_i <= 0; dec_q <= 0;
        dec_valid <= 0;
    end else begin
        dec_valid <= 0;
        if (iq_valid) begin
            if (dec_cnt == 2'd0) begin
                acc_i <= {{2{iq_i[15]}}, iq_i};
                acc_q <= {{2{iq_q[15]}}, iq_q};
            end else begin
                acc_i <= acc_i + {{2{iq_i[15]}}, iq_i};
                acc_q <= acc_q + {{2{iq_q[15]}}, iq_q};
            end
            if (dec_cnt == 2'd3) begin
                dec_i <= (acc_i + {{2{iq_i[15]}}, iq_i}) >>> 2;
                dec_q <= (acc_q + {{2{iq_q[15]}}, iq_q}) >>> 2;
                dec_valid <= 1;
                dec_cnt <= 0;
            end else begin
                dec_cnt <= dec_cnt + 1;
            end
        end
    end
end

// ============================================================================
// Section 2: STF Detector (energy-based, full 16-bit precision)
// ============================================================================
(* use_dsp = "yes" *) wire [31:0] cur_power = dec_i * dec_i + dec_q * dec_q;

reg [31:0] stf_power;
reg [5:0]  stf_plateau_cnt;
reg        stf_detected;
localparam STF_MIN_PLATEAU = 6'd32;
wire [31:0] stf_th = (stf_power_th_reg != 0) ? stf_power_th_reg : 32'd5000;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        stf_power <= 0;
        stf_plateau_cnt <= 0;
        stf_detected <= 0;
    end else begin
        stf_detected <= 0;
        if (dec_valid) begin
            stf_power <= stf_power - (stf_power >> 4) + cur_power;
            if (stf_power > stf_th) begin
                if (stf_plateau_cnt < STF_MIN_PLATEAU)
                    stf_plateau_cnt <= stf_plateau_cnt + 1;
                else
                    stf_detected <= 1;
            end else begin
                stf_plateau_cnt <= 0;
            end
        end
    end
end

// ============================================================================
// Section 3: Main FSM
// ============================================================================

localparam [3:0]
    M_IDLE        = 4'd0,
    M_WAIT_LTF    = 4'd1,
    M_COLLECT_LTF = 4'd2,
    M_DFT_ADDR    = 4'd3,
    M_DFT_CALC    = 4'd4,
    M_VITERBI     = 4'd5,
    M_CHECK_SIG   = 4'd6,
    M_DURATION    = 4'd7,
    M_COUNTDOWN   = 4'd8;

reg [3:0]  main_fsm;
reg [5:0]  sample_cnt;
reg [6:0]  skip_cnt;

// ── LTF sample buffer: 8-bit I + 8-bit Q = 16 bits per entry ──
(* ram_style = "block" *) reg [15:0] ltf_buf [0:63];
reg [5:0]  ltf_rd_addr;
reg [15:0] ltf_rd_data;
always @(posedge clk)
    ltf_rd_data <= ltf_buf[ltf_rd_addr];

// ── Sign-only channel estimation ──
reg [47:0] ch_sign;

// ── DFT computation: 16-bit accumulator, I-channel only ──
reg [5:0]  dft_k_idx;
reg [5:0]  dft_n;
reg signed [15:0] dft_sum_i;

// ── Running twiddle phase counter (replaces 2× 6x6 multiply) ──
reg [5:0]  tw_phase;

reg        doing_lsig;
reg [47:0] bpsk_bits;
reg [47:0] coded_bits;

reg        vit_start;
wire       vit_done;
wire [23:0] vit_decoded;

reg [15:0] duration_timer_us;
reg        timer_running;

integer mi;

// ── Subcarrier map ──
function [5:0] subcarrier_map;
    input [5:0] k;
    begin
        case (k)
            0: subcarrier_map = 38; 1: subcarrier_map = 39;
            2: subcarrier_map = 40; 3: subcarrier_map = 41;
            4: subcarrier_map = 42;
            5: subcarrier_map = 44; 6: subcarrier_map = 45;
            7: subcarrier_map = 46; 8: subcarrier_map = 47;
            9: subcarrier_map = 48; 10: subcarrier_map = 49;
            11: subcarrier_map = 50; 12: subcarrier_map = 51;
            13: subcarrier_map = 52; 14: subcarrier_map = 53;
            15: subcarrier_map = 54; 16: subcarrier_map = 55;
            17: subcarrier_map = 56;
            18: subcarrier_map = 58; 19: subcarrier_map = 59;
            20: subcarrier_map = 60; 21: subcarrier_map = 61;
            22: subcarrier_map = 62; 23: subcarrier_map = 63;
            24: subcarrier_map = 1; 25: subcarrier_map = 2;
            26: subcarrier_map = 3; 27: subcarrier_map = 4;
            28: subcarrier_map = 5; 29: subcarrier_map = 6;
            30: subcarrier_map = 8; 31: subcarrier_map = 9;
            32: subcarrier_map = 10; 33: subcarrier_map = 11;
            34: subcarrier_map = 12; 35: subcarrier_map = 13;
            36: subcarrier_map = 14; 37: subcarrier_map = 15;
            38: subcarrier_map = 16; 39: subcarrier_map = 17;
            40: subcarrier_map = 18; 41: subcarrier_map = 19;
            42: subcarrier_map = 20;
            43: subcarrier_map = 22; 44: subcarrier_map = 23;
            45: subcarrier_map = 24; 46: subcarrier_map = 25;
            47: subcarrier_map = 26;
            default: subcarrier_map = 0;
        endcase
    end
endfunction

// ── Known LTF data subcarrier values ──
function signed [1:0] ltf_known;
    input [5:0] k;
    begin
        case (k)
            0: ltf_known =  2'sd1; 1: ltf_known =  2'sd1;
            2: ltf_known = -2'sd1; 3: ltf_known = -2'sd1;
            4: ltf_known =  2'sd1;
            5: ltf_known = -2'sd1; 6: ltf_known =  2'sd1;
            7: ltf_known = -2'sd1; 8: ltf_known =  2'sd1;
            9: ltf_known =  2'sd1; 10: ltf_known =  2'sd1;
            11: ltf_known =  2'sd1; 12: ltf_known =  2'sd1;
            13: ltf_known =  2'sd1;
            14: ltf_known = -2'sd1; 15: ltf_known = -2'sd1;
            16: ltf_known =  2'sd1; 17: ltf_known =  2'sd1;
            18: ltf_known =  2'sd1; 19: ltf_known = -2'sd1;
            20: ltf_known =  2'sd1; 21: ltf_known =  2'sd1;
            22: ltf_known =  2'sd1; 23: ltf_known =  2'sd1;
            24: ltf_known =  2'sd1; 25: ltf_known = -2'sd1;
            26: ltf_known = -2'sd1; 27: ltf_known =  2'sd1;
            28: ltf_known =  2'sd1; 29: ltf_known = -2'sd1;
            30: ltf_known = -2'sd1; 31: ltf_known =  2'sd1;
            32: ltf_known = -2'sd1; 33: ltf_known = -2'sd1;
            34: ltf_known = -2'sd1; 35: ltf_known = -2'sd1;
            36: ltf_known = -2'sd1; 37: ltf_known =  2'sd1;
            38: ltf_known =  2'sd1; 39: ltf_known = -2'sd1;
            40: ltf_known = -2'sd1; 41: ltf_known =  2'sd1;
            42: ltf_known = -2'sd1;
            43: ltf_known = -2'sd1; 44: ltf_known =  2'sd1;
            45: ltf_known =  2'sd1; 46: ltf_known =  2'sd1;
            47: ltf_known =  2'sd1;
            default: ltf_known = 2'sd1;
        endcase
    end
endfunction

// ── Deinterleaver ──
function [5:0] deint_map;
    input [5:0] k;
    begin
        case (k)
            0: deint_map = 0;  1: deint_map = 3;  2: deint_map = 6;
            3: deint_map = 9;  4: deint_map = 12; 5: deint_map = 15;
            6: deint_map = 18; 7: deint_map = 21; 8: deint_map = 24;
            9: deint_map = 27; 10: deint_map = 30; 11: deint_map = 33;
            12: deint_map = 36; 13: deint_map = 39; 14: deint_map = 42;
            15: deint_map = 45; 16: deint_map = 1;  17: deint_map = 4;
            18: deint_map = 7;  19: deint_map = 10; 20: deint_map = 13;
            21: deint_map = 16; 22: deint_map = 19; 23: deint_map = 22;
            24: deint_map = 25; 25: deint_map = 28; 26: deint_map = 31;
            27: deint_map = 34; 28: deint_map = 37; 29: deint_map = 40;
            30: deint_map = 43; 31: deint_map = 46; 32: deint_map = 2;
            33: deint_map = 5;  34: deint_map = 8;  35: deint_map = 11;
            36: deint_map = 14; 37: deint_map = 17; 38: deint_map = 20;
            39: deint_map = 23; 40: deint_map = 26; 41: deint_map = 29;
            42: deint_map = 32; 43: deint_map = 35; 44: deint_map = 38;
            45: deint_map = 41; 46: deint_map = 44; 47: deint_map = 47;
            default: deint_map = 0;
        endcase
    end
endfunction

// ── Twiddle factor ROM: 8-bit cos + 8-bit sin, Q7 (×127) ──
// Format: {sin[7:0], cos[7:0]}, 16-bit per entry
// Negative values in 2's complement: e.g. -12 → 244 (8'd256-12)
(* rom_style = "block" *) reg [15:0] twiddle_rom [0:63];
reg [5:0]  tw_rd_addr;
reg [15:0] tw_rd_data;
always @(posedge clk)
    tw_rd_data <= twiddle_rom[tw_rd_addr];

initial begin
    //              {  sin,     cos  }   angle = 2*pi*k/64
    twiddle_rom[0]  = {8'd0,   8'd127}; twiddle_rom[1]  = {8'd12,  8'd126};
    twiddle_rom[2]  = {8'd25,  8'd125}; twiddle_rom[3]  = {8'd37,  8'd122};
    twiddle_rom[4]  = {8'd49,  8'd117}; twiddle_rom[5]  = {8'd60,  8'd112};
    twiddle_rom[6]  = {8'd71,  8'd106}; twiddle_rom[7]  = {8'd81,  8'd98};
    twiddle_rom[8]  = {8'd90,  8'd90};  twiddle_rom[9]  = {8'd98,  8'd81};
    twiddle_rom[10] = {8'd106, 8'd71};  twiddle_rom[11] = {8'd112, 8'd60};
    twiddle_rom[12] = {8'd117, 8'd49};  twiddle_rom[13] = {8'd122, 8'd37};
    twiddle_rom[14] = {8'd125, 8'd25};  twiddle_rom[15] = {8'd126, 8'd12};
    twiddle_rom[16] = {8'd127, 8'd0};   twiddle_rom[17] = {8'd126, 8'd244};
    twiddle_rom[18] = {8'd125, 8'd231}; twiddle_rom[19] = {8'd122, 8'd219};
    twiddle_rom[20] = {8'd117, 8'd207}; twiddle_rom[21] = {8'd112, 8'd196};
    twiddle_rom[22] = {8'd106, 8'd185}; twiddle_rom[23] = {8'd98,  8'd175};
    twiddle_rom[24] = {8'd90,  8'd166}; twiddle_rom[25] = {8'd81,  8'd158};
    twiddle_rom[26] = {8'd71,  8'd150}; twiddle_rom[27] = {8'd60,  8'd144};
    twiddle_rom[28] = {8'd49,  8'd139}; twiddle_rom[29] = {8'd37,  8'd134};
    twiddle_rom[30] = {8'd25,  8'd131}; twiddle_rom[31] = {8'd12,  8'd130};
    twiddle_rom[32] = {8'd0,   8'd129}; twiddle_rom[33] = {8'd244, 8'd130};
    twiddle_rom[34] = {8'd231, 8'd131}; twiddle_rom[35] = {8'd219, 8'd134};
    twiddle_rom[36] = {8'd207, 8'd139}; twiddle_rom[37] = {8'd196, 8'd144};
    twiddle_rom[38] = {8'd185, 8'd150}; twiddle_rom[39] = {8'd175, 8'd158};
    twiddle_rom[40] = {8'd166, 8'd166}; twiddle_rom[41] = {8'd158, 8'd175};
    twiddle_rom[42] = {8'd150, 8'd185}; twiddle_rom[43] = {8'd144, 8'd196};
    twiddle_rom[44] = {8'd139, 8'd207}; twiddle_rom[45] = {8'd134, 8'd219};
    twiddle_rom[46] = {8'd131, 8'd231}; twiddle_rom[47] = {8'd130, 8'd244};
    twiddle_rom[48] = {8'd129, 8'd0};   twiddle_rom[49] = {8'd130, 8'd12};
    twiddle_rom[50] = {8'd131, 8'd25};  twiddle_rom[51] = {8'd134, 8'd37};
    twiddle_rom[52] = {8'd139, 8'd49};  twiddle_rom[53] = {8'd144, 8'd60};
    twiddle_rom[54] = {8'd150, 8'd71};  twiddle_rom[55] = {8'd158, 8'd81};
    twiddle_rom[56] = {8'd166, 8'd90};  twiddle_rom[57] = {8'd175, 8'd98};
    twiddle_rom[58] = {8'd185, 8'd106}; twiddle_rom[59] = {8'd196, 8'd112};
    twiddle_rom[60] = {8'd207, 8'd117}; twiddle_rom[61] = {8'd219, 8'd122};
    twiddle_rom[62] = {8'd231, 8'd125}; twiddle_rom[63] = {8'd244, 8'd126};
end

// ── DFT addressing ──
wire [5:0] dft_k_bin = subcarrier_map(dft_k_idx);

// ── Unpack BRAM data (8-bit signed) ──
wire signed [7:0] ltf_sample_i = ltf_rd_data[7:0];
wire signed [7:0] ltf_sample_q = ltf_rd_data[15:8];
wire signed [7:0] tw_cos = tw_rd_data[7:0];
wire signed [7:0] tw_sin = tw_rd_data[15:8];

// ── Complex multiply: I-channel ONLY (Q not needed for BPSK sign) ──
// 8s × 8s = 16s, two products summed = 17s, >>>7 = 10s effective
(* use_dsp = "yes" *) wire signed [15:0] cmult_i =
    (ltf_sample_i * tw_cos + ltf_sample_q * tw_sin) >>> 7;

// ── Final DFT real part ──
wire signed [15:0] final_dft_i = dft_sum_i + cmult_i;

// ── LTF known sign ──
wire [1:0] ltf_val = ltf_known(dft_k_idx);
wire ltf_is_neg = ltf_val[1];

// ── BPSK demod with sign-only equalization ──
wire bpsk_demod_bit = ~(final_dft_i[15] ^ ch_sign[dft_k_idx]);

// ── Duration computation ──
wire [15:0] total_data_bits = 16'd22 + ({4'b0, lsig_length} << 3);
reg [11:0] recip;
reg [7:0]  n_dbps_m1;
always @(*) begin
    case (lsig_rate)
        4'b1011: begin recip = 12'd2731; n_dbps_m1 = 8'd23;  end
        4'b1111: begin recip = 12'd1821; n_dbps_m1 = 8'd35;  end
        4'b1010: begin recip = 12'd1366; n_dbps_m1 = 8'd47;  end
        4'b1110: begin recip = 12'd911;  n_dbps_m1 = 8'd71;  end
        4'b1001: begin recip = 12'd683;  n_dbps_m1 = 8'd95;  end
        4'b1101: begin recip = 12'd456;  n_dbps_m1 = 8'd143; end
        4'b1000: begin recip = 12'd342;  n_dbps_m1 = 8'd191; end
        4'b1100: begin recip = 12'd304;  n_dbps_m1 = 8'd215; end
        default: begin recip = 12'd2731; n_dbps_m1 = 8'd23;  end
    endcase
end
wire [15:0] div_numer = total_data_bits + {8'b0, n_dbps_m1};
(* use_dsp = "yes" *) wire [27:0] div_prod = div_numer * recip;
wire [15:0] computed_duration_us = {div_prod[27:16], 4'b0};

// ── Viterbi decoder instance ──
opp_viterbi_k7 viterbi_inst (
    .clk(clk),
    .rst_n(rst_n),
    .start(vit_start),
    .coded_bits(coded_bits),
    .done(vit_done),
    .decoded_bits(vit_decoded)
);

// ============================================================================
// Section 4: Main FSM
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        main_fsm      <= M_IDLE;
        lsig_valid    <= 0;
        pkt_end_pulse <= 0;
        timer_running <= 0;
        vit_start     <= 0;
        doing_lsig    <= 0;
        sample_cnt    <= 0;
        skip_cnt      <= 0;
        dft_k_idx     <= 0;
        dft_n         <= 0;
        dft_sum_i     <= 0;
        tw_phase      <= 0;
        duration_timer_us <= 0;
        pkt_remaining_us  <= 0;
    end else begin
        lsig_valid    <= 0;
        pkt_end_pulse <= 0;
        vit_start     <= 0;

        // Duration countdown timer (runs independently)
        if (timer_running && tsf_pulse_1M) begin
            if (duration_timer_us == 16'd1) begin
                pkt_end_pulse <= 1;
                timer_running <= 0;
                duration_timer_us <= 0;
            end else begin
                duration_timer_us <= duration_timer_us - 1;
            end
            pkt_remaining_us <= duration_timer_us - 1;
        end

        case (main_fsm)

        M_IDLE: begin
            if (stf_detected) begin
                skip_cnt   <= 7'd48;
                sample_cnt <= 0;
                main_fsm   <= M_WAIT_LTF;
            end
        end

        M_WAIT_LTF: begin
            if (dec_valid) begin
                if (skip_cnt > 0) begin
                    skip_cnt <= skip_cnt - 1;
                end else begin
                    sample_cnt <= 0;
                    main_fsm   <= M_COLLECT_LTF;
                    doing_lsig <= 0;
                end
            end
        end

        M_COLLECT_LTF: begin
            if (dec_valid) begin
                // Store truncated 8-bit I/Q in LTF buffer
                ltf_buf[sample_cnt] <= {dec_q[15:8], dec_i[15:8]};
                if (sample_cnt == 6'd63) begin
                    dft_k_idx   <= 0;
                    dft_n       <= 0;
                    dft_sum_i   <= 0;
                    tw_phase    <= 0;
                    ltf_rd_addr <= 6'd0;
                    tw_rd_addr  <= 6'd0;
                    main_fsm    <= M_DFT_ADDR;
                end else begin
                    sample_cnt <= sample_cnt + 1;
                end
            end
        end

        // ── DFT Pipeline Stage 1: BRAM address setup ──
        M_DFT_ADDR: begin
            main_fsm <= M_DFT_CALC;
        end

        // ── DFT Pipeline Stage 2: Multiply + Accumulate (I-only) ──
        M_DFT_CALC: begin
            dft_sum_i <= dft_sum_i + cmult_i;

            if (dft_n == 6'd63) begin
                // Finished one subcarrier DFT
                if (!doing_lsig) begin
                    // LTF: sign-only channel estimate
                    ch_sign <= {final_dft_i[15] ^ ltf_is_neg, ch_sign[47:1]};
                end else begin
                    // L-SIG: BPSK demod
                    bpsk_bits <= {bpsk_demod_bit, bpsk_bits[47:1]};
                end

                if (dft_k_idx == 6'd47) begin
                    if (!doing_lsig) begin
                        // LTF done → skip to L-SIG
                        skip_cnt   <= 7'd80;
                        sample_cnt <= 0;
                        doing_lsig <= 1;
                        main_fsm   <= M_WAIT_LTF;
                    end else begin
                        // L-SIG done → deinterleave + decode
                        sample_cnt <= 0;
                        main_fsm   <= M_VITERBI;
                    end
                end else begin
                    // Next subcarrier: reset accumulator and phase
                    dft_k_idx   <= dft_k_idx + 1;
                    dft_n       <= 0;
                    dft_sum_i   <= 0;
                    tw_phase    <= 0;
                    ltf_rd_addr <= 6'd0;
                    tw_rd_addr  <= 6'd0;
                    main_fsm    <= M_DFT_ADDR;
                end
            end else begin
                // Advance to next sample, update running phase
                dft_n       <= dft_n + 1;
                tw_phase    <= tw_phase + dft_k_bin;
                ltf_rd_addr <= dft_n + 1;
                tw_rd_addr  <= tw_phase + dft_k_bin;
                main_fsm    <= M_DFT_ADDR;
            end
        end

        M_VITERBI: begin
            if (sample_cnt == 0) begin
                for (mi = 0; mi < 48; mi = mi + 1)
                    coded_bits[mi] <= bpsk_bits[deint_map(mi[5:0])];
                sample_cnt <= 1;
            end else if (sample_cnt == 1) begin
                vit_start  <= 1;
                sample_cnt <= 2;
            end else if (vit_done) begin
                main_fsm <= M_CHECK_SIG;
            end
        end

        M_CHECK_SIG: begin
            if (^vit_decoded[17:0] != 1'b0) begin
                main_fsm <= M_IDLE;
            end else if (vit_decoded[4] != 1'b0) begin
                main_fsm <= M_IDLE;
            end else if (vit_decoded[3] != 1'b1) begin
                main_fsm <= M_IDLE;
            end else begin
                lsig_rate   <= vit_decoded[3:0];
                lsig_length <= vit_decoded[16:5];
                lsig_valid  <= 1;
                main_fsm    <= M_DURATION;
            end
        end

        M_DURATION: begin
            main_fsm <= M_COUNTDOWN;
        end

        M_COUNTDOWN: begin
            if (!timer_running) begin
                duration_timer_us <= computed_duration_us;
                pkt_remaining_us  <= computed_duration_us;
                timer_running     <= 1;
                main_fsm          <= M_IDLE;
            end
        end

        default: main_fsm <= M_IDLE;
        endcase
    end
end

// v17 debug: sticky flag for lsig_valid (cleared on reset only)
reg lsig_valid_sticky;
always @(posedge clk or negedge rst_n)
    if (!rst_n) lsig_valid_sticky <= 0;
    else if (lsig_valid) lsig_valid_sticky <= 1;

assign debug_fsm               = main_fsm;
assign debug_stf_plateau       = stf_plateau_cnt;
assign debug_stf_power_hi      = stf_power[31:16];
assign debug_lsig_valid_sticky = lsig_valid_sticky;

endmodule
