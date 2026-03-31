// opp_5mhz_lsig.v — 5MHz L-SIG decoder for OPP packet-end prediction
// v22: v19 dual-DFT architecture (LTF ch_sign + L-SIG demod) + v21 ce gating
//
// v19 had correct L-SIG decoding but WNS=-1.861ns (timing violations)
// v21 had ce gating solving timing (WNS=-0.423ns) but no LTF → decode failed
// v22 combines both: dual DFT with ce gating for timing + channel estimation
//
// Architecture:
//   CIC 4:1 (20→5MHz) → STF detect → collect LTF(64) + skip(80) + L-SIG(64)
//   → DFT pass 1: LTF → ch_sign[47:0] (channel estimation)
//   → DFT pass 2: L-SIG → equalized BPSK demod
//   → Viterbi → parity check → duration countdown → pkt_end_pulse
//
// CE gating: DFT/Viterbi/Duration states run at effective 50MHz (20ns budget)
//            CIC, STF, sample collection remain at full 100MHz

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
    // debug outputs
    output wire [3:0]  debug_fsm,
    output wire [5:0]  debug_stf_plateau,
    output wire [15:0] debug_stf_power_hi,
    output wire        debug_lsig_valid_sticky,
    input  wire [31:0] stf_power_th_reg
);

// ============================================================================
// Section 0: Clock Enable — halves effective frequency for DFT+ states
// ============================================================================
reg ce;
always @(posedge clk or negedge rst_n)
    if (!rst_n) ce <= 0;
    else        ce <= ~ce;

// ============================================================================
// Section 1: CIC 4:1 Decimator (20MHz → 5MHz) — UNGATED
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
// Section 2: STF Detector (energy-based) — UNGATED
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
// Section 3: Main FSM declarations
// ============================================================================

localparam [3:0]
    M_IDLE         = 4'd0,
    M_WAIT_LTF     = 4'd1,
    M_COLLECT      = 4'd2,
    M_DFT_ADDR     = 4'd3,
    M_DFT_MULT     = 4'd9,
    M_DFT_CALC     = 4'd4,
    M_VITERBI      = 4'd5,
    M_CHECK_SIG    = 4'd6,
    M_DURATION     = 4'd7,
    M_COUNTDOWN    = 4'd8,
    M_RETRY        = 4'd10;

// Dual DFT at 50MHz effective: 2 × (48×64×6) = 36864 clocks ≈ 369µs
localparam [15:0] DFT_DELAY_US = 16'd372;

reg [3:0]  main_fsm;
reg [6:0]  sample_cnt;      // 0..127 (128-entry buffer)
reg [8:0]  skip_cnt;        // skip between LTF and L-SIG
reg [1:0]  collect_phase;   // 0=LTF, 1=skip CP, 2=L-SIG

// ── Sample buffer: 128 entries (LTF [0:63] + L-SIG [64:127]) ──
(* ram_style = "distributed" *) reg [15:0] ltf_buf [0:127];
reg [6:0]  buf_rd_addr;
reg [15:0] buf_rd_data;
always @(posedge clk)
    buf_rd_data <= ltf_buf[buf_rd_addr];

// ── DFT computation ──
reg [5:0]  dft_k_idx;
reg [5:0]  dft_n;
reg signed [15:0] dft_sum_i;
reg [5:0]  tw_phase;

// ── Channel estimation from LTF ──
reg [47:0] ch_sign;         // 1 bit per subcarrier: sign of LTF DFT real part
reg        doing_lsig;      // 0=LTF DFT pass, 1=L-SIG DFT pass
reg [6:0]  dft_base;        // 0 for LTF, 64 for L-SIG

reg [47:0] bpsk_bits;
reg [47:0] coded_bits;
reg        phase_flipped;

reg        vit_start;
wire       vit_done;
wire [23:0] vit_decoded;

reg [15:0] duration_timer_us;
reg        timer_running;
reg [15:0] computed_duration_reg;

integer mi;

// ── Known LTF subcarrier pattern (802.11a Table L-6) ──
// Returns expected sign: 0 = positive, 1 = negative
function ltf_known;
    input [5:0] k;
    begin
        case (k)
            6'd0:  ltf_known = 0; 6'd1:  ltf_known = 0;
            6'd2:  ltf_known = 1; 6'd3:  ltf_known = 1;
            6'd4:  ltf_known = 0; 6'd5:  ltf_known = 0;
            6'd6:  ltf_known = 0; 6'd7:  ltf_known = 0;
            6'd8:  ltf_known = 0; 6'd9:  ltf_known = 0;
            6'd10: ltf_known = 0; 6'd11: ltf_known = 1;
            6'd12: ltf_known = 1; 6'd13: ltf_known = 0;
            6'd14: ltf_known = 0; 6'd15: ltf_known = 0;
            6'd16: ltf_known = 0; 6'd17: ltf_known = 1;
            6'd18: ltf_known = 0; 6'd19: ltf_known = 0;
            6'd20: ltf_known = 0; 6'd21: ltf_known = 0;
            6'd22: ltf_known = 0; 6'd23: ltf_known = 0;
            6'd24: ltf_known = 0; 6'd25: ltf_known = 0;
            6'd26: ltf_known = 0; 6'd27: ltf_known = 0;
            6'd28: ltf_known = 1; 6'd29: ltf_known = 1;
            6'd30: ltf_known = 0; 6'd31: ltf_known = 0;
            6'd32: ltf_known = 0; 6'd33: ltf_known = 0;
            6'd34: ltf_known = 0; 6'd35: ltf_known = 0;
            6'd36: ltf_known = 1; 6'd37: ltf_known = 1;
            6'd38: ltf_known = 0; 6'd39: ltf_known = 0;
            6'd40: ltf_known = 0; 6'd41: ltf_known = 0;
            6'd42: ltf_known = 0; 6'd43: ltf_known = 0;
            6'd44: ltf_known = 1; 6'd45: ltf_known = 1;
            6'd46: ltf_known = 1; 6'd47: ltf_known = 0;
            default: ltf_known = 0;
        endcase
    end
endfunction

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

// ── Twiddle factor ROM ──
(* rom_style = "distributed" *) reg [15:0] twiddle_rom [0:63];
reg [5:0]  tw_rd_addr;
reg [15:0] tw_rd_data;
always @(posedge clk)
    tw_rd_data <= twiddle_rom[tw_rd_addr];

initial begin
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

// ── Unpack buffer data (8-bit signed) ──
wire signed [7:0] buf_sample_i = buf_rd_data[7:0];
wire signed [7:0] buf_sample_q = buf_rd_data[15:8];
wire signed [7:0] tw_cos = tw_rd_data[7:0];
wire signed [7:0] tw_sin = tw_rd_data[15:8];

// ── Complex multiply: I-channel ONLY ──
(* use_dsp = "yes" *) wire signed [15:0] cmult_i =
    (buf_sample_i * tw_cos + buf_sample_q * tw_sin) >>> 7;

// ── Pipeline register for cmult_i ──
reg signed [15:0] cmult_i_pipe;

// ── Final DFT real part ──
wire signed [15:0] final_dft_i = dft_sum_i + cmult_i_pipe;

// ── Equalized BPSK demod: use ch_sign from LTF ──
// ch_sign[k]=0 means LTF DFT was positive (channel non-inverting)
// ch_sign[k]=1 means LTF DFT was negative (channel inverting) → flip L-SIG
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
        main_fsm       <= M_IDLE;
        lsig_valid     <= 0;
        pkt_end_pulse  <= 0;
        timer_running  <= 0;
        vit_start      <= 0;
        sample_cnt     <= 0;
        skip_cnt       <= 0;
        collect_phase  <= 0;
        phase_flipped  <= 0;
        doing_lsig     <= 0;
        dft_base       <= 0;
        dft_k_idx      <= 0;
        dft_n          <= 0;
        dft_sum_i      <= 0;
        tw_phase       <= 0;
        ch_sign        <= 0;
        duration_timer_us <= 0;
        pkt_remaining_us  <= 0;
    end else begin
        // ── Ungated defaults (every cycle) ──
        lsig_valid    <= 0;
        pkt_end_pulse <= 0;
        vit_start     <= 0;

        // ── Duration countdown timer — UNGATED (needs 1µs precision) ──
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

        // ── UNGATED states: sample collection ──

        M_IDLE: begin
            if (stf_detected) begin
                // After STF (160 samples plateau), skip LTF GI (32 samples)
                // then collect LTF1 (64), skip CP (16), collect L-SIG (64)
                skip_cnt      <= 9'd32;   // skip LTF guard interval
                sample_cnt    <= 0;
                collect_phase <= 2'd0;    // start with LTF collection
                phase_flipped <= 0;
                doing_lsig    <= 0;
                dft_base      <= 7'd0;
                ch_sign       <= 0;
                main_fsm      <= M_WAIT_LTF;
            end
        end

        M_WAIT_LTF: begin
            if (dec_valid) begin
                if (skip_cnt > 0) begin
                    skip_cnt <= skip_cnt - 1;
                end else begin
                    sample_cnt <= 0;
                    main_fsm   <= M_COLLECT;
                end
            end
        end

        M_COLLECT: begin
            if (dec_valid) begin
                case (collect_phase)
                2'd0: begin
                    // Collecting LTF (64 samples → buf[0:63])
                    ltf_buf[sample_cnt[6:0]] <= {dec_q[11:4], dec_i[11:4]};
                    if (sample_cnt[5:0] == 6'd63) begin
                        // LTF done, skip LTF2(64) + L-SIG CP(16) = 80 samples
                        skip_cnt      <= 9'd80;
                        collect_phase <= 2'd2;
                        main_fsm      <= M_WAIT_LTF;
                    end else begin
                        sample_cnt <= sample_cnt + 1;
                    end
                end
                2'd2: begin
                    // Collecting L-SIG (64 samples → buf[64:127])
                    ltf_buf[{1'b1, sample_cnt[5:0]}] <= {dec_q[11:4], dec_i[11:4]};
                    if (sample_cnt[5:0] == 6'd63) begin
                        // Both LTF and L-SIG collected, start LTF DFT
                        dft_k_idx   <= 0;
                        dft_n       <= 0;
                        dft_sum_i   <= 0;
                        tw_phase    <= 0;
                        doing_lsig  <= 0;
                        dft_base    <= 7'd0;    // LTF at buf[0:63]
                        buf_rd_addr <= 7'd0;
                        tw_rd_addr  <= 6'd0;
                        main_fsm    <= M_DFT_ADDR;
                    end else begin
                        sample_cnt <= sample_cnt + 1;
                    end
                end
                default: begin
                    main_fsm <= M_IDLE;
                end
                endcase
            end
        end

        // ── CE-GATED states: DFT + decode (effective 50MHz) ──

        M_DFT_ADDR: if (ce) begin
            main_fsm <= M_DFT_MULT;
        end

        M_DFT_MULT: if (ce) begin
            cmult_i_pipe <= cmult_i;
            main_fsm <= M_DFT_CALC;
        end

        M_DFT_CALC: if (ce) begin
            dft_sum_i <= dft_sum_i + cmult_i_pipe;

            if (dft_n == 6'd63) begin
                if (!doing_lsig) begin
                    // LTF pass: store channel sign
                    // ch_sign = observed_sign XOR known_sign
                    ch_sign[dft_k_idx] <= final_dft_i[15] ^ ltf_known(dft_k_idx);
                end else begin
                    // L-SIG pass: equalized BPSK demod
                    bpsk_bits <= {bpsk_demod_bit, bpsk_bits[47:1]};
                end

                if (dft_k_idx == 6'd47) begin
                    if (!doing_lsig) begin
                        // LTF DFT done → start L-SIG DFT
                        doing_lsig  <= 1;
                        dft_base    <= 7'd64;   // L-SIG at buf[64:127]
                        dft_k_idx   <= 0;
                        dft_n       <= 0;
                        dft_sum_i   <= 0;
                        tw_phase    <= 0;
                        buf_rd_addr <= 7'd64;
                        tw_rd_addr  <= 6'd0;
                        main_fsm    <= M_DFT_ADDR;
                    end else begin
                        // L-SIG DFT done → Viterbi
                        sample_cnt <= 0;
                        main_fsm   <= M_VITERBI;
                    end
                end else begin
                    dft_k_idx   <= dft_k_idx + 1;
                    dft_n       <= 0;
                    dft_sum_i   <= 0;
                    tw_phase    <= 0;
                    buf_rd_addr <= dft_base;
                    tw_rd_addr  <= 6'd0;
                    main_fsm    <= M_DFT_ADDR;
                end
            end else begin
                dft_n       <= dft_n + 1;
                tw_phase    <= tw_phase + dft_k_bin;
                buf_rd_addr <= dft_base + {1'b0, dft_n[5:0]} + 7'd1;
                tw_rd_addr  <= tw_phase + dft_k_bin;
                main_fsm    <= M_DFT_ADDR;
            end
        end

        M_VITERBI: if (ce) begin
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

        M_CHECK_SIG: if (ce) begin
            if (^vit_decoded[17:0] != 1'b0) begin
                if (!phase_flipped)
                    main_fsm <= M_RETRY;
                else
                    main_fsm <= M_IDLE;
            end else if (vit_decoded[4] != 1'b0) begin
                if (!phase_flipped)
                    main_fsm <= M_RETRY;
                else
                    main_fsm <= M_IDLE;
            end else if (vit_decoded[3] != 1'b1) begin
                if (!phase_flipped)
                    main_fsm <= M_RETRY;
                else
                    main_fsm <= M_IDLE;
            end else begin
                lsig_rate   <= vit_decoded[3:0];
                lsig_length <= vit_decoded[16:5];
                lsig_valid  <= 1;
                main_fsm    <= M_DURATION;
            end
        end

        M_RETRY: if (ce) begin
            bpsk_bits     <= ~bpsk_bits;
            phase_flipped <= 1;
            sample_cnt    <= 0;
            main_fsm      <= M_VITERBI;
        end

        M_DURATION: if (ce) begin
            computed_duration_reg <= computed_duration_us;
            main_fsm <= M_COUNTDOWN;
        end

        M_COUNTDOWN: if (ce) begin
            if (computed_duration_reg > DFT_DELAY_US) begin
                duration_timer_us <= computed_duration_reg - DFT_DELAY_US;
                pkt_remaining_us  <= computed_duration_reg - DFT_DELAY_US;
                timer_running     <= 1;
            end else begin
                pkt_end_pulse <= 1;
            end
            main_fsm <= M_IDLE;
        end

        default: main_fsm <= M_IDLE;
        endcase
    end
end

// Debug: sticky flag for lsig_valid (cleared on reset only)
reg lsig_valid_sticky;
always @(posedge clk or negedge rst_n)
    if (!rst_n) lsig_valid_sticky <= 0;
    else if (lsig_valid) lsig_valid_sticky <= 1;

assign debug_fsm               = main_fsm;
assign debug_stf_plateau       = stf_plateau_cnt;
assign debug_stf_power_hi      = stf_power[31:16];
assign debug_lsig_valid_sticky = lsig_valid_sticky;

endmodule
