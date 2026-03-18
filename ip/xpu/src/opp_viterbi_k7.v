// opp_viterbi_k7.v — Feedback decoder for 802.11 L-SIG
// Replaces Viterbi K=7 with sequential feedback decoding for area savings.
// Rate 1/2, g0=133o (0x5B), g1=171o (0x79)
// 48 coded bits → 24 data bits
//
// Architecture: Sequential feedback using g1 polynomial
//   u[n] = coded_bits[2n+1] ^ u[n-3] ^ u[n-4] ^ u[n-5] ^ u[n-6]
//
// No error correction (errors propagate), but:
//   - At >10dB SNR (typical for nearby 5MHz TX), coded BER ~0.2%
//   - L-SIG parity check catches decode errors → packet skipped
//   - Much smaller than Viterbi: ~37 FFs vs ~2500 FFs
//
// Latency: 24 clocks (1 per data bit)

module opp_viterbi_k7 (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start,          // pulse: begin decoding
    input  wire [47:0] coded_bits,     // 48 coded bits, bit 0 first (LSB)
    output reg         done,           // pulse: decoding complete
    output reg  [23:0] decoded_bits    // 24 data bits, bit 0 first (LSB)
);

reg [4:0] cnt;        // 0..23 data bit counter
reg [5:0] state;      // {u[n-1], u[n-2], u[n-3], u[n-4], u[n-5], u[n-6]}
reg       running;

// g1 = 171o = 1111001 in binary
// Taps at positions {0, 3, 4, 5, 6} of the 7-bit register
// output_1 = u[n] ^ u[n-3] ^ u[n-4] ^ u[n-5] ^ u[n-6]
// Solving: u[n] = coded_bits[2n+1] ^ u[n-3] ^ u[n-4] ^ u[n-5] ^ u[n-6]
//                = coded_bits[2n+1] ^ state[2] ^ state[3] ^ state[4] ^ state[5]
wire decoded_bit = coded_bits[{cnt, 1'b1}] ^ state[2] ^ state[3] ^ state[4] ^ state[5];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        done    <= 0;
        running <= 0;
        cnt     <= 0;
        state   <= 6'd0;
    end else begin
        done <= 0;

        if (start) begin
            running <= 1;
            cnt     <= 5'd0;
            state   <= 6'd0;  // encoder starts in all-zero state
        end else if (running) begin
            decoded_bits[cnt] <= decoded_bit;
            state <= {decoded_bit, state[5:1]};

            if (cnt == 5'd23) begin
                done    <= 1;
                running <= 0;
            end else begin
                cnt <= cnt + 1;
            end
        end
    end
end

endmodule
