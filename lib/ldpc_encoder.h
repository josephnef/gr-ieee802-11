// Systematic GF(2) encoder for the IEEE 802.11 R=1/2 n=648 QC-LDPC code.
// Reuses LDPC_BASE_R12_648 from ldpc_min_sum.h; builds the parity-check matrix H,
// derives the systematic generator (P matrix) by column-pivoted Gaussian elimination,
// and encodes 324 info bits -> 648 codeword bits. This is a faithful C++ port of
// sdr2wifi/ldpc.py (build_H / gf2_systematic / encode); validate with a round-trip
// through ldpc_decode_648 (see ldpc_test.cc).
#ifndef INCLUDED_IEEE802_11_LDPC_ENCODER_H
#define INCLUDED_IEEE802_11_LDPC_ENCODER_H

#include "ldpc_min_sum.h"
#include <cstdint>
#include <vector>

namespace gr {
namespace ieee802_11 {

struct LdpcEncoder648 {
    static const int Z = 27;
    static const int M = 12 * Z; // 324 parity rows
    static const int N = 24 * Z; // 648 codeword bits
    std::vector<int> info_cols;  // K columns carrying info bits
    std::vector<int> par_cols;   // M columns carrying parity bits
    std::vector<uint8_t> P;      // K x M row-major: parity = info @ P (mod 2)

    LdpcEncoder648() { build(); }

    void build()
    {
        // H = build_H(LDPC_BASE_R12_648, Z): each base entry s>=0 -> identity cyclically
        // shifted right by s (np.roll(I, s, axis=1) -> H[block r][i][(i+s)%Z] = 1).
        std::vector<std::vector<uint8_t>> H(M, std::vector<uint8_t>(N, 0));
        for (int br = 0; br < 12; br++) {
            for (int bc = 0; bc < 24; bc++) {
                int s = LDPC_BASE_R12_648[br][bc];
                if (s < 0)
                    continue;
                for (int i = 0; i < Z; i++)
                    H[br * Z + i][bc * Z + ((i + s) % Z)] = 1;
            }
        }
        // gf2_systematic: column-pivoted elimination preferring the rightmost (parity)
        // columns as pivots, so the chosen pivot columns end up as an identity and the
        // remaining columns are the info columns.
        std::vector<char> used(N, 0);
        par_cols.clear();
        for (int r = 0; r < M; r++) {
            int pc = -1;
            for (int c = N - 1; c >= 0; c--) {
                if (!used[c] && H[r][c]) {
                    pc = c;
                    break;
                }
            }
            if (pc < 0)
                continue;
            used[pc] = 1;
            par_cols.push_back(pc);
            for (int rr = 0; rr < M; rr++) {
                if (rr != r && H[rr][pc]) {
                    for (int c = 0; c < N; c++)
                        H[rr][c] ^= H[r][c];
                }
            }
        }
        info_cols.clear();
        for (int c = 0; c < N; c++)
            if (!used[c])
                info_cols.push_back(c);

        std::vector<int> rowof(N, -1);
        for (int r = 0; r < (int)par_cols.size(); r++)
            rowof[par_cols[r]] = r;

        int kk = (int)info_cols.size(), mm = (int)par_cols.size();
        P.assign((size_t)kk * mm, 0);
        for (int pi = 0; pi < mm; pi++) {
            int r = rowof[par_cols[pi]];
            for (int ic = 0; ic < kk; ic++)
                P[(size_t)ic * mm + pi] = H[r][info_cols[ic]];
        }
    }

    int k() const { return (int)info_cols.size(); } // 324

    // info[k()] -> cw[N]. cw[info_cols]=info; cw[par_cols]=info @ P (mod 2).
    void encode(const uint8_t* info, uint8_t* cw) const
    {
        for (int i = 0; i < N; i++)
            cw[i] = 0;
        int kk = (int)info_cols.size(), mm = (int)par_cols.size();
        for (int ic = 0; ic < kk; ic++)
            cw[info_cols[ic]] = info[ic];
        for (int pi = 0; pi < mm; pi++) {
            uint8_t s = 0;
            for (int ic = 0; ic < kk; ic++)
                s ^= (uint8_t)(info[ic] & P[(size_t)ic * mm + pi]);
            cw[par_cols[pi]] = (uint8_t)(s & 1);
        }
    }
};

} // namespace ieee802_11
} // namespace gr

#endif
