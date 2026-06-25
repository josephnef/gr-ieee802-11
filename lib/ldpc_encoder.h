// Systematic GF(2) encoder for the IEEE 802.11 QC-LDPC codes (all 12: R=1/2,2/3,3/4,5/6
// x n=648,1296,1944). Builds the parity-check matrix H from a base matrix + subblock size
// Z, derives the systematic generator by column-pivoted Gaussian elimination, and encodes
// k info bits -> n codeword bits. Faithful C++ port of sdr2wifi/ldpc.py
// (build_H / gf2_systematic / encode); validate with the bit-exact KAT in ldpc_test.cc.
#ifndef INCLUDED_IEEE802_11_LDPC_ENCODER_H
#define INCLUDED_IEEE802_11_LDPC_ENCODER_H

#include "ldpc_bases.h"
#include <cstdint>
#include <vector>

namespace gr {
namespace ieee802_11 {

struct LdpcEncoder {
    int Z = 0, rows = 0, M = 0, N = 0;
    std::vector<int> info_cols; // K columns carrying info bits
    std::vector<int> par_cols;  // M columns carrying parity bits
    std::vector<uint8_t> P;     // K x M row-major: parity = info @ P (mod 2)

    LdpcEncoder() {}
    explicit LdpcEncoder(const ldpc_code& c) { build(c); }

    void build(const ldpc_code& c)
    {
        Z = c.Z;
        rows = c.rows;
        M = rows * Z; // parity rows
        N = 24 * Z;   // codeword bits
        // H = build_H(base, Z): each base entry s>=0 -> identity cyclically shifted right
        // by s (np.roll(I, s, axis=1) -> H[block r][i][(i+s)%Z] = 1).
        std::vector<std::vector<uint8_t>> H(M, std::vector<uint8_t>(N, 0));
        for (int br = 0; br < rows; br++) {
            for (int bc = 0; bc < 24; bc++) {
                int s = c.base[br * 24 + bc];
                if (s < 0)
                    continue;
                for (int i = 0; i < Z; i++)
                    H[br * Z + i][bc * Z + ((i + s) % Z)] = 1;
            }
        }
        // gf2_systematic: column-pivoted elimination preferring the rightmost (parity)
        // columns as pivots, so they end up as an identity and the rest are info columns.
        std::vector<char> used(N, 0);
        par_cols.clear();
        for (int r = 0; r < M; r++) {
            int pc = -1;
            for (int c2 = N - 1; c2 >= 0; c2--) {
                if (!used[c2] && H[r][c2]) {
                    pc = c2;
                    break;
                }
            }
            if (pc < 0)
                continue;
            used[pc] = 1;
            par_cols.push_back(pc);
            for (int rr = 0; rr < M; rr++) {
                if (rr != r && H[rr][pc]) {
                    for (int c2 = 0; c2 < N; c2++)
                        H[rr][c2] ^= H[r][c2];
                }
            }
        }
        info_cols.clear();
        for (int c2 = 0; c2 < N; c2++)
            if (!used[c2])
                info_cols.push_back(c2);

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

    int k() const { return (int)info_cols.size(); }
    int n() const { return N; }

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

// Back-compat: the R=1/2 n=648 encoder used by the single-codeword HT20-MCS0 path.
struct LdpcEncoder648 : LdpcEncoder {
    static const int N = 648, M = 324;
    LdpcEncoder648() : LdpcEncoder(ldpc_code_for(648, 1, 2)) {}
};

} // namespace ieee802_11
} // namespace gr

#endif
