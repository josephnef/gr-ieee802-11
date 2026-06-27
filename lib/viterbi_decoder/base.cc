/*
 * Copyright (C) 2016 Bastian Bloessl <bloessl@ccs-labs.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "base.h"
#include <cstdlib>
#include <cstring>
#include <iostream>

using namespace gr::ieee802_11;

base::base() : d_store_pos(0) {}

base::~base() {}

uint8_t* base::depuncture(uint8_t* in)
{

    int count;
    int n_cbps = d_ofdm->n_cbps;
    uint8_t* depunctured;

    if (d_ntraceback == 5) {
        count = d_frame->n_sym * n_cbps;
        depunctured = in;

    } else {
        depunctured = d_depunctured;
        count = 0;
        for (int i = 0; i < d_frame->n_sym; i++) {
            for (int k = 0; k < n_cbps; k++) {
                while (d_depuncture_pattern[count % (2 * d_k)] == 0) {
                    depunctured[count] = 2;
                    count++;
                }

                // Insert received bits
                depunctured[count] = in[i * n_cbps + k];
                count++;

                while (d_depuncture_pattern[count % (2 * d_k)] == 0) {
                    depunctured[count] = 2;
                    count++;
                }
            }
        }
    }

    return depunctured;
}

// ---------------------------------------------------------------------------
// Soft-decision Viterbi (scalar). A near-copy of the generic hard decoder, but
// the branch metric is a soft correlation on inputs q in [0, SOFT_Q] instead of
// a Hamming XOR on hard bits {0,1,2}. Punctured positions carry SOFT_NEUTRAL
// (= SOFT_Q/2), which contributes equally to every branch (no information), so
// the hard decoder's `== 2` puncture special-case disappears. Metrics stay in
// uint8 because SOFT_Q is small and the per-output min-normalisation bounds the
// spread, exactly as in the hard path.
// ---------------------------------------------------------------------------

uint8_t* base::depuncture_soft(uint8_t* in)
{
    int count;
    int n_cbps = d_ofdm->n_cbps;
    uint8_t* depunctured;

    if (d_ntraceback == 5) {
        depunctured = in; // rate 1/2 — no puncturing, soft values pass through
    } else {
        depunctured = d_depunctured;
        count = 0;
        for (int i = 0; i < d_frame->n_sym; i++) {
            for (int k = 0; k < n_cbps; k++) {
                while (d_depuncture_pattern[count % (2 * d_k)] == 0) {
                    depunctured[count] = SOFT_NEUTRAL; // erasure = no information
                    count++;
                }
                depunctured[count] = in[i * n_cbps + k];
                count++;
                while (d_depuncture_pattern[count % (2 * d_k)] == 0) {
                    depunctured[count] = SOFT_NEUTRAL;
                    count++;
                }
            }
        }
    }
    return depunctured;
}

void base::soft_init()
{
    for (int i = 0; i < 64; i++) {
        d_soft_metric0[i] = 0;
        d_soft_path0[i] = 0;
    }
    int polys[2] = { 0x6d, 0x4f };
    for (int i = 0; i < 32; i++) {
        d_soft_branchtab[0].c[i] =
            (polys[0] < 0) ^ PARTAB[(2 * i) & abs(polys[0])] ? 1 : 0;
        d_soft_branchtab[1].c[i] =
            (polys[1] < 0) ^ PARTAB[(2 * i) & abs(polys[1])] ? 1 : 0;
    }
    for (int i = 0; i < 64; i++) {
        d_mmresult[i] = 0;
        for (int j = 0; j < TRACEBACK_MAX; j++) {
            d_ppresult[j][i] = 0;
        }
    }
    d_store_pos = 0;
}

// One soft branch metric pair (the only divergence from the hard butterfly):
// mismatch_b = b ? (Q - q) : q ; metsvm = sum over the two coded bits ;
// metsv = 2Q - metsvm. ACS maximises metsv, exactly like the hard path.
#define SOFT_METRIC_BLOCK(SY0, SY1)                                              \
    for (j = 0; j < 16; j++) {                                                   \
        int b0 = d_soft_branchtab[0].c[(i * 16) + j];                            \
        int b1 = d_soft_branchtab[1].c[(i * 16) + j];                            \
        int mm = (b0 ? (SOFT_Q - (SY0)[j]) : (SY0)[j]) +                         \
                 (b1 ? (SOFT_Q - (SY1)[j]) : (SY1)[j]);                          \
        metsvm[j] = mm;                                                          \
        metsv[j] = 2 * SOFT_Q - mm;                                              \
    }

void base::soft_butterfly(unsigned char* symbols,
                          unsigned char* mm0,
                          unsigned char* mm1,
                          unsigned char* pp0,
                          unsigned char* pp1)
{
    int i, j, k;
    unsigned char *metric0, *metric1, *path0, *path1;
    metric0 = mm0;
    path0 = pp0;
    metric1 = mm1;
    path1 = pp1;

    unsigned char m0[16], m1[16], m2[16], m3[16], decision0[16], decision1[16],
        survivor0[16], survivor1[16];
    unsigned char metsv[16], metsvm[16];
    unsigned char shift0[16], shift1[16];
    unsigned char tmp0[16], tmp1[16];
    unsigned char sym0v[16], sym1v[16];
    unsigned short simd_epi16;

    for (j = 0; j < 16; j++) {
        sym0v[j] = symbols[0];
        sym1v[j] = symbols[1];
    }

    for (i = 0; i < 2; i++) {
        SOFT_METRIC_BLOCK(sym0v, sym1v)

        for (j = 0; j < 16; j++) {
            m0[j] = metric0[(i * 16) + j] + metsv[j];
            m1[j] = metric0[((i + 2) * 16) + j] + metsvm[j];
            m2[j] = metric0[(i * 16) + j] + metsvm[j];
            m3[j] = metric0[((i + 2) * 16) + j] + metsv[j];
        }
        for (j = 0; j < 16; j++) {
            decision0[j] = ((m0[j] - m1[j]) > 0) ? 0xff : 0x0;
            decision1[j] = ((m2[j] - m3[j]) > 0) ? 0xff : 0x0;
            survivor0[j] = (decision0[j] & m0[j]) | ((~decision0[j]) & m1[j]);
            survivor1[j] = (decision1[j] & m2[j]) | ((~decision1[j]) & m3[j]);
        }
        for (j = 0; j < 16; j += 2) {
            simd_epi16 = path0[(i * 16) + j];
            simd_epi16 |= path0[(i * 16) + (j + 1)] << 8;
            simd_epi16 <<= 1;
            shift0[j] = simd_epi16;
            shift0[j + 1] = simd_epi16 >> 8;
            simd_epi16 = path0[((i + 2) * 16) + j];
            simd_epi16 |= path0[((i + 2) * 16) + (j + 1)] << 8;
            simd_epi16 <<= 1;
            shift1[j] = simd_epi16;
            shift1[j + 1] = simd_epi16 >> 8;
        }
        for (j = 0; j < 16; j++) {
            shift1[j] = shift1[j] + 1;
        }
        for (j = 0, k = 0; j < 16; j += 2, k++) {
            metric1[(2 * i * 16) + j] = survivor0[k];
            metric1[(2 * i * 16) + (j + 1)] = survivor1[k];
        }
        for (j = 0; j < 16; j++) {
            tmp0[j] = (decision0[j] & shift0[j]) | ((~decision0[j]) & shift1[j]);
        }
        for (j = 0, k = 8; j < 16; j += 2, k++) {
            metric1[((2 * i + 1) * 16) + j] = survivor0[k];
            metric1[((2 * i + 1) * 16) + (j + 1)] = survivor1[k];
        }
        for (j = 0; j < 16; j++) {
            tmp1[j] = (decision1[j] & shift0[j]) | ((~decision1[j]) & shift1[j]);
        }
        for (j = 0, k = 0; j < 16; j += 2, k++) {
            path1[(2 * i * 16) + j] = tmp0[k];
            path1[(2 * i * 16) + (j + 1)] = tmp1[k];
        }
        for (j = 0, k = 8; j < 16; j += 2, k++) {
            path1[((2 * i + 1) * 16) + j] = tmp0[k];
            path1[((2 * i + 1) * 16) + (j + 1)] = tmp1[k];
        }
    }

    metric0 = mm1;
    path0 = pp1;
    metric1 = mm0;
    path1 = pp0;

    for (j = 0; j < 16; j++) {
        sym0v[j] = symbols[2];
        sym1v[j] = symbols[3];
    }

    for (i = 0; i < 2; i++) {
        SOFT_METRIC_BLOCK(sym0v, sym1v)

        for (j = 0; j < 16; j++) {
            m0[j] = metric0[(i * 16) + j] + metsv[j];
            m1[j] = metric0[((i + 2) * 16) + j] + metsvm[j];
            m2[j] = metric0[(i * 16) + j] + metsvm[j];
            m3[j] = metric0[((i + 2) * 16) + j] + metsv[j];
        }
        for (j = 0; j < 16; j++) {
            decision0[j] = ((m0[j] - m1[j]) > 0) ? 0xff : 0x0;
            decision1[j] = ((m2[j] - m3[j]) > 0) ? 0xff : 0x0;
            survivor0[j] = (decision0[j] & m0[j]) | ((~decision0[j]) & m1[j]);
            survivor1[j] = (decision1[j] & m2[j]) | ((~decision1[j]) & m3[j]);
        }
        for (j = 0; j < 16; j += 2) {
            simd_epi16 = path0[(i * 16) + j];
            simd_epi16 |= path0[(i * 16) + (j + 1)] << 8;
            simd_epi16 <<= 1;
            shift0[j] = simd_epi16;
            shift0[j + 1] = simd_epi16 >> 8;
            simd_epi16 = path0[((i + 2) * 16) + j];
            simd_epi16 |= path0[((i + 2) * 16) + (j + 1)] << 8;
            simd_epi16 <<= 1;
            shift1[j] = simd_epi16;
            shift1[j + 1] = simd_epi16 >> 8;
        }
        for (j = 0; j < 16; j++) {
            shift1[j] = shift1[j] + 1;
        }
        for (j = 0, k = 0; j < 16; j += 2, k++) {
            metric1[(2 * i * 16) + j] = survivor0[k];
            metric1[(2 * i * 16) + (j + 1)] = survivor1[k];
        }
        for (j = 0; j < 16; j++) {
            tmp0[j] = (decision0[j] & shift0[j]) | ((~decision0[j]) & shift1[j]);
        }
        for (j = 0, k = 8; j < 16; j += 2, k++) {
            metric1[((2 * i + 1) * 16) + j] = survivor0[k];
            metric1[((2 * i + 1) * 16) + (j + 1)] = survivor1[k];
        }
        for (j = 0; j < 16; j++) {
            tmp1[j] = (decision1[j] & shift0[j]) | ((~decision1[j]) & shift1[j]);
        }
        for (j = 0, k = 0; j < 16; j += 2, k++) {
            path1[(2 * i * 16) + j] = tmp0[k];
            path1[(2 * i * 16) + (j + 1)] = tmp1[k];
        }
        for (j = 0, k = 8; j < 16; j += 2, k++) {
            path1[((2 * i + 1) * 16) + j] = tmp0[k];
            path1[((2 * i + 1) * 16) + (j + 1)] = tmp1[k];
        }
    }
}
#undef SOFT_METRIC_BLOCK

unsigned char base::soft_get_output(unsigned char* mm0,
                                    unsigned char* pp0,
                                    int ntraceback,
                                    unsigned char* outbuf)
{
    int i, j, pos = 0;
    int bestmetric, minmetric, beststate = 0;

    d_store_pos = (d_store_pos + 1) % ntraceback;

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 16; j++) {
            d_mmresult[(i * 16) + j] = mm0[(i * 16) + j];
            d_ppresult[d_store_pos][(i * 16) + j] = pp0[(i * 16) + j];
        }
    }

    bestmetric = d_mmresult[beststate];
    minmetric = d_mmresult[beststate];
    for (i = 1; i < 64; i++) {
        if (d_mmresult[i] > bestmetric) {
            bestmetric = d_mmresult[i];
            beststate = i;
        }
        if (d_mmresult[i] < minmetric) {
            minmetric = d_mmresult[i];
        }
    }

    for (i = 0, pos = d_store_pos; i < (ntraceback - 1); i++) {
        beststate = d_ppresult[pos][beststate] >> 2;
        pos = (pos - 1 + ntraceback) % ntraceback;
    }
    *outbuf = d_ppresult[pos][beststate];

    for (i = 0; i < 4; i++) {
        for (j = 0; j < 16; j++) {
            pp0[(i * 16) + j] = 0;
            mm0[(i * 16) + j] = mm0[(i * 16) + j] - minmetric;
        }
    }
    return bestmetric;
}

uint8_t* base::decode_soft(ofdm_param* ofdm, frame_param* frame, uint8_t* in)
{
    d_ofdm = ofdm;
    d_frame = frame;
    reset();     // derived: sets d_ntraceback / d_depuncture_pattern / d_k
    soft_init(); // set up the independent soft state

    uint8_t* depunctured = depuncture_soft(in);
    int in_count = 0, out_count = 0, n_decoded = 0;
    const int max_in = MAX_ENCODED_BITS;

    while (n_decoded < d_frame->n_data_bits && in_count < max_in) {
        if ((in_count % 4) == 0) {
            soft_butterfly(&depunctured[in_count & 0xfffffffc],
                           d_soft_metric0,
                           d_soft_metric1,
                           d_soft_path0,
                           d_soft_path1);
            if ((in_count > 0) && (in_count % 16) == 8) {
                unsigned char c;
                soft_get_output(d_soft_metric0, d_soft_path0, d_ntraceback, &c);
                if ((out_count >= d_ntraceback) &&
                    ((out_count - d_ntraceback) * 8 + 8 <= MAX_ENCODED_BITS * 3 / 4)) {
                    for (int i = 0; i < 8; i++) {
                        d_decoded[(out_count - d_ntraceback) * 8 + i] =
                            (c >> (7 - i)) & 0x1;
                        n_decoded++;
                    }
                }
                out_count++;
            }
        }
        in_count++;
    }
    return d_decoded;
}

/* Parity lookup table */
const unsigned char base::PARTAB[256] = {
    0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1,
    0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0,
    0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0,
    1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1,
    0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0,
    1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1,
    1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0,
    1, 0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
    0, 1, 1, 0, 1, 0, 0, 1, 0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0,
};

const unsigned char base::PUNCTURE_1_2[2] = { 1, 1 };
const unsigned char base::PUNCTURE_2_3[4] = { 1, 1, 1, 0 };
const unsigned char base::PUNCTURE_3_4[6] = { 1, 1, 1, 0, 0, 1 };
// 5/6: keep 6 of every 10 mother-code bits (drop indices 3,4,7,8) -> the TX
// puncturing in tools_gen_wifi.py matches this.
const unsigned char base::PUNCTURE_5_6[10] = { 1, 1, 1, 0, 0, 1, 1, 0, 0, 1 };
