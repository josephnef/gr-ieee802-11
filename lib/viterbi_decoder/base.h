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
#ifndef INCLUDED_IEEE802_11_VITERBI_DECODER_BASE_H
#define INCLUDED_IEEE802_11_VITERBI_DECODER_BASE_H

#include "../utils.h"

namespace gr {
namespace ieee802_11 {

// Maximum number of traceback bytes
#define TRACEBACK_MAX 24

/* This Viterbi decoder was taken from the gr-dvbt module of
 * GNU Radio. It is an SSE2 version of the Viterbi Decoder
 * created by Phil Karn. The SSE2 version was made by Bogdan
 * Diaconescu. For more info see: gr-dvbt/lib/d_viterbi.h
 */
class base
{
public:
    base();
    ~base();
    virtual uint8_t* decode(ofdm_param* ofdm, frame_param* frame, uint8_t* in) = 0;

    // Soft-decision decode (GR_SOFT_VITERBI). `in` carries one soft value per
    // coded bit in [0, SOFT_Q]: 0 = confident bit 0, SOFT_Q = confident bit 1,
    // SOFT_NEUTRAL = no information. A full-confidence soft input (q = bit ?
    // SOFT_Q : 0) decodes identically to the hard decode(). Implemented here in
    // the base (scalar) so it is independent of the hard SSE/generic path —
    // the per-frame Viterbi cost makes SSE unnecessary for correctness.
    uint8_t* decode_soft(ofdm_param* ofdm, frame_param* frame, uint8_t* in);

    // Soft scale. SOFT_Q small keeps the metric in the uint8 accumulators with
    // the existing per-output min-normalisation (no int16 repacking); 8-level
    // soft already captures ~all of the soft-decision coding gain.
    static const int SOFT_Q = 8;
    static const int SOFT_NEUTRAL = SOFT_Q / 2;

protected:
    // Position in circular buffer where the current decoded byte is stored
    int d_store_pos;
    // Metrics for each state
    alignas(16) unsigned char d_mmresult[64];
    // Paths for each state
    alignas(16) unsigned char d_ppresult[TRACEBACK_MAX][64];

    int d_ntraceback;
    int d_k;
    ofdm_param* d_ofdm;
    frame_param* d_frame;
    const unsigned char* d_depuncture_pattern;

    uint8_t d_depunctured[MAX_ENCODED_BITS];
    uint8_t d_decoded[MAX_ENCODED_BITS * 3 / 4];

    static const unsigned char PARTAB[256];
    static const unsigned char PUNCTURE_1_2[2];
    static const unsigned char PUNCTURE_2_3[4];
    static const unsigned char PUNCTURE_3_4[6];
    static const unsigned char PUNCTURE_5_6[10];

    virtual void reset() = 0;
    uint8_t* depuncture(uint8_t* in);

    // Soft decoder state — scalar, independent of the derived hard decoder's
    // branch table / metrics (so it works whether x86 or generic is compiled).
    union soft_branchtab {
        unsigned char c[32];
    } d_soft_branchtab[2];
    alignas(16) unsigned char d_soft_metric0[64];
    alignas(16) unsigned char d_soft_metric1[64];
    alignas(16) unsigned char d_soft_path0[64];
    alignas(16) unsigned char d_soft_path1[64];

    void soft_init();
    uint8_t* depuncture_soft(uint8_t* in);
    void soft_butterfly(unsigned char* symbols,
                        unsigned char* mm0,
                        unsigned char* mm1,
                        unsigned char* pp0,
                        unsigned char* pp1);
    unsigned char soft_get_output(unsigned char* mm0,
                                  unsigned char* pp0,
                                  int ntraceback,
                                  unsigned char* outbuf);
};

} // namespace ieee802_11
} // namespace gr

#endif /* INCLUDED_IEEE802_11_VITERBI_DECODER_BASE_H */
