/*
 * Copyright (C) 2013, 2016 Bastian Bloessl <bloessl@ccs-labs.org>
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
#ifndef INCLUDED_IEEE802_11_MAPPER_H
#define INCLUDED_IEEE802_11_MAPPER_H

#include <cstdint>
#include <gnuradio/block.h>
#include <ieee802_11/api.h>

namespace gr {
namespace ieee802_11 {

enum Encoding {
    // --- legacy 802.11a/g/p (values 0..7 are FIXED: binary-compatible with the
    // SIGNAL rate_field switch, the viterbi puncture map and the TX path) ---
    BPSK_1_2 = 0,
    BPSK_3_4 = 1,
    QPSK_1_2 = 2,
    QPSK_3_4 = 3,
    QAM16_1_2 = 4,
    QAM16_3_4 = 5,
    QAM64_2_3 = 6,
    QAM64_3_4 = 7,
    // --- 802.11n HT MCS, offset by 8 (HT_MCS_0..HT_MCS_31). MCS 0..7 = 1 stream,
    // 8..15 = 2 streams, 16..23 = 3, 24..31 = 4. We implement RX for 0..15. ---
    HT_MCS_0 = 8,   HT_MCS_1,  HT_MCS_2,  HT_MCS_3,  HT_MCS_4,  HT_MCS_5,
    HT_MCS_6,  HT_MCS_7,  HT_MCS_8,  HT_MCS_9,  HT_MCS_10, HT_MCS_11,
    HT_MCS_12, HT_MCS_13, HT_MCS_14, HT_MCS_15, HT_MCS_16, HT_MCS_17,
    HT_MCS_18, HT_MCS_19, HT_MCS_20, HT_MCS_21, HT_MCS_22, HT_MCS_23,
    HT_MCS_24, HT_MCS_25, HT_MCS_26, HT_MCS_27, HT_MCS_28, HT_MCS_29,
    HT_MCS_30, HT_MCS_31 = 39,
    // --- reserved for 802.11ac VHT (Phase 4); not implemented yet ---
    VHT_MCS_0 = 40,
};

// True for the HT MCS range (HT_MCS_0..HT_MCS_31).
inline bool is_ht_encoding(Encoding e) { return e >= HT_MCS_0 && e <= HT_MCS_31; }
// HT MCS index 0..31 for an HT encoding (undefined for legacy).
inline int ht_mcs_index(Encoding e) { return static_cast<int>(e) - HT_MCS_0; }

// Required for fmt 10
inline uint8_t format_as(Encoding e) {
  return static_cast<uint8_t>(e);
}

class IEEE802_11_API mapper : virtual public block
{
public:
    typedef std::shared_ptr<mapper> sptr;
    static sptr make(Encoding mcs, bool debug = false);
    virtual void set_encoding(Encoding mcs) = 0;
};

} // namespace ieee802_11
} // namespace gr

#endif /* INCLUDED_IEEE802_11_MAPPER_H */
