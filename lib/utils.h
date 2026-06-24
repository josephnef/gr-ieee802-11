/*
 * Copyright (C) 2013 Bastian Bloessl <bloessl@ccs-labs.org>
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
#ifndef INCLUDED_IEEE802_11_UTILS_H
#define INCLUDED_IEEE802_11_UTILS_H

#include <gnuradio/config.h>
#include <ieee802_11/api.h>
#include <ieee802_11/mapper.h>
#include <cinttypes>
#include <iostream>

using gr::ieee802_11::Encoding;

#define MAX_PAYLOAD_SIZE 1500
#define MAX_PSDU_SIZE (MAX_PAYLOAD_SIZE + 28) // MAC, CRC
#define MAX_SYM (((16 + 8 * MAX_PSDU_SIZE + 6) / 24) + 1)
// Coded bits per OFDM symbol upper bound. Legacy max is 288 (64-QAM, 48 SC).
// HT40 64-QAM 2 streams = 108 SC * 6 bpsc * 2 ss = 1296 -> size for the widest
// modern mode we decode so the interleaver/depuncture buffers never overflow.
#define MAX_BITS_PER_SYM 1296
#define MAX_ENCODED_BITS ((16 + 8 * MAX_PSDU_SIZE + 6) * 2 + MAX_BITS_PER_SYM)

// Widest data-subcarrier count carried out of frame_equalizer per OFDM symbol
// (HT40 = 108 data SC * up to 2 spatial streams = 216). Legacy uses 48 of these.
#define MAX_DATA_CARRIERS 216

#define dout d_debug&& std::cout
#define mylog(...)                      \
    do {                                \
        if (d_log) {                    \
            d_logger->info(__VA_ARGS__); \
        }                               \
    } while (0);

#pragma pack(push, 1)
struct mac_header {
    // protocol version, type, subtype, to_ds, from_ds, ...
    uint16_t frame_control;
    uint16_t duration;
    uint8_t addr1[6];
    uint8_t addr2[6];
    uint8_t addr3[6];
    uint16_t seq_nr;
};
#pragma pack(pop)

/**
 * WIFI parameters
 */
class ofdm_param
{
public:
    // Legacy 802.11a/g/p (20 MHz, 1 stream, 48 data SC).
    ofdm_param(Encoding e);
    // 802.11n HT. e must be an HT_MCS_* value; bw is 20 or 40 (MHz). The number
    // of spatial streams is derived from the MCS index (0..7=1ss, 8..15=2ss).
    ofdm_param(Encoding e, int bw);

    // data rate
    Encoding encoding;
    // rate field of the SIGNAL header
    char rate_field;
    // number of coded bits per sub carrier (per stream)
    int n_bpsc;
    // number of coded bits per OFDM symbol (across all streams)
    int n_cbps;
    // number of data bits per OFDM symbol (across all streams)
    int n_dbps;

    // --- format / geometry (legacy defaults set by the 1-arg constructor) ---
    bool is_ht = false;     // true for HT_MCS_*
    int n_ss = 1;           // spatial streams
    int bw = 20;            // channel bandwidth in MHz (20 or 40)
    int n_data_sc = 48;     // data subcarriers (legacy 48, HT20 52, HT40 108)
    int n_pilot_sc = 4;     // pilot subcarriers (legacy/HT20 4, HT40 6)
    int fft_len = 64;       // OFDM FFT size (20 MHz 64, 40 MHz 128)

    void print();
};

/**
 * packet specific parameters
 */
class frame_param
{
public:
    frame_param(ofdm_param& ofdm, int psdu_length);
    // PSDU size in bytes
    int psdu_size;
    // number of OFDM symbols (17-11)
    int n_sym;
    // number of padding bits in the DATA field (17-13)
    int n_pad;
    int n_encoded_bits;
    // number of data bits, including service and padding (17-12)
    int n_data_bits;

    void print();
};

/**
 * Given a payload, generates a MAC data frame (i.e., a PSDU) to be given
 * to the physical layer for encoding.
 *
 * \param msdu the payload for the MAC frame
 * \param msdu_size the size of the msdu in bytes
 * \param psdu pointer to a byte array where to store the MAC frame. Memory
 * will be alloced by the function
 * \param psdu_size pointer to an integer where the size of the psdu in bytes
 * will be stored
 * \param seq sequence number of the frame
 */
void generate_mac_data_frame(
    const char* msdu, int msdu_size, char** psdu, int* psdu_size, char seq);

void scramble(const char* input, char* out, frame_param& frame, char initial_state);

void reset_tail_bits(char* scrambled_data, frame_param& frame);

void convolutional_encoding(const char* input, char* out, frame_param& frame);

void puncturing(const char* input, char* out, frame_param& frame, ofdm_param& ofdm);

void interleave(const char* input,
                char* out,
                frame_param& frame,
                ofdm_param& ofdm,
                bool reverse = false);

void split_symbols(const char* input, char* out, frame_param& frame, ofdm_param& ofdm);

void generate_bits(const char* psdu, char* data_bits, frame_param& frame);

#endif /* INCLUDED_IEEE802_11_UTILS_H */
