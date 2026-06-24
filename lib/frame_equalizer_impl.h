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

#ifndef INCLUDED_IEEE802_11_FRAME_EQUALIZER_IMPL_H
#define INCLUDED_IEEE802_11_FRAME_EQUALIZER_IMPL_H

#include "equalizer/base.h"
#include "viterbi_decoder/viterbi_decoder.h"
#include <ieee802_11/constellations.h>
#include <ieee802_11/frame_equalizer.h>

namespace gr {
namespace ieee802_11 {

class frame_equalizer_impl : virtual public frame_equalizer
{

public:
    frame_equalizer_impl(
        Equalizer algo, double freq, double bw, bool log, bool debug, int fft_len);
    ~frame_equalizer_impl();

    void set_algorithm(Equalizer algo);
    void set_bandwidth(double bw);
    void set_frequency(double freq);

    void forecast(int noutput_items, gr_vector_int& ninput_items_required);
    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items);

private:
    bool parse_signal(uint8_t* signal);
    bool decode_signal_field(uint8_t* rx_bits);
    void deinterleave(uint8_t* rx_bits);

    // --- 802.11n HT (Phase 1a/b: HT-SIG sniffer over a clean HT equalization) ---
    // Decode the two captured QBPSK HT-SIG symbols; log MCS/length and CRC.
    void sniff_ht_sig();
    // Equalize one raw HT-SIG symbol (48 data carriers) with the L-LTF channel,
    // pilot-derotated to the real axis.
    void equalize_ht_sig(const gr_complex* raw, int sym_idx, const gr_complex* h,
                         gr_complex* out48);
    // Decode the two pilot-derotated HT-SIG symbols (flat 96 = 2x48).
    bool try_ht_sig(const gr_complex* eq, int& mcs, int& cbw, int& len, int& stbc,
                    int& fec, int& sgi);

    int d_fft_len;              // 64 (20 MHz) or 128 (HT40)
    gr_complex d_ht_raw[2][128]; // raw FFT bins of the 2 candidate HT-SIG symbols
    double d_er_frozen;         // residual-CFO estimate frozen at L-SIG (pre HT-SIG)

    // HT DATA decode (Phase 1b-ii). After HT-SIG validates we re-equalize the
    // HT DATA symbols (52 data SC for HT20) off the raw FFT, demap, deinterleave,
    // Viterbi-decode, descramble and CRC-32 check -- entirely inside this block
    // (the legacy 48-wide stream path to decode_mac is left untouched).
    void ht_begin(int mcs, int len);
    void ht_data_symbol(const gr_complex* raw, int sym_idx);
    void ht_finish();
    void ht_estimate_ltf40(const gr_complex* raw); // HT40 channel est from HT-LTF
    gr_complex d_ht_h[128];   // HT40 data channel estimate (from the HT-LTF)
    bool d_ht_active = false;
    int d_ht_mcs = 0;
    int d_ht_len = 0;
    int d_ht_nsym = 0;   // number of HT DATA OFDM symbols
    int d_ht_dsym = 0;   // HT DATA symbols collected so far
    int d_ht_nbpsc = 1;  // bits per subcarrier for the HT MCS
    int d_ht_ncbps = 52; // coded bits per HT OFDM symbol
    int d_ht_ndbps = 26; // data bits per HT OFDM symbol
    uint8_t d_ht_rx_bits[MAX_ENCODED_BITS]; // accumulated coded bits

    equalizer::base* d_equalizer;
    gr::thread::mutex d_mutex;
    std::vector<gr::tag_t> tags;
    bool d_debug;
    bool d_log;
    int d_current_symbol;
    viterbi_decoder d_decoder;

    // freq offset
    double d_freq;                      // Hz
    double d_freq_offset_from_synclong; // Hz, estimation from "sync_long" block
    double d_bw;                        // Hz
    double d_er;
    double d_epsilon0;
    gr_complex d_prev_pilots[4];

    int d_frame_bytes;
    int d_frame_symbols;
    int d_frame_encoding;

    uint8_t d_deinterleaved[48];
    gr_complex symbols[48];

    std::shared_ptr<gr::digital::constellation> d_frame_mod;
    constellation_bpsk::sptr d_bpsk;
    constellation_qpsk::sptr d_qpsk;
    constellation_16qam::sptr d_16qam;
    constellation_64qam::sptr d_64qam;

    static const int interleaver_pattern[48];
};

} // namespace ieee802_11
} // namespace gr

#endif /* INCLUDED_IEEE802_11_FRAME_EQUALIZER_IMPL_H */
