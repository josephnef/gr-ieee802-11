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

#include "equalizer/base.h"
#include "equalizer/comb.h"
#include "equalizer/lms.h"
#include "equalizer/ls.h"
#include "equalizer/sta.h"
#include "frame_equalizer_impl.h"
#include "ldpc_min_sum.h"
#include "utils.h"
#include <gnuradio/io_signature.h>
#include <boost/crc.hpp>
#include <cstdlib>
#include <vector>

namespace gr {
namespace ieee802_11 {

frame_equalizer::sptr frame_equalizer::make(
    Equalizer algo, double freq, double bw, bool log, bool debug, int fft_len, int n_rx)
{
    return gnuradio::get_initial_sptr(
        new frame_equalizer_impl(algo, freq, bw, log, debug, fft_len, n_rx));
}


frame_equalizer_impl::frame_equalizer_impl(
    Equalizer algo, double freq, double bw, bool log, bool debug, int fft_len, int n_rx)
    : gr::block("frame_equalizer",
                gr::io_signature::make(n_rx, n_rx, fft_len * sizeof(gr_complex)),
                gr::io_signature::make(1, 1, 48)),
      d_current_symbol(0),
      d_log(log),
      d_debug(debug),
      d_equalizer(NULL),
      d_freq(freq),
      d_bw(bw),
      d_fft_len(fft_len),
      d_frame_bytes(0),
      d_frame_symbols(0),
      d_freq_offset_from_synclong(0.0),
      d_n_rx(n_rx)
{

    message_port_register_out(pmt::mp("symbols"));
    message_port_register_out(pmt::mp("pdu"));

    d_bpsk = constellation_bpsk::make();
    d_qpsk = constellation_qpsk::make();
    d_16qam = constellation_16qam::make();
    d_64qam = constellation_64qam::make();

    d_frame_mod = d_bpsk;

    set_tag_propagation_policy(block::TPP_DONT);
    set_algorithm(algo);
}

frame_equalizer_impl::~frame_equalizer_impl() {}


void frame_equalizer_impl::set_algorithm(Equalizer algo)
{
    gr::thread::scoped_lock lock(d_mutex);
    delete d_equalizer;

    switch (algo) {

    case COMB:
        dout << "Comb" << std::endl;
        d_equalizer = new equalizer::comb();
        break;
    case LS:
        dout << "LS" << std::endl;
        d_equalizer = new equalizer::ls();
        break;
    case LMS:
        dout << "LMS" << std::endl;
        d_equalizer = new equalizer::lms();
        break;
    case STA:
        dout << "STA" << std::endl;
        d_equalizer = new equalizer::sta();
        break;
    default:
        throw std::runtime_error("Algorithm not implemented");
    }
    d_equalizer->set_fft_len(d_fft_len);
}

void frame_equalizer_impl::set_bandwidth(double bw)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_bw = bw;
}

void frame_equalizer_impl::set_frequency(double freq)
{
    gr::thread::scoped_lock lock(d_mutex);
    d_freq = freq;
}

void frame_equalizer_impl::forecast(int noutput_items,
                                    gr_vector_int& ninput_items_required)
{
    ninput_items_required[0] = noutput_items;
    if (d_n_rx == 2) {
        ninput_items_required[1] = noutput_items;
    }
}

int frame_equalizer_impl::general_work(int noutput_items,
                                       gr_vector_int& ninput_items,
                                       gr_vector_const_void_star& input_items,
                                       gr_vector_void_star& output_items)
{

    gr::thread::scoped_lock lock(d_mutex);

    const gr_complex* in = (const gr_complex*)input_items[0];
    const gr_complex* in1 = (d_n_rx == 2) ? (const gr_complex*)input_items[1] : nullptr;
    uint8_t* out = (uint8_t*)output_items[0];

    int i = 0;
    int o = 0;
    gr_complex symbols[48];
    gr_complex current_symbol[128];
    const int fft = d_fft_len;        // 64 or 128
    const int dc = fft / 2;           // DC bin
    const int slen = fft + fft / 4;   // OFDM symbol length in samples (80 or 160)

    dout << "FRAME EQUALIZER: input " << ninput_items[0] << "  output " << noutput_items
         << std::endl;

    while ((i < ninput_items[0]) && (o < noutput_items)) {

        get_tags_in_window(tags, 0, i, i + 1, pmt::string_to_symbol("wifi_start"));

        // new frame
        if (tags.size()) {
            d_current_symbol = 0;
            d_frame_symbols = 0;
            d_frame_mod = d_bpsk;

            d_freq_offset_from_synclong =
                pmt::to_double(tags.front().value) * d_bw / (2 * M_PI);
            d_epsilon0 = pmt::to_double(tags.front().value) * d_bw / (2 * M_PI * d_freq);
            d_er = 0;

            dout << "epsilon: " << d_epsilon0 << std::endl;
        }

        // new frame resets the HT decode state
        if (tags.size()) {
            d_ht_active = false;
            d_mimo_active = false;
            d_vht_pending = false;
            d_vht = false;
            d_stbc_active = false;
            d_stbc_ltf_seen = 0;
            d_stbc_have_y0 = false;
            d_ht_fec = false;
        }

        // not interesting -> skip (but keep going while an HT/MIMO/VHT/STBC frame is
        // decoding or a VHT-SIG-B length read is still pending)
        if (d_current_symbol > (d_frame_symbols + 2) && !d_ht_active &&
            !d_mimo_active && !d_vht_pending && !d_stbc_active) {
            i++;
            continue;
        }

        std::memcpy(current_symbol, in + i * fft, fft * sizeof(gr_complex));

        // HT: stash the *raw* FFT of the two symbols after L-SIG
        // (candidate HT-SIG) before any legacy ramp/pilot correction mutates
        // current_symbol, and freeze the residual-CFO estimate as it stood at
        // L-SIG. The legacy pilot tracking corrupts d_er on QBPSK pilots, so the
        // HT path re-equalizes these raw symbols itself (see sniff_ht_sig).
        if (d_current_symbol == 3) {
            d_er_frozen = d_er;
            std::memcpy(d_ht_raw[0], in + i * fft, fft * sizeof(gr_complex));
        } else if (d_current_symbol == 4) {
            std::memcpy(d_ht_raw[1], in + i * fft, fft * sizeof(gr_complex));
        }


        // compensate sampling offset
        for (int k = 0; k < fft; k++) {
            current_symbol[k] *= exp(gr_complex(0,
                                                2 * M_PI * d_current_symbol * slen *
                                                    (d_epsilon0 + d_er) * (k - dc) / fft));
        }

        // POLARITY is the pilot-polarity sequence for SIGNAL/DATA symbols (index
        // d_current_symbol-2). For the two L-LTF symbols (0,1) the index would be
        // negative and C++'s % preserves the sign -> an out-of-bounds read (caught
        // by AddressSanitizer; can fault depending on global layout). The value is
        // unused for symbols <2, so clamp the index to 0 there.
        gr_complex p =
            equalizer::base::POLARITY[d_current_symbol < 2 ? 0 : (d_current_symbol - 2) % 127];

        double beta;
        if (d_current_symbol < 2) {
            beta = arg(current_symbol[11] - current_symbol[25] + current_symbol[39] +
                       current_symbol[53]);

        } else {
            beta = arg((current_symbol[11] * p) + (current_symbol[39] * p) +
                       (current_symbol[25] * p) + (current_symbol[53] * -p));
        }

        double er = arg((conj(d_prev_pilots[0]) * current_symbol[11] * p) +
                        (conj(d_prev_pilots[1]) * current_symbol[25] * p) +
                        (conj(d_prev_pilots[2]) * current_symbol[39] * p) +
                        (conj(d_prev_pilots[3]) * current_symbol[53] * -p));

        er *= d_bw / (2 * M_PI * d_freq * slen);

        if (d_current_symbol < 2) {
            d_prev_pilots[0] = current_symbol[11];
            d_prev_pilots[1] = -current_symbol[25];
            d_prev_pilots[2] = current_symbol[39];
            d_prev_pilots[3] = current_symbol[53];
        } else {
            d_prev_pilots[0] = current_symbol[11] * p;
            d_prev_pilots[1] = current_symbol[25] * p;
            d_prev_pilots[2] = current_symbol[39] * p;
            d_prev_pilots[3] = current_symbol[53] * -p;
        }

        // compensate residual frequency offset
        for (int k = 0; k < fft; k++) {
            current_symbol[k] *= exp(gr_complex(0, -beta));
        }

        // update estimate of residual frequency offset
        if (d_current_symbol >= 2) {

            double alpha = 0.1;
            d_er = (1 - alpha) * d_er + alpha * er;
        }

        // do equalization
        d_equalizer->equalize(
            current_symbol, d_current_symbol, symbols, out + o * 48, d_frame_mod);

        // HT: once both candidate HT-SIG symbols are stashed, decode them off the
        // raw FFT (clean HT equalization) and, on a valid CRC-8, start the HT/MIMO
        // data decode. Runs alongside the untouched legacy 48-wide output flow.
        if (d_current_symbol == 4) {
            sniff_ht_sig();
        }
        // HT40 needs a real channel estimate from the HT-LTF (sym6): the duplicate
        // L-LTF has a center gap the HT40 data fills. (HT20 reuses the L-LTF.)
        if (d_ht_active && fft == 128 && d_current_symbol == 6) {
            ht_estimate_ltf40(in + i * fft);
        }
        // HT20 estimates its data channel from the HT-LTF (sym6) too -- get_h() is
        // corrupted by the QBPSK HT-SIG tracking on real frames. (VHT, whose
        // d_ht_active isn't set until sym7, keeps get_h() in ht_data_symbol.)
        if (d_ht_active && fft == 64 && !d_vht && d_current_symbol == 6) {
            ht_estimate_ltf20(in + i * fft);
        }
        // VHT: read the PSDU length from VHT-SIG-B (sym7), then start the data
        // decode. This sets d_ht_active + d_vht, so it must run before the HT-data
        // dispatch below (whose VHT offset then skips sym7 itself).
        if (d_vht_pending && d_current_symbol == 7) {
            vht_read_sig_b(in + i * fft);
        }
        // HT DATA starts after L-SIG(2), HT-SIG(3,4), HT-STF(5), HT-LTF(6) at sym7.
        // VHT DATA starts one symbol later (sym8) because VHT-SIG-B occupies sym7.
        if (d_ht_active && d_current_symbol >= (d_vht ? 8 : 7)) {
            ht_data_symbol(in + i * fft, d_current_symbol);
        }

        // 2x2 MIMO (MCS 8-15): 2 HT-LTF symbols (6,7) give the 2x2 channel; HT DATA
        // starts at symbol 8. Both RX antennas are read off the 2 input streams.
        if (d_mimo_active && (d_current_symbol == 6 || d_current_symbol == 7)) {
            int t = d_current_symbol - 6;
            std::memcpy(d_mimo_ltf[t][0], in + i * fft, 64 * sizeof(gr_complex));
            std::memcpy(d_mimo_ltf[t][1], in1 + i * fft, 64 * sizeof(gr_complex));
            if (++d_mimo_ltf_seen == 2) {
                mimo_estimate_ltf();
            }
        }
        if (d_mimo_active && d_current_symbol >= 8) {
            mimo_data_symbol(in + i * fft, in1 + i * fft, d_current_symbol);
        }

        // HT STBC (1 SS -> 2 STS): the 2 HT-LTF symbols (6,7) give h0,h1 on the single
        // antenna via the P matrix; Alamouti data starts at symbol 8.
        if (d_stbc_active && (d_current_symbol == 6 || d_current_symbol == 7)) {
            std::memcpy(d_stbc_ltf[d_current_symbol - 6], in + i * fft,
                        64 * sizeof(gr_complex));
            if (++d_stbc_ltf_seen == 2) {
                stbc_estimate_ltf();
            }
        }
        if (d_stbc_active && d_current_symbol >= 8) {
            stbc_data_symbol(in + i * fft, d_current_symbol);
        }

        // signal field
        if (d_current_symbol == 2) {

            if (decode_signal_field(out + o * 48)) {

                pmt::pmt_t dict = pmt::make_dict();
                dict = pmt::dict_add(
                    dict, pmt::mp("frame bytes"), pmt::from_uint64(d_frame_bytes));
                dict = pmt::dict_add(
                    dict, pmt::mp("encoding"), pmt::from_uint64(d_frame_encoding));
                dict = pmt::dict_add(
                    dict, pmt::mp("snr"), pmt::from_double(d_equalizer->get_snr()));
                dict = pmt::dict_add(
                    dict, pmt::mp("nominal frequency"), pmt::from_double(d_freq));
                dict = pmt::dict_add(dict,
                                     pmt::mp("frequency offset"),
                                     pmt::from_double(d_freq_offset_from_synclong));
                dict = pmt::dict_add(dict, pmt::mp("beta"), pmt::from_double(beta));

                std::vector<gr_complex> csi = d_equalizer->get_csi();
                dict = pmt::dict_add(
                    dict, pmt::mp("csi"), pmt::init_c32vector(csi.size(), csi));

                pmt::pmt_t pairs = pmt::dict_items(dict);
                for (int i = 0; i < pmt::length(pairs); i++) {
                    pmt::pmt_t pair = pmt::nth(i, pairs);
                    add_item_tag(0,
                                 nitems_written(0) + o,
                                 pmt::car(pair),
                                 pmt::cdr(pair),
                                 alias_pmt());
                }
            }
        }

        if (d_current_symbol > 2) {
            o++;
            pmt::pmt_t pdu = pmt::make_dict();
            message_port_pub(
                pmt::mp("symbols"),
                pmt::cons(pmt::make_dict(), pmt::init_c32vector(48, symbols)));
        }

        i++;
        d_current_symbol++;
    }

    consume(0, i);
    if (d_n_rx == 2) {
        consume(1, i);
    }
    return o;
}

bool frame_equalizer_impl::decode_signal_field(uint8_t* rx_bits)
{

    static ofdm_param ofdm(BPSK_1_2);
    static frame_param frame(ofdm, 0);

    deinterleave(rx_bits);
    uint8_t* decoded_bits = d_decoder.decode(&ofdm, &frame, d_deinterleaved);

    return parse_signal(decoded_bits);
}

void frame_equalizer_impl::deinterleave(uint8_t* rx_bits)
{
    for (int i = 0; i < 48; i++) {
        d_deinterleaved[i] = rx_bits[interleaver_pattern[i]];
    }
}

bool frame_equalizer_impl::parse_signal(uint8_t* decoded_bits)
{

    int r = 0;
    d_frame_bytes = 0;
    bool parity = false;
    for (int i = 0; i < 17; i++) {
        parity ^= decoded_bits[i];

        if ((i < 4) && decoded_bits[i]) {
            r = r | (1 << i);
        }

        if (decoded_bits[i] && (i > 4) && (i < 17)) {
            d_frame_bytes = d_frame_bytes | (1 << (i - 5));
        }
    }

    if (parity != decoded_bits[17]) {
        dout << "SIGNAL: wrong parity" << std::endl;
        return false;
    }

    switch (r) {
    case 11:
        d_frame_encoding = 0;
        d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)24);
        d_frame_mod = d_bpsk;
        dout << "Encoding: 3 Mbit/s   ";
        break;
    case 15:
        d_frame_encoding = 1;
        d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)36);
        d_frame_mod = d_bpsk;
        dout << "Encoding: 4.5 Mbit/s   ";
        break;
    case 10:
        d_frame_encoding = 2;
        d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)48);
        d_frame_mod = d_qpsk;
        dout << "Encoding: 6 Mbit/s   ";
        break;
    case 14:
        d_frame_encoding = 3;
        d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)72);
        d_frame_mod = d_qpsk;
        dout << "Encoding: 9 Mbit/s   ";
        break;
    case 9:
        d_frame_encoding = 4;
        d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)96);
        d_frame_mod = d_16qam;
        dout << "Encoding: 12 Mbit/s   ";
        break;
    case 13:
        d_frame_encoding = 5;
        d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)144);
        d_frame_mod = d_16qam;
        dout << "Encoding: 18 Mbit/s   ";
        break;
    case 8:
        d_frame_encoding = 6;
        d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)192);
        d_frame_mod = d_64qam;
        dout << "Encoding: 24 Mbit/s   ";
        break;
    case 12:
        d_frame_encoding = 7;
        d_frame_symbols = (int)ceil((16 + 8 * d_frame_bytes + 6) / (double)216);
        d_frame_mod = d_64qam;
        dout << "Encoding: 27 Mbit/s   ";
        break;
    default:
        dout << "unknown encoding" << std::endl;
        return false;
    }

    mylog("encoding: {} - length: {} - symbols: {}",
          d_frame_encoding,
          d_frame_bytes,
          d_frame_symbols);
    return true;
}

const int frame_equalizer_impl::interleaver_pattern[48] = {
    0, 3, 6, 9,  12, 15, 18, 21, 24, 27, 30, 33, 36, 39, 42, 45,
    1, 4, 7, 10, 13, 16, 19, 22, 25, 28, 31, 34, 37, 40, 43, 46,
    2, 5, 8, 11, 14, 17, 20, 23, 26, 29, 32, 35, 38, 41, 44, 47
};

// ---- 802.11n HT-SIG decode over a clean HT equalization --------

// HT-SIG CRC-8: G(x)=x^8+x^2+x+1 (0x07), register init all-ones, over the first
// 34 HT-SIG bits, result is the bit-complement of the register (802.11 19.3.9.4.3).
static uint8_t ht_sig_crc8(const uint8_t* bits, int n)
{
    uint8_t c = 0xff;
    for (int i = 0; i < n; i++) {
        uint8_t fb = (bits[i] & 1) ^ ((c >> 7) & 1);
        c = (uint8_t)(c << 1);
        if (fb) {
            c ^= 0x07;
        }
    }
    return (uint8_t)(~c);
}

// HT-SIG occupies the same 52 subcarriers as L-SIG: 48 data + 4 pilots
// {11,25,39,53}; the same legacy 48-bit interleaver applies. Equalize the raw
// FFT of an HT-SIG symbol (index sym_idx) with the L-LTF channel `h` and the
// frozen sampling-offset correction, returning the 48 data-carrier symbols.
void frame_equalizer_impl::equalize_ht_sig(const gr_complex* raw,
                                           int sym_idx,
                                           const gr_complex* h,
                                           gr_complex* out48)
{
    // HT-SIG sits on the LOWER 20 MHz subchannel (bins 6..58, pilots 11/25/39/53)
    // in both HT20 and HT40, so the bin layout is identical; only the sampling
    // ramp depends on the FFT size.
    const int dc = d_fft_len / 2;
    const int slen = d_fft_len + d_fft_len / 4;
    gr_complex eqd[128];
    // NOTE: the per-subcarrier sampling-offset ramp (d_epsilon0 = CFO/f_carrier)
    // assumes the TX and RX share a reference oscillator -- valid for two radios on
    // one board, but FALSE when an SDR captures an independent transmitter (the B210
    // sample clock and the chip LO are unrelated, so eps0 derived from CFO is a bogus
    // SFO). Applying it injects a spurious linear phase that flips HT-SIG band-edge
    // bits (the CRC-8 has no FEC margin). The true SFO is negligible over the 3-4
    // symbols from L-LTF to HT-SIG, so we drop the ramp here. (DISABLE_HT_SIG_RAMP)
    for (int i = 6; i <= 58; i++) {
        eqd[i] = raw[i] / h[i];
    }
    (void)slen;
    // Common-phase derotation from the 4 HT-SIG pilots {11,25,39,53}, which are
    // QBPSK {+1,+1,+1,-1}*Pn*j. Using the known pilots (not a blind square)
    // resolves the phase unambiguously and is robust to sync-alignment jitter --
    // and the pilots' own j cancels the data's QBPSK rotation, landing data on
    // the real axis. (Same scheme as the HT DATA path.)
    gr_complex p = equalizer::base::POLARITY[(sym_idx - 2) % 127];
    gr_complex acc = eqd[11] * p + eqd[25] * p + eqd[39] * p + eqd[53] * (-p);
    gr_complex derot = std::polar(1.0f, (float)(-std::arg(acc)));
    int c = 0;
    for (int i = 6; i <= 58; i++) {
        if (i == 11 || i == 25 || i == 32 || i == 39 || i == 53) {
            continue;
        }
        out48[c++] = eqd[i] * derot;
    }
}

// Decode the two equalized HT-SIG symbols at the given per-symbol derotation
// phases. Returns true (and fills out params) on valid CRC-8 + sane fields.
bool frame_equalizer_impl::try_ht_sig(const gr_complex* eq,
                                      int& mcs,
                                      int& cbw,
                                      int& len,
                                      int& stbc,
                                      int& fec,
                                      int& sgi)
{
    uint8_t coded[96];
    for (int s = 0; s < 2; s++) {
        uint8_t raw[48];
        for (int c = 0; c < 48; c++) {
            raw[c] = std::real(eq[s * 48 + c]) > 0 ? 1 : 0; // pilot-derotated
        }
        for (int i = 0; i < 48; i++) {
            coded[s * 48 + i] = raw[interleaver_pattern[i]];
        }
    }

    // Viterbi decode 96 -> 48 info bits (BPSK rate 1/2 over 2 OFDM symbols).
    static ofdm_param ofdm(BPSK_1_2);
    static frame_param frame(ofdm, 0);
    frame.n_sym = 2;
    frame.n_data_bits = 48;
    frame.n_encoded_bits = 96;
    frame.psdu_size = 0;
    frame.n_pad = 0;
    uint8_t* bits = d_decoder.decode(&ofdm, &frame, coded);

    mcs = 0;
    for (int i = 0; i < 7; i++) {
        mcs |= bits[i] << i;
    }
    cbw = bits[7];
    len = 0;
    for (int i = 0; i < 16; i++) {
        len |= bits[8 + i] << i;
    }
    stbc = bits[28] | (bits[29] << 1);
    fec = bits[30];
    sgi = bits[31];

    uint8_t crc_calc = ht_sig_crc8(bits, 34);
    int crc_msb = 0;
    for (int i = 0; i < 8; i++) {
        crc_msb |= bits[34 + i] << (7 - i); // CRC transmitted MSB-first (c7..c0)
    }
    // fec (BCC vs LDPC) is returned for the caller to dispatch on -- not gated here.
    return (crc_calc == crc_msb) && mcs <= 31 && len >= 1 && len <= 4095;
}

void frame_equalizer_impl::sniff_ht_sig()
{
    // PHY-format detection seam: symbols 3,4 after L-SIG carry HT-SIG
    // (QBPSK x2) for HT-mixed and VHT-SIG-A (BPSK then QBPSK) for VHT. We try HT-SIG
    // here (CRC-8 gate). A VHT branch would, on HT-SIG CRC failure, attempt
    // VHT-SIG-A decode and, on success, set d_format=FORMAT_VHT and dispatch to a
    // vht_begin() parallel to ht_begin()/mimo_begin(). The 2x2 channel machinery
    // (mimo_estimate_ltf / MMSE / stream de-parse) is format-agnostic and reused for
    // VHT-LTF; only the SIG parse, subcarrier geometry and LDPC FEC differ.
    const gr_complex* h = d_equalizer->get_h();

    // Pilot-derotated HT equalization of the two candidate symbols (3,4).
    gr_complex eq[96];
    equalize_ht_sig(d_ht_raw[0], 3, h, eq);
    equalize_ht_sig(d_ht_raw[1], 4, h, eq + 48);

    int mcs, cbw, len, stbc, fec, sgi;

    // Real 802.11 HT-SIG / VHT-SIG-A carry BPSK pilots + QBPSK data, so after
    // pilot-phase derotation the QBPSK data lands on the IMAGINARY axis (verified
    // against real Realtek frames: a clean 6M-L-SIG beacon decodes to its true MCS
    // only on the imag axis). The synthetic generator instead rotates the pilots too,
    // landing data on the REAL axis. Decode robustly by trying the real axis first
    // (matches the synthetic harness) then the imaginary axis -- rotating eq by -j so
    // try_*_sig's real-axis slicer reads the imaginary component.
    gr_complex eq_q[96];
    for (int k = 0; k < 96; k++) {
        eq_q[k] = eq[k] * gr_complex(0, -1);
    }

    bool ht_ok = try_ht_sig(eq, mcs, cbw, len, stbc, fec, sgi) ||
                 try_ht_sig(eq_q, mcs, cbw, len, stbc, fec, sgi);

    if (d_debug) {
        // Per-sniff diagnostic for over-air HT-SIG bring-up: axis-agnostic coherence of
        // the raw-equalized SIG (|mean(d^2)|/mean|d|^2; ~1 = clean symbols, ~0 = bad
        // sync/FFT-window) plus each axis's decode. Gated; no effect on dispatch.
        static int dbg_n = 0;
        gr_complex acc2 = 0;
        double pwr = 0;
        for (int i = 6; i <= 58; i++) {
            if (i == 11 || i == 25 || i == 32 || i == 39 || i == 53)
                continue;
            gr_complex d = d_ht_raw[0][i] / h[i];
            acc2 += d * d;
            pwr += std::norm(d);
        }
        int m0, c0, l0, s0, f0, g0, m1, c1, l1, s1, f1, g1;
        bool a0 = try_ht_sig(eq, m0, c0, l0, s0, f0, g0);
        bool a1 = try_ht_sig(eq_q, m1, c1, l1, s1, f1, g1);
        std::cerr << "[ht-dbg] sniff#" << ++dbg_n
                  << " rawCoh=" << std::abs(acc2) / (pwr + 1e-9)
                  << " real{ok=" << a0 << " mcs=" << m0 << "}"
                  << " imag{ok=" << a1 << " mcs=" << m1 << "}" << std::endl;
    }

    if (ht_ok) {
        std::cout << "[HT-SIG] CRC-OK mcs=" << mcs << " cbw" << (cbw ? 40 : 20)
                  << " len=" << len << " stbc=" << stbc
                  << " fec=" << (fec ? "LDPC" : "BCC") << " sgi=" << sgi << std::endl;
        // 2x2 MIMO (MCS 8-15, HT20, BCC) when 2 RX antennas are wired.
        if (d_n_rx == 2 && mcs >= 8 && mcs <= 15 && cbw == 0 && stbc == 0 && fec == 0) {
            mimo_begin(mcs, len);
            return;
        }
        // SISO BCC, bandwidth matching this block's FFT (HT20 cbw=0 / HT40 cbw=1).
        const int want_cbw = (d_fft_len == 128) ? 1 : 0;
        if (cbw == want_cbw && stbc == 0 && fec == 0) {
            ht_begin(mcs, len);
        }
        // HT STBC (1 SS -> 2 STS Alamouti), HT20 SISO, BCC, MCS 0-7.
        else if (d_fft_len == 64 && cbw == 0 && stbc == 1 && fec == 0 && mcs <= 7) {
            stbc_begin(mcs, len);
        }
        // HT LDPC (R=1/2 n=648, single codeword, BPSK MCS0): collect the data bits
        // like BCC, then run the min-sum LDPC decoder in ht_finish.
        else if (d_fft_len == 64 && cbw == 0 && stbc == 0 && fec == 1 && mcs == 0) {
            ht_begin(mcs, len);
            if (d_ht_active) {
                d_ht_fec = true;
                d_ht_nsym = 13; // ceil(648 / 52) BPSK data symbols for one codeword
            }
        }
        return;
    }

    // Not HT -> try VHT-SIG-A. VHT20 SISO only on the 64-FFT block. Spec VHT-SIG-A is
    // MIXED-axis: A1 BPSK (real axis), A2 QBPSK (imag axis) -- verified against
    // GR-WiFi's spec generator AND matched by our (now spec-compliant) synthetic
    // generator. eq_mix carries symbol-1 from eq (real) and symbol-2 from eq_q (imag).
    // A SINGLE attempt (not a blind multi-axis trial) keeps the CRC-8 false-positive
    // surface minimal so the synthetic regression stays stable.
    gr_complex eq_mix[96];
    for (int k = 0; k < 48; k++) {
        eq_mix[k] = eq[k];             // VHT-SIG-A1: real axis
        eq_mix[48 + k] = eq_q[48 + k]; // VHT-SIG-A2: imag axis (eq_q = eq * -j)
    }
    int vmcs, vbw, vstbc, vfec, vsgi, vnss;
    if (d_fft_len == 64 &&
        try_vht_sig(eq_mix, vmcs, vbw, vstbc, vfec, vsgi, vnss)) {
        std::cout << "[VHT-SIG-A] CRC-OK mcs=" << vmcs << " nss=" << vnss
                  << " bw" << (20 << vbw) << " stbc=" << vstbc
                  << " fec=" << (vfec ? "LDPC" : "BCC") << " sgi=" << vsgi << std::endl;
        // First cut: SU, 20 MHz, single stream, BCC, MCS 0-7. The PSDU length is
        // not in VHT-SIG-A; defer the data decode until VHT-SIG-B (sym7).
        if (vbw == 0 && vnss == 1 && vstbc == 0 && vfec == 0 && vmcs <= 7) {
            d_vht_pending = true;
            d_vht_mcs = vmcs;
            d_vht_nss = vnss;
        }
    }
}

// Decode the two equalized SIG symbols as VHT-SIG-A (SU). The BCC decode is
// identical to try_ht_sig (same 96 -> 48); only the field map + validity differ.
// VHT-SIG-A1(24): B0-1 BW, B3 STBC, B10-12 NSTS(=NSS-1).  VHT-SIG-A2(24): B0 SGI,
// B2 coding(0=BCC), B4-7 VHT-MCS, B10-17 CRC-8 over B0..B33.
bool frame_equalizer_impl::try_vht_sig(const gr_complex* eq,
                                       int& mcs,
                                       int& bw,
                                       int& stbc,
                                       int& fec,
                                       int& sgi,
                                       int& nss)
{
    uint8_t coded[96];
    for (int s = 0; s < 2; s++) {
        uint8_t raw[48];
        for (int c = 0; c < 48; c++) {
            raw[c] = std::real(eq[s * 48 + c]) > 0 ? 1 : 0;
        }
        for (int i = 0; i < 48; i++) {
            coded[s * 48 + i] = raw[interleaver_pattern[i]];
        }
    }
    static ofdm_param ofdm(BPSK_1_2);
    static frame_param frame(ofdm, 0);
    frame.n_sym = 2;
    frame.n_data_bits = 48;
    frame.n_encoded_bits = 96;
    frame.psdu_size = 0;
    frame.n_pad = 0;
    uint8_t* bits = d_decoder.decode(&ofdm, &frame, coded);

    bw = bits[0] | (bits[1] << 1);
    stbc = bits[3];
    nss = (bits[10] | (bits[11] << 1) | (bits[12] << 2)) + 1; // NSTS+1
    sgi = bits[24];
    fec = bits[26];
    mcs = bits[28] | (bits[29] << 1) | (bits[30] << 2) | (bits[31] << 3);

    uint8_t crc_calc = ht_sig_crc8(bits, 34);
    int crc_msb = 0;
    for (int i = 0; i < 8; i++) {
        crc_msb |= bits[34 + i] << (7 - i);
    }
    return (crc_calc == crc_msb) && mcs <= 9 && nss >= 1 && nss <= 4;
}

// VHT-SIG-B (sym7, 20 MHz SU): BPSK over the 52 data tones, BCC rate-1/2 (52 coded
// -> 26 info, no interleave in this harness). B0-16 = APEP-Length in 4-octet units.
// Recovers the PSDU length VHT-SIG-A lacks, then starts the VHT data decode.
void frame_equalizer_impl::vht_read_sig_b(const gr_complex* raw)
{
    const gr_complex* h = d_equalizer->get_h();
    const int slen = d_fft_len + d_fft_len / 4;
    gr_complex eqd[64];
    for (int i = 4; i <= 60; i++) {
        gr_complex hi = h[i];
        if (i < 6) {
            hi = h[6];
        } else if (i > 58) {
            hi = h[58];
        }
        gr_complex r = raw[i] * exp(gr_complex(0,
                                               2 * M_PI * 7 * slen *
                                                   (d_epsilon0 + d_er_frozen) *
                                                   (i - 32) / d_fft_len));
        eqd[i] = r / hi;
    }
    gr_complex p = equalizer::base::POLARITY[(7 - 2) % 127];
    gr_complex acc = (eqd[11] + eqd[25] + eqd[39] - eqd[53]) * p;
    gr_complex derot = std::polar(1.0f, (float)(-std::arg(acc)));

    uint8_t coded[52];
    int c = 0;
    for (int i = 4; i <= 60; i++) {
        if (i == 11 || i == 25 || i == 32 || i == 39 || i == 53) {
            continue;
        }
        coded[c++] = std::real(eqd[i] * derot) > 0 ? 1 : 0;
    }
    static ofdm_param ofdm(BPSK_1_2);
    static frame_param frame(ofdm, 0);
    frame.n_sym = 1;
    frame.n_data_bits = 26;
    frame.n_encoded_bits = 52;
    frame.psdu_size = 0;
    frame.n_pad = 0;
    uint8_t* bits = d_decoder.decode(&ofdm, &frame, coded);
    int apep = 0;
    for (int i = 0; i < 17; i++) {
        apep |= bits[i] << i;
    }
    int len = apep * 4;
    d_vht_pending = false;
    if (len >= 1 && len <= 4095) {
        vht_begin(d_vht_mcs, len);
    }
}

// Start the VHT data decode. VHT20 NSS=1 MCS 0-7 data is identical to HT20 MCS 0-7
// (52 SC, same constellations + interleaver), so reuse ht_begin's machinery via the
// equivalent HT encoding; d_vht just shifts the data start (+1 for VHT-SIG-B) and
// the finish log.
void frame_equalizer_impl::vht_begin(int mcs, int len)
{
    ht_begin(mcs, len); // sets d_ht_active + d_ht_* using HT_MCS_0+mcs, bw=20
    if (d_ht_active) {
        d_vht = true;
    }
}

// HT20 data subcarriers: FFT bins 4..60 excluding DC(32) and pilots {11,25,39,53}.
static bool ht20_is_data(int i)
{
    return i >= 4 && i <= 60 && i != 32 && i != 11 && i != 25 && i != 39 && i != 53;
}
// HT40 (128-FFT) data subcarriers: bins 6..62 and 66..122 minus the 6 pilots.
static bool ht40_is_data(int i)
{
    if (!((i >= 6 && i <= 62) || (i >= 66 && i <= 122))) {
        return false;
    }
    return i != 11 && i != 39 && i != 53 && i != 75 && i != 89 && i != 117;
}

void frame_equalizer_impl::ht_begin(int mcs, int len)
{
    if (mcs > 7) { // single-stream path (MCS 0..7); MCS 8-15 go to mimo_begin
        return;
    }
    const int bw = (d_fft_len == 128) ? 40 : 20;
    ofdm_param ofdm(Encoding(gr::ieee802_11::HT_MCS_0 + mcs), bw);
    frame_param frame(ofdm, len);
    if (frame.n_sym <= 0 || frame.n_sym > MAX_SYM ||
        frame.n_encoded_bits > MAX_ENCODED_BITS) {
        return;
    }
    d_ht_active = true;
    d_ht_fec = false; // BCC by default; the LDPC dispatch flips this on
    d_ht_mcs = mcs;
    d_ht_len = len;
    d_ht_nsym = frame.n_sym;
    d_ht_dsym = 0;
    d_ht_nbpsc = ofdm.n_bpsc;
    d_ht_ncbps = ofdm.n_cbps;
    d_ht_ndbps = ofdm.n_dbps;
}

// HT40 channel estimate from the HT-LTF (sym6): d_H[i] = raw[i]*ramp/HTLTF40[i]
// over the 114 occupied bins. Needed because the duplicate L-LTF leaves the
// center carriers null.
void frame_equalizer_impl::ht_estimate_ltf40(const gr_complex* raw)
{
    const int dc = 64, slen = 160;
    for (int i = 0; i < 128; i++) {
        gr_complex ref = equalizer::base::HTLTF40[i];
        if (ref == gr_complex(0, 0)) {
            continue;
        }
        gr_complex r = raw[i] * exp(gr_complex(0,
                                               2 * M_PI * 6 * slen *
                                                   (d_epsilon0 + d_er_frozen) *
                                                   (i - dc) / 128));
        d_ht_h[i] = r / ref;
    }
}

// HT20 channel estimate from the HT-LTF (sym6): 64-FFT analogue of ht_estimate_ltf40
// over the HTLTF20 occupied bins (L-LTF tones + the HT edge tones +-27,+-28). Real
// Realtek HT frames need this -- the legacy get_h() is corrupted by the QBPSK HT-SIG
// pilot tracking (measured: it left ~57% EVM on real data; the HT-LTF estimate ~30%).
void frame_equalizer_impl::ht_estimate_ltf20(const gr_complex* raw)
{
    const int dc = 32, slen = 80;
    for (int i = 0; i < 64; i++) {
        gr_complex ref = equalizer::base::HTLTF20[i];
        if (ref == gr_complex(0, 0)) {
            continue;
        }
        gr_complex r = raw[i] * exp(gr_complex(0,
                                               2 * M_PI * 6 * slen *
                                                   (d_epsilon0 + d_er_frozen) *
                                                   (i - dc) / 64));
        d_ht_h[i] = r / ref;
    }
}

// Equalize + demap one HT DATA OFDM symbol (raw FFT), appending n_cbps coded bits,
// with pilot-based common-phase correction. HT (20 + 40) uses the HT-LTF channel
// estimate (d_ht_h, covers the edge tones); VHT keeps the legacy L-LTF channel
// (get_h) with edge extrapolation, since VHT estimates no d_ht_h at sym6.
void frame_equalizer_impl::ht_data_symbol(const gr_complex* raw, int sym_idx)
{
    const bool ht40 = (d_fft_len == 128);
    const int dc = d_fft_len / 2;
    const int slen = d_fft_len + d_fft_len / 4;
    const bool use_lltf = d_vht;
    const gr_complex* h = use_lltf ? d_equalizer->get_h() : d_ht_h;

    gr_complex eqd[128];
    const int lo = ht40 ? 6 : 4;
    const int hi_bin = ht40 ? 122 : 60;
    for (int i = lo; i <= hi_bin; i++) {
        gr_complex hi = h[i];
        if (use_lltf && i < 6) {
            hi = h[6]; // VHT/L-LTF edge carriers (-28,-27) not in L-LTF
        } else if (use_lltf && i > 58) {
            hi = h[58]; // VHT/L-LTF edge carriers (27,28)
        }
        gr_complex r = raw[i] * exp(gr_complex(0,
                                               2 * M_PI * sym_idx * slen *
                                                   (d_epsilon0 + d_er_frozen) *
                                                   (i - dc) / d_fft_len));
        eqd[i] = r / hi;
    }

    // Common phase from the DATA pilots, 802.11-2016 cycling convention: the base pilot
    // pattern is cyclically rotated by the data-symbol index and multiplied by the
    // polarity sequence at POLARITY[sym_idx-4] (HT data starts at OFDM sym 7, VHT at 8).
    // HT20/VHT base {1,1,1,-1} at {11,25,39,53}; HT40 {1,1,1,-1,-1,1} at
    // {11,39,53,75,89,117}. This matches real Realtek silicon and the fork's TX; the old
    // fixed {+,+,+,-} pattern only matched a decoder-targeted generator.
    int jdata = d_vht ? (sym_idx - 8) : (sym_idx - 7);
    gr_complex p = equalizer::base::POLARITY[((sym_idx - 4) % 127 + 127) % 127];
    gr_complex acc(0, 0);
    if (ht40) {
        static const int sc[6] = { 11, 39, 53, 75, 89, 117 };
        static const int base[6] = { 1, 1, 1, -1, -1, 1 };
        for (int i = 0; i < 6; i++)
            acc += eqd[sc[i]] * (float)base[((i + jdata) % 6 + 6) % 6];
    } else {
        static const int sc[4] = { 11, 25, 39, 53 };
        static const int base[4] = { 1, 1, 1, -1 };
        for (int i = 0; i < 4; i++)
            acc += eqd[sc[i]] * (float)base[((i + jdata) % 4 + 4) % 4];
    }
    acc *= p;
    gr_complex derot = std::polar(1.0f, (float)(-std::arg(acc)));

    std::shared_ptr<gr::digital::constellation> mod = d_64qam;
    if (d_ht_nbpsc == 1) {
        mod = d_bpsk;
    } else if (d_ht_nbpsc == 2) {
        mod = d_qpsk;
    } else if (d_ht_nbpsc == 4) {
        mod = d_16qam;
    }
    int b0 = d_ht_dsym * d_ht_ncbps;
    int c = 0;
    for (int i = lo; i <= hi_bin; i++) {
        if (!(ht40 ? ht40_is_data(i) : ht20_is_data(i))) {
            continue;
        }
        gr_complex sym = eqd[i] * derot;
        unsigned int val = mod->decision_maker(&sym);
        for (int k = 0; k < d_ht_nbpsc; k++) {
            d_ht_rx_bits[b0 + c + k] = (val >> k) & 1;
        }
        c += d_ht_nbpsc;
    }
    d_ht_dsym++;
    if (d_ht_dsym >= d_ht_nsym) {
        ht_finish();
        d_ht_active = false;
    }
}

void frame_equalizer_impl::publish_pdu(const uint8_t* psdu, int len,
                                       bool crc_ok, int encoding)
{
    // HT/VHT/MIMO are decoded HERE (not in decode_mac), so the PDU delivery +
    // GR_KEEP_CORRUPTED surfacing live here too. Read the env once; default
    // (unset) drops FCS failures exactly as before — zero behaviour change.
    static const bool keep = (std::getenv("GR_KEEP_CORRUPTED") != nullptr);
    if (len < 4 || (!crc_ok && !keep)) {
        return;
    }
    pmt::pmt_t meta = pmt::make_dict();
    meta = pmt::dict_add(meta, pmt::mp("crc_ok"), pmt::from_bool(crc_ok));
    meta = pmt::dict_add(meta, pmt::mp("encoding"), pmt::from_uint64(encoding));
    meta = pmt::dict_add(meta, pmt::mp("dlt"), pmt::from_long(105)); // LINKTYPE_IEEE802_11
    pmt::pmt_t blob = pmt::make_blob(psdu, len - 4); // strip FCS, like decode_mac
    message_port_pub(pmt::mp("pdu"), pmt::cons(meta, blob));
}

void frame_equalizer_impl::ht_finish()
{
    // LDPC path: the collected data bits (in tone order, no BCC interleave) hold one
    // R=1/2 n=648 codeword in the first 648 positions. Min-sum decode it, take the
    // first 324 columns as the info block (SERVICE + scrambled PSDU), descramble, FCS.
    if (d_ht_fec) {
        float llr[648];
        for (int i = 0; i < 648; i++) {
            llr[i] = d_ht_rx_bits[i] ? -1.0f : 1.0f; // bit1 -> -1, bit0 -> +1
        }
        uint8_t cw[648];
        ldpc_decode_648(llr, cw);
        int state = 0;
        for (int i = 0; i < 7; i++) {
            if (cw[i]) {
                state |= 1 << (6 - i);
            }
        }
        static uint8_t lb[MAX_PSDU_SIZE + 2];
        std::memset(lb, 0, d_ht_len + 2);
        lb[0] = state;
        for (int i = 7; i < d_ht_len * 8 + 16; i++) {
            int fb = (!!(state & 64)) ^ (!!(state & 8));
            lb[i / 8] |= (fb ^ (cw[i] & 0x1)) << (i % 8);
            state = ((state << 1) & 0x7e) | fb;
        }
        boost::crc_32_type lcrc;
        lcrc.process_bytes(lb + 2, d_ht_len);
        bool lok = (lcrc.checksum() == 558161692);
        std::cout << "[LDPC-DATA] mcs=" << d_ht_mcs << " len=" << d_ht_len
                  << " (R1/2 n=648)  CRC-32 " << (lok ? "PASS" : "fail") << std::endl;
        publish_pdu(lb + 2, d_ht_len, lok, gr::ieee802_11::HT_MCS_0 + d_ht_mcs);
        d_ht_fec = false;
        return;
    }

    const int bw = (d_fft_len == 128) ? 40 : 20;
    ofdm_param ofdm(Encoding(gr::ieee802_11::HT_MCS_0 + d_ht_mcs), bw);
    frame_param frame(ofdm, d_ht_len);
    const int ncbps = d_ht_ncbps;

    // HT BCC deinterleaver (802.11-2016 19.3.11.7). 20 MHz: N_COL=13,
    // N_ROW=4*N_BPSCS; 40 MHz: N_COL=18, N_ROW=6*N_BPSCS. The interleaver maps
    // coded-bit index k -> tx subcarrier position j; deinterleave inverts it.
    static uint8_t deint[MAX_ENCODED_BITS];
    const int s = std::max(d_ht_nbpsc / 2, 1);
    const int n_col = (bw == 40) ? 18 : 13;
    const int n_row = (bw == 40 ? 6 : 4) * d_ht_nbpsc;
    for (int sym = 0; sym < d_ht_nsym; sym++) {
        for (int k = 0; k < ncbps; k++) {
            int i = n_row * (k % n_col) + (k / n_col);
            int j = s * (i / s) + (i + ncbps - (n_col * i) / ncbps) % s;
            deint[sym * ncbps + k] = d_ht_rx_bits[sym * ncbps + j];
        }
    }

    uint8_t* decoded = d_decoder.decode(&ofdm, &frame, deint);

    // descramble (7-bit LFSR): SERVICE(16) + PSDU + tail
    int state = 0;
    static uint8_t out_bytes[MAX_PSDU_SIZE + 2];
    std::memset(out_bytes, 0, d_ht_len + 2);
    for (int i = 0; i < 7; i++) {
        if (decoded[i]) {
            state |= 1 << (6 - i);
        }
    }
    out_bytes[0] = state;
    for (int i = 7; i < d_ht_len * 8 + 16; i++) {
        int fb = (!!(state & 64)) ^ (!!(state & 8));
        int bit = fb ^ (decoded[i] & 0x1);
        out_bytes[i / 8] |= bit << (i % 8);
        state = ((state << 1) & 0x7e) | fb;
    }

    // CRC-32 over the PSDU (skip the 2-byte SERVICE field). The check value is the
    // CRC-32/ISO-HDLC residue (0x2144DF1C): running the CRC over [MPDU || its FCS]
    // yields it for ANY payload, so this is a genuine per-frame FCS check.
    boost::crc_32_type crc;
    crc.process_bytes(out_bytes + 2, d_ht_len);
    bool ok = (crc.checksum() == 558161692);
    if (d_stbc_active) {
        std::cout << "[STBC-DATA] mcs=" << d_ht_mcs << " len=" << d_ht_len
                  << " nsym=" << d_ht_nsym << "  CRC-32 " << (ok ? "PASS" : "fail")
                  << std::endl;
    } else if (d_vht) {
        std::cout << "[VHT-DATA] mcs=" << d_ht_mcs << " nss=" << d_vht_nss
                  << " len=" << d_ht_len << " nsym=" << d_ht_nsym << "  CRC-32 "
                  << (ok ? "PASS" : "fail") << std::endl;
        d_vht = false;
    } else {
        std::cout << "[HT-DATA] mcs=" << d_ht_mcs << " len=" << d_ht_len
                  << " nsym=" << d_ht_nsym << "  CRC-32 " << (ok ? "PASS" : "fail")
                  << std::endl;
    }
    publish_pdu(out_bytes + 2, d_ht_len, ok, gr::ieee802_11::HT_MCS_0 + d_ht_mcs);
}

// ---- 802.11n 2x2 MIMO (MCS 8-15, HT20) ----------------------------------------

void frame_equalizer_impl::mimo_begin(int mcs, int len)
{
    const int per = mcs - 8; // per-stream MCS (8->0 .. 15->7)
    ofdm_param ofdm(Encoding(gr::ieee802_11::HT_MCS_0 + per), 20);
    const int ndbps_tot = 2 * ofdm.n_dbps;
    const int nsym = (int)ceil((16 + 8 * len + 6) / (double)ndbps_tot);
    if (nsym <= 0 || nsym > MAX_SYM)
        return;
    if (nsym * 2 * ofdm.n_cbps > MAX_ENCODED_BITS)
        return;
    d_mimo_active = true;
    d_mimo_ltf_seen = 0;
    d_ht_mcs = per; // store per-stream values; *2 for totals at finish
    d_ht_len = len;
    d_ht_nsym = nsym;
    d_ht_dsym = 0;
    d_ht_nbpsc = ofdm.n_bpsc;
    d_ht_ncbps = ofdm.n_cbps; // per-stream coded bits / symbol (N_CBPSS)
    d_ht_ndbps = ofdm.n_dbps;
}

// Per-subcarrier 2x2 channel from the 2 HT-LTF symbols (P-matrix). With
// P=[[1,-1],[1,1]], P^-1 = 0.5*[[1,1],[-1,1]]:  [H_r0;H_r1] = P^-1 [y_r0;y_r1]/LTF.
void frame_equalizer_impl::mimo_estimate_ltf()
{
    const int dc = 32, slen = 80;
    for (int sc = 4; sc <= 60; sc++) {
        gr_complex ref = equalizer::base::HTLTF20[sc];
        if (ref == gr_complex(0, 0))
            continue;
        for (int r = 0; r < 2; r++) {
            gr_complex y0 = d_mimo_ltf[0][r][sc], y1 = d_mimo_ltf[1][r][sc];
            // sampling-offset deramp at the LTF symbol positions (6,7)
            y0 *= exp(gr_complex(
                0, 2 * M_PI * 6 * slen * (d_epsilon0 + d_er_frozen) * (sc - dc) / 64));
            y1 *= exp(gr_complex(
                0, 2 * M_PI * 7 * slen * (d_epsilon0 + d_er_frozen) * (sc - dc) / 64));
            d_mimo_H[sc][r][0] = gr_complex(0.5f, 0) * (y0 + y1) / ref;
            d_mimo_H[sc][r][1] = gr_complex(0.5f, 0) * (y1 - y0) / ref;
        }
    }
}

// MMSE-separate one HT DATA symbol into 2 streams, demap each, store coded bits.
void frame_equalizer_impl::mimo_data_symbol(const gr_complex* r0,
                                            const gr_complex* r1,
                                            int sym_idx)
{
    const int dc = 32, slen = 80;
    const int dsym = sym_idx - 8; // 0-based HT DATA symbol index
    const float s2 = 1e-3f;       // MMSE noise floor (near-ZF for clean signal)

    std::shared_ptr<gr::digital::constellation> mod = d_64qam;
    if (d_ht_nbpsc == 1)
        mod = d_bpsk;
    else if (d_ht_nbpsc == 2)
        mod = d_qpsk;
    else if (d_ht_nbpsc == 4)
        mod = d_16qam;

    int c[2] = { dsym * d_ht_ncbps, dsym * d_ht_ncbps };
    for (int sc = 4; sc <= 60; sc++) {
        if (!ht20_is_data(sc))
            continue;
        gr_complex ramp = exp(gr_complex(
            0, 2 * M_PI * sym_idx * slen * (d_epsilon0 + d_er_frozen) * (sc - dc) / 64));
        gr_complex y0 = r0[sc] * ramp, y1 = r1[sc] * ramp;
        gr_complex H00 = d_mimo_H[sc][0][0], H01 = d_mimo_H[sc][0][1];
        gr_complex H10 = d_mimo_H[sc][1][0], H11 = d_mimo_H[sc][1][1];
        // MMSE: x = (H^H H + s2 I)^-1 H^H y
        gr_complex G00 = std::norm(H00) + std::norm(H10) + s2;
        gr_complex G11 = std::norm(H01) + std::norm(H11) + s2;
        gr_complex G01 = conj(H00) * H01 + conj(H10) * H11;
        gr_complex G10 = conj(H01) * H00 + conj(H11) * H10;
        gr_complex b0 = conj(H00) * y0 + conj(H10) * y1;
        gr_complex b1 = conj(H01) * y0 + conj(H11) * y1;
        gr_complex det = G00 * G11 - G01 * G10;
        gr_complex x0 = (G11 * b0 - G01 * b1) / det;
        gr_complex x1 = (-G10 * b0 + G00 * b1) / det;
        gr_complex xs[2] = { x0, x1 };
        for (int ss = 0; ss < 2; ss++) {
            gr_complex sym = xs[ss];
            unsigned int val = mod->decision_maker(&sym);
            for (int k = 0; k < d_ht_nbpsc; k++) {
                d_mimo_bits[ss][c[ss] + k] = (val >> k) & 1;
            }
            c[ss] += d_ht_nbpsc;
        }
    }
    d_ht_dsym++;
    if (d_ht_dsym >= d_ht_nsym) {
        mimo_finish();
        d_mimo_active = false;
    }
}

void frame_equalizer_impl::mimo_finish()
{
    const int ncbps = d_ht_ncbps; // per-stream
    const int s = std::max(d_ht_nbpsc / 2, 1);
    const int n_col = 13;
    const int n_row = 4 * d_ht_nbpsc;
    const int n_rot = 11;

    // per-stream deinterleave (with the 3rd-permutation rotation for stream>0)
    static uint8_t deint[2][MAX_ENCODED_BITS];
    for (int ss = 0; ss < 2; ss++) {
        const int issp = ss + 1;
        const int jrot =
            (((issp - 1) * 2) % 3 + 3 * ((issp - 1) / 3)) * n_rot * d_ht_nbpsc;
        for (int sym = 0; sym < d_ht_nsym; sym++) {
            for (int k = 0; k < ncbps; k++) {
                int ii = n_row * (k % n_col) + (k / n_col);
                int j = s * (ii / s) + (ii + ncbps - (n_col * ii) / ncbps) % s;
                int r = ((j - jrot) % ncbps + ncbps) % ncbps; // forward k -> position
                deint[ss][sym * ncbps + k] = d_mimo_bits[ss][sym * ncbps + r];
            }
        }
    }

    // stream de-parse: s bits at a time, round-robin from the 2 streams -> one BCC seq
    static uint8_t deparsed[MAX_ENCODED_BITS];
    int pos[2] = { 0, 0 }, idx = 0;
    const int blocks_per_sym = (2 * ncbps) / s;
    for (int sym = 0; sym < d_ht_nsym; sym++) {
        for (int blk = 0; blk < blocks_per_sym; blk++) {
            int ss = blk % 2;
            for (int b = 0; b < s; b++) {
                deparsed[idx++] = deint[ss][pos[ss]++];
            }
        }
    }

    // single Viterbi decode over the combined stream (n_cbps/n_dbps doubled)
    ofdm_param ofdm(Encoding(gr::ieee802_11::HT_MCS_0 + d_ht_mcs), 20);
    ofdm.n_cbps *= 2;
    ofdm.n_dbps *= 2;
    frame_param frame(ofdm, d_ht_len);
    uint8_t* decoded = d_decoder.decode(&ofdm, &frame, deparsed);

    int state = 0;
    static uint8_t out_bytes[MAX_PSDU_SIZE + 2];
    std::memset(out_bytes, 0, d_ht_len + 2);
    for (int i = 0; i < 7; i++) {
        if (decoded[i])
            state |= 1 << (6 - i);
    }
    out_bytes[0] = state;
    for (int i = 7; i < d_ht_len * 8 + 16; i++) {
        int fb = (!!(state & 64)) ^ (!!(state & 8));
        int bit = fb ^ (decoded[i] & 0x1);
        out_bytes[i / 8] |= bit << (i % 8);
        state = ((state << 1) & 0x7e) | fb;
    }

    boost::crc_32_type crc;
    crc.process_bytes(out_bytes + 2, d_ht_len);
    bool ok = (crc.checksum() == 558161692);
    std::cout << "[HT-MIMO] mcs=" << (d_ht_mcs + 8) << " (2x2) len=" << d_ht_len
              << " nsym=" << d_ht_nsym << "  CRC-32 " << (ok ? "PASS" : "fail")
              << std::endl;
    publish_pdu(out_bytes + 2, d_ht_len, ok,
                gr::ieee802_11::HT_MCS_0 + d_ht_mcs + 8);
}

// ---- 802.11n HT STBC (1 SS -> 2 STS Alamouti, HT20, 1 RX antenna) -------------

void frame_equalizer_impl::stbc_begin(int mcs, int len)
{
    ofdm_param ofdm(Encoding(gr::ieee802_11::HT_MCS_0 + mcs), 20);
    frame_param frame(ofdm, len);
    int nsym = frame.n_sym;
    if (nsym & 1) {
        nsym++; // Alamouti needs an even number of OFDM symbols (symbol pairs)
    }
    if (nsym <= 0 || nsym > MAX_SYM || nsym * ofdm.n_cbps > MAX_ENCODED_BITS) {
        return;
    }
    d_stbc_active = true;
    d_stbc_ltf_seen = 0;
    d_stbc_have_y0 = false;
    d_ht_mcs = mcs;
    d_ht_len = len;
    d_ht_nsym = nsym;
    d_ht_dsym = 0;
    d_ht_nbpsc = ofdm.n_bpsc;
    d_ht_ncbps = ofdm.n_cbps;
    d_ht_ndbps = ofdm.n_dbps;
}

// Estimate the two STS channels h0,h1 from the 2 HT-LTF symbols (P matrix) on the
// single antenna: y0 = (h0 - h1)*HTLTF20, y1 = (h0 + h1)*HTLTF20  (P2 rows {1,-1}/{1,1})
// -> h0 = (a + b)/2, h1 = (b - a)/2 with a = y0/ref, b = y1/ref.
void frame_equalizer_impl::stbc_estimate_ltf()
{
    const int dc = 32, slen = 80;
    for (int i = 0; i < 64; i++) {
        gr_complex ref = equalizer::base::HTLTF20[i];
        if (ref == gr_complex(0, 0)) {
            d_stbc_h0[i] = d_stbc_h1[i] = gr_complex(0, 0);
            continue;
        }
        gr_complex a = d_stbc_ltf[0][i] *
                       exp(gr_complex(0, 2 * M_PI * 6 * slen *
                                             (d_epsilon0 + d_er_frozen) * (i - dc) / 64)) /
                       ref;
        gr_complex b = d_stbc_ltf[1][i] *
                       exp(gr_complex(0, 2 * M_PI * 7 * slen *
                                             (d_epsilon0 + d_er_frozen) * (i - dc) / 64)) /
                       ref;
        // Standard 802.11 HT-LTF P-matrix [[1,-1],[1,1]] (STS0=[+,-], STS1=[+,+]) as used
        // by real Realtek silicon and the fork's TX: a=h0+h1, b=-h0+h1 -> recover via
        // h0=(a-b)/2, h1=(a+b)/2. (The old [[1,1],[-1,1]] gave (a+b)/2, (b-a)/2.)
        d_stbc_h0[i] = (a - b) * gr_complex(0.5f, 0);
        d_stbc_h1[i] = (a + b) * gr_complex(0.5f, 0);
    }
}

// One HT STBC DATA symbol. Symbols arrive in pairs: buffer the first (y0), and on
// the second (y1) Alamouti-combine per subcarrier to recover the pair (s0,s1):
//   s0 = (h0* y0 + h1 y1*) / g ;  s1 = -conj((h1* y0 - h0 y1*) / g) ;  g=|h0|^2+|h1|^2
// then demap both recovered symbols into the coded-bit buffer.
void frame_equalizer_impl::stbc_data_symbol(const gr_complex* raw, int sym_idx)
{
    const int dc = 32, slen = 80;
    gr_complex y[64];
    for (int i = 4; i <= 60; i++) {
        y[i] = raw[i] * exp(gr_complex(0, 2 * M_PI * sym_idx * slen *
                                              (d_epsilon0 + d_er_frozen) * (i - dc) / 64));
    }
    if (!d_stbc_have_y0) {
        std::memcpy(d_stbc_y0, y, sizeof(y));
        d_stbc_y0_sym = sym_idx;
        d_stbc_have_y0 = true;
        return;
    }
    // second of the pair -> combine. Pilots ride STS0 (h0); use them for a per-pair
    // common-phase correction on both recovered streams.
    std::shared_ptr<gr::digital::constellation> mod = d_64qam;
    if (d_ht_nbpsc == 1) {
        mod = d_bpsk;
    } else if (d_ht_nbpsc == 2) {
        mod = d_qpsk;
    } else if (d_ht_nbpsc == 4) {
        mod = d_16qam;
    }
    gr_complex s0[64], s1[64];
    for (int i = 4; i <= 60; i++) {
        gr_complex h0 = d_stbc_h0[i], h1 = d_stbc_h1[i];
        float g = std::norm(h0) + std::norm(h1);
        if (g < 1e-6f) {
            s0[i] = s1[i] = gr_complex(0, 0);
            continue;
        }
        gr_complex y0 = d_stbc_y0[i], y1 = y[i];
        s0[i] = (std::conj(h0) * y0 + h1 * std::conj(y1)) / g;
        gr_complex s1h = (std::conj(h1) * y0 - h0 * std::conj(y1)) / g;
        s1[i] = -std::conj(s1h);
    }
    // common-phase from the STS0 pilots, spec cycling convention (base {1,1,1,-1} rotated
    // by the data-symbol index * POLARITY[sym-5]; STBC data starts at OFDM sym 8, so the
    // SISO data index = sym-8 and the polarity start is 3 -> POLARITY[3+(sym-8)]).
    static const int psc[4] = { 11, 25, 39, 53 };
    static const int pbase[4] = { 1, 1, 1, -1 };
    auto pilot_acc = [&](const gr_complex* s, int sym) {
        int j = sym - 8;
        gr_complex pol = equalizer::base::POLARITY[((sym - 5) % 127 + 127) % 127];
        gr_complex a(0, 0);
        for (int k = 0; k < 4; k++)
            a += s[psc[k]] * (float)pbase[((k + j) % 4 + 4) % 4];
        return a * pol;
    };
    gr_complex a0 = pilot_acc(s0, d_stbc_y0_sym);
    gr_complex a1 = pilot_acc(s1, sym_idx);
    gr_complex d0 = std::polar(1.0f, (float)(-std::arg(a0)));
    gr_complex d1 = std::polar(1.0f, (float)(-std::arg(a1)));

    for (int pass = 0; pass < 2; pass++) {
        const gr_complex* s = pass ? s1 : s0;
        gr_complex derot = pass ? d1 : d0;
        int b0 = d_ht_dsym * d_ht_ncbps;
        int c = 0;
        for (int i = 4; i <= 60; i++) {
            if (!ht20_is_data(i)) {
                continue;
            }
            gr_complex sym = s[i] * derot;
            unsigned int val = mod->decision_maker(&sym);
            for (int k = 0; k < d_ht_nbpsc; k++) {
                d_ht_rx_bits[b0 + c + k] = (val >> k) & 1;
            }
            c += d_ht_nbpsc;
        }
        d_ht_dsym++;
    }
    d_stbc_have_y0 = false;
    if (d_ht_dsym >= d_ht_nsym) {
        ht_finish(); // reuses d_ht_* deinterleave/Viterbi/descramble/CRC; logs [STBC-DATA]
        d_stbc_active = false;
    }
}

} /* namespace ieee802_11 */
} /* namespace gr */
