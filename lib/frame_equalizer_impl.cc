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
#include "utils.h"
#include <gnuradio/io_signature.h>

namespace gr {
namespace ieee802_11 {

frame_equalizer::sptr
frame_equalizer::make(Equalizer algo, double freq, double bw, bool log, bool debug)
{
    return gnuradio::get_initial_sptr(
        new frame_equalizer_impl(algo, freq, bw, log, debug));
}


frame_equalizer_impl::frame_equalizer_impl(
    Equalizer algo, double freq, double bw, bool log, bool debug)
    : gr::block("frame_equalizer",
                gr::io_signature::make(1, 1, 64 * sizeof(gr_complex)),
                gr::io_signature::make(1, 1, 48)),
      d_current_symbol(0),
      d_log(log),
      d_debug(debug),
      d_equalizer(NULL),
      d_freq(freq),
      d_bw(bw),
      d_frame_bytes(0),
      d_frame_symbols(0),
      d_freq_offset_from_synclong(0.0)
{

    message_port_register_out(pmt::mp("symbols"));

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
}

int frame_equalizer_impl::general_work(int noutput_items,
                                       gr_vector_int& ninput_items,
                                       gr_vector_const_void_star& input_items,
                                       gr_vector_void_star& output_items)
{

    gr::thread::scoped_lock lock(d_mutex);

    const gr_complex* in = (const gr_complex*)input_items[0];
    uint8_t* out = (uint8_t*)output_items[0];

    int i = 0;
    int o = 0;
    gr_complex symbols[48];
    gr_complex current_symbol[64];

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

        // not interesting -> skip
        if (d_current_symbol > (d_frame_symbols + 2)) {
            i++;
            continue;
        }

        std::memcpy(current_symbol, in + i * 64, 64 * sizeof(gr_complex));

        // HT (Phase 1b): stash the *raw* FFT of the two symbols after L-SIG
        // (candidate HT-SIG) before any legacy ramp/pilot correction mutates
        // current_symbol, and freeze the residual-CFO estimate as it stood at
        // L-SIG. The legacy pilot tracking corrupts d_er on QBPSK pilots, so the
        // HT path re-equalizes these raw symbols itself (see sniff_ht_sig).
        if (d_current_symbol == 3) {
            d_er_frozen = d_er;
            std::memcpy(d_ht_raw[0], in + i * 64, 64 * sizeof(gr_complex));
        } else if (d_current_symbol == 4) {
            std::memcpy(d_ht_raw[1], in + i * 64, 64 * sizeof(gr_complex));
        }

        // compensate sampling offset
        for (int i = 0; i < 64; i++) {
            current_symbol[i] *= exp(gr_complex(0,
                                                2 * M_PI * d_current_symbol * 80 *
                                                    (d_epsilon0 + d_er) * (i - 32) / 64));
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

        er *= d_bw / (2 * M_PI * d_freq * 80);

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
        for (int i = 0; i < 64; i++) {
            current_symbol[i] *= exp(gr_complex(0, -beta));
        }

        // update estimate of residual frequency offset
        if (d_current_symbol >= 2) {

            double alpha = 0.1;
            d_er = (1 - alpha) * d_er + alpha * er;
        }

        // do equalization
        d_equalizer->equalize(
            current_symbol, d_current_symbol, symbols, out + o * 48, d_frame_mod);

        // HT (Phase 1b): once both candidate HT-SIG symbols are stashed, decode
        // them off the raw FFT (clean HT equalization). Diagnostic only for now --
        // does not touch the legacy output flow.
        if (d_current_symbol == 4) {
            sniff_ht_sig();
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

// ---- 802.11n HT-SIG decode over a clean HT equalization (Phase 1b) --------

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
    int c = 0;
    for (int i = 0; i < 64; i++) {
        if ((i == 11) || (i == 25) || (i == 32) || (i == 39) || (i == 53) || (i < 6) ||
            (i > 58)) {
            continue;
        }
        // replicate the legacy sampling-offset ramp with the frozen d_er
        gr_complex r =
            raw[i] * exp(gr_complex(0,
                                    2 * M_PI * sym_idx * 80 *
                                        (d_epsilon0 + d_er_frozen) * (i - 32) / 64));
        out48[c++] = r / h[i];
    }
}

// Decode the two equalized HT-SIG symbols at the given per-symbol derotation
// phases. Returns true (and fills out params) on valid CRC-8 + sane fields.
bool frame_equalizer_impl::try_ht_sig(const gr_complex* eq,
                                      const double* phi,
                                      int& mcs,
                                      int& cbw,
                                      int& len,
                                      int& stbc,
                                      int& fec,
                                      int& sgi)
{
    uint8_t coded[96];
    for (int s = 0; s < 2; s++) {
        gr_complex rot = std::polar(1.0f, (float)(-phi[s]));
        uint8_t raw[48];
        for (int c = 0; c < 48; c++) {
            raw[c] = std::real(eq[s * 48 + c] * rot) > 0 ? 1 : 0;
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
    return (crc_calc == crc_msb) && mcs <= 31 && len >= 1 && len <= 4095 && fec == 0;
}

void frame_equalizer_impl::sniff_ht_sig()
{
    const gr_complex* h = d_equalizer->get_h();

    // Clean HT equalization of the two raw candidate symbols (indices 3,4).
    gr_complex eq[96];
    equalize_ht_sig(d_ht_raw[0], 3, h, eq);
    equalize_ht_sig(d_ht_raw[1], 4, h, eq + 48);

    // HT-SIG is QBPSK. Blind-estimate each symbol's axis by squaring (removes the
    // +/-1 modulation -> 2*axis): phi = 0.5*arg(sum sym^2). The square has a
    // 180-degree ambiguity, so try both resolutions per symbol (4 combinations)
    // and accept the one whose HT-SIG CRC-8 passes.
    double base[2];
    for (int s = 0; s < 2; s++) {
        gr_complex acc(0, 0);
        for (int c = 0; c < 48; c++) {
            acc += eq[s * 48 + c] * eq[s * 48 + c];
        }
        base[s] = 0.5 * std::arg(acc);
    }

    for (int a = 0; a < 2; a++) {
        for (int b = 0; b < 2; b++) {
            double phi[2] = { base[0] + a * M_PI, base[1] + b * M_PI };
            int mcs, cbw, len, stbc, fec, sgi;
            if (try_ht_sig(eq, phi, mcs, cbw, len, stbc, fec, sgi)) {
                std::cout << "[HT-SIG] CRC-OK mcs=" << mcs << " cbw" << (cbw ? 40 : 20)
                          << " len=" << len << " stbc=" << stbc
                          << " fec=" << (fec ? "LDPC" : "BCC") << " sgi=" << sgi
                          << std::endl;
                return;
            }
        }
    }
}

} /* namespace ieee802_11 */
} /* namespace gr */
