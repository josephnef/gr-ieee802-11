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
#include <ieee802_11/decode_mac.h>

#include "utils.h"
#include "viterbi_decoder/viterbi_decoder.h"

#include <gnuradio/io_signature.h>
#include <boost/crc.hpp>
#include <cstdlib>
#include <iomanip>

using namespace gr::ieee802_11;

#define LINKTYPE_IEEE802_11 105 /* http://www.tcpdump.org/linktypes.html */

class decode_mac_impl : public decode_mac
{

public:
    decode_mac_impl(bool log, bool debug)
        : block("decode_mac",
                // in0: 48 packed legacy symbol values (hard). in1 (OPTIONAL):
                // per-coded-bit soft values from frame_equalizer for the
                // GR_SOFT_VITERBI decode (same item rate; consumed in lockstep,
                // used only when soft is on). Optional so existing single-input
                // flowgraphs keep working untouched.
                gr::io_signature::make2(1, 2, 48, LEGACY_SOFT_STRIDE),
                gr::io_signature::make(0, 0, 0)),
          d_log(log),
          d_debug(debug),
          d_ofdm(BPSK_1_2),
          d_frame(d_ofdm, 0),
          d_frame_complete(true)
    {
        message_port_register_out(pmt::mp("out"));
    }

    int general_work(int noutput_items,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items)
    {

        const uint8_t* in = (const uint8_t*)input_items[0];
        // in1 (soft) is optional; present only when an upstream connects it.
        const uint8_t* in_soft =
            (input_items.size() > 1) ? (const uint8_t*)input_items[1] : nullptr;
        d_soft_in_present = (in_soft != nullptr);

        int i = 0;

        std::vector<gr::tag_t> tags;
        const uint64_t nread = this->nitems_read(0);

        dout << "Decode MAC: input " << ninput_items[0] << std::endl;

        while (i < ninput_items[0]) {

            get_tags_in_range(tags, 0, nread + i, nread + i + 1);

            if (tags.size()) {
                if (d_frame_complete == false) {
                    dout << "Warning: starting to receive new frame before old frame was "
                            "complete"
                         << std::endl;
                    dout << "Already copied " << copied << " out of " << d_frame.n_sym
                         << " symbols of last frame" << std::endl;
                }
                d_frame_complete = false;

                // Enter tags into metadata dictionary
                d_meta = pmt::make_dict();
                for (auto tag : tags)
                    d_meta = pmt::dict_add(d_meta, tag.key, tag.value);

                int len_data = pmt::to_uint64(pmt::dict_ref(
                    d_meta, pmt::mp("frame bytes"), pmt::from_uint64(MAX_PSDU_SIZE + 1)));
                int encoding = pmt::to_uint64(
                    pmt::dict_ref(d_meta, pmt::mp("encoding"), pmt::from_uint64(0)));

                // Only legacy encodings (0..7) are decodable on this path; an
                // out-of-range value would leave ofdm_param uninitialized (the
                // assert is compiled out in release) and yield a garbage frame.
                if (encoding > QAM64_3_4) {
                    dout << "Dropping frame with non-legacy encoding " << encoding
                         << std::endl;
                } else {
                    ofdm_param ofdm = ofdm_param((Encoding)encoding);
                    frame_param frame = frame_param(ofdm, len_data);

                    // Accept only fully self-consistent frames that fit every fixed
                    // decode buffer. This guards the Viterbi/deinterleave buffers
                    // against noise-triggered false detections.
                    if (frame.n_sym > 0 && frame.n_sym <= MAX_SYM &&
                        frame.psdu_size > 0 && frame.psdu_size <= MAX_PSDU_SIZE &&
                        frame.n_data_bits > 0 && frame.n_data_bits <= MAX_ENCODED_BITS &&
                        frame.n_encoded_bits > 0 &&
                        frame.n_encoded_bits <= MAX_ENCODED_BITS) {
                        d_ofdm = ofdm;
                        d_frame = frame;
                        copied = 0;
                        dout << "Decode MAC: frame start -- len " << len_data
                             << "  symbols " << frame.n_sym << "  encoding " << encoding
                             << std::endl;
                    } else {
                        dout << "Dropping frame which is too large/inconsistent "
                                "(symbols or bits)"
                             << std::endl;
                    }
                }
            }

            if (copied < d_frame.n_sym) {
                dout << "copy one symbol, copied " << copied << " out of "
                     << d_frame.n_sym << std::endl;
                std::memcpy(d_rx_symbols + (copied * 48), in, 48);
                if (in_soft)
                    std::memcpy(d_rx_soft_raw + (copied * LEGACY_SOFT_STRIDE),
                                in_soft, LEGACY_SOFT_STRIDE);
                copied++;

                if (copied == d_frame.n_sym) {
                    dout << "received complete frame - decoding" << std::endl;
                    decode();
                    in += 48;
                    if (in_soft)
                        in_soft += LEGACY_SOFT_STRIDE;
                    i++;
                    d_frame_complete = true;
                    break;
                }
            }

            in += 48;
            if (in_soft)
                in_soft += LEGACY_SOFT_STRIDE;
            i++;
        }

        consume(0, i);
        if (d_soft_in_present)
            consume(1, i);

        return 0;
    }

    void decode()
    {

        // GR_SOFT_VITERBI: deinterleave the per-coded-bit soft values and run the
        // soft Viterbi. Default (unset) keeps the hard unpack + decode bit-identical.
        static const bool soft_on = std::getenv("GR_SOFT_VITERBI") != nullptr;
        const int n_cbps = d_ofdm.n_cbps; // = 48 * n_bpsc for legacy
        uint8_t* decoded;
        if (soft_on && d_soft_in_present) {
            for (int s = 0; s < d_frame.n_sym; s++) {
                std::memcpy(d_rx_soft + s * n_cbps,
                            d_rx_soft_raw + s * LEGACY_SOFT_STRIDE, n_cbps);
            }
            deinterleave(d_rx_soft, d_deinterleaved_soft);
            decoded = d_decoder.decode_soft(&d_ofdm, &d_frame, d_deinterleaved_soft);
        } else {
            for (int i = 0; i < d_frame.n_sym * 48; i++) {
                for (int k = 0; k < d_ofdm.n_bpsc; k++) {
                    d_rx_bits[i * d_ofdm.n_bpsc + k] = !!(d_rx_symbols[i] & (1 << k));
                }
            }
            deinterleave(d_rx_bits, d_deinterleaved_bits);
            decoded = d_decoder.decode(&d_ofdm, &d_frame, d_deinterleaved_bits);
        }
        descramble(decoded);
        print_output();

        // skip service field
        boost::crc_32_type result;
        result.process_bytes(out_bytes + 2, d_frame.psdu_size);
        const bool crc_ok = (result.checksum() == 558161692);

        // GR_KEEP_CORRUPTED (mirror of devourer's DEVOURER_RX_KEEP_CORRUPTED):
        // normally an FCS-failed frame is dropped here, hiding the
        // mostly-correct bytes a fused-FEC sub-block-integrity layer could
        // still salvage. With the env var set we publish the PSDU anyway,
        // tagged crc_ok=#f, so the downstream SBI decoder can keep the
        // surviving sub-blocks. Read once; default (unset) = unchanged.
        static const bool keep_corrupted =
            (std::getenv("GR_KEEP_CORRUPTED") != nullptr);
        if (!crc_ok && !keep_corrupted) {
            dout << "checksum wrong -- dropping" << std::endl;
            return;
        }

        mylog("encoding: {} - length: {} - symbols: {} - crc_ok: {}",
              d_ofdm.encoding,
              d_frame.psdu_size,
              d_frame.n_sym,
              crc_ok);

        // create PDU. crc_ok lets a consumer distinguish clean frames from
        // kept-corrupt ones without re-checking the FCS itself.
        d_meta = pmt::dict_add(d_meta, pmt::mp("crc_ok"), pmt::from_bool(crc_ok));
        pmt::pmt_t blob = pmt::make_blob(out_bytes + 2, d_frame.psdu_size - 4);
        d_meta =
            pmt::dict_add(d_meta, pmt::mp("dlt"), pmt::from_long(LINKTYPE_IEEE802_11));

        message_port_pub(pmt::mp("out"), pmt::cons(d_meta, blob));
    }

    // Deinterleave src -> dst (one byte per coded bit). Used for both the hard
    // bits and the GR_SOFT_VITERBI soft values (same permutation).
    void deinterleave(const uint8_t* src, uint8_t* dst)
    {

        int n_cbps = d_ofdm.n_cbps;
        int first[MAX_BITS_PER_SYM];
        int second[MAX_BITS_PER_SYM];
        int s = std::max(d_ofdm.n_bpsc / 2, 1);

        for (int j = 0; j < n_cbps; j++) {
            first[j] = s * (j / s) + ((j + int(floor(16.0 * j / n_cbps))) % s);
        }

        for (int i = 0; i < n_cbps; i++) {
            second[i] = 16 * i - (n_cbps - 1) * int(floor(16.0 * i / n_cbps));
        }

        for (int i = 0; i < d_frame.n_sym; i++) {
            for (int k = 0; k < n_cbps; k++) {
                dst[i * n_cbps + second[first[k]]] = src[i * n_cbps + k];
            }
        }
    }


    void descramble(uint8_t* decoded_bits)
    {

        int state = 0;
        std::memset(out_bytes, 0, d_frame.psdu_size + 2);

        for (int i = 0; i < 7; i++) {
            if (decoded_bits[i]) {
                state |= 1 << (6 - i);
            }
        }
        out_bytes[0] = state;

        int feedback;
        int bit;

        for (int i = 7; i < d_frame.psdu_size * 8 + 16; i++) {
            feedback = ((!!(state & 64))) ^ (!!(state & 8));
            bit = feedback ^ (decoded_bits[i] & 0x1);
            out_bytes[i / 8] |= bit << (i % 8);
            state = ((state << 1) & 0x7e) | feedback;
        }
    }

    void print_output()
    {

        dout << std::endl;
        dout << "psdu size" << d_frame.psdu_size << std::endl;
        for (int i = 2; i < d_frame.psdu_size + 2; i++) {
            dout << std::setfill('0') << std::setw(2) << std::hex
                 << ((unsigned int)out_bytes[i] & 0xFF) << std::dec << " ";
            if (i % 16 == 15) {
                dout << std::endl;
            }
        }
        dout << std::endl;
        for (int i = 2; i < d_frame.psdu_size + 2; i++) {
            if ((out_bytes[i] > 31) && (out_bytes[i] < 127)) {
                dout << ((char)out_bytes[i]);
            } else {
                dout << ".";
            }
        }
        dout << std::endl;
    }

private:
    bool d_debug;
    bool d_log;

    pmt::pmt_t d_meta;

    frame_param d_frame;
    ofdm_param d_ofdm;

    viterbi_decoder d_decoder;

    uint8_t d_rx_symbols[48 * MAX_SYM];
    uint8_t d_rx_bits[MAX_ENCODED_BITS];
    uint8_t d_deinterleaved_bits[MAX_ENCODED_BITS];
    // GR_SOFT_VITERBI legacy path: per-coded-bit soft values (raw per-symbol from
    // frame_equalizer's soft port, then packed + deinterleaved).
    uint8_t d_rx_soft_raw[LEGACY_SOFT_STRIDE * MAX_SYM];
    uint8_t d_rx_soft[MAX_ENCODED_BITS];
    uint8_t d_deinterleaved_soft[MAX_ENCODED_BITS];
    bool d_soft_in_present = false; // is the optional soft input connected?
    uint8_t out_bytes[MAX_PSDU_SIZE + 2]; // 2 for signal field

    int copied;
    bool d_frame_complete;
};

decode_mac::sptr decode_mac::make(bool log, bool debug)
{
    return gnuradio::get_initial_sptr(new decode_mac_impl(log, debug));
}
