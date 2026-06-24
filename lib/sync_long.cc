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
#include "utils.h"
#include <gnuradio/fft/fft.h>
#include <gnuradio/filter/fir_filter.h>
#include <gnuradio/io_signature.h>
#include <ieee802_11/sync_long.h>
#include <volk/volk.h>

#include <list>
#include <tuple>

using namespace gr::ieee802_11;
using namespace std;


bool compare_abs(const std::pair<gr_complex, int>& first,
                 const std::pair<gr_complex, int>& second)
{
    return abs(get<0>(first)) > abs(get<0>(second));
}

class sync_long_impl : public sync_long
{

public:
    sync_long_impl(unsigned int sync_length, bool log, bool debug, int fft_len)
        : block("sync_long",
                gr::io_signature::make2(2, 2, sizeof(gr_complex), sizeof(gr_complex)),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
          d_fir(gr::filter::kernel::fir_filter_ccc(fft_len == 128 ? LONG128 : LONG)),
          d_log(log),
          d_debug(debug),
          d_offset(0),
          d_state(SYNC),
          SYNC_LENGTH(sync_length),
          d_fft_len(fft_len)
    {

        set_tag_propagation_policy(block::TPP_DONT);
        d_correlation = (gr_complex*)volk_malloc(sizeof(gr_complex) * 8192, volk_get_alignment());
    }

    ~sync_long_impl() {
        volk_free(d_correlation);
    }

    int general_work(int noutput,
                     gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items)
    {

        const gr_complex* in = (const gr_complex*)input_items[0];
        const gr_complex* in_delayed = (const gr_complex*)input_items[1];
        gr_complex* out = (gr_complex*)output_items[0];

        dout << "LONG ninput[0] " << ninput_items[0] << "   ninput[1] " << ninput_items[1]
             << "  noutput " << noutput << "   state " << d_state << std::endl;

        int ninput = std::min(std::min(ninput_items[0], ninput_items[1]), 8192);

        const uint64_t nread = nitems_read(0);
        get_tags_in_range(d_tags, 0, nread, nread + ninput);
        if (d_tags.size()) {
            std::sort(d_tags.begin(), d_tags.end(), gr::tag_t::offset_compare);

            const uint64_t offset = d_tags.front().offset;

            if (offset > nread) {
                ninput = offset - nread;
            } else {
                if (d_offset && (d_state == SYNC)) {
                    throw std::runtime_error("wtf");
                }
                if (d_state == COPY) {
                    d_state = RESET;
                }
                d_freq_offset_short = pmt::to_double(d_tags.front().value);
            }
        }


        int i = 0;
        int o = 0;

        switch (d_state) {

        case SYNC:
            d_fir.filterN(d_correlation,
                          in,
                          std::min(SYNC_LENGTH, std::max(ninput - (d_fft_len - 1), 0)));

            while (i + (d_fft_len - 1) < ninput) {

                d_cor.push_back(pair<gr_complex, int>(d_correlation[i], d_offset));

                i++;
                d_offset++;

                if (d_offset == SYNC_LENGTH) {
                    search_frame_start();
                    mylog("LONG: frame start at {}",d_frame_start);
                    d_offset = 0;
                    d_count = 0;
                    d_state = COPY;

                    break;
                }
            }

            break;

        case COPY:
            while (i < ninput && o < noutput) {

                int rel = d_offset - d_frame_start;

                if (!rel) {
                    add_item_tag(0,
                                 nitems_written(0),
                                 pmt::string_to_symbol("wifi_start"),
                                 pmt::from_double(d_freq_offset_short - d_freq_offset),
                                 pmt::string_to_symbol(name()));
                }

                if (rel >= 0 && (rel < 2 * d_fft_len ||
                                 ((rel - 2 * d_fft_len) % (d_fft_len + d_fft_len / 4)) >
                                     (d_fft_len / 4 - 1))) {
                    out[o] = in_delayed[i] * exp(gr_complex(0, d_offset * d_freq_offset));
                    o++;
                }

                i++;
                d_offset++;
            }

            break;

        case RESET: {
            while (o < noutput) {
                if (((d_count + o) % d_fft_len) == 0) {
                    d_offset = 0;
                    d_state = SYNC;
                    break;
                } else {
                    out[o] = 0;
                    o++;
                }
            }

            break;
        }
        }

        dout << "produced : " << o << " consumed: " << i << std::endl;

        d_count += o;
        consume(0, i);
        consume(1, i);
        return o;
    }

    void forecast(int noutput_items, gr_vector_int& ninput_items_required)
    {

        // in sync state we need at least a symbol to correlate
        // with the pattern
        if (d_state == SYNC) {
            ninput_items_required[0] = d_fft_len;
            ninput_items_required[1] = d_fft_len;

        } else {
            ninput_items_required[0] = noutput_items;
            ninput_items_required[1] = noutput_items;
        }
    }

    void search_frame_start()
    {

        // sort list (highest correlation first)
        assert(d_cor.size() == SYNC_LENGTH);
        d_cor.sort(compare_abs);

        // copy list in vector for nicer access
        vector<pair<gr_complex, int>> vec(d_cor.begin(), d_cor.end());
        d_cor.clear();

        // in case we don't find anything use SYNC_LENGTH
        d_frame_start = SYNC_LENGTH;

        for (int i = 0; i < 3; i++) {
            for (int k = i + 1; k < 4; k++) {
                gr_complex first;
                gr_complex second;
                if (get<1>(vec[i]) > get<1>(vec[k])) {
                    first = get<0>(vec[k]);
                    second = get<0>(vec[i]);
                } else {
                    first = get<0>(vec[i]);
                    second = get<0>(vec[k]);
                }
                int diff = abs(get<1>(vec[i]) - get<1>(vec[k]));
                if (diff == d_fft_len) {
                    d_frame_start = min(get<1>(vec[i]), get<1>(vec[k]));
                    d_freq_offset = arg(first * conj(second)) / d_fft_len;
                    // nice match found, return immediately
                    return;

                } else if (diff == d_fft_len - 1) {
                    d_frame_start = min(get<1>(vec[i]), get<1>(vec[k]));
                    d_freq_offset = arg(first * conj(second)) / (d_fft_len - 1);
                } else if (diff == d_fft_len + 1) {
                    d_frame_start = min(get<1>(vec[i]), get<1>(vec[k]));
                    d_freq_offset = arg(first * conj(second)) / (d_fft_len + 1);
                }
            }
        }
    }

private:
    enum { SYNC, COPY, RESET } d_state;
    int d_count;
    int d_offset;
    int d_frame_start;
    float d_freq_offset;
    double d_freq_offset_short;

    gr_complex* d_correlation;
    list<pair<gr_complex, int>> d_cor;
    std::vector<gr::tag_t> d_tags;
    gr::filter::kernel::fir_filter_ccc d_fir;

    const bool d_log;
    const bool d_debug;
    const int SYNC_LENGTH;
    const int d_fft_len; // 64 (=20 MHz) or 128 (=HT40); selects matched filter & geometry

    static const std::vector<gr_complex> LONG;
    static const std::vector<gr_complex> LONG128; // HT40 non-HT-duplicate L-LTF (128 taps)
};

sync_long::sptr sync_long::make(unsigned int sync_length, bool log, bool debug, int fft_len)
{
    return gnuradio::get_initial_sptr(
        new sync_long_impl(sync_length, log, debug, fft_len));
}

const std::vector<gr_complex> sync_long_impl::LONG = {
    gr_complex(-0.0455, -1.0679), gr_complex(0.3528, -0.9865),
    gr_complex(0.8594, 0.7348),   gr_complex(0.1874, 0.2475),
    gr_complex(0.5309, -0.7784),  gr_complex(-1.0218, -0.4897),
    gr_complex(-0.3401, -0.9423), gr_complex(0.8657, -0.2298),
    gr_complex(0.4734, 0.0362),   gr_complex(0.0088, -1.0207),
    gr_complex(-1.2142, -0.4205), gr_complex(0.2172, -0.5195),
    gr_complex(0.5207, -0.1326),  gr_complex(-0.1995, 1.4259),
    gr_complex(1.0583, -0.0363),  gr_complex(0.5547, -0.5547),
    gr_complex(0.3277, 0.8728),   gr_complex(-0.5077, 0.3488),
    gr_complex(-1.1650, 0.5789),  gr_complex(0.7297, 0.8197),
    gr_complex(0.6173, 0.1253),   gr_complex(-0.5353, 0.7214),
    gr_complex(-0.5011, -0.1935), gr_complex(-0.3110, -1.3392),
    gr_complex(-1.0818, -0.1470), gr_complex(-1.1300, -0.1820),
    gr_complex(0.6663, -0.6571),  gr_complex(-0.0249, 0.4773),
    gr_complex(-0.8155, 1.0218),  gr_complex(0.8140, 0.9396),
    gr_complex(0.1090, 0.8662),   gr_complex(-1.3868, -0.0000),
    gr_complex(0.1090, -0.8662),  gr_complex(0.8140, -0.9396),
    gr_complex(-0.8155, -1.0218), gr_complex(-0.0249, -0.4773),
    gr_complex(0.6663, 0.6571),   gr_complex(-1.1300, 0.1820),
    gr_complex(-1.0818, 0.1470),  gr_complex(-0.3110, 1.3392),
    gr_complex(-0.5011, 0.1935),  gr_complex(-0.5353, -0.7214),
    gr_complex(0.6173, -0.1253),  gr_complex(0.7297, -0.8197),
    gr_complex(-1.1650, -0.5789), gr_complex(-0.5077, -0.3488),
    gr_complex(0.3277, -0.8728),  gr_complex(0.5547, 0.5547),
    gr_complex(1.0583, 0.0363),   gr_complex(-0.1995, -1.4259),
    gr_complex(0.5207, 0.1326),   gr_complex(0.2172, 0.5195),
    gr_complex(-1.2142, 0.4205),  gr_complex(0.0088, 1.0207),
    gr_complex(0.4734, -0.0362),  gr_complex(0.8657, 0.2298),
    gr_complex(-0.3401, 0.9423),  gr_complex(-1.0218, 0.4897),
    gr_complex(0.5309, 0.7784),   gr_complex(0.1874, -0.2475),
    gr_complex(0.8594, -0.7348),  gr_complex(0.3528, 0.9865),
    gr_complex(-0.0455, 1.0679),  gr_complex(1.3868, -0.0000),
};

// HT40 non-HT-duplicate L-LTF matched filter (128 taps): conj(ifft(ifftshift(LONG40)))
// reversed, scaled to ~unit mean magnitude. The two identical L-LTF40 symbols are 128
// samples apart, so search_frame_start's diff==fft_len pair test locates the frame.
// Generated by tools_gen_wifi.py's LONG40 (must stay in sync with equalizer base.cc).
const std::vector<gr_complex> sync_long_impl::LONG128 = {
    gr_complex(0.1796, -0.7441),  gr_complex(0.5906, 0.5424),
    gr_complex(0.8547, 0.5615),   gr_complex(-0.3362, -0.7105),
    gr_complex(0.4657, -0.5642),  gr_complex(-0.8457, 0.0661),
    gr_complex(-0.6344, -0.2760), gr_complex(0.2307, 0.0319),
    gr_complex(-0.0009, -0.5236), gr_complex(0.1313, 0.6946),
    gr_complex(0.4104, 0.2438),   gr_complex(-0.8018, 0.2823),
    gr_complex(-0.9659, 0.2358),  gr_complex(0.6803, 0.3195),
    gr_complex(0.1189, 0.7220),   gr_complex(0.3373, -0.5811),
    gr_complex(0.4593, -0.2389),  gr_complex(-0.2703, 0.2319),
    gr_complex(0.1048, 0.5188),   gr_complex(-0.5368, -0.5461),
    gr_complex(-0.8527, -0.0677), gr_complex(0.8672, -0.4210),
    gr_complex(0.5619, -0.2812),  gr_complex(-0.1603, -0.3908),
    gr_complex(0.0930, -0.7858),  gr_complex(-0.2059, 0.3466),
    gr_complex(-0.3772, -0.5152), gr_complex(0.6505, 0.8622),
    gr_complex(0.7777, 0.3055),   gr_complex(-0.5421, 0.5807),
    gr_complex(-0.1322, 0.9487),  gr_complex(0.0000, -0.5885),
    gr_complex(0.3014, -0.0055),  gr_complex(-0.6368, -0.2892),
    gr_complex(-0.4980, -0.3262), gr_complex(-0.0843, 0.4544),
    gr_complex(-0.5157, 0.7617),  gr_complex(0.3109, -0.9251),
    gr_complex(-0.3447, -0.6402), gr_complex(0.8219, 0.0477),
    gr_complex(0.7609, -0.3670),  gr_complex(-0.3940, 0.2610),
    gr_complex(-0.1470, -0.2379), gr_complex(0.0988, 0.6666),
    gr_complex(-0.0196, 0.6406),  gr_complex(0.3684, -0.1631),
    gr_complex(0.7490, 0.3858),   gr_complex(-0.8754, -0.5454),
    gr_complex(-0.7558, -0.1494), gr_complex(0.6518, -0.4959),
    gr_complex(0.6931, -0.8242),  gr_complex(-0.6960, 0.5029),
    gr_complex(-0.4357, -0.2215), gr_complex(-0.0049, 0.7020),
    gr_complex(-0.2912, 0.4827),  gr_complex(0.2400, 0.2664),
    gr_complex(0.0570, 0.9154),   gr_complex(-0.1094, -0.9746),
    gr_complex(-0.5001, -0.5225), gr_complex(0.9303, 0.0666),
    gr_complex(0.9835, 0.0335),   gr_complex(-0.5173, -0.4017),
    gr_complex(0.2070, -0.7718),  gr_complex(-0.7356, 0.7356),
    gr_complex(-0.7718, 0.2070),  gr_complex(0.4017, 0.5173),
    gr_complex(0.0335, 0.9835),   gr_complex(-0.0666, -0.9303),
    gr_complex(-0.5225, -0.5001), gr_complex(0.9746, 0.1094),
    gr_complex(0.9154, 0.0570),   gr_complex(-0.2664, -0.2400),
    gr_complex(0.4827, -0.2912),  gr_complex(-0.7020, 0.0049),
    gr_complex(-0.2215, -0.4357), gr_complex(-0.5029, 0.6960),
    gr_complex(-0.8242, 0.6931),  gr_complex(0.4959, -0.6518),
    gr_complex(-0.1494, -0.7558), gr_complex(0.5454, 0.8754),
    gr_complex(0.3858, 0.7490),   gr_complex(0.1631, -0.3684),
    gr_complex(0.6406, -0.0196),  gr_complex(-0.6666, -0.0988),
    gr_complex(-0.2379, -0.1470), gr_complex(-0.2610, 0.3940),
    gr_complex(-0.3670, 0.7609),  gr_complex(-0.0477, -0.8219),
    gr_complex(-0.6402, -0.3447), gr_complex(0.9251, -0.3109),
    gr_complex(0.7617, -0.5157),  gr_complex(-0.4544, 0.0843),
    gr_complex(-0.3262, -0.4980), gr_complex(0.2892, 0.6368),
    gr_complex(-0.0055, 0.3014),  gr_complex(0.5885, 0.0000),
    gr_complex(0.9487, -0.1322),  gr_complex(-0.5807, 0.5421),
    gr_complex(0.3055, 0.7777),   gr_complex(-0.8622, -0.6505),
    gr_complex(-0.5152, -0.3772), gr_complex(-0.3466, 0.2059),
    gr_complex(-0.7858, 0.0930),  gr_complex(0.3908, 0.1603),
    gr_complex(-0.2812, 0.5619),  gr_complex(0.4210, -0.8672),
    gr_complex(-0.0677, -0.8527), gr_complex(0.5461, 0.5368),
    gr_complex(0.5188, 0.1048),   gr_complex(-0.2319, 0.2703),
    gr_complex(-0.2389, 0.4593),  gr_complex(0.5811, -0.3373),
    gr_complex(0.7220, 0.1189),   gr_complex(-0.3195, -0.6803),
    gr_complex(0.2358, -0.9659),  gr_complex(-0.2823, 0.8018),
    gr_complex(0.2438, 0.4104),   gr_complex(-0.6946, -0.1313),
    gr_complex(-0.5236, -0.0009), gr_complex(-0.0319, -0.2307),
    gr_complex(-0.2760, -0.6344), gr_complex(-0.0661, 0.8457),
    gr_complex(-0.5642, 0.4657),  gr_complex(0.7105, 0.3362),
    gr_complex(0.5615, 0.8547),   gr_complex(-0.5424, -0.5906),
    gr_complex(-0.7441, 0.1796),  gr_complex(0.7356, -0.7356),
};
