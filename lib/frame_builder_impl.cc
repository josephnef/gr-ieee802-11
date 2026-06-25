/*
 * Copyright (C) 2026 devourer / gr-ieee802-11 fork
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
 */
#include "tx/wifi_tx.h"
#include <gnuradio/io_signature.h>
#include <ieee802_11/frame_builder.h>
#include <cstdio>
#include <vector>

using namespace gr::ieee802_11;

class frame_builder_impl : public frame_builder
{
public:
    frame_builder_impl(Format format, int mcs, int bw, FecType fec, bool debug)
        : block("frame_builder",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(1, 1, sizeof(gr_complex))),
          d_format(format), d_mcs(mcs), d_bw(bw), d_fec(fec), d_debug(debug),
          d_offset(0)
    {
        message_port_register_in(pmt::mp("in"));
    }

    int general_work(int noutput, gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override
    {
        gr_complex* out = static_cast<gr_complex*>(output_items[0]);

        // Build the next frame when idle.
        while (d_samples.empty()) {
            pmt::pmt_t msg(delete_head_nowait(pmt::intern("in")));
            if (!msg.get())
                return 0;
            if (!pmt::is_pair(msg))
                continue;

            int psdu_len = pmt::blob_length(pmt::cdr(msg));
            const uint8_t* psdu =
                static_cast<const uint8_t*>(pmt::blob_data(pmt::cdr(msg)));

            tx::tx_params p;
            p.format = (d_format == FORMAT_VHT) ? tx::TX_VHT : tx::TX_HT;
            p.fec = (d_fec == FEC_LDPC) ? tx::TX_LDPC : tx::TX_BCC;
            p.mcs = d_mcs;
            p.bw = d_bw;
            try {
                gr::thread::scoped_lock lock(d_mutex);
                auto ants = tx::build_frame(psdu, psdu_len, p);
                if (!ants.empty())
                    d_samples = ants[0]; // SISO: single TX stream (STBC/MIMO later)
            } catch (const std::exception& e) {
                if (d_debug)
                    std::fprintf(stderr, "frame_builder: drop PDU (%s)\n", e.what());
                continue; // unsupported params -> drop this PDU, keep running
            }
            if (d_samples.empty())
                continue;

            pmt::pmt_t srcid = pmt::string_to_symbol(alias());
            add_item_tag(0, nitems_written(0), pmt::mp("packet_len"),
                         pmt::from_long((long)d_samples.size()), srcid);
            add_item_tag(0, nitems_written(0), pmt::mp("psdu_len"),
                         pmt::from_long((long)psdu_len), srcid);
            d_offset = 0;
        }

        int n = std::min<int>(noutput, (int)d_samples.size() - d_offset);
        std::memcpy(out, d_samples.data() + d_offset, n * sizeof(gr_complex));
        d_offset += n;
        if (d_offset == (int)d_samples.size()) {
            d_samples.clear();
            d_offset = 0;
        }
        return n;
    }

    void set_format(Format f) override { lock_set([&] { d_format = f; }); }
    void set_mcs(int m) override { lock_set([&] { d_mcs = m; }); }
    void set_bw(int b) override { lock_set([&] { d_bw = b; }); }
    void set_fec(FecType f) override { lock_set([&] { d_fec = f; }); }

private:
    template <typename F> void lock_set(F&& f)
    {
        gr::thread::scoped_lock lock(d_mutex);
        f();
    }

    Format d_format;
    int d_mcs;
    int d_bw;
    FecType d_fec;
    bool d_debug;
    std::vector<gr_complex> d_samples;
    int d_offset;
    gr::thread::mutex d_mutex;
};

frame_builder::sptr frame_builder::make(Format format, int mcs, int bw, FecType fec,
                                        bool debug)
{
    return gnuradio::get_initial_sptr(
        new frame_builder_impl(format, mcs, bw, fec, debug));
}
