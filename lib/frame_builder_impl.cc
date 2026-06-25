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
    frame_builder_impl(Format format, int mcs, int bw, FecType fec, int n_tx, bool debug)
        : block("frame_builder",
                gr::io_signature::make(0, 0, 0),
                gr::io_signature::make(n_tx, n_tx, sizeof(gr_complex))),
          d_format(format), d_mcs(mcs), d_bw(bw), d_fec(fec),
          d_ntx(n_tx < 1 ? 1 : (n_tx > 2 ? 2 : n_tx)), d_debug(debug), d_offset(0)
    {
        message_port_register_in(pmt::mp("in"));
        d_ant.resize(d_ntx);
    }

    int general_work(int noutput, gr_vector_int& ninput_items,
                     gr_vector_const_void_star& input_items,
                     gr_vector_void_star& output_items) override
    {
        // Build the next frame when idle. d_ant[k] is antenna k's samples; for 2-output
        // STBC/MIMO that's the two TX antennas, for SISO antenna 1 (if requested) is 0.
        while (d_ant[0].empty()) {
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
                if (!ants.empty()) {
                    d_ant[0] = ants[0];
                    if (d_ntx == 2)
                        d_ant[1] = (ants.size() > 1) ? ants[1]
                                                     : std::vector<gr_complex>(
                                                           ants[0].size(), gr_complex(0, 0));
                }
            } catch (const std::exception& e) {
                if (d_debug)
                    std::fprintf(stderr, "frame_builder: drop PDU (%s)\n", e.what());
                continue; // unsupported params -> drop this PDU, keep running
            }
            if (d_ant[0].empty())
                continue;

            pmt::pmt_t srcid = pmt::string_to_symbol(alias());
            for (int ch = 0; ch < d_ntx; ch++) {
                add_item_tag(ch, nitems_written(ch), pmt::mp("packet_len"),
                             pmt::from_long((long)d_ant[ch].size()), srcid);
                add_item_tag(ch, nitems_written(ch), pmt::mp("psdu_len"),
                             pmt::from_long((long)psdu_len), srcid);
            }
            d_offset = 0;
        }

        int n = std::min<int>(noutput, (int)d_ant[0].size() - d_offset);
        for (int ch = 0; ch < d_ntx; ch++)
            std::memcpy(static_cast<gr_complex*>(output_items[ch]),
                        d_ant[ch].data() + d_offset, n * sizeof(gr_complex));
        d_offset += n;
        if (d_offset == (int)d_ant[0].size()) {
            for (auto& a : d_ant)
                a.clear();
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
    int d_ntx;
    bool d_debug;
    std::vector<std::vector<gr_complex>> d_ant;
    int d_offset;
    gr::thread::mutex d_mutex;
};

frame_builder::sptr frame_builder::make(Format format, int mcs, int bw, FecType fec,
                                        int n_tx, bool debug)
{
    return gnuradio::get_initial_sptr(
        new frame_builder_impl(format, mcs, bw, fec, n_tx, debug));
}
