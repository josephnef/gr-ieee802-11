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
#ifndef INCLUDED_IEEE802_11_FRAME_BUILDER_H
#define INCLUDED_IEEE802_11_FRAME_BUILDER_H

#include <gnuradio/block.h>
#include <ieee802_11/api.h>
#include <ieee802_11/mapper.h> // Format, FecType

namespace gr {
namespace ieee802_11 {

/*!
 * \brief Modern-format (HT/VHT) TX frame builder.
 *
 * Mirrors the legacy `mapper` convention: takes a PSDU as a PDU message on port
 * "in" and emits the finished TIME-DOMAIN frame as a tagged-stream burst of
 * gr_complex (packet_len tag), ready for a USRP/file sink. Wraps the chip-validated
 * TX core in lib/tx/ (build_frame): HT20/HT40/VHT20 SISO BCC and HT20 LDPC, with the
 * Realtek-compatible SIG/pilot recipe. The legacy a/g/p TX chain is untouched.
 */
class IEEE802_11_API frame_builder : virtual public block
{
public:
    typedef std::shared_ptr<frame_builder> sptr;

    /*!
     * \param format  FORMAT_HT or FORMAT_VHT
     * \param mcs     MCS index (HT 0-7 / VHT 0-7; per-stream)
     * \param bw      bandwidth in MHz (20 or 40; VHT 20 only so far)
     * \param fec     FEC_BCC or FEC_LDPC (LDPC: HT20 MCS0 so far)
     * \param debug   verbose logging
     */
    static sptr make(Format format = FORMAT_HT, int mcs = 0, int bw = 20,
                     FecType fec = FEC_BCC, bool debug = false);

    virtual void set_format(Format format) = 0;
    virtual void set_mcs(int mcs) = 0;
    virtual void set_bw(int bw) = 0;
    virtual void set_fec(FecType fec) = 0;
};

} // namespace ieee802_11
} // namespace gr

#endif /* INCLUDED_IEEE802_11_FRAME_BUILDER_H */
