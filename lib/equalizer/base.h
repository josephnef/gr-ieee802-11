/*
 * Copyright (C) 2015 Bastian Bloessl <bloessl@ccs-labs.org>
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

#ifndef INCLUDED_IEEE802_11_EQUALIZER_BASE_H
#define INCLUDED_IEEE802_11_EQUALIZER_BASE_H

#include <gnuradio/digital/constellation.h>
#include <gnuradio/gr_complex.h>

namespace gr {
namespace ieee802_11 {
namespace equalizer {

class base
{
public:
    virtual ~base(){};
    virtual void equalize(gr_complex* in,
                          int n,
                          gr_complex* symbols,
                          uint8_t* bits,
                          std::shared_ptr<gr::digital::constellation> mod) = 0;
    virtual double get_snr() = 0;

    static const gr_complex POLARITY[127];
    static const gr_complex HTLTF40[128]; // HT40 HT-LTF (all 114 occupied bins)

    std::vector<gr_complex> get_csi();

    // L-LTF channel estimate (FFT bins). Valid after the 2 LTF symbols; used by
    // the HT path to equalize HT-SIG/HT data without the legacy pilot tracking.
    const gr_complex* get_h() const { return d_H; }
    // FFT size: 64 (20 MHz) or 128 (HT40). Selects which L-LTF the estimator uses.
    void set_fft_len(int n) { d_fft_len = n; }

protected:
    static const gr_complex LONG[64];
    static const gr_complex LONG40[128];   // HT40 non-HT-duplicate L-LTF (128 bins)

    int d_fft_len = 64;
    gr_complex d_H[128];
};

} // namespace equalizer
} /* namespace ieee802_11 */
} /* namespace gr */

#endif /* INCLUDED_IEEE802_11_EQUALIZER_BASE_H */
