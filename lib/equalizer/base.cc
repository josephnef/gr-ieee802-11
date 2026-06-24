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

#include "base.h"
#include <cstring>
#include <iostream>

using namespace gr::ieee802_11::equalizer;

const gr_complex base::LONG[] = { 0,  0,  0,  0,  0,  0,  1,  1,  -1, -1, 1,  1,  -1,
                                  1,  -1, 1,  1,  1,  1,  1,  1,  -1, -1, 1,  1,  -1,
                                  1,  -1, 1,  1,  1,  1,  0,  1,  -1, -1, 1,  1,  -1,
                                  1,  -1, 1,  -1, -1, -1, -1, -1, 1,  1,  -1, -1, 1,
                                  -1, 1,  -1, 1,  1,  1,  1,  0,  0,  0,  0,  0 };

// HT40 L-LTF (128 FFT bins, DC at 64): the 20 MHz L-LTF replicated on both
// subchannels (lower bins 6..58, upper 70..122 rotated by j). Generated to match
// tools_gen_wifi.py's LONG40. Used for the HT40 channel estimate.
const gr_complex base::LONG40[128] = {
    {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {1,0}, {1,0},
    {-1,0}, {-1,0}, {1,0}, {1,0}, {-1,0}, {1,0}, {-1,0}, {1,0},
    {1,0}, {1,0}, {1,0}, {1,0}, {1,0}, {-1,0}, {-1,0}, {1,0},
    {1,0}, {-1,0}, {1,0}, {-1,0}, {1,0}, {1,0}, {1,0}, {1,0},
    {0,0}, {1,0}, {-1,0}, {-1,0}, {1,0}, {1,0}, {-1,0}, {1,0},
    {-1,0}, {1,0}, {-1,0}, {-1,0}, {-1,0}, {-1,0}, {-1,0}, {1,0},
    {1,0}, {-1,0}, {-1,0}, {1,0}, {-1,0}, {1,0}, {-1,0}, {1,0},
    {1,0}, {1,0}, {1,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0},
    {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,1}, {0,1},
    {0,-1}, {0,-1}, {0,1}, {0,1}, {0,-1}, {0,1}, {0,-1}, {0,1},
    {0,1}, {0,1}, {0,1}, {0,1}, {0,1}, {0,-1}, {0,-1}, {0,1},
    {0,1}, {0,-1}, {0,1}, {0,-1}, {0,1}, {0,1}, {0,1}, {0,1},
    {0,0}, {0,1}, {0,-1}, {0,-1}, {0,1}, {0,1}, {0,-1}, {0,1},
    {0,-1}, {0,1}, {0,-1}, {0,-1}, {0,-1}, {0,-1}, {0,-1}, {0,1},
    {0,1}, {0,-1}, {0,-1}, {0,1}, {0,-1}, {0,1}, {0,-1}, {0,1},
    {0,1}, {0,1}, {0,1}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0},
};

const gr_complex base::POLARITY[127] = {
    1,  1,  1,  1,  -1, -1, -1, 1,  -1, -1, -1, -1, 1,  1,  -1, 1,  -1, -1, 1, 1,  -1, 1,
    1,  -1, 1,  1,  1,  1,  1,  1,  -1, 1,  1,  1,  -1, 1,  1,  -1, -1, 1,  1, 1,  -1, 1,
    -1, -1, -1, 1,  -1, 1,  -1, -1, 1,  -1, -1, 1,  1,  1,  1,  1,  -1, -1, 1, 1,  -1, -1,
    1,  -1, 1,  -1, 1,  1,  -1, -1, -1, 1,  1,  -1, -1, -1, -1, 1,  -1, -1, 1, -1, 1,  1,
    1,  1,  -1, 1,  -1, 1,  -1, 1,  -1, -1, -1, -1, -1, 1,  -1, 1,  1,  -1, 1, -1, 1,  1,
    1,  -1, -1, 1,  -1, -1, -1, 1,  1,  1,  -1, -1, -1, -1, -1, -1, -1
};

std::vector<gr_complex> base::get_csi()
{
    std::vector<gr_complex> csi;
    csi.reserve(52);
    for (int i = 0; i < 64; i++) {
        if ((i == 32) || (i < 6) || (i > 58)) {
            continue;
        }
        csi.push_back(d_H[i]);
    }
    return csi;
}
