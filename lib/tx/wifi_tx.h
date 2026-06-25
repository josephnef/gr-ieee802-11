/* -*- c++ -*- */
/*
 * Modern-format (802.11n HT / 802.11ac VHT) transmit core for the gr-ieee802-11
 * fork. Builds a spec-compliant, chip-receivable baseband frame from a PSDU.
 *
 * This is the DSP engine; the GNU Radio block (frame_builder) wraps it, and a
 * standalone tx_gen tool exercises it for self-loop + over-air validation.
 *
 * The algorithm mirrors sdr2wifi/tools_gen_wifi.py (validated against this repo's
 * RX) but fixes the chip-compat gaps that generator left for a lenient decoder:
 *   - HT-STF/VHT-STF use the real L-STF short sequence (not the L-LTF placeholder),
 *   - VHT-SIG-B CRC seeds the DATA SERVICE field,
 *   - HT-SIG/VHT-SIG-A carry QBPSK data with BPSK pilots,
 *   - L-SIG LENGTH is the spec duration value.
 */
#ifndef INCLUDED_IEEE802_11_WIFI_TX_H
#define INCLUDED_IEEE802_11_WIFI_TX_H

#include <complex>
#include <cstdint>
#include <vector>

namespace gr {
namespace ieee802_11 {
namespace tx {

using cf = std::complex<float>;

// Local format/FEC enums so the TX core has no GNU Radio dependency (it links into a
// standalone tool too). The frame_builder block maps the gr enums onto these.
enum TxFormat { TX_HT = 1, TX_VHT = 2 };
enum TxFec { TX_BCC = 0, TX_LDPC = 1 };

// Per-frame PHY parameters. Defaults = HT20 SISO MCS0 BCC.
struct tx_params {
    TxFormat format = TX_HT;       // TX_HT or TX_VHT
    int mcs = 0;                   // HT MCS 0..15 (>=8 => 2 streams), or VHT MCS 0..9
    int n_ss = 1;                  // VHT spatial streams (HT derives n_ss from mcs)
    int bw = 20;                   // 20 or 40 (MHz)
    bool sgi = false;              // short guard interval (signalled; LGI samples for now)
    TxFec fec = TX_BCC;            // TX_BCC or TX_LDPC
    int stbc = 0;                  // 0 (none) or 1 (1 SS -> 2 STS)
};

// Build the time-domain frame(s). Returns one complex vector per TX antenna
// (1 for SISO, 2 for STBC/2x2 MIMO). The PSDU must already include its FCS.
std::vector<std::vector<cf>> build_frame(const uint8_t* psdu, int psdu_len,
                                         const tx_params& p);

} // namespace tx
} // namespace ieee802_11
} // namespace gr

#endif /* INCLUDED_IEEE802_11_WIFI_TX_H */
