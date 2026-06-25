/* Standalone dev tool: build a modern-format frame and write cf32 for self-loop
 * validation through this repo's RX (sdr2wifi/tools_replay_iq.py) and over-air.
 *   tx_gen <out.cf32> [format ht|vht] [mcs] [bw 20|40]
 */
#include "wifi_tx.h"
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace tx = gr::ieee802_11::tx;
using gr::ieee802_11::tx::cf;

// zlib-compatible CRC-32 (reflected, init/xor 0xFFFFFFFF) == 802.11 FCS the RX checks.
static uint32_t crc32(const uint8_t* d, int n)
{
    uint32_t c = 0xFFFFFFFFu;
    for (int i = 0; i < n; i++) {
        c ^= d[i];
        for (int k = 0; k < 8; k++)
            c = (c & 1) ? ((c >> 1) ^ 0xEDB88320u) : (c >> 1); // reflected CRC-32 poly
    }
    return ~c;
}

// minimal canonical-SA MPDU + FCS (matches sdr2wifi make_psdu).
static std::vector<uint8_t> make_psdu(int payload_len)
{
    std::vector<uint8_t> m = { 0x08, 0x01, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
                               0x57, 0x42, 0x75, 0x05, 0xd6, 0x00, 0x11, 0x22, 0x33, 0x44,
                               0x55, 0x66, 0x00, 0x00 };
    m.resize(payload_len, 0);
    uint32_t fcs = crc32(m.data(), (int)m.size());
    for (int i = 0; i < 4; i++)
        m.push_back((fcs >> (8 * i)) & 0xff); // little-endian
    return m;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <out.cf32> [ht|vht] [mcs] [bw]\n", argv[0]);
        return 2;
    }
    const char* out = argv[1];
    std::string fmt = argc > 2 ? argv[2] : "ht";
    int mcs = argc > 3 ? atoi(argv[3]) : 0;
    int bw = argc > 4 ? atoi(argv[4]) : 20;

    tx::tx_params p;
    // "ldpc" selects HT20 MCS0 LDPC; "stbc" selects HT20 1->2 STBC; "vht"/"vht2" = VHT
    // SU NSS=1 / NSS=2; else BCC HT.
    p.format = (fmt == "vht" || fmt == "vht2") ? tx::TX_VHT : tx::TX_HT;
    p.fec = (fmt == "ldpc") ? tx::TX_LDPC : tx::TX_BCC;
    p.stbc = (fmt == "stbc") ? 1 : 0;
    p.n_ss = (fmt == "vht2") ? 2 : 1;
    // "mimo" = 2x2 HT MIMO; the per-stream MCS arg maps to HT MCS 8+mcs.
    p.mcs = (fmt == "mimo") ? 8 + mcs : mcs;
    p.bw = bw;

    // TX_LEN overrides the MPDU payload length (>24 exercises multi-codeword LDPC).
    const char* lenenv = std::getenv("TX_LEN");
    auto psdu = make_psdu(lenenv ? atoi(lenenv) : 24);
    auto built = tx::build_frame(psdu.data(), (int)psdu.size(), p);
    // STBC is 2 antennas. STBC_2ANT=1 keeps them separate (-> <out> = ant0, <out>.ant1
    // = ant1) for the 2-channel B210 harness; otherwise the headless self-loop uses the
    // single-RX-antenna view (their sum, h0=h1=1). Non-STBC -> antenna 0 only.
    const bool two_ant = std::getenv("STBC_2ANT") != nullptr;
    std::vector<std::vector<tx::cf>> ants;
    if (built.size() == 2 && two_ant) {
        ants = built; // ant0, ant1 separate (STBC/MIMO 2-channel harness)
    } else if (p.stbc && built.size() == 2) {
        std::vector<tx::cf> s(built[0].size()); // STBC self-loop: single-RX-antenna sum
        for (size_t i = 0; i < s.size(); i++)
            s[i] = built[0][i] + built[1][i];
        ants.push_back(s);
    } else {
        ants.push_back(built[0]);
    }

    // raw single-frame dump of antenna 0 (unscaled, no gap) for bit-exact comparison.
    { std::string one = std::string(out) + ".one"; FILE* g = fopen(one.c_str(), "wb");
      for (auto& s : ants[0]) { float v[2] = { s.real(), s.imag() }; fwrite(v, 4, 2, g); }
      fclose(g); }

    // common normalization to ~0.5 peak across ALL antennas (preserves the STBC
    // amplitude relationship between the two streams).
    float peak = 1e-9f;
    for (auto& a : ants)
        for (auto& s : a)
            peak = std::max(peak, std::max(std::abs(s.real()), std::abs(s.imag())));
    float scale = 0.5f / peak;
    std::vector<float> gap(4000 * 2, 0.f);

    if (ants.size() == 2) {
        // Two-antenna (STBC/MIMO): write ONE interleaved file ant0[t],ant1[t],... so a
        // single GNU Radio file_source + deinterleave keeps the two B210 TX channels
        // sample-locked (two independent file_sources drift, overlapping ant1's data
        // onto ant0's preamble). rf_tx_air2 --interleaved consumes this.
        FILE* f = fopen(out, "wb");
        if (!f) { perror("fopen"); return 1; }
        std::vector<float> gap2(4000 * 4, 0.f); // interleaved gap (both channels zero)
        fwrite(gap2.data(), sizeof(float), gap2.size(), f);
        for (int rep = 0; rep < 20; rep++) {
            for (size_t i = 0; i < ants[0].size(); i++) {
                float v[4] = { ants[0][i].real() * scale, ants[0][i].imag() * scale,
                               ants[1][i].real() * scale, ants[1][i].imag() * scale };
                fwrite(v, sizeof(float), 4, f);
            }
            fwrite(gap2.data(), sizeof(float), gap2.size(), f);
        }
        fclose(f);
    } else {
        FILE* f = fopen(out, "wb");
        if (!f) { perror("fopen"); return 1; }
        fwrite(gap.data(), sizeof(float), gap.size(), f);
        for (int rep = 0; rep < 20; rep++) {
            for (auto& s : ants[0]) {
                float v[2] = { s.real() * scale, s.imag() * scale };
                fwrite(v, sizeof(float), 2, f);
            }
            fwrite(gap.data(), sizeof(float), gap.size(), f);
        }
        fclose(f);
    }
    fprintf(stderr, "[tx_gen] wrote %s: %s mcs=%d bw=%d, %zu antenna(s)%s, frame=%zu samples x20\n",
            out, fmt.c_str(), mcs, bw, ants.size(),
            ants.size() == 2 ? " interleaved" : "", ants[0].size());
    return 0;
}
