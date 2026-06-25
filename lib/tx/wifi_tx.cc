/* -*- c++ -*- */
/*
 * Modern-format TX core. See wifi_tx.h. Phase 1: HT20 SISO BCC (MCS 0-7).
 * Ported from sdr2wifi/tools_gen_wifi.py (validated against this repo's RX),
 * with the chip-compat fixes baked in (spec HT-STF, BPSK SIG pilots, spec L-SIG).
 */
#include "wifi_tx.h"
#include "../ldpc_encoder.h"
#include <cmath>
#include <cstdlib>
#include <stdexcept>

namespace gr {
namespace ieee802_11 {
namespace tx {

namespace {

constexpr double PI = 3.14159265358979323846;

// ---- frequency-domain tables (DC at index 32; must match lib/equalizer/base.cc) ----
const int LONG[64] = {
    0, 0, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1,
    -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 0, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1,
    -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 0, 0, 0, 0, 0
};

const float POLARITY[127] = {
    1, 1, 1, 1, -1, -1, -1, 1, -1, -1, -1, -1, 1, 1, -1, 1, -1, -1, 1, 1, -1, 1, 1,
    -1, 1, 1, 1, 1, 1, 1, -1, 1, 1, 1, -1, 1, 1, -1, -1, 1, 1, 1, -1, 1, -1, -1, -1,
    1, -1, 1, -1, -1, 1, -1, -1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1,
    1, 1, -1, -1, -1, 1, 1, -1, -1, -1, -1, 1, -1, -1, 1, -1, 1, 1, 1, 1, -1, 1, -1,
    1, -1, 1, -1, -1, -1, -1, -1, 1, -1, 1, 1, -1, 1, -1, 1, 1, 1, -1, -1, 1, -1, -1,
    -1, 1, 1, 1, -1, -1, -1, -1, -1, -1, -1
};

// data / pilot subcarrier maps (DC at 32)
const int DATA_SC_LEGACY[48] = {
    6, 7, 8, 9, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 26, 27, 28, 29,
    30, 31, 33, 34, 35, 36, 37, 38, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52,
    54, 55, 56, 57, 58
};
const int DATA_SC_HT[52] = {
    4, 5, 6, 7, 8, 9, 10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 26, 27, 28,
    29, 30, 31, 33, 34, 35, 36, 37, 38, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52,
    54, 55, 56, 57, 58, 59, 60
};
const int PILOT_SC[4] = { 11, 25, 39, 53 };
const float PILOT_VAL[4] = { 1, 1, 1, -1 };

// HT MCS (per stream, 20 MHz): {n_bpsc, rate_num, rate_den, n_cbps, n_dbps}
struct mcs_row { int n_bpsc, rnum, rden, n_cbps, n_dbps; };
const mcs_row HT_MCS20[8] = {
    { 1, 1, 2, 52, 26 }, { 2, 1, 2, 104, 52 }, { 2, 3, 4, 104, 78 }, { 4, 1, 2, 208, 104 },
    { 4, 3, 4, 208, 156 }, { 6, 2, 3, 312, 208 }, { 6, 3, 4, 312, 234 }, { 6, 5, 6, 312, 260 }
};
// VHT per-stream MCS (20 MHz, 52 data SC). MCS0-7 == HT20; MCS8 adds 256-QAM (r3/4).
// VHT20 NSS=1 MCS9 (256-QAM r5/6) has a non-integer n_dbps (416*5/6) and is excluded by
// the spec -> it is only valid at >=40 MHz (see build_vht40).
const mcs_row VHT_MCS20[9] = {
    { 1, 1, 2, 52, 26 },   { 2, 1, 2, 104, 52 },  { 2, 3, 4, 104, 78 },
    { 4, 1, 2, 208, 104 }, { 4, 3, 4, 208, 156 }, { 6, 2, 3, 312, 208 },
    { 6, 3, 4, 312, 234 }, { 6, 5, 6, 312, 260 }, { 8, 3, 4, 416, 312 }
};

// ---- small DSP helpers ----
typedef std::vector<cf> vcf;

// inverse FFT of a 64/128-bin frequency vector given DC at index N/2 (fft-shifted).
// Mirrors np.fft.ifft(np.fft.ifftshift(freq)). Naive O(N^2) -- N is 64/128 and we
// build only a few dozen symbols per frame, so speed is irrelevant.
vcf ifft_sym(const vcf& freq)
{
    int N = (int)freq.size();
    vcf shifted(N), out(N);
    for (int k = 0; k < N; k++)
        shifted[k] = freq[(k + N / 2) % N]; // ifftshift
    for (int n = 0; n < N; n++) {
        cf acc(0, 0);
        for (int k = 0; k < N; k++) {
            double ang = 2.0 * PI * k * n / N;
            acc += shifted[k] * cf((float)std::cos(ang), (float)std::sin(ang));
        }
        out[n] = acc / (float)N;
    }
    return out;
}

// time-domain OFDM symbol = CP (last `cp` samples) + the N IFFT samples.
vcf ofdm_symbol(const vcf& freq, int cp = -1)
{
    int N = (int)freq.size();
    if (cp < 0)
        cp = N / 4;
    vcf t = ifft_sym(freq);
    vcf out;
    out.reserve(N + cp);
    for (int i = N - cp; i < N; i++)
        out.push_back(t[i]);
    for (int i = 0; i < N; i++)
        out.push_back(t[i]);
    return out;
}

// Cyclic-shift diversity (802.11-2016 19.3.11.9): a tcs-sample cyclic time shift applied
// as a per-subcarrier phase ramp exp(-j2pi (i-N/2) tcs / N) in the frequency domain
// (centered subcarrier index i-N/2). Used to decorrelate TX antenna 1's (STS1) legacy
// preamble + HT fields from antenna 0 so a 2-stream transmission does not beamform a null;
// the receiver's per-STS channel estimate absorbs the shift. STS0 uses tcs=0 (identity).
vcf apply_csd(vcf freq, int tcs)
{
    if (tcs == 0)
        return freq;
    int N = (int)freq.size();
    for (int i = 0; i < N; i++) {
        double ph = -2.0 * M_PI * (i - N / 2.0) * tcs / N;
        freq[i] *= cf((float)std::cos(ph), (float)std::sin(ph));
    }
    return freq;
}

vcf lstf_freq()
{
    vcf f(64, cf(0, 0));
    const float s = (float)std::sqrt(13.0 / 6.0);
    auto set = [&](int k, float re, float im) { f[32 + k] = cf(re, im) * s; };
    set(-24, 1, 1);  set(-20, -1, -1); set(-16, 1, 1); set(-12, -1, -1);
    set(-8, -1, -1); set(-4, 1, 1);    set(4, -1, -1); set(8, -1, -1);
    set(12, 1, 1);   set(16, 1, 1);    set(20, 1, 1);  set(24, 1, 1);
    return f;
}

typedef std::vector<uint8_t> vb;

vb conv_encode(const vb& bits)
{
    vb out(bits.size() * 2);
    int state = 0;
    for (size_t i = 0; i < bits.size(); i++) {
        state = ((state << 1) & 0x7e) | (bits[i] & 1);
        out[2 * i] = (uint8_t)(__builtin_popcount(state & 0x6d) & 1);     // 0155
        out[2 * i + 1] = (uint8_t)(__builtin_popcount(state & 0x4f) & 1); // 0117
    }
    return out;
}

vb scramble(const vb& bits, int init = 0x5d)
{
    vb out(bits.size());
    int state = init;
    for (size_t i = 0; i < bits.size(); i++) {
        int fb = ((state & 64) ? 1 : 0) ^ ((state & 8) ? 1 : 0);
        out[i] = (uint8_t)(fb ^ (bits[i] & 1));
        state = ((state << 1) & 0x7e) | fb;
    }
    return out;
}

vb puncture(const vb& coded, int num, int den)
{
    if (num == 1 && den == 2)
        return coded;
    vb out;
    out.reserve(coded.size());
    for (size_t i = 0; i < coded.size(); i++) {
        int m6 = i % 6, m4 = i % 4, m10 = i % 10;
        bool keep = true;
        if (num == 2 && den == 3)
            keep = (m4 != 3);
        else if (num == 3 && den == 4)
            keep = (m6 != 3 && m6 != 4);
        else if (num == 5 && den == 6)
            keep = (m10 != 3 && m10 != 4 && m10 != 7 && m10 != 8);
        if (keep)
            out.push_back(coded[i]);
    }
    return out;
}

// legacy forward interleave (N_COL=16), one OFDM symbol (n_cbps bits) at a time.
vb interleave_legacy(const vb& bits, int n_cbps, int n_bpsc)
{
    int s = std::max(n_bpsc / 2, 1);
    std::vector<int> first(n_cbps), second(n_cbps);
    for (int j = 0; j < n_cbps; j++)
        first[j] = s * (j / s) + ((j + (int)std::floor(16.0 * j / n_cbps)) % s);
    for (int i = 0; i < n_cbps; i++)
        second[i] = 16 * i - (n_cbps - 1) * (int)std::floor(16.0 * i / n_cbps);
    vb out(bits.size());
    int nsym = (int)bits.size() / n_cbps;
    for (int sym = 0; sym < nsym; sym++) {
        int base = sym * n_cbps;
        for (int k = 0; k < n_cbps; k++)
            out[base + k] = bits[base + second[first[k]]];
    }
    return out;
}

// HT forward interleave (802.11 19.3.11.7). 20 MHz: N_COL=13, N_ROW=4*N_BPSCS.
vb interleave_ht(const vb& bits, int n_cbps, int n_bpsc, int bw = 20)
{
    int s = std::max(n_bpsc / 2, 1);
    int n_col = (bw == 40) ? 18 : 13;
    int n_row = (bw == 40 ? 6 : 4) * n_bpsc;
    vb out(bits.size());
    int nsym = (int)bits.size() / n_cbps;
    for (int sym = 0; sym < nsym; sym++) {
        int base = sym * n_cbps;
        for (int k = 0; k < n_cbps; k++) {
            int i = n_row * (k % n_col) + (k / n_col);
            int j = s * (i / s) + (i + n_cbps - (n_col * i) / n_cbps) % s;
            out[base + j] = bits[base + k];
        }
    }
    return out;
}

uint8_t ht_sig_crc8(const vb& bits34)
{
    int c = 0xff;
    for (uint8_t b : bits34) {
        int fb = (b & 1) ^ ((c >> 7) & 1);
        c = (c << 1) & 0xff;
        if (fb)
            c ^= 0x07;
    }
    return (uint8_t)((~c) & 0xff);
}

vcf map_bpsk(const vb& bits)
{
    vcf out(bits.size());
    for (size_t i = 0; i < bits.size(); i++)
        out[i] = cf(bits[i] > 0 ? 1.f : -1.f, 0);
    return out;
}

// constellation point for a value, matching lib/constellations_impl.cc / the Python.
cf qam_point(int v, int n_bpsc)
{
    if (n_bpsc == 1)
        return cf(v ? 1.f : -1.f, 0);
    if (n_bpsc == 2) {
        float l = (float)std::sqrt(0.5);
        float re = (v & 1) ? l : -l;
        float im = (v & 2) ? l : -l;
        return cf(re, im);
    }
    if (n_bpsc == 4) {
        float l = (float)std::sqrt(0.1);
        const int lvl[4] = { -3, 3, -1, 1 };
        int r = lvl[(v & 1) | ((v >> 1 & 1) << 1)];
        int i = lvl[((v >> 2) & 1) | ((v >> 3 & 1) << 1)];
        return cf(r * l, i * l);
    }
    if (n_bpsc == 6) {
        float l = (float)std::sqrt(1.0 / 42.0);
        const int lvl[8] = { -7, 7, -1, 1, -5, 5, -3, 3 };
        int r = lvl[(v & 1) | ((v >> 1 & 1) << 1) | ((v >> 2 & 1) << 2)];
        int i = lvl[((v >> 3) & 1) | ((v >> 4 & 1) << 1) | ((v >> 5 & 1) << 2)];
        return cf(r * l, i * l);
    }
    if (n_bpsc == 8) {
        // 256-QAM (VHT MCS 8-9): 16-level Gray-coded PAM per axis, scale 1/sqrt(170).
        // Levels are the 4-bit I/Q tables from IEEE 802.11 (== GR-WiFi C_QAM_MODU_TAB[5]).
        float l = (float)std::sqrt(1.0 / 170.0);
        const int lvl[16] = { -15, 15, -1, 1,  -9, 9,  -7, 7,
                              -13, 13, -3, 3, -11, 11, -5, 5 };
        int r = lvl[v & 0xF];
        int i = lvl[(v >> 4) & 0xF];
        return cf(r * l, i * l);
    }
    throw std::runtime_error("unsupported n_bpsc");
}

vcf map_qam(const vb& coded, int n_bpsc)
{
    int n = (int)coded.size() / n_bpsc;
    vcf out(n);
    for (int c = 0; c < n; c++) {
        int v = 0;
        for (int k = 0; k < n_bpsc; k++)
            v |= (coded[c * n_bpsc + k] & 1) << k;
        out[c] = qam_point(v, n_bpsc);
    }
    return out;
}

// one OFDM data/SIG symbol from pre-mapped points + pilots (DATA_SC list).
// pilot_override (4 values, already incl. polarity) is used as-is when non-null; else
// the legacy/SIG pilots PILOT_VAL * POLARITY[sym_idx-2] are used. HT DATA symbols pass
// an override (cyclically-rotated pattern + the HT polarity index) -- the spec the chip
// requires; the legacy/SIG path keeps the simple computation.
// The frequency-domain content of one (L-)SIG / data symbol, before IFFT. Split out so
// the CSD path (apply_csd) can shift antenna 1's copy before ofdm_symbol().
vcf data_symbol_freq(vcf pts, const int* data_sc, int n_data, int sym_idx, bool rotate,
                     bool pilot_rotate, const cf* pilot_override = nullptr)
{
    vcf freq(64, cf(0, 0));
    if (rotate)
        for (auto& p : pts)
            p *= cf(0, 1);
    for (int c = 0; c < n_data; c++)
        freq[data_sc[c]] = pts[c];
    if (pilot_override) {
        for (int i = 0; i < 4; i++)
            freq[PILOT_SC[i]] = pilot_override[i];
    } else {
        float p = POLARITY[((sym_idx - 2) % 127 + 127) % 127];
        for (int i = 0; i < 4; i++)
            freq[PILOT_SC[i]] =
                cf(PILOT_VAL[i] * p, 0) * (pilot_rotate ? cf(0, 1) : cf(1, 0));
    }
    return freq;
}

vcf data_symbol(vcf pts, const int* data_sc, int n_data, int sym_idx, bool rotate,
                bool pilot_rotate, const cf* pilot_override = nullptr)
{
    return ofdm_symbol(
        data_symbol_freq(pts, data_sc, n_data, sym_idx, rotate, pilot_rotate, pilot_override));
}

vb bytes_to_bits(const uint8_t* data, int len)
{
    vb bits(len * 8, 0);
    for (int i = 0; i < len; i++)
        for (int b = 0; b < 8; b++)
            bits[i * 8 + b] = (data[i] >> b) & 1;
    return bits;
}

vb build_sig_bits(int rate4, int length12)
{
    vb bits(24, 0);
    for (int i = 0; i < 4; i++)
        bits[i] = (rate4 >> i) & 1;
    for (int i = 0; i < 12; i++)
        bits[5 + i] = (length12 >> i) & 1;
    int par = 0;
    for (int i = 0; i < 17; i++)
        par ^= bits[i];
    bits[17] = par & 1;
    return bits;
}

// 48 interleaved BPSK bits for the (L-)SIG OFDM symbol.
vb sig_field(int rate4, int length12)
{
    return interleave_legacy(conv_encode(build_sig_bits(rate4, length12)), 48, 1);
}

void append(vcf& dst, const vcf& src) { dst.insert(dst.end(), src.begin(), src.end()); }

// Legacy preamble (L-STF 160 + L-LTF GI2+2x64) with an optional cyclic shift tcs applied
// to both fields (for STS1 CSD). tcs=0 is the unshifted antenna-0 / SISO preamble.
vcf preamble_csd(int tcs)
{
    vcf st = ifft_sym(apply_csd(lstf_freq(), tcs));
    vcf out;
    for (int rep = 0; rep < 10; rep++)
        for (int i = 0; i < 16; i++)
            out.push_back(st[i]); // L-STF: 160 samples (10x first 16)
    vcf longv(64);
    for (int i = 0; i < 64; i++)
        longv[i] = cf((float)LONG[i], 0);
    vcf lt = ifft_sym(apply_csd(longv, tcs));
    for (int i = 32; i < 64; i++)
        out.push_back(lt[i]); // GI2
    append(out, lt);
    append(out, lt); // L-LTF: 2x64
    return out;
}

vcf preamble() { return preamble_csd(0); }

// HT-LTF20 = L-LTF + the HT edge tones (sc +-27,+-28 -> bins 4,5,59,60).
vcf htltf20_freq()
{
    vcf f(64);
    for (int i = 0; i < 64; i++)
        f[i] = cf((float)LONG[i], 0);
    f[4] = 1; f[5] = 1; f[59] = -1; f[60] = -1;
    return f;
}

// Spec L-SIG LENGTH for an HT-mixed PPDU: legacy deferral must cover the whole
// frame. L_LENGTH = 3*ceil((TXTIME-20us)/4us) - 3, RATE = 6 Mbps.
// TXTIME = 20 (L-pre) + 8 (HT-SIG) + 4 (HT-STF) + 4*N_LTF + 4*N_SYM (us).
int lsig_length_ht(int n_sym, int n_ltf)
{
    int txtime = 20 + 8 + 4 + 4 * n_ltf + 4 * n_sym;
    int sig_us = txtime - 20;
    int n = (sig_us + 3) / 4; // ceil(sig_us/4)
    return 3 * n - 3;
}

// VHT-LTF (20 MHz), subcarriers -28..+28 (DC=0), from GR-WiFi C_LTF_VHT_28.
const int VHTLTF20[57] = {
    1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1,
    1, 1, 0, 1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1,
    1, 1, 1, 1, -1, -1
};
vcf vhtltf20_freq()
{
    vcf f(64, cf(0, 0));
    for (int i = 0; i < 57; i++)
        f[4 + i] = cf((float)VHTLTF20[i], 0); // SC -28..+28 -> bins 4..60
    return f;
}

// Wrap a single MPDU in an A-MPDU (VHT data is always an A-MPDU). One subframe:
// 4-byte delimiter (EOF + MPDU length + CRC-8 + 0x4E signature) + MPDU + pad-to-4.
// Mirrors GR-WiFi mac80211.genAmpduVHT.
vb ampdu_wrap(const uint8_t* mpdu, int len)
{
    vb d; // 32 delimiter bits
    d.push_back(1);                       // EOF (single MPDU)
    d.push_back(0);                       // reserved
    d.push_back((len >> 12) & 1);         // len bits 12,13 ...
    d.push_back((len >> 13) & 1);
    for (int i = 0; i < 12; i++)          // ... then len bits 0..11
        d.push_back((len >> i) & 1);
    uint8_t crc = ht_sig_crc8(d);         // CRC-8 over the 16 delimiter bits
    for (int i = 0; i < 8; i++)
        d.push_back((crc >> (7 - i)) & 1); // MSB-first
    for (int i = 0; i < 8; i++)
        d.push_back((0x4e >> i) & 1);      // signature, LSB-first
    std::vector<uint8_t> out(4, 0);
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 8; j++)
            out[i] |= d[i * 8 + j] << j;   // pack LSB-first
    for (int i = 0; i < len; i++)
        out.push_back(mpdu[i]);
    while (out.size() % 4)
        out.push_back(0);                  // pad to 4-byte boundary
    return out;
}

// VHT20 SU SISO (MCS 0-7, BCC). The MPDU is wrapped in an A-MPDU (VHT data is always
// aggregated); VHT-SIG-B carries the APEP length and its CRC-8 seeds DATA SERVICE[8:16].
std::vector<vcf> build_vht20(const uint8_t* mpdu, int mpdu_len, const tx_params& p)
{
    const mcs_row& m = VHT_MCS20[p.mcs]; // VHT MCS0-7 == HT20; MCS8 = 256-QAM r3/4
    vb ampdu = ampdu_wrap(mpdu, mpdu_len);
    int alen = (int)ampdu.size();
    int nsym = (16 + 8 * alen + 6 + m.n_dbps - 1) / m.n_dbps;
    int n_ltf = 1;

    vcf frame = preamble();

    // L-SIG. VHT TXTIME = 20 + 8(SIG-A) + 4(VHT-STF) + 4*N_LTF + 4(SIG-B) + 4*nsym.
    int txtime = 20 + 8 + 4 + 4 * n_ltf + 4 + 4 * nsym;
    int lsig_len = 3 * ((txtime - 20 + 3) / 4) - 3;
    append(frame, data_symbol(map_bpsk(sig_field(11, lsig_len)), DATA_SC_LEGACY, 48, 2,
                              false, false));

    // VHT-SIG-A (48 bits). Reserved bits B2/B23/B33 must be 1 (chip validates them).
    vb a(48, 0);
    a[2] = 1;
    a[23] = 1;
    for (int i = 0; i < 4; i++)
        a[28 + i] = (p.mcs >> i) & 1; // B28-31 MCS
    a[33] = 1;
    uint8_t acrc = ht_sig_crc8(vb(a.begin(), a.begin() + 34));
    for (int i = 0; i < 8; i++)
        a[34 + i] = (acrc >> (7 - i)) & 1;
    vb a_il = interleave_legacy(conv_encode(a), 48, 1); // 96
    const cf sigpil[4] = { cf(1, 0), cf(1, 0), cf(1, 0), cf(-1, 0) };
    vb a0(a_il.begin(), a_il.begin() + 48), a1(a_il.begin() + 48, a_il.end());
    append(frame, data_symbol(map_bpsk(a0), DATA_SC_LEGACY, 48, 3, false, false, sigpil));
    append(frame, data_symbol(map_bpsk(a1), DATA_SC_LEGACY, 48, 4, true, false, sigpil));

    // VHT-STF (= L-STF) + VHT-LTF.
    append(frame, ofdm_symbol(lstf_freq()));
    append(frame, ofdm_symbol(vhtltf20_freq()));

    // VHT-SIG-B (26 bits): B0-16 APEP length, B17-19 reserved=1, B20-25 tail. The CRC-8
    // over B0-19 seeds DATA SERVICE[8:16].
    int apep = alen / 4;
    vb b(26, 0);
    for (int i = 0; i < 17; i++)
        b[i] = (apep >> i) & 1;
    b[17] = b[18] = b[19] = 1;
    uint8_t bcrc = ht_sig_crc8(vb(b.begin(), b.begin() + 20));
    vb b_il = interleave_ht(conv_encode(b), 52, 1, 20); // 26->52 BCC, HT interleaver
    append(frame, data_symbol(map_bpsk(b_il), DATA_SC_HT, 52, 7, false, false, sigpil));

    // VHT DATA: SERVICE(16) = [0]*8 + SIG-B CRC(8); + A-MPDU bits + pad; scrambled.
    vb data_bits(nsym * m.n_dbps, 0);
    for (int i = 0; i < 8; i++)
        data_bits[8 + i] = (bcrc >> (7 - i)) & 1; // SERVICE[8:16]
    vb abits = bytes_to_bits(ampdu.data(), alen);
    for (size_t i = 0; i < abits.size(); i++)
        data_bits[16 + i] = abits[i];
    vb scr = scramble(data_bits);
    for (int i = 0; i < 6; i++)
        scr[16 + 8 * alen + i] = 0; // tail
    vb coded = puncture(conv_encode(scr), m.rnum, m.rden);
    vb il = interleave_ht(coded, m.n_cbps, m.n_bpsc, 20);
    // VHT data pilots: base {1,1,1,-1} rotated by symbol index, times the polarity
    // sequence starting at index 4 (VHT-SIG-B is an extra pilot-carrying symbol between
    // the LTF and DATA, so the index is one past HT's 3).
    const int htp_base[4] = { 1, 1, 1, -1 };
    for (int j = 0; j < nsym; j++) {
        vb chunk(il.begin() + j * m.n_cbps, il.begin() + (j + 1) * m.n_cbps);
        float pol = POLARITY[(4 + j) % 127];
        cf pil[4];
        for (int i = 0; i < 4; i++)
            pil[i] = cf(htp_base[(i + j) % 4] * pol, 0);
        append(frame, data_symbol(map_qam(chunk, m.n_bpsc), DATA_SC_HT, 52, 8 + j, false,
                                  false, pil));
    }
    return { frame };
}

// ---- HT40 (128-FFT, 40 MHz) ----
// per-stream {n_bpsc, rnum, rden, n_cbps, n_dbps} for 108 data SC.
const mcs_row HT_MCS40[8] = {
    { 1, 1, 2, 108, 54 }, { 2, 1, 2, 216, 108 }, { 2, 3, 4, 216, 162 }, { 4, 1, 2, 432, 216 },
    { 4, 3, 4, 432, 324 }, { 6, 2, 3, 648, 432 }, { 6, 3, 4, 648, 486 }, { 6, 5, 6, 648, 540 }
};
// HT40 data subcarriers (108) and pilots {-53,-25,-11,11,25,53} -> bins, base {1,1,1,-1,-1,1}.
const int PILOT_SC_HT40[6] = { 11, 39, 53, 75, 89, 117 };
const int PILOT40_BASE[6] = { 1, 1, 1, -1, -1, 1 };
bool is_pilot40(int b)
{
    for (int i = 0; i < 6; i++)
        if (b == PILOT_SC_HT40[i])
            return true;
    return false;
}

// 20 MHz freq symbol (DC@32) -> 40 MHz non-HT-duplicate (lower bins 6..58, upper 70..122 *j).
vcf dup40(const vcf& f64)
{
    vcf f(128, cf(0, 0));
    for (int i = 6; i <= 58; i++) {
        f[i] = f64[i];
        f[64 + i] = f64[i] * cf(0, 1);
    }
    return f;
}

vcf long40_freq()
{
    vcf f64(64);
    for (int i = 0; i < 64; i++)
        f64[i] = cf((float)LONG[i], 0);
    return dup40(f64);
}

// HT-LTF40 fills all 114 occupied bins (incl the centre tones the duplicate L-LTF nulls).
vcf htltf40_freq()
{
    vcf f = long40_freq();
    f[32] = 1;
    f[96] = cf(0, 1);
    for (int b = 59; b <= 62; b++)
        f[b] = 1;
    for (int b = 66; b <= 69; b++)
        f[b] = cf(0, 1);
    return f;
}

// HT40 data SC list: 6..122 (both subchannels) minus DC region and the 6 pilots.
std::vector<int> data_sc_ht40()
{
    std::vector<int> v;
    for (int i = 6; i <= 122; i++) {
        if (i >= 63 && i <= 65)
            continue; // DC/null centre
        if (is_pilot40(i))
            continue;
        v.push_back(i);
    }
    return v; // 108
}

vcf preamble40()
{
    vcf st = ifft_sym(dup40(lstf_freq()));
    vcf out;
    for (int rep = 0; rep < 10; rep++)
        for (int i = 0; i < 32; i++)
            out.push_back(st[i]); // L-STF40: 320 samples (10x first 32)
    vcf lt = ifft_sym(long40_freq());
    for (int i = 64; i < 128; i++)
        out.push_back(lt[i]); // GI2(64)
    append(out, lt);
    append(out, lt); // 2x128
    return out;
}

// 128-FFT data/SIG symbol over the HT40 geometry. pilot_override = 6 explicit pilots.
vcf data_symbol40(vcf pts, const std::vector<int>& data_sc, int sym_idx, bool rotate,
                  const cf* pilot_override)
{
    vcf freq(128, cf(0, 0));
    if (rotate)
        for (auto& p : pts)
            p *= cf(0, 1);
    for (size_t c = 0; c < data_sc.size(); c++)
        freq[data_sc[c]] = pts[c];
    float p = POLARITY[((sym_idx - 2) % 127 + 127) % 127];
    for (int i = 0; i < 6; i++)
        freq[PILOT_SC_HT40[i]] = pilot_override
                                     ? pilot_override[i]
                                     : cf(PILOT40_BASE[i] * p, 0) * (rotate ? cf(0, 1) : cf(1, 0));
    return ofdm_symbol(freq);
}

// L-SIG / HT-SIG in HT40 non-HT-duplicate form (the 20 MHz symbol duplicated).
vcf sig_symbol_dup40(const vb& bits48, int sym_idx, bool rotate)
{
    vcf f64(64, cf(0, 0));
    vcf pts = map_bpsk(bits48);
    if (rotate)
        for (auto& p : pts)
            p *= cf(0, 1);
    for (int c = 0; c < 48; c++)
        f64[DATA_SC_LEGACY[c]] = pts[c];
    float p = POLARITY[((sym_idx - 2) % 127 + 127) % 127];
    for (int i = 0; i < 4; i++)
        f64[PILOT_SC[i]] = cf(PILOT_VAL[i] * p, 0) * (rotate ? cf(0, 1) : cf(1, 0));
    return ofdm_symbol(dup40(f64));
}

std::vector<vcf> build_ht40(const uint8_t* psdu, int length, const tx_params& p)
{
    const mcs_row& m = HT_MCS40[p.mcs];
    int nsym = (16 + 8 * length + 6 + m.n_dbps - 1) / m.n_dbps;
    std::vector<int> DSC = data_sc_ht40();

    vcf frame = preamble40();

    // L-SIG (dup40), spec length.
    int txtime = 20 + 8 + 4 + 4 * 1 + 4 * nsym;
    int lsig_len = 3 * ((txtime - 20 + 3) / 4) - 3;
    append(frame, sig_symbol_dup40(sig_field(11, lsig_len), 2, false));

    // HT-SIG (dup40): MCS, CBW40=bit7=1, length, reserved bits 24/25/26=1, CRC.
    vb hsig(48, 0);
    for (int i = 0; i < 7; i++)
        hsig[i] = (p.mcs >> i) & 1;
    hsig[7] = 1; // CBW40
    for (int i = 0; i < 16; i++)
        hsig[8 + i] = (length >> i) & 1;
    hsig[24] = hsig[25] = hsig[26] = 1;
    uint8_t crc = ht_sig_crc8(vb(hsig.begin(), hsig.begin() + 34));
    for (int i = 0; i < 8; i++)
        hsig[34 + i] = (crc >> (7 - i)) & 1;
    vb hsig_il = interleave_legacy(conv_encode(hsig), 48, 1);
    append(frame, sig_symbol_dup40(vb(hsig_il.begin(), hsig_il.begin() + 48), 3, true));
    append(frame, sig_symbol_dup40(vb(hsig_il.begin() + 48, hsig_il.end()), 4, true));

    // HT-STF40 (spec: L-STF dup) + HT-LTF40.
    append(frame, ofdm_symbol(dup40(lstf_freq())));
    append(frame, ofdm_symbol(htltf40_freq()));

    // HT40 DATA.
    vb data_bits(nsym * m.n_dbps, 0);
    vb pbits = bytes_to_bits(psdu, length);
    for (size_t i = 0; i < pbits.size(); i++)
        data_bits[16 + i] = pbits[i];
    vb scr = scramble(data_bits);
    for (int i = 0; i < 6; i++)
        scr[16 + 8 * length + i] = 0;
    vb coded = puncture(conv_encode(scr), m.rnum, m.rden);
    vb il = interleave_ht(coded, m.n_cbps, m.n_bpsc, 40);
    for (int j = 0; j < nsym; j++) {
        vb chunk(il.begin() + j * m.n_cbps, il.begin() + (j + 1) * m.n_cbps);
        float pol = POLARITY[(3 + j) % 127];
        cf pil[6];
        for (int i = 0; i < 6; i++)
            pil[i] = cf(PILOT40_BASE[(i + j) % 6] * pol, 0);
        append(frame, data_symbol40(map_qam(chunk, m.n_bpsc), DSC, 7 + j, false, pil));
    }
    return { frame };
}

// A SIG symbol in HT40 non-HT-duplicate form with EXPLICIT legacy pilots (the VHT-SIG-A
// duplicate uses the fixed sigpil rather than the polarity sequence).
vcf sig_dup40_pil(const vb& bits48, bool rotate, const cf* pil4)
{
    vcf f64(64, cf(0, 0));
    vcf pts = map_bpsk(bits48);
    if (rotate)
        for (auto& q : pts)
            q *= cf(0, 1);
    for (int c = 0; c < 48; c++)
        f64[DATA_SC_LEGACY[c]] = pts[c];
    for (int i = 0; i < 4; i++)
        f64[PILOT_SC[i]] = pil4[i];
    return ofdm_symbol(dup40(f64));
}

// VHT40 SU SISO (MCS 0-9, BCC). Reuses the HT40 geometry (128-FFT, 108 data SC, 6 pilots)
// with the VHT preamble + SIG fields. VHT-LTF40 == HT-LTF40 (C_LTF_VHT_58 = C_LTF_HT_58).
std::vector<vcf> build_vht40(const uint8_t* mpdu, int mpdu_len, const tx_params& p)
{
    // VHT40 per-stream MCS (108 data SC): MCS0-7 == HT40; MCS8 256-QAM r3/4, MCS9 r5/6.
    static const mcs_row VHT_MCS40[10] = {
        { 1, 1, 2, 108, 54 },  { 2, 1, 2, 216, 108 }, { 2, 3, 4, 216, 162 },
        { 4, 1, 2, 432, 216 }, { 4, 3, 4, 432, 324 }, { 6, 2, 3, 648, 432 },
        { 6, 3, 4, 648, 486 }, { 6, 5, 6, 648, 540 }, { 8, 3, 4, 864, 648 },
        { 8, 5, 6, 864, 720 }
    };
    const mcs_row& m = VHT_MCS40[p.mcs];
    vb ampdu = ampdu_wrap(mpdu, mpdu_len);
    int alen = (int)ampdu.size();
    int nsym = (16 + 8 * alen + 6 + m.n_dbps - 1) / m.n_dbps;
    int n_ltf = 1;
    std::vector<int> DSC = data_sc_ht40();
    const cf sigpil[4] = { cf(1, 0), cf(1, 0), cf(1, 0), cf(-1, 0) };
    cf pil40_base[6];
    for (int i = 0; i < 6; i++)
        pil40_base[i] = cf((float)PILOT40_BASE[i], 0);

    vcf frame = preamble40();

    // L-SIG (dup40), spec VHT length.
    int txtime = 20 + 8 + 4 + 4 * n_ltf + 4 + 4 * nsym;
    int lsig_len = 3 * ((txtime - 20 + 3) / 4) - 3;
    append(frame, sig_symbol_dup40(sig_field(11, lsig_len), 2, false));

    // VHT-SIG-A (dup40): B0-1 BW=01 (40 MHz), reserved B2/B23/B33=1, MCS B28-31.
    vb a(48, 0);
    a[0] = 1; // BW = 40 MHz (B0=1, B1=0)
    a[2] = 1;
    a[23] = 1;
    a[33] = 1;
    for (int i = 0; i < 4; i++)
        a[28 + i] = (p.mcs >> i) & 1;
    uint8_t acrc = ht_sig_crc8(vb(a.begin(), a.begin() + 34));
    for (int i = 0; i < 8; i++)
        a[34 + i] = (acrc >> (7 - i)) & 1;
    vb a_il = interleave_legacy(conv_encode(a), 48, 1);
    append(frame, sig_dup40_pil(vb(a_il.begin(), a_il.begin() + 48), false, sigpil));
    append(frame, sig_dup40_pil(vb(a_il.begin() + 48, a_il.end()), true, sigpil));

    // VHT-STF (= dup40 L-STF) + VHT-LTF40 (== HT-LTF40).
    append(frame, ofdm_symbol(dup40(lstf_freq())));
    append(frame, ofdm_symbol(htltf40_freq()));

    // VHT-SIG-B (40 MHz): 19 len + 2 reserved=1 + 6 tail = 27 bits, repeated x2 -> 54,
    // BCC -> 108 coded, BPSK over the 108 data SC. CRC-8 over the 21 len+reserved bits
    // seeds DATA SERVICE[8:16].
    int apep = alen / 4;
    vb b(27, 0);
    for (int i = 0; i < 19; i++)
        b[i] = (apep >> i) & 1;
    b[19] = b[20] = 1; // reserved
    uint8_t bcrc = ht_sig_crc8(vb(b.begin(), b.begin() + 21));
    vb b2(b);
    b2.insert(b2.end(), b.begin(), b.end()); // x2 -> 54
    vb b_il = interleave_ht(conv_encode(b2), 108, 1, 40); // 54 -> 108
    append(frame, data_symbol40(map_bpsk(b_il), DSC, 7, false, pil40_base));

    // VHT DATA: SERVICE(16) = [0]*8 + SIG-B CRC(8) + A-MPDU + pad; scrambled.
    vb data_bits(nsym * m.n_dbps, 0);
    for (int i = 0; i < 8; i++)
        data_bits[8 + i] = (bcrc >> (7 - i)) & 1; // SERVICE[8:16]
    vb abits = bytes_to_bits(ampdu.data(), alen);
    for (size_t i = 0; i < abits.size(); i++)
        data_bits[16 + i] = abits[i];
    vb scr = scramble(data_bits);
    for (int i = 0; i < 6; i++)
        scr[16 + 8 * alen + i] = 0; // tail
    vb coded = puncture(conv_encode(scr), m.rnum, m.rden);
    vb il = interleave_ht(coded, m.n_cbps, m.n_bpsc, 40);
    for (int j = 0; j < nsym; j++) {
        vb chunk(il.begin() + j * m.n_cbps, il.begin() + (j + 1) * m.n_cbps);
        float pol = POLARITY[(4 + j) % 127]; // VHT data starts at sym 8 (VHT-SIG-B at 7)
        cf pil[6];
        for (int i = 0; i < 6; i++)
            pil[i] = cf(PILOT40_BASE[(i + j) % 6] * pol, 0);
        append(frame, data_symbol40(map_qam(chunk, m.n_bpsc), DSC, 8 + j, false, pil));
    }
    return { frame };
}

} // namespace

// HT20 SISO LDPC (MCS0, R=1/2 n=648 single codeword). IEEE 802.11-2016 19.3.11.7.5:
// scramble SERVICE+PSDU, pad to k=324 with shortening zeros, systematic LDPC-encode,
// drop the N_shrt shortening info bits + the last N_punc parity bits, then apply the
// 20 MHz LDPC tone mapping (D_TM=4) per OFDM symbol. HT-SIG FEC bit (B30) = 1.
std::vector<vcf> build_ht_ldpc(const uint8_t* psdu, int length, const tx_params& p)
{
    // MCS0: BPSK, R=1/2 -> N_DBPS=26, N_CBPS=52 on 52 data SC.
    const int N_DBPS = 26, N_CBPS = 52, K = 324, Lcw = 648;
    int N_pld = 16 + 8 * length; // SERVICE(16) + PSDU
    int N_sym = (N_pld + N_DBPS - 1) / N_DBPS;
    int N_avbits = N_sym * N_CBPS;
    int N_shrt = K - N_pld;                 // shortening zeros in the single codeword
    int N_punc = Lcw - N_avbits - N_shrt;   // parity bits punctured from the tail
    // Puncture-too-aggressive guard (bump one OFDM symbol). R/(1-R)=1 for R=1/2.
    if ((N_punc > 0.1 * Lcw * 0.5 && N_shrt < 1.2 * N_punc) || N_punc > 0.3 * Lcw * 0.5) {
        N_sym += 1;
        N_avbits = N_sym * N_CBPS;
        N_punc = Lcw - N_avbits - N_shrt;
    }
    if (N_shrt < 0 || N_punc < 0 || N_avbits > Lcw)
        throw std::runtime_error("wifi_tx: LDPC length out of single-648-codeword range");

    vcf frame = preamble();
    int lsig_len = lsig_length_ht(N_sym, 1);
    append(frame, data_symbol(map_bpsk(sig_field(11, lsig_len)), DATA_SC_LEGACY, 48, 2,
                              false, false));
    // HT-SIG: same as BCC HT20 but B30 (FEC) = 1 (LDPC).
    vb hsig(48, 0);
    for (int i = 0; i < 7; i++)
        hsig[i] = (p.mcs >> i) & 1;
    for (int i = 0; i < 16; i++)
        hsig[8 + i] = (length >> i) & 1;
    hsig[24] = hsig[25] = hsig[26] = 1; // smoothing / not-sounding / reserved
    hsig[30] = 1;                        // FEC = LDPC
    vb hsig34(hsig.begin(), hsig.begin() + 34);
    uint8_t crc = ht_sig_crc8(hsig34);
    for (int i = 0; i < 8; i++)
        hsig[34 + i] = (crc >> (7 - i)) & 1;
    vb hsig_il = interleave_legacy(conv_encode(hsig), 48, 1);
    append(frame, data_symbol(map_bpsk(vb(hsig_il.begin(), hsig_il.begin() + 48)),
                              DATA_SC_LEGACY, 48, 3, true, false));
    append(frame, data_symbol(map_bpsk(vb(hsig_il.begin() + 48, hsig_il.end())),
                              DATA_SC_LEGACY, 48, 4, true, false));
    append(frame, ofdm_symbol(lstf_freq()));
    append(frame, ofdm_symbol(htltf20_freq()));

    // LDPC encode: info = scramble(SERVICE+PSDU) ++ N_shrt shortening zeros.
    static LdpcEncoder648 enc;
    vb db(N_pld, 0);
    vb pbits = bytes_to_bits(psdu, length);
    for (size_t i = 0; i < pbits.size(); i++)
        db[16 + i] = pbits[i];
    vb scr = scramble(db);
    std::vector<uint8_t> info(K, 0), cw(Lcw, 0);
    for (int i = 0; i < N_pld; i++)
        info[i] = scr[i];
    enc.encode(info.data(), cw.data());
    // Transmitted coded bits: info[0:N_pld] (drop shortening) ++ parity[0:m-N_punc].
    vb coded;
    coded.reserve(N_avbits);
    for (int i = 0; i < N_pld; i++)
        coded.push_back(cw[i]);
    for (int i = K; i < Lcw - N_punc; i++)
        coded.push_back(cw[i]);

    // Per symbol: BPSK map straight onto the 52 data SC, then cycling DATA pilots.
    // 20 MHz HT LDPC uses NO tone mapping (D_TM=1, identity) -- confirmed against real
    // RTL8812AU silicon: D_TM=4 (the value sometimes quoted) and every other D_TM>1
    // permutation makes the chip's LDPC decoder fail the DATA CRC; only the in-order
    // mapping decodes clean. (LDPC tone mapping only applies at 40 MHz and wider.)
    const int htp_base[4] = { 1, 1, 1, -1 };
    for (int j = 0; j < N_sym; j++) {
        vb chunk(coded.begin() + j * N_CBPS, coded.begin() + (j + 1) * N_CBPS);
        vcf pts = map_bpsk(chunk);
        float pol = POLARITY[(3 + j) % 127];
        cf pil[4];
        for (int i = 0; i < 4; i++)
            pil[i] = cf(htp_base[(i + j) % 4] * pol, 0);
        append(frame, data_symbol(pts, DATA_SC_HT, 52, 7 + j, false, false, pil));
    }
    return { frame };
}

// Frequency-domain HT20 DATA symbol (64-bin: 52 data SC + 4 cycling pilots), no IFFT.
// Used by STBC, which must apply the Alamouti transform per subcarrier before the IFFT.
vcf ht20_data_freq(const vcf& pts, int j)
{
    vcf freq(64, cf(0, 0));
    for (int c = 0; c < 52; c++)
        freq[DATA_SC_HT[c]] = pts[c];
    // Chip cycling DATA pilots: base {1,1,1,-1} rotated by the data-symbol index, times
    // POLARITY[3+j] (same as non-STBC HT; chip-validated).
    const int htp_base[4] = { 1, 1, 1, -1 };
    float pol = POLARITY[(3 + j) % 127];
    for (int i = 0; i < 4; i++)
        freq[PILOT_SC[i]] = cf(htp_base[(i + j) % 4] * pol, 0);
    return freq;
}

// HT20 STBC: 1 spatial stream -> 2 space-time streams (Alamouti), 2 TX antennas.
// Per the RX inverse (frame_equalizer stbc_estimate_ltf / stbc_data_symbol): standard
// HT-LTF P-matrix STS0=[+,-] STS1=[+,+]; Alamouti slot A: ant0=s0, ant1=-conj(s1); slot B:
// ant0=s1, ant1=conj(s0). MCS 0-7 BCC. (Pilots ride STS0.) Returns {ant0, ant1}.
std::vector<vcf> build_stbc(const uint8_t* psdu, int length, const tx_params& p)
{
    const mcs_row& m = HT_MCS20[p.mcs];
    int nsym = (16 + 8 * length + 6 + m.n_dbps - 1) / m.n_dbps;
    if (nsym % 2)
        nsym++; // Alamouti pairs OFDM symbols -> need an even count
    int n_ltf = 2;

    // Preamble + L-SIG + HT-SIG (STBC=1) + HT-STF, built in the frequency domain so
    // antenna 1 (STS1) can be cyclic-shifted independently (CSD, below).
    int lsig_len = lsig_length_ht(nsym, n_ltf);
    vcf lsig_f = data_symbol_freq(map_bpsk(sig_field(11, lsig_len)), DATA_SC_LEGACY, 48, 2,
                                  false, false);
    vb hsig(48, 0);
    for (int i = 0; i < 7; i++)
        hsig[i] = (p.mcs >> i) & 1;
    for (int i = 0; i < 16; i++)
        hsig[8 + i] = (length >> i) & 1;
    hsig[24] = hsig[25] = hsig[26] = 1;
    hsig[28] = 1; // STBC field = 1 (one extra space-time stream: 1 SS -> 2 STS)
    uint8_t crc = ht_sig_crc8(vb(hsig.begin(), hsig.begin() + 34));
    for (int i = 0; i < 8; i++)
        hsig[34 + i] = (crc >> (7 - i)) & 1;
    vb hsig_il = interleave_legacy(conv_encode(hsig), 48, 1);
    vcf hs0_f = data_symbol_freq(map_bpsk(vb(hsig_il.begin(), hsig_il.begin() + 48)),
                                 DATA_SC_LEGACY, 48, 3, true, false);
    vcf hs1_f = data_symbol_freq(map_bpsk(vb(hsig_il.begin() + 48, hsig_il.end())),
                                 DATA_SC_LEGACY, 48, 4, true, false);

    // HT-LTF P-matrix = standard 802.11 [[1,-1],[1,1]]: STS0=[+,-], STS1=[+,+]. A real
    // RTL8812AU uses this (confirmed by separating a chip-transmitted STBC frame on two
    // B210 RX channels). Wrong P -> the chip's per-STS channel estimate swaps/negates
    // -> the Alamouti DATA combine fails CRC.
    vcf ltf_p = htltf20_freq();
    vcf ltf_n(64);
    for (int i = 0; i < 64; i++)
        ltf_n[i] = -ltf_p[i];

    // SISO data chain -> per-symbol frequency-domain constellations.
    vb data_bits(nsym * m.n_dbps, 0);
    vb pbits = bytes_to_bits(psdu, length);
    for (size_t i = 0; i < pbits.size(); i++)
        data_bits[16 + i] = pbits[i];
    vb scr = scramble(data_bits);
    for (int i = 0; i < 6; i++)
        scr[16 + 8 * length + i] = 0;
    vb coded = puncture(conv_encode(scr), m.rnum, m.rden);
    vb il = interleave_ht(coded, m.n_cbps, m.n_bpsc, 20);
    std::vector<vcf> F(nsym);
    for (int j = 0; j < nsym; j++) {
        vb chunk(il.begin() + j * m.n_cbps, il.begin() + (j + 1) * m.n_cbps);
        F[j] = ht20_data_freq(map_qam(chunk, m.n_bpsc), j);
    }

    // Antenna 0 (STS0): no cyclic shift. Antenna 1 (STS1): legacy fields (L-STF/L-LTF/
    // L-SIG/HT-SIG) shifted by C_L, HT fields (HT-STF/HT-LTF/DATA) by C_HT -- 802.11
    // 2-stream cyclic-shift diversity. The chip absorbs the shift into its per-STS
    // channel estimate, so STBC still decodes; CSD decorrelates the antennas over the air.
    const int C_L = -4, C_HT = -8; // -200 ns / -400 ns at 20 MHz (C_CYCLIC_SHIFT[1])
    vcf a0 = preamble(), a1 = preamble_csd(C_L);
    append(a0, ofdm_symbol(lsig_f));
    append(a0, ofdm_symbol(hs0_f));
    append(a0, ofdm_symbol(hs1_f));
    append(a0, ofdm_symbol(lstf_freq())); // HT-STF
    append(a0, ofdm_symbol(ltf_p));
    append(a0, ofdm_symbol(ltf_n)); // STS0 LTF [+,-]
    append(a1, ofdm_symbol(apply_csd(lsig_f, C_L)));
    append(a1, ofdm_symbol(apply_csd(hs0_f, C_L)));
    append(a1, ofdm_symbol(apply_csd(hs1_f, C_L)));
    append(a1, ofdm_symbol(apply_csd(lstf_freq(), C_HT))); // HT-STF
    append(a1, ofdm_symbol(apply_csd(ltf_p, C_HT)));
    append(a1, ofdm_symbol(apply_csd(ltf_p, C_HT))); // STS1 LTF [+,+]
    for (int i = 0; i < nsym; i += 2) {
        const vcf& s0 = F[i];
        const vcf& s1 = F[i + 1];
        vcf ms1c(64), s0c(64);
        for (int k = 0; k < 64; k++) {
            ms1c[k] = -std::conj(s1[k]);
            s0c[k] = std::conj(s0[k]);
        }
        append(a0, ofdm_symbol(s0));
        append(a0, ofdm_symbol(s1)); // STS0: s0, s1
        append(a1, ofdm_symbol(apply_csd(ms1c, C_HT)));
        append(a1, ofdm_symbol(apply_csd(s0c, C_HT))); // STS1: -conj(s1), conj(s0)
    }
    return { a0, a1 };
}

// ---- 2x2 HT MIMO (MCS 8-15, HT20) ----
// per-stream {n_bpsc, rnum, rden, n_cbps_ss, n_dbps_ss} = HT20 SISO MCS 0-7 params.
const mcs_row HT_MCS_MIMO[8] = { { 1, 1, 2, 52, 26 },   { 2, 1, 2, 104, 52 },
                                 { 2, 3, 4, 104, 78 },  { 4, 1, 2, 208, 104 },
                                 { 4, 3, 4, 208, 156 }, { 6, 2, 3, 312, 208 },
                                 { 6, 3, 4, 312, 234 }, { 6, 5, 6, 312, 260 } };
// HT20 DATA pilot pattern per spatial stream, N_SS=2 (802.11 Table 19-21).
const int MIMO_PILOT2[2][4] = { { 1, 1, -1, -1 }, { 1, -1, -1, 1 } };
const int N_ROT_20 = 11;

// 802.11n stream parser (19.3.11.6): one BCC stream -> n_ss streams, s bits round-robin.
std::vector<vb> stream_parse(const vb& coded, int n_ss, int n_bpsc, int n_cbps_ss)
{
    int s = std::max(n_bpsc / 2, 1);
    int nsym = (int)coded.size() / (n_ss * n_cbps_ss);
    std::vector<vb> out(n_ss, vb(nsym * n_cbps_ss, 0));
    std::vector<int> pos(n_ss, 0);
    int idx = 0, blocks_per_sym = (n_ss * n_cbps_ss) / s;
    for (int sym = 0; sym < nsym; sym++)
        for (int blk = 0; blk < blocks_per_sym; blk++) {
            int ss = blk % n_ss;
            for (int b = 0; b < s; b++)
                out[ss][pos[ss]++] = coded[idx++];
        }
    return out;
}

// HT interleave for spatial stream iss (0-based), with the 3rd-permutation frequency
// rotation (19.3.11.7.3) for iss>0.
vb interleave_ht_ss(const vb& bits, int n_cbps_ss, int n_bpsc, int iss)
{
    int s = std::max(n_bpsc / 2, 1), n_col = 13, n_row = 4 * n_bpsc;
    int issp = iss + 1;
    int jrot = (((issp - 1) * 2) % 3 + 3 * ((issp - 1) / 3)) * N_ROT_20 * n_bpsc;
    vb out(bits.size(), 0);
    int nsym = (int)bits.size() / n_cbps_ss;
    for (int sym = 0; sym < nsym; sym++) {
        int base = sym * n_cbps_ss;
        for (int k = 0; k < n_cbps_ss; k++) {
            int i = n_row * (k % n_col) + (k / n_col);
            int j = s * (i / s) + (i + n_cbps_ss - (n_col * i) / n_cbps_ss) % s;
            int r = ((j - jrot) % n_cbps_ss + n_cbps_ss) % n_cbps_ss;
            out[base + r] = bits[base + k];
        }
    }
    return out;
}

// HT20 MIMO DATA symbol (64-bin freq) for spatial stream ss, data-symbol index j:
// 52 data SC + per-stream cycling pilots (MIMO_PILOT2[ss] rotated by j, * POLARITY[3+j]).
vcf mimo_data_freq(const vcf& pts, int ss, int j)
{
    vcf freq(64, cf(0, 0));
    for (int c = 0; c < 52; c++)
        freq[DATA_SC_HT[c]] = pts[c];
    float pol = POLARITY[(3 + j) % 127];
    for (int i = 0; i < 4; i++)
        freq[PILOT_SC[i]] = cf(MIMO_PILOT2[ss][(i + j) % 4] * pol, 0);
    return freq;
}

// 2x2 HT MIMO (MCS 8-15). Returns {ant0=STS0, ant1=STS1}. STS0 has zero cyclic shift
// (matches a spec stream-0 capture); STS1 is cyclic-shifted (CSD) to decorrelate the two
// TX antennas over the air -- legacy fields -200 ns, HT fields -400 ns.
std::vector<vcf> build_mimo(const uint8_t* psdu, int length, const tx_params& p)
{
    int mcs8 = p.mcs - 8; // 0..7 per-stream index
    const mcs_row& m = HT_MCS_MIMO[mcs8];
    const int n_ss = 2;
    int n_dbps_tot = n_ss * m.n_dbps;
    int nsym = (16 + 8 * length + 6 + n_dbps_tot - 1) / n_dbps_tot;
    int n_ltf = 2;

    int lsig_len = lsig_length_ht(nsym, n_ltf);
    vcf lsig_f = data_symbol_freq(map_bpsk(sig_field(11, lsig_len)), DATA_SC_LEGACY, 48, 2,
                                  false, false);
    // HT-SIG: MCS 8-15 (2 SS), reserved bits 24/25/26=1, no STBC/LDPC.
    vb hsig(48, 0);
    for (int i = 0; i < 7; i++)
        hsig[i] = (p.mcs >> i) & 1;
    for (int i = 0; i < 16; i++)
        hsig[8 + i] = (length >> i) & 1;
    hsig[24] = hsig[25] = hsig[26] = 1;
    uint8_t crc = ht_sig_crc8(vb(hsig.begin(), hsig.begin() + 34));
    for (int i = 0; i < 8; i++)
        hsig[34 + i] = (crc >> (7 - i)) & 1;
    vb hi = interleave_legacy(conv_encode(hsig), 48, 1);
    vcf hs0_f = data_symbol_freq(map_bpsk(vb(hi.begin(), hi.begin() + 48)), DATA_SC_LEGACY,
                                 48, 3, true, false);
    vcf hs1_f = data_symbol_freq(map_bpsk(vb(hi.begin() + 48, hi.end())), DATA_SC_LEGACY, 48,
                                 4, true, false);

    // HT-LTF: 2 symbols, standard P-matrix [[1,-1],[1,1]] (STS0=[+,-], STS1=[+,+]).
    vcf ltf_p = htltf20_freq(), ltf_n(64);
    for (int i = 0; i < 64; i++)
        ltf_n[i] = -ltf_p[i];

    // STS0 (ant0): no cyclic shift. STS1 (ant1): legacy fields shifted by C_L, HT fields
    // by C_HT -- 802.11 2-stream cyclic-shift diversity to decorrelate the antennas.
    const int C_L = -4, C_HT = -8;
    vcf a0 = preamble(), a1 = preamble_csd(C_L);
    append(a0, ofdm_symbol(lsig_f));
    append(a0, ofdm_symbol(hs0_f));
    append(a0, ofdm_symbol(hs1_f));
    append(a0, ofdm_symbol(lstf_freq())); // HT-STF
    append(a0, ofdm_symbol(ltf_p));
    append(a0, ofdm_symbol(ltf_n)); // STS0 LTF [+,-]
    append(a1, ofdm_symbol(apply_csd(lsig_f, C_L)));
    append(a1, ofdm_symbol(apply_csd(hs0_f, C_L)));
    append(a1, ofdm_symbol(apply_csd(hs1_f, C_L)));
    append(a1, ofdm_symbol(apply_csd(lstf_freq(), C_HT))); // HT-STF
    append(a1, ofdm_symbol(apply_csd(ltf_p, C_HT)));
    append(a1, ofdm_symbol(apply_csd(ltf_p, C_HT))); // STS1 LTF [+,+]

    // DATA: scramble -> BCC -> puncture -> stream-parse(2) -> per-stream interleave -> QAM.
    vb data_bits(nsym * n_dbps_tot, 0);
    vb pbits = bytes_to_bits(psdu, length);
    for (size_t i = 0; i < pbits.size(); i++)
        data_bits[16 + i] = pbits[i];
    vb scr = scramble(data_bits);
    for (int i = 0; i < 6; i++)
        scr[16 + 8 * length + i] = 0;
    vb coded = puncture(conv_encode(scr), m.rnum, m.rden);
    std::vector<vb> parts = stream_parse(coded, n_ss, m.n_bpsc, m.n_cbps);
    vcf* outs[2] = { &a0, &a1 };
    const int data_tcs[2] = { 0, C_HT };
    for (int ss = 0; ss < n_ss; ss++) {
        vb il = interleave_ht_ss(parts[ss], m.n_cbps, m.n_bpsc, ss);
        for (int j = 0; j < nsym; j++) {
            vb chunk(il.begin() + j * m.n_cbps, il.begin() + (j + 1) * m.n_cbps);
            append(*outs[ss], ofdm_symbol(apply_csd(
                                  mimo_data_freq(map_qam(chunk, m.n_bpsc), ss, j), data_tcs[ss])));
        }
    }
    return { a0, a1 };
}

// VHT-LTF (20 MHz) for NSS=2, spatial stream ss, LTF symbol ltf. Data tones are scaled by
// the VHT-LTF P-matrix C_P_LTF_VHT (stream0=[1,-1], stream1=[1,1]); the 4 pilot tones by
// C_R_LTF_VHT=[1,-1] -- so stream1's 2nd LTF has its pilot tones negated vs its data tones.
vcf vht_ltf_nss2_freq(int ss, int ltf)
{
    static const int Pm[2][2] = { { 1, -1 }, { 1, 1 } };
    static const int Rm[2] = { 1, -1 };
    vcf base = vhtltf20_freq();
    vcf f(64, cf(0, 0));
    for (int b = 4; b <= 60; b++) {
        bool pilot = (b == PILOT_SC[0] || b == PILOT_SC[1] || b == PILOT_SC[2] || b == PILOT_SC[3]);
        f[b] = base[b] * (float)(pilot ? Rm[ltf] : Pm[ss][ltf]);
    }
    return f;
}

// VHT20 NSS=2 (2 spatial streams, MCS 0-8 per stream, BCC). Mirrors build_mimo (HT) with
// VHT framing: VHT-SIG-A with NSTS=2, 2 VHT-LTF via the C_P/C_R LTF matrices, VHT-SIG-B on
// both streams, VHT DATA stream-parsed across 2 antennas. STS0 unshifted; STS1 gets CSD.
// The VHT DATA pilots are stream-independent (the same cycling {1,1,1,-1} x POLARITY[4+j]
// on both streams -- streams are separated by the VHT-LTF + CSD, not the pilots).
std::vector<vcf> build_vht_mimo(const uint8_t* mpdu, int mpdu_len, const tx_params& p)
{
    const mcs_row& m = VHT_MCS20[p.mcs];
    const int n_ss = 2;
    vb ampdu = ampdu_wrap(mpdu, mpdu_len);
    int alen = (int)ampdu.size();
    int n_dbps_tot = n_ss * m.n_dbps;
    int nsym = (16 + 8 * alen + 6 + n_dbps_tot - 1) / n_dbps_tot;
    int n_ltf = 2;
    const cf sigpil[4] = { cf(1, 0), cf(1, 0), cf(1, 0), cf(-1, 0) };
    const int C_L = -4, C_HT = -8;

    // L-SIG.
    int txtime = 20 + 8 + 4 + 4 * n_ltf + 4 + 4 * nsym;
    int lsig_len = 3 * ((txtime - 20 + 3) / 4) - 3;
    vcf lsig_f = data_symbol_freq(map_bpsk(sig_field(11, lsig_len)), DATA_SC_LEGACY, 48, 2,
                                  false, false);

    // VHT-SIG-A: B0-1 BW=20, reserved B2/B23/B33=1, NSTS-1=1 -> B10=1, MCS B28-31.
    vb a(48, 0);
    a[2] = 1;
    a[10] = 1; // NSTS - 1 = 1 (2 streams)
    a[23] = 1;
    a[33] = 1;
    for (int i = 0; i < 4; i++)
        a[28 + i] = (p.mcs >> i) & 1;
    uint8_t acrc = ht_sig_crc8(vb(a.begin(), a.begin() + 34));
    for (int i = 0; i < 8; i++)
        a[34 + i] = (acrc >> (7 - i)) & 1;
    vb a_il = interleave_legacy(conv_encode(a), 48, 1);
    vcf sa1_f = data_symbol_freq(map_bpsk(vb(a_il.begin(), a_il.begin() + 48)), DATA_SC_LEGACY,
                                 48, 3, false, false, sigpil);
    vcf sa2_f = data_symbol_freq(map_bpsk(vb(a_il.begin() + 48, a_il.end())), DATA_SC_LEGACY,
                                 48, 4, true, false, sigpil);

    // VHT-SIG-B (26 bits, 20 MHz); CRC-8 over B0-19 seeds DATA SERVICE[8:16].
    int apep = alen / 4;
    vb b(26, 0);
    for (int i = 0; i < 17; i++)
        b[i] = (apep >> i) & 1;
    b[17] = b[18] = b[19] = 1;
    uint8_t bcrc = ht_sig_crc8(vb(b.begin(), b.begin() + 20));
    vb b_il = interleave_ht(conv_encode(b), 52, 1, 20);
    vcf sb_f = data_symbol_freq(map_bpsk(b_il), DATA_SC_HT, 52, 7, false, false, sigpil);

    // DATA bits -> scramble -> BCC -> puncture -> stream-parse(2).
    vb data_bits(nsym * n_dbps_tot, 0);
    for (int i = 0; i < 8; i++)
        data_bits[8 + i] = (bcrc >> (7 - i)) & 1; // SERVICE[8:16]
    vb abits = bytes_to_bits(ampdu.data(), alen);
    for (size_t i = 0; i < abits.size(); i++)
        data_bits[16 + i] = abits[i];
    vb scr = scramble(data_bits);
    for (int i = 0; i < 6; i++)
        scr[16 + 8 * alen + i] = 0;
    vb coded = puncture(conv_encode(scr), m.rnum, m.rden);
    std::vector<vb> parts = stream_parse(coded, n_ss, m.n_bpsc, m.n_cbps);
    vb il0 = interleave_ht_ss(parts[0], m.n_cbps, m.n_bpsc, 0);
    vb il1 = interleave_ht_ss(parts[1], m.n_cbps, m.n_bpsc, 1);

    // VHT DATA symbol (stream-independent cycling pilots, VHT polarity start 4).
    static const int htp_base[4] = { 1, 1, 1, -1 };
    auto vht_data = [&](const vb& il, int j) {
        vb chunk(il.begin() + j * m.n_cbps, il.begin() + (j + 1) * m.n_cbps);
        vcf pts = map_qam(chunk, m.n_bpsc);
        vcf freq(64, cf(0, 0));
        for (int c = 0; c < 52; c++)
            freq[DATA_SC_HT[c]] = pts[c];
        float pol = POLARITY[(4 + j) % 127];
        for (int i = 0; i < 4; i++)
            freq[PILOT_SC[i]] = cf(htp_base[(i + j) % 4] * pol, 0);
        return freq;
    };

    // Assemble. STS0 (ant0) unshifted; STS1 (ant1) CSD: legacy fields C_L, VHT fields C_HT.
    vcf a0 = preamble(), a1 = preamble_csd(C_L);
    append(a0, ofdm_symbol(lsig_f));
    append(a0, ofdm_symbol(sa1_f));
    append(a0, ofdm_symbol(sa2_f));
    append(a0, ofdm_symbol(lstf_freq())); // VHT-STF
    append(a1, ofdm_symbol(apply_csd(lsig_f, C_L)));
    append(a1, ofdm_symbol(apply_csd(sa1_f, C_L)));
    append(a1, ofdm_symbol(apply_csd(sa2_f, C_L)));
    append(a1, ofdm_symbol(apply_csd(lstf_freq(), C_HT))); // VHT-STF
    for (int ltf = 0; ltf < 2; ltf++) {
        append(a0, ofdm_symbol(vht_ltf_nss2_freq(0, ltf)));
        append(a1, ofdm_symbol(apply_csd(vht_ltf_nss2_freq(1, ltf), C_HT)));
    }
    append(a0, ofdm_symbol(sb_f)); // VHT-SIG-B (same content on both streams)
    append(a1, ofdm_symbol(apply_csd(sb_f, C_HT)));
    for (int j = 0; j < nsym; j++) {
        append(a0, ofdm_symbol(vht_data(il0, j)));
        append(a1, ofdm_symbol(apply_csd(vht_data(il1, j), C_HT)));
    }
    return { a0, a1 };
}

std::vector<std::vector<cf>> build_frame(const uint8_t* psdu, int psdu_len,
                                         const tx_params& p)
{
    if (p.stbc == 0 && p.fec == TX_BCC && p.format == TX_HT && p.bw == 20 &&
        p.mcs >= 8 && p.mcs <= 15)
        return build_mimo(psdu, psdu_len, p);
    if (p.stbc != 0) {
        if (p.format != TX_HT || p.bw != 20 || p.fec != TX_BCC || p.mcs > 7)
            throw std::runtime_error("wifi_tx: STBC only HT20 BCC MCS0-7 (so far)");
        return build_stbc(psdu, psdu_len, p);
    }
    if (p.format == TX_VHT) {
        if (p.fec != TX_BCC)
            throw std::runtime_error("wifi_tx: VHT LDPC not supported (so far)");
        if (p.n_ss == 2) {
            if (p.bw != 20 || p.mcs > 8)
                throw std::runtime_error("wifi_tx: VHT NSS=2 only 20 MHz MCS0-8 (so far)");
            return build_vht_mimo(psdu, psdu_len, p);
        }
        if (p.bw == 20) {
            // VHT20 NSS=1: MCS 0-8 (MCS9 256-QAM r5/6 has non-integer n_dbps at 20 MHz).
            if (p.mcs > 8)
                throw std::runtime_error("wifi_tx: VHT20 MCS0-8 (so far)");
            return build_vht20(psdu, psdu_len, p);
        }
        if (p.bw == 40) {
            if (p.mcs > 9)
                throw std::runtime_error("wifi_tx: VHT40 MCS0-9");
            return build_vht40(psdu, psdu_len, p);
        }
        throw std::runtime_error("wifi_tx: VHT only 20/40 MHz (so far)");
    }
    if (p.mcs > 7)
        throw std::runtime_error("wifi_tx: SISO MCS0-7 only (so far)");
    if (p.fec == TX_LDPC) {
        if (p.format != TX_HT || p.bw != 20 || p.mcs != 0)
            throw std::runtime_error("wifi_tx: LDPC only HT20 MCS0 (so far)");
        return build_ht_ldpc(psdu, psdu_len, p);
    }
    if (p.fec != TX_BCC)
        throw std::runtime_error("wifi_tx: unknown FEC");
    if (p.format != TX_HT)
        throw std::runtime_error("wifi_tx: unsupported format");
    if (p.bw == 40)
        return build_ht40(psdu, psdu_len, p);
    if (p.bw != 20)
        throw std::runtime_error("wifi_tx: HT only 20/40 MHz");
    const mcs_row& m = HT_MCS20[p.mcs];
    int length = psdu_len;
    int nsym = (16 + 8 * length + 6 + m.n_dbps - 1) / m.n_dbps; // ceil
    int n_ltf = 1;

    vcf frame = preamble();

    // L-SIG (6 Mbps, spec length).
    int lsig_len = lsig_length_ht(nsym, n_ltf);
    append(frame, data_symbol(map_bpsk(sig_field(11, lsig_len)), DATA_SC_LEGACY, 48, 2,
                              false, false));

    // HT-SIG: 48 info bits (mcs[0:7], cbw20[7]=0, length[8:24], rest 0, crc[34:42]).
    vb hsig(48, 0);
    for (int i = 0; i < 7; i++)
        hsig[i] = (p.mcs >> i) & 1;
    for (int i = 0; i < 16; i++)
        hsig[8 + i] = (length >> i) & 1;
    // HT-SIG2 flag bits. Real Realtek silicon validates these -- the synthetic-
    // generator habit of leaving them 0 makes a frame the software RX accepts but
    // the chip rejects. Per 802.11-2016 19.3.9.4.3: B24 channel-smoothing=1,
    // B25 not-sounding=1, B26 reserved=1 (must be 1). B27 aggregation, B28-29 STBC,
    // B30 FEC, B31 SGI, B32-33 N-ESS stay 0 for HT20 SISO BCC long-GI no-aggregation.
    hsig[24] = 1; // channel smoothing
    hsig[25] = 1; // not sounding
    hsig[26] = 1; // reserved (must be 1)
    vb hsig34(hsig.begin(), hsig.begin() + 34);
    uint8_t crc = ht_sig_crc8(hsig34);
    for (int i = 0; i < 8; i++)
        hsig[34 + i] = (crc >> (7 - i)) & 1;
    vb hsig_il = interleave_legacy(conv_encode(hsig), 48, 1); // 96
    // QBPSK data, BPSK pilots (spec / chip-compat): rotate=true, pilot_rotate=false.
    vb h0(hsig_il.begin(), hsig_il.begin() + 48);
    vb h1(hsig_il.begin() + 48, hsig_il.end());
    append(frame, data_symbol(map_bpsk(h0), DATA_SC_LEGACY, 48, 3, true, false));
    append(frame, data_symbol(map_bpsk(h1), DATA_SC_LEGACY, 48, 4, true, false));

    // HT-STF (spec: the L-STF short sequence, NOT the L-LTF placeholder) + HT-LTF.
    append(frame, ofdm_symbol(lstf_freq()));
    append(frame, ofdm_symbol(htltf20_freq()));

    // HT DATA: SERVICE(16) + PSDU + pad -> scramble -> tail-reset -> BCC -> puncture
    // -> HT-interleave -> QAM -> OFDM symbols.
    vb data_bits(nsym * m.n_dbps, 0);
    vb pbits = bytes_to_bits(psdu, length);
    for (size_t i = 0; i < pbits.size(); i++)
        data_bits[16 + i] = pbits[i];
    vb scr = scramble(data_bits);
    for (int i = 0; i < 6; i++)
        scr[16 + 8 * length + i] = 0; // tail
    vb coded = puncture(conv_encode(scr), m.rnum, m.rden);
    vb il = interleave_ht(coded, m.n_cbps, m.n_bpsc, 20);
    // HT DATA pilots (802.11-2016 19.3.11.10): base pattern {1,1,1,-1} cyclically
    // rotated left by the data-symbol index, times the polarity sequence starting at
    // index 3 (the non-pilot HT-STF/HT-LTF symbols do NOT advance it). Real Realtek
    // tracks phase across the DATA on these pilots; a fixed/mis-indexed pattern (what a
    // decoder-targeted generator uses) makes the chip fail the long DATA field even
    // though the 2-symbol HT-SIG survives.
    const int htp_base[4] = { 1, 1, 1, -1 };
    for (int j = 0; j < nsym; j++) {
        vb chunk(il.begin() + j * m.n_cbps, il.begin() + (j + 1) * m.n_cbps);
        float pol = POLARITY[(3 + j) % 127];
        cf pil[4];
        for (int i = 0; i < 4; i++)
            pil[i] = cf(htp_base[(i + j) % 4] * pol, 0);
        append(frame, data_symbol(map_qam(chunk, m.n_bpsc), DATA_SC_HT, 52, 7 + j, false,
                                  false, pil));
    }

    return { frame };
}

} // namespace tx
} // namespace ieee802_11
} // namespace gr
