// Minimal 802.11 QC-LDPC min-sum decoder (R=1/2, n=648) for the HT/VHT fec==1 path.
// Ported from the Python reference (sdr2wifi/ldpc.py), which validated all 12 codes
// bit-exact; this single-codeword R12_648 entry point is the first fork integration.
#ifndef INCLUDED_IEEE802_11_LDPC_MIN_SUM_H
#define INCLUDED_IEEE802_11_LDPC_MIN_SUM_H

#include <cmath>
#include <cstdint>
#include <vector>

namespace gr {
namespace ieee802_11 {

// R=1/2, n=648, Z=27 base matrix (IEEE 802.11-2016 Table F-1).
static const int LDPC_BASE_R12_648[12][24] = {
    { 0,-1,-1,-1, 0, 0,-1,-1, 0,-1,-1, 0, 1, 0,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1},
    {22, 0,-1,-1,17,-1, 0, 0,12,-1,-1,-1,-1, 0, 0,-1,-1,-1,-1,-1,-1,-1,-1,-1},
    { 6,-1, 0,-1,10,-1,-1,-1,24,-1, 0,-1,-1,-1, 0, 0,-1,-1,-1,-1,-1,-1,-1,-1},
    { 2,-1,-1, 0,20,-1,-1,-1,25, 0,-1,-1,-1,-1,-1, 0, 0,-1,-1,-1,-1,-1,-1,-1},
    {23,-1,-1,-1, 3,-1,-1,-1, 0,-1, 9,11,-1,-1,-1,-1, 0, 0,-1,-1,-1,-1,-1,-1},
    {24,-1,23, 1,17,-1, 3,-1,10,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0,-1,-1,-1,-1,-1},
    {25,-1,-1,-1, 8,-1,-1,-1, 7,18,-1,-1, 0,-1,-1,-1,-1,-1, 0, 0,-1,-1,-1,-1},
    {13,24,-1,-1, 0,-1, 8,-1, 6,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0,-1,-1,-1},
    { 7,20,-1,16,22,10,-1,-1,23,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0,-1,-1},
    {11,-1,-1,-1,19,-1,-1,-1,13,-1, 3,17,-1,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0,-1},
    {25,-1, 8,-1,23,18,-1,14, 9,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, 0, 0},
    { 3,-1,-1,-1,16,-1,-1, 2,25, 5,-1,-1, 1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1, 0},
};

// Decode one R=1/2 n=648 codeword from soft LLRs (llr>0 favours bit 0) into the 648
// hard bits. The first 324 columns are the (info+parity-systematic) bits per the
// Python gf2_systematic mapping; the caller extracts the info columns it needs. We
// decode the full codeword and return all 648 hard bits.
inline void ldpc_decode_648(const float* llr, uint8_t* out648, int iters = 50)
{
    const int Z = 27, M = 12 * Z, N = 24 * Z;
    static std::vector<std::vector<int>> chk_vars, var_chks;
    if (chk_vars.empty()) {
        chk_vars.assign(M, {});
        var_chks.assign(N, {});
        for (int br = 0; br < 12; br++)
            for (int bc = 0; bc < 24; bc++) {
                int s = LDPC_BASE_R12_648[br][bc];
                if (s < 0)
                    continue;
                for (int i = 0; i < Z; i++) {
                    int r = br * Z + i, c = bc * Z + (i + s) % Z;
                    chk_vars[r].push_back(c);
                    var_chks[c].push_back(r);
                }
            }
    }
    std::vector<std::vector<float>> msg(M);
    for (int r = 0; r < M; r++)
        msg[r].assign(chk_vars[r].size(), 0.0f);
    auto posof = [&](int r, int v) {
        for (size_t i = 0; i < chk_vars[r].size(); i++)
            if (chk_vars[r][i] == v)
                return (int)i;
        return -1;
    };
    std::vector<float> total(N);
    for (int it = 0; it < iters; it++) {
        for (int r = 0; r < M; r++) {
            int deg = chk_vars[r].size();
            std::vector<float> vc(deg);
            for (int i = 0; i < deg; i++) {
                int v = chk_vars[r][i];
                float s = llr[v];
                for (int r2 : var_chks[v])
                    if (r2 != r)
                        s += msg[r2][posof(r2, v)];
                vc[i] = s;
            }
            float sgn = 1.0f;
            for (int i = 0; i < deg; i++)
                sgn *= (vc[i] >= 0 ? 1.0f : -1.0f);
            for (int i = 0; i < deg; i++) {
                float mn = 1e18f;
                for (int j = 0; j < deg; j++)
                    if (j != i && std::fabs(vc[j]) < mn)
                        mn = std::fabs(vc[j]);
                msg[r][i] = sgn * (vc[i] >= 0 ? 1.0f : -1.0f) * mn;
            }
        }
        for (int v = 0; v < N; v++)
            total[v] = llr[v];
        for (int r = 0; r < M; r++)
            for (size_t i = 0; i < chk_vars[r].size(); i++)
                total[chk_vars[r][i]] += msg[r][i];
        for (int v = 0; v < N; v++)
            out648[v] = total[v] < 0 ? 1 : 0;
        bool ok = true;
        for (int r = 0; r < M && ok; r++) {
            int p = 0;
            for (int v : chk_vars[r])
                p ^= out648[v];
            if (p)
                ok = false;
        }
        if (ok)
            break;
    }
}

} // namespace ieee802_11
} // namespace gr

#endif
