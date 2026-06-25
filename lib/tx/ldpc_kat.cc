// Known-answer test: encode a fixed info pattern with each of the 12 IEEE 802.11 QC-LDPC
// codes and print "<name> <hex codeword>". Compared bit-exact against sdr2wifi/ldpc.py by
// scripts/ldpc_kat.sh. Build: g++ -I../../include -I.. lib/tx/ldpc_kat.cc -o ldpc_kat
#include "../ldpc_encoder.h"
#include <cstdio>
#include <vector>
using namespace gr::ieee802_11;

int main()
{
    struct { const char* name; int n, rn, rd; } codes[] = {
        { "R12_648", 648, 1, 2 },   { "R23_648", 648, 2, 3 },   { "R34_648", 648, 3, 4 },
        { "R56_648", 648, 5, 6 },   { "R12_1296", 1296, 1, 2 }, { "R23_1296", 1296, 2, 3 },
        { "R34_1296", 1296, 3, 4 }, { "R56_1296", 1296, 5, 6 }, { "R12_1944", 1944, 1, 2 },
        { "R23_1944", 1944, 2, 3 }, { "R34_1944", 1944, 3, 4 }, { "R56_1944", 1944, 5, 6 },
    };
    for (auto& c : codes) {
        LdpcEncoder enc(ldpc_code_for(c.n, c.rn, c.rd));
        int k = enc.k();
        std::vector<uint8_t> info(k), cw(enc.n());
        for (int i = 0; i < k; i++)
            info[i] = (uint8_t)((i * 7 + 3) % 2); // deterministic pattern
        enc.encode(info.data(), cw.data());
        printf("%s ", c.name);
        // pack the codeword MSB-first into hex nibbles
        for (int i = 0; i < enc.n(); i += 4) {
            int v = 0;
            for (int b = 0; b < 4 && i + b < enc.n(); b++)
                v |= cw[i + b] << (3 - b);
            printf("%x", v);
        }
        printf("\n");
    }
    return 0;
}
