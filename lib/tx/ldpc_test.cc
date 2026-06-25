// Round-trip self-test for LdpcEncoder648: encode random info -> clean BPSK LLRs ->
// ldpc_decode_648 -> assert the recovered 648-bit codeword is bit-exact. If encode
// produced a non-codeword the min-sum decoder would settle on a different nearest
// codeword, so an exact match proves both (a) cw is a valid codeword and (b) the
// systematic info/parity column split is correct.
#include "../ldpc_encoder.h"
#include "../ldpc_min_sum.h"
#include <cstdio>
#include <cstdlib>

using namespace gr::ieee802_11;

int main()
{
    LdpcEncoder648 enc;
    int K = enc.k();
    printf("LDPC R12_648: K=%d (expect 324), info_cols=%zu par_cols=%zu\n", K,
           enc.info_cols.size(), enc.par_cols.size());
    if (K != 324 || enc.par_cols.size() != 324u) {
        printf("FAIL: bad dimensions\n");
        return 1;
    }
    srand(12345);
    int trials = 500, fails = 0, bit_errs = 0;
    for (int t = 0; t < trials; t++) {
        uint8_t info[324], cw[648], dec[648];
        for (int i = 0; i < K; i++)
            info[i] = (uint8_t)(rand() & 1);
        enc.encode(info, cw);
        float llr[648];
        for (int i = 0; i < 648; i++)
            llr[i] = cw[i] ? -4.0f : 4.0f; // clean: bit0 -> +llr, bit1 -> -llr
        ldpc_decode_648(llr, dec, 50);
        bool ok = true;
        for (int i = 0; i < 648; i++)
            if (dec[i] != cw[i]) {
                ok = false;
                bit_errs++;
            }
        if (!ok)
            fails++;
    }
    printf("round-trip encode->decode: %d/%d codewords exact (%d residual bit errors)\n",
           trials - fails, trials, bit_errs);
    return fails ? 1 : 0;
}
