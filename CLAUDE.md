# CLAUDE.md

Guidance for Claude Code (claude.ai/code) when working in this repository.

## What this is

A fork of **bastibl/gr-ieee802-11** that extends the **receive** path from legacy
802.11a/g/p to modern OFDM — 802.11n HT (20/40 MHz, MCS 0–15 incl. 2×2 MIMO),
802.11ac VHT (SU SISO, MCS 0–7), plus STBC and LDPC — while keeping bastibl's C++
GNU Radio block architecture and ecosystem. It is **RX-only for the modern formats**;
TX is still legacy. The point is *blind* decode: recover a frame's format / MCS /
coding / FCS from a capture, to verify what a transmitter actually emitted.

See `README.md` for the fork-vs-bastibl-vs-cloud9477 (GR-WiFi) comparison.

## Build

Standard GNU Radio 3.10 OOT module (depends on gr-foo). The companion repo
**sdr2wifi** pins exact dependency commits and provides a one-shot build:

```sh
mkdir build && cd build && cmake .. && make && sudo make install && sudo ldconfig
```

## Where each format is decoded

The RX chain is `sync_short` → `sync_long` → FFT → `frame_equalizer` →
`decode_mac`. Almost all the modern-format work lives in **one block**:
`lib/frame_equalizer_impl.cc`. The legacy 48-carrier stream path to `decode_mac`
is left untouched; the HT/VHT/STBC/LDPC/MIMO decode runs *alongside* it inside the
equalizer and prints its results (and an FCS check) to stdout.

- **`sniff_ht_sig()`** — the format-detect seam, run at OFDM symbol 4 (after L-SIG).
  Re-equalizes the two stashed raw HT-SIG symbols off the L-LTF channel and tries
  HT-SIG (CRC-8); on failure tries VHT-SIG-A. Dispatches to the right `*_begin`.
- **HT-SIG two-axis trick** — real Realtek HT-SIG / VHT-SIG-A carry BPSK pilots +
  QBPSK data, so after pilot derotation the data lands on the **imaginary** axis.
  `try_ht_sig` is tried on `eq` then on `eq * -j` (the `eq_q` path). The synthetic
  generator's default lands data on the real axis, so the `eq_q` path is only
  exercised by real frames (or sdr2wifi's `--mode ht_spec`). Mind this when testing.
- **`ht_begin` / `ht_data_symbol` / `ht_finish`** — HT20/HT40 SISO data decode.
- **`mimo_*`** — 2×2 MIMO (MCS 8–15): P-matrix channel from the 2 HT-LTFs, MMSE
  stream separation, stream-deparse → BCC → FCS.
- **`stbc_*`** — Alamouti combine (1 SS → 2 STS), HT20 SISO.
- **`try_vht_sig` / `vht_read_sig_b` / `vht_begin`** — VHT-SIG-A (mixed BPSK/QBPSK),
  VHT-SIG-B length, VHT data.
- **LDPC** — `lib/ldpc_min_sum.h` (`ldpc_decode_648`, R=1/2 n=648 min-sum). Selected
  by the `fec` bit; `try_ht_sig` no longer gates on `fec==0`.

## Over-the-air gotchas (the real-RF fixes)

Both were "0 decodes over the air, fine in simulation" bugs:

- **`sync_short.cc` `MIN_GAP`** — in COPY state, sync re-triggers a new frame start
  when the L-STF plateau fires again and `d_copied > MIN_GAP`. An HT/VHT frame has a
  **second** short-training field (HT-STF) at L-STF+560 samples; with the old gap
  (480) it re-triggered mid-packet and mis-windowed the frame. Legacy (no HT-STF) was
  unaffected. `MIN_GAP` is 700 (past the HT-STF); a real next frame is a full PPDU away.
- **`equalize_ht_sig` sampling ramp** — the per-subcarrier ramp used
  `eps0 = CFO / f_carrier`, which assumes the TX and RX share a reference oscillator.
  That's false when an **SDR captures an independent transmitter** (the B210 sample
  clock and the chip LO are unrelated), so the ramp injected a spurious linear phase
  that flipped HT-SIG band-edge bits (the CRC-8 has no FEC margin). It is dropped for
  the HT-SIG re-equalization; the true SFO is negligible over 3–4 symbols.

## Testing

- **Synthetic, no SDR**: the asserting regression lives in the **sdr2wifi** repo
  (`scripts/run-tests.sh`) — per-format gates (legacy / HT20 / HT20-spec imag-axis /
  HT40 / VHT20 / STBC / LDPC) plus bit-exact `--selftest`s for MIMO/STBC/LDPC. Run it
  after any decode change.
- **Replay a capture**: `sdr2wifi/tools_replay_iq.py --in cap.cf32 --bw 20e6`
  (`--debug` enables the gated HT-SIG sniff diagnostic in `sniff_ht_sig`).
- **Over the air**: sdr2wifi drives a B210 to capture a real devourer transmission
  and decode it; HT/VHT SIG fields decode to the transmitted MCS on a quiet channel.

## Conventions

- Keep bastibl's architecture and the legacy path intact; add modern decode beside it.
- A decode result must end in a real FCS/CRC check — never a fixed-payload match.
- The synthetic generator and this decoder share math, so a synthetic round-trip only
  proves self-consistency. Confirm against **real** captures and an *independent*
  reference (GR-WiFi as TX, or the kernel driver) before claiming a format works.
