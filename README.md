Hi!

This an IEEE 802.11 a/g/p transceiver for GNU Radio that is fitted for operation
with Ettus N210s and B210s. Interoperability was tested with many off-the-shelf
WiFi cards and IEEE 802.11p prototypes. The code can also be used in
simulations.

---

# This fork: modern-OFDM RX decode + TX (HT / VHT / LDPC / STBC)

This is a fork of **[bastibl/gr-ieee802-11](https://github.com/bastibl/gr-ieee802-11)**
that extends both the **receive** and the **transmit** path from legacy 802.11a/g/p
to modern OFDM, while keeping bastibl's proven C++ block architecture (`sync_short` →
`sync_long` → `frame_equalizer` → `decode_mac` on RX) and the surrounding ecosystem
(gr-foo, the Wireshark connector, the `.grc` flow graphs). The RX side blind-decodes a
captured frame to recover its format / MCS / coding / FCS; the TX side builds
spec/chip-compatible HT/VHT/LDPC frames that a real Realtek RTL8812AU receives over the
air. The legacy a/g/p TX chain is untouched.

**What this fork adds on RX (validated to CRC/FCS):**

- **802.11n HT**: HT20 and HT40 SISO (MCS 0–7), HT20 2×2 **MIMO** (MCS 8–15, MMSE).
- **802.11ac VHT**: SU SISO, 20 MHz, NSS=1, MCS 0–7 (VHT-SIG-A + VHT-SIG-B + VHT data).
- **STBC** (Alamouti, 1 SS → 2 STS) and **LDPC** (the `fec=1` path, R=1/2 n=648).
- **Real-RF fixes** so the above decode over the air, not just in simulation:
  `sync_short` no longer re-triggers on the HT/VHT mid-frame short-training field
  (HT-STF), and the HT-SIG re-equalization drops a sampling-offset ramp that is
  invalid when an SDR captures an independent (non-co-referenced) transmitter.

Validated end-to-end against real Realtek silicon (RTL8812AU via the
[devourer](https://github.com/OpenIPC/devourer) userspace driver) captured on a
B210: HT and VHT SIG fields blind-decode to the transmitted MCS. The synthetic
test harness and the over-the-air bring-up tooling live in a companion repo,
**[sdr2wifi](https://github.com/josephnef/sdr2wifi)**.

**Not (yet) decoded:** 256-QAM (VHT MCS 8–9), 40/80/160 MHz VHT, NSS > 1 VHT,
HT MCS > 15.

## Modern-format TX (the `frame_builder` block)

The TX side is one new C++ block, **`ieee802_11::frame_builder`** (mirroring the RX's
single-block design). It takes a PSDU as a PDU message on port `in` and emits the
finished time-domain frame as a tagged-stream burst of `gr_complex` (a `packet_len`
tag), ready for a USRP or file sink. The DSP core (`lib/tx/wifi_tx.cc`) is decoupled
from GNU Radio and reused by a standalone generator (`lib/tx/tx_gen.cc`).

Getting a frame that *real silicon* accepts (not just a lenient software RX) needed a
specific recipe — the same two bugs sink every "decoder-targeted" generator:
SIG-field **reserved bits must be 1** (HT-SIG B24–26; VHT-SIG-A), and the DATA
**pilots must cycle** (base pattern rotated by the data-symbol index × the polarity
sequence at the right start index). Plus per-format details: spec HT-STF/VHT-STF (the
L-STF, not a placeholder), VHT-SIG-B CRC seeding the DATA SERVICE field, the spec L-SIG
length, and — found empirically against the chip — **no LDPC tone mapping at 20 MHz**
(D_TM = 1, in-order).

**Chip-validated over the air** (B210 TX → RTL8812AU via [devourer](https://github.com/OpenIPC/devourer),
decoded with FCS = 0 at the expected DESC_RATE/bw/ldpc):

- **HT20** SISO BCC, MCS 0–7
- **HT40** SISO BCC, MCS 0–7 (needs a 40 MHz monitor channel on the RX side)
- **VHT20** SU SISO BCC, MCS 0–7
- **HT20 LDPC**, MCS 0 (R=1/2 n=648; header-only GF(2) systematic encoder in
  `lib/ldpc_encoder.h`, round-trip tested against the in-tree min-sum decoder)
- **HT20 STBC**, MCS 0–7 (1 SS → 2 STS Alamouti, transmitted from two synchronized
  B210 channels). The chip's STBC convention was reverse-engineered by transmitting
  STBC *with the chip itself* and separating its two TX streams from a 2-channel B210
  capture — that's how the HT-LTF P-matrix bug (`[[1,1],[-1,1]]` vs the standard
  `[[1,-1],[1,1]]`) was found.
- **HT20 2×2 MIMO**, MCS 8–15 (two spatial streams, stream parser + per-stream
  interleaver + per-stream pilots, standard HT-LTF P-matrix). Antenna 0 is
  sample-identical to GR-WiFi's stream 0; over the air the RTL8812AU's two RX antennas
  separate the streams and decode at the expected MCS with FCS = 0.

So the full modern-format TX set — **HT20/HT40/VHT20 BCC, HT20 LDPC, HT20 STBC, and
HT20 2×2 MIMO** — is over-the-air validated against real Realtek silicon.

### TX limitations (current)
- **STBC / 2×2 MIMO produce two antenna streams**, transmitted over the B210's two
  synchronized TX channels (see `sdr2wifi`'s `rf_tx_air2.py`). The `frame_builder`
  block takes an `n_tx` parameter — set it to 2 to emit both antennas (output 0/1 →
  USRP sink ch0/ch1). Antenna 1 (STS1) is cyclic-shifted (CSD) per 802.11 — legacy
  fields −200 ns, HT fields −400 ns — matching GR-WiFi's `procCSD`.
- **LDPC TX** is **HT20 MCS 0 only** so far (single R=1/2 n=648 codeword; the
  multi-codeword / 1296 / 1944 / shorten-puncture-repeat selection and 40 MHz LDPC
  tone mapping are not yet wired up).
- **VHT TX** covers 20/40 MHz, NSS = 1–2, MCS 0–9 (256-QAM included). 80/160 MHz are not
  yet wired up. (256-QAM and 40 MHz at high MCS need a high link SNR to decode over the air
  — VHT20 MCS8, VHT40 MCS0 and VHT20 NSS=2 MCS0 are chip-validated; the modulation is
  bit-exact to GR-WiFi.)
- The RX, the chip-correct TX and the sdr2wifi synthetic generator all use the 802.11
  spec cycling DATA pilots and the standard STBC HT-LTF P-matrix, so the TX↔RX self-loop
  decodes HT/HT40/VHT/STBC/LDPC-HT DATA. Decoding a *real chip* HT40/VHT frame's SIG
  fields end-to-end still needs the HT40 HT-SIG sniff to accept the spec reserved bits
  and the VHT-SIG-B decode to match silicon — a pending RX-SIG follow-up.

## How this relates to the two reference projects

| | **bastibl/gr-ieee802-11** (upstream) | **this fork** | **cloud9477/gr-ieee80211** (“GR-WiFi”) |
|---|---|---|---|
| Lineage | original | fork of bastibl | independent, from scratch (not a bastibl fork) |
| Language | C++ GNU Radio OOT | same | Python PHY (`phy80211`) + GNU Radio |
| Formats | 802.11a/g/p (legacy) | + HT/VHT (SU)/LDPC/STBC **RX**; HT20/HT40/VHT20/LDPC/STBC **TX** (all chip-validated) | 802.11a/n/ac, spec-compliant **TX + RX** |
| Focus | a working legacy transceiver | blind RX decode + chip-compatible TX for driver/PHY **completeness testing** | a complete, spec-faithful soft-PHY |
| Weight | light | light (minimal extension of upstream) | heavier / more complete |

If you want a complete, spec-compliant 802.11 soft-radio (full TX and RX, all
NSS/bandwidths), use **GR-WiFi**. If you want a light extension of the classic bastibl
C++ transceiver that blind-decodes HT/VHT frames off the air *and* transmits
chip-compatible HT/VHT/LDPC frames to check a driver/PHY end to end, this fork is for
you. GR-WiFi also makes an excellent *independent* reference TX for cross-checking this
fork's RX (its PHY shares no code with this one).

See [`CLAUDE.md`](CLAUDE.md) for the internal architecture and where each format is
decoded.

---

# Table of Contents
1. [Development](#development)

1. [Installation](#installation)

1. [Usage](#usage)

1. [Troubleshooting](#troubleshooting)

1. [Further information](#further-information)

# Development

Like GNU Radio, this module uses *maint* branches for development.
These branches are supposed to be used with the corresponding GNU Radio
branches. This means: the *maint-3.7* branch is compatible with GNU Radio 3.7,
*maint-3.8* is compatible with GNU Radio 3.8, etc.


# Installation


## Dependencies


### GNU Radio

There are several ways to install GNU Radio. You can use

- [pybombs](http://gnuradio.org/redmine/projects/pybombs/wiki)
- [pre-compiled binaries](http://gnuradio.org/redmine/projects/gnuradio/wiki/BinaryPackages)
- [from source](http://gnuradio.org/redmine/projects/gnuradio/wiki/InstallingGRFromSource)


### gr-foo

I have some non project specific GNU Radio blocks in my gr-foo repo that are
needed. For example the Wireshark connector. You can find these blocks at
[https://github.com/bastibl/gr-foo](https://github.com/bastibl/gr-foo). They are
installed with the typical command sequence:

    git clone https://github.com/bastibl/gr-foo
    cd gr-foo
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig


## Installation of gr-ieee802-11

To actually install the blocks do

    git clone https://github.com/bastibl/gr-ieee802-11
    cd gr-ieee802-11
    mkdir build
    cd build
    cmake ..
    make
    sudo make install
    sudo ldconfig

### Adjust Maximum Shared Memory
Since the transmitter is using the Tagged Stream blocks it has to store a
complete frame in the buffer before processing it. The default maximum shared
memory might not be enough on most Linux systems. It can be increased with

    sudo sysctl -w kernel.shmmax=2147483648


### OFDM PHY

The physical layer is encapsulated in a hierarchical block to allow for a
clearer transceiver structure in GNU Radio Companion. This hierarchical block is
not included in the installation process. You have to open
```/examples/wifi_phy_hier.grc``` with GNU Radio Companion and build it. This
will install the block in ```~/.grc_gnuradio/```.


### Check message port connections

Sometime the connections between the message ports (the gray ones in GNU Radio
Companion) break. Therefore, please open the flow graphs and assert that
everything is connected. It should be pretty obvious how the blocks are supposed
to be wired. Actually this should not happen anymore, so if your ports are still
unconnected please drop me a mail.


### Python OpenGL

If you want to run the receive demo (the one that plots the subcarrier
constellations), please assert that you have python-opengl installed. The nongl
version of the plot does not work for me.


### Run volk_profile

volk_profile is part of GNU Radio. It benchmarks different SIMD implementations
on your PC and creates a configuration file that stores the fastest version of
every function. This can speed up the computation considerably and is required
in order to deal with the high rate of incoming samples.


### Calibrate your daughterboard

If you have a WBX, SBX, or CBX daughterboard you should calibrate it in order to
minimize IQ imbalance and TX DC offsets. See the [application
notes](http://files.ettus.com/manual/page_calibration.html).


# Checking your installation

As a first step I recommend to test the ```wifi_loopback.grc``` flow graph. This
flow graph does not need any hardware and allows you to ensure that the software
part is installed correctly. So open the flow graph and run it. If everything
works as intended you should see some decoded packets on the console.


## Troubleshooting

If GRC complains that it can't find some blocks (other than performance counters
and hierarchical blocks) like

    >>> Error: Block key "ieee802_11_ofdm_mac" not found in Platform - grc(GNU Radio Companion)
    >>> Error: Block key "foo_packet_pad" not found in Platform - grc(GNU Radio Companion)

Most likely you used a different ```CMAKE_INSTALL_PREFIX``` for the module than
for GNU Radio. Therefore, the blocks of the module ended up in a different
directory and GRC can't find them. You have to tell GRC where these blocks are
by creating/adding to your ```~/.gnuradio/config.conf``` something like

    [grc]
    global_blocks_path = /opt/local/share/gnuradio/grc/blocks
    local_blocks_path = /Users/basti/usr/share/gnuradio/grc/blocks

But with the directories that match your installation.


# Usage


## Simulation

The loopback flow graph should give you an idea of how simulations can be
conducted. To ease use, most blocks have debugging and logging capabilities that
can generate traces of the simulation. You can read about the logging feature
and how to use it on the [GNU Radio
Wiki](https://wiki.gnuradio.org/index.php/Logging).


## Unidirectional communication

As first over the air test I recommend to try ```wifi_rx.grc``` and
```wifi_tx.grc```. Just open the flow graphs in GNU Radio companion and execute
them. If it does not work out of the box, try to play around with the gain. If
everything works as intended you should see similar output as in the
```wifi_loopback.grc``` example.


## Ad Hoc Network with WiFi card

- The transceiver is currently connected to a TAP device, i.e. is a virtual
  Ethernet interface. Therefore, we have no WiFi signaling like association
  requests and hence, the transceiver can not "join" an ad hoc network. You have
  to make some small changes to the kernel in order to convince you WiFi card to
  send to this hosts nevertheless.
- The transceiver can not respond to ACKs in time. This is kind of an
  architectural limitation of USRP + GNU Radio since Ethernet and computations
  on a normal CPU introduce some latency. You can set the number of ACK retries
  to zero and handle retransmits on higher layers (-> TCP).
- RTS/CTS is not working for the same reason. You can however just disable this
  mechanism.
- Currently, there is no CSMA/CA mechanism, but this can be implemented on the
  FPGA.


# Troubleshooting

- Please check compile and installation logs. They might contain interesting
  information.
- Did you calibrate your daughterboard?
- Did you run volk_profile?
- Did you try different gain settings?
- Did you close the case of the devices?
- Did you try real-time priority?
- Did you compile GNU Radio and gr-ieee802-11 in release mode?
- If you see warnings that ```blocks_ctrlport_monitor_performance``` is missing
  that means that you installed GNU Radio without control port or performance
  counters. These blocks allow you to monitor the performance of the transceiver
  while it is running, but are not required. You can just delete them from the
  flow graph.
- The message

    You must now use ifconfig to set its IP address. E.g.,
    $ sudo ifconfig tap0 192.168.200.1

is normal and is output by the TUN/Tap Block during startup. The configuration
of the TUN/TAP interface is handled by the scripts in the ```apps``` folder.
- Did you try to tune the RF frequency out of the band of interest (i.e. used
  the LO offset menu of the flow graphs)?
- If 'D's appear, it might be related to your Ethernet card. Assert that you
  made the sysconf changes recommended by Ettus. Did you try to connect you PC
  directly to the USRP without a switch in between?


# Further information

For further information please checkout our project page
[https://www.wime-project.net](https://www.wime-project.net)

For information and an implementation surrounding IEEE 802.11ah, please refer to [gr-ieee802-11ah](https://github.com/irongiant33/gr-ieee802-11ah).