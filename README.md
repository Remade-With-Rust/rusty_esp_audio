# rusty_esp_audio

[![Remade With Rust](https://img.shields.io/badge/Remade%20With-Rust-000?logo=rust&logoColor=fff)](https://github.com/remade-with-rust) [![By Mata Network](https://img.shields.io/badge/by-Mata%20Network-5b2be0)](https://www.mata.network) [![crates.io](https://img.shields.io/crates/v/rusty_esp_audio.svg)](https://crates.io/crates/rusty_esp_audio) [![docs.rs](https://docs.rs/rusty_esp_audio/badge.svg)](https://docs.rs/rusty_esp_audio) [![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue)](https://github.com/Remade-With-Rust/rusty_esp_audio/blob/main/LICENSE-MIT)

Audio on an ESP32: a PDM microphone, WAV and FLAC writing, ADPCM, resampling,
voice activity detection, and raw PCM over the network. Pure Rust, no C, no
FFI, `no_std` by default. Every codec path is gated **byte-identical against
FFmpeg**, in both directions.

* **Byte-identical, both ways.** Our decoder over FFmpeg's blocks equals
  FFmpeg's own decode; FFmpeg's decode of our WAV equals ours. Not close —
  identical.
* **FLAC that FFmpeg reads.** Mono at level 5 compresses 64,000 bytes to 52,503
  (82.0%), stereo at level 8 to 110,348 of 128,000 (86.2%), each decoding
  byte-identically and probing as the format it claims to be.
* **Resampling with exact counts**, not approximate ones: 48 kHz to 16 kHz of
  480 frames gives exactly 160; 44.1 kHz to 16 kHz over a hundred blocks lands
  within one sample of the ideal.
* **A microphone that keeps time.** Ten minutes on the chip: **30,000 blocks,
  none short, dropped or errored**, at 50.000 blocks per second against a
  nominal 50.

## What has run on hardware

| what | measured |
|---|---|
| a ten-minute soak | **30,000 blocks, none lost**, 50.000 per second |
| the audio clock against the system clock | **0.52 parts per million** apart over ten minutes |
| audio over the network, ten minutes | **7,201 datagrams, none lost**, at the rate the board sent |
| the speech ratio, short window against long | **0.469 falling to 0.235** — the same room, the same threshold, nothing changed but the duration |

That last row is why the soak was worth running. Ten seconds sits comfortably
inside a single conversation, so a short capture does not estimate a room — it
measures the minute it was taken in. Any figure of that kind quoted from a
brief recording describes something narrower than it appears to.

**A known gap, measured and recorded:** the generated sketch's loop is paced by
its camera and reads one audio block per frame, so it sends twelve blocks a
second from a microphone producing fifty. Three quarters of the audio is
discarded at the source before any radio is involved.

Every number, with the run that produced it:
[`docs/LEDGER.md`](https://github.com/Remade-With-Rust/rusty_esp_audio/blob/main/docs/LEDGER.md).

## Using it

```rust
use rusty_esp_audio::prelude::*;

let mut mic = PdmIn::new(pins, PcmFormat::mono(16_000))?;
while let Some(block) = mic.read()? {     // borrowed, frame-aligned by construction
    if vad.speaking(&block) {
        wav.write(&block)?;               // or flac, or adpcm, or straight to UDP
    }
}
```

## Two tracks

| track | what it is | this crate |
|---|---|---|
| **A** | `std` on ESP-IDF — the PDM driver, the sockets, the writers | `rusty_esp_audio-esp --features esp-idf` |
| **B** | `no_std` on `esp-hal` — the codecs, the resampler, the detector | `rusty_esp_audio-core`, default |

## Part of Janus

**Janus** rebuilds the Espressif ESP32 and Arduino application portfolio as
independent, memory-safe Rust packages — so a hardware maker can ship a device
that the [MATA](https://www.mata.network) home computer discovers, catalogs honestly, adopts
under its own identity, and pays for. Ten packages, three layers, and the
dependency direction never reverses.

| layer | packages |
|---|---|
| **0 — the vocabulary** | [`rusty_esp_core`](https://crates.io/crates/rusty_esp_core) · [`rusty_esp_dsp`](https://crates.io/crates/rusty_esp_dsp) |
| **1 — the functions** | [`rusty_esp_image`](https://crates.io/crates/rusty_esp_image) · [`rusty_esp_video`](https://crates.io/crates/rusty_esp_video) · [`rusty_esp_audio`](https://crates.io/crates/rusty_esp_audio) · [`rusty_esp_signal`](https://crates.io/crates/rusty_esp_signal) · [`rusty_esp_mid`](https://crates.io/crates/rusty_esp_mid) · [`rusty_esp_iroh`](https://crates.io/crates/rusty_esp_iroh) |
| **2 — the surfaces** | [`rusty_esp_arduino`](https://crates.io/crates/rusty_esp_arduino) — the sketch facade · [`espino`](https://crates.io/crates/espino) — the maker's CLI |

Every package is host-verified against an external oracle and keeps a ledger
in which no number appears without the run that produced it. **Five of seven
device profiles have now run their kill tests on real silicon**, three of them
over a Wi-Fi network the board hosts itself.

Also check out the rest of [Remade With Rust](https://github.com/remade-with-rust) — including
[`rusty_alloc`](https://crates.io/crates/rusty_alloc), the pure-Rust rebuild of
mimalloc that these firmwares run on, and
[`rusty_jpeg`](https://crates.io/crates/rusty_jpeg), the JPEG engine behind the
camera path — and our sister project
[remade_ffmpeg_rs](https://github.com/Remade-With-Rust/remade_ffmpeg_rs), a ground-up Rust rebuild of FFmpeg.

## About Mata Network

[Mata Network](https://www.mata.network) builds sovereign, self-hostable infrastructure.
**Remade With Rust** is our open-source home for the permissively-licensed
building blocks that work depends on.

## License

MIT OR Apache-2.0, at your option. See [LICENSE-MIT](https://github.com/Remade-With-Rust/rusty_esp_audio/blob/main/LICENSE-MIT)
and [LICENSE-APACHE](https://github.com/Remade-With-Rust/rusty_esp_audio/blob/main/LICENSE-APACHE).
