# rusty_esp_audio-core

[![Remade With Rust](https://img.shields.io/badge/Remade%20With-Rust-000?logo=rust&logoColor=fff)](https://github.com/remade-with-rust) [![By Mata Network](https://img.shields.io/badge/by-Mata%20Network-5b2be0)](https://www.mata.network) [![crates.io](https://img.shields.io/crates/v/rusty_esp_audio-core.svg)](https://crates.io/crates/rusty_esp_audio-core) [![docs.rs](https://docs.rs/rusty_esp_audio-core/badge.svg)](https://docs.rs/rusty_esp_audio-core) [![License: MIT OR Apache-2.0](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue)](https://github.com/Remade-With-Rust/rusty_esp_audio/blob/main/LICENSE-MIT)

The pure half of the audio package: WAV, FLAC and ADPCM framing, a linear resampler, voice activity detection, and the fixed-block pipeline they feed — with no drivers, no allocator and no product types in it. `no_std` + `alloc`, `forbid(unsafe)`.

Every codec path here is gated **byte-identical against FFmpeg in both directions**: our decode of their blocks equals their decode, and their decode of our file equals ours.

## Where the evidence is

This crate is part of [`rusty_esp_audio`](https://crates.io/crates/rusty_esp_audio). The
hardware results, the method lines and the open defects live in that package's
[README](https://github.com/Remade-With-Rust/rusty_esp_audio#readme) and in
[`docs/LEDGER.md`](https://github.com/Remade-With-Rust/rusty_esp_audio/blob/main/docs/LEDGER.md), where no number
appears without the run that produced it.

## On an ESP32-S3

Turn on `pie-s3` and the elements run the chip's 128-bit vector twins instead
of their scalar arms — same bytes out, gated byte-identical, and on any other
target the scalar IS what runs, so the feature is safe to leave on.

```toml
rusty_esp_audio-core = { version = "0.1", features = ["pie-s3"] }
```

Measured through the element, on a Seeed XIAO ESP32-S3 Sense:

| call site | scalar | vector | |
|---|---:|---:|---:|
| `peak_abs_i16` | 78,259 | 10,272 | **−86.9%** |
| `mix_i16` | 108,364 | 16,783 | **−84.5%** |
| `Convert` integer pairs | 193,418 | 29,240 | **−84.9%** |
| `rms_dbfs_i16` | 215,142 | 38,312 | **−82.2%** |
| `Gain::process` | 134,920 | 27,896 | **−79.3%** |
| `StereoToMono::process` | 96,008 | 42,897 | **−55.3%** |
| `MonoToStereo::process` | 55,259 | 27,165 | **−50.8%** |

(picoseconds per sample; lower is better)

`rms_dbfs_i16` is the one that pays for itself: a capture loop calls it once
per block, right after `Pipeline::process`.

Two arms are deliberately NOT accelerated, and the source says why rather
than leaving it to inference. `Biquad`, `DcBlock` and the AGC filter are
sequential — `y[n]` depends on `y[n-1]` — and `LinearResampler` gathers at
fractional positions, which this unit has no instruction for.

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
| **2 — the surfaces** | [`rusty_esp_arduino`](https://crates.io/crates/rusty_esp_arduino) — the sketch facade · `espino` — the maker's CLI (not published) |

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
