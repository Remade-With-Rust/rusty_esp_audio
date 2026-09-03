# rusty_esp_audio

[![crates.io](https://img.shields.io/crates/v/rusty_esp_audio.svg)](https://crates.io/crates/rusty_esp_audio)
[![docs.rs](https://docs.rs/rusty_esp_audio/badge.svg)](https://docs.rs/rusty_esp_audio)
[![license](https://img.shields.io/badge/license-MIT%20OR%20Apache--2.0-blue.svg)](#license)

ESP-ADF / ESP-GMF / esp_codec_dev remade in Rust: I2S/PDM capture and playback, a fixed-block audio pipeline, codec-chip drivers, Opus/FLAC/ADPCM framing via the Remade codecs. Memory safe, no_std core.

Part of **Janus**, the Remade-With-Rust programme that rebuilds the Espressif
ESP32 and Arduino application portfolio in memory-safe Rust so hardware makers
can ship products that plug straight into the MATA home computer.

- This package's plan: [docs/plans/rusty_esp_audio.md](docs/plans/rusty_esp_audio.md)
- The family plan: Janus `docs/plans/janus-mission.md` (umbrella repo)

**Claims discipline:** this README makes no performance or capability claim that
is not backed by a test, a benchmark ledger entry, or a kill test recorded in the
plan. "Scaffold" means scaffold.

## Status

**A0 shipped on the host (2026-09-01); A1's host half done.** The fixed-block
pipeline, its elements and the PCM/ADPCM/WAV codecs pass 45 unit tests plus 6
external-oracle tests: IMA ADPCM is **byte-identical to ffmpeg in both
directions**, PCM conversions byte-identical to swresample, biquads within
1 LSB of ffmpeg and scipy, the VAD within 3 points of ffmpeg's
`silencedetect` on recorded speech. The Track A transport (raw PCM over UDP that
`ffplay` reads directly, WAV files ffprobe reads), the PDM backend and the
XIAO ESP32-S3 Sense firmware are written, and the firmware **builds**
(986,688 B image, 64 % of the factory partition). FLAC is in behind the
`flac` feature: `rusty_flac` went `no_std` upstream (PR #8) and our chunks
decode in ffmpeg to the exact source PCM. Nothing has run on a chip yet;
`docs/LEDGER.md` has every number.

**A2's host half (2026-09-02):** the ES8311 and ES7210 codec chips exist as
register data with their vendor sequences (bring-up, clocking for every
MCLK/rate pair in the tables, serial-port format and width, start, stop,
gain, mute) reproduced register for register against a fake I²C bus; the
moved kernels the pipeline leans on (PCM conversion, the dBFS meter) come
from `rusty_esp_dsp`. What waits for a board: the Korvo-2 / S3-EYE loopback
and its measured latency.

## What is in it

| module | what |
|---|---|
| `Element`, `Pipeline<N>` | ESP-ADF's element/pipeline: fixed blocks over two halves of one caller scratch buffer, zero heap |
| `RingBuffer` | whole-frame ring with drop counter and high-water mark |
| `elements` | `Gain`, `DcBlock`, `Biquad` (RBJ low/high/peak/notch), `Agc`, `EnergyVad`, mono↔stereo, `mix_i16`, `LinearResampler`, `Convert` |
| `codec::pcm` | I16 ↔ I24In32 ↔ I32 ↔ F32 with ffmpeg's rules |
| `codec::adpcm_ima` | IMA ADPCM encoder/decoder in the WAV block layout |
| `codec::wav` | RIFF/WAVE headers (PCM, float, IMA), write and parse |
| `chip` | codec chips as register data over `embedded-hal` I²C: `es8311` (mono ADC + DAC; 75-row clock table, bring-up / start / stop / format / volume / mic gain), `es7210` (4-channel ADC; 25-row table, mic select, TDM, gain, mute), `I2cRegs`, `CodecChip` — re-derived from Espressif's `esp-adf` drivers and attributed; the vendor sequences reproduced register for register against a fake bus |
| `codec::flac` (feature `flac`) | chunked FLAC streams through the house `rusty_flac` (`no-std` branch); ffmpeg decodes them to the exact source |
| `-esp` `net` (`std`) | `UdpPcmSender` / `UdpPcmReceiver`: raw s16le datagrams, `ffplay -f s16le -ar 16000 -ch_layout mono -i udp://0.0.0.0:5004` |
| `-esp` `wavfile` (`std`) | `WavWriter` (an `AudioSink`), `read_all` |
| `-esp` `idf::PdmIn` | Track A PDM microphone over esp-idf-hal 0.46 |
| examples | `tone_send` (a fake device), `pcm_record` (the Pi record path) |

## What it is

- A pure-Rust remake of the *application* layer Espressif ships in C for this
  function. Same job, same protocols and file formats, new code, permissive
  licence, `forbid(unsafe)` in the core.
- Track-agnostic: the core crate is `no_std + alloc` and knows nothing about
  ESP-IDF or `esp-hal`. Backends are thin and feature-gated.

## What it is not

- Not a rewrite of the radio PHY, the ROM, or Espressif's Wi-Fi/BT controller
  blob. Where the silicon must be touched, the `-esp` crate **wraps** the
  esp-rs HAL or ESP-IDF and says so.
- Not a fork of esp-hal, esp-radio, espflash or ESP-IDF. Those are dependencies.

## Layout

```text
crates/rusty_esp_audio          facade: re-exports + prelude; the crate you depend on
crates/rusty_esp_audio-core     no_std + alloc; forbid(unsafe); types, traits, algorithms
crates/rusty_esp_audio-esp      the WRAP crate: `esp-hal` (Track B) | `esp-idf` (Track A)
firmware/                per-chip example projects, excluded from the workspace
docs/plans/              the mission plan for this package
```

## Two tracks, one core

| Track | Feature | Runtime | Use when |
|---|---|---|---|
| **A** | `esp-idf` | `std` on ESP-IDF (FreeRTOS) | you need iroh, TLS, or a driver ESP-IDF has and esp-hal lacks |
| **B** | `esp-hal` | `no_std` + Embassy | the purity path; every driver upstream in esp-rs |

The core compiles on both and on the host, which is where its tests run.

## Build

```sh
cargo test --workspace                                   # host: the tests
cargo check -p rusty_esp_audio-core --no-default-features \
  --target riscv32imac-unknown-none-elf                  # ESP32-C6 class, no alloc
cargo check -p rusty_esp_audio-core --no-default-features --features alloc \
  --target riscv32imac-unknown-none-elf
```

Firmware examples (Xtensa needs `espup`; RISC-V works on stable) are built from
their own directories under `firmware/`.

## License

MIT OR Apache-2.0, at your option.
