# rusty_esp_audio — mission plan

**One sentence:** ESP-ADF, ESP-GMF, `esp_codec_dev` and `esp_audio_codec`
remade in Rust — a fixed-block audio pipeline with elements that never
allocate, I2S/PDM capture and playback, codec-chip drivers as register data,
and PCM/ADPCM/FLAC framing through the Remade codecs — so a microphone or a
speaker on an ESP32 is a `PcmBlock` source or sink like any other.

Family plan: Janus `docs/plans/janus-mission.md`. Layer 1 · media. Depends on
`rusty_esp_core` only.

Written 2026-09-01. Status: **scaffold.**

---

## 1. Espressif map

| Espressif item | Job | Class | Janus |
|---|---|---|---|
| ESP-ADF `audio_pipeline`, `audio_element`, `ringbuf`, `i2s_stream`, `raw_stream`, `http_stream` | the graph | **REMAKE** | `Pipeline<const N>`, `Element`, `RingBuffer<const N>`, `AudioSource`/`AudioSink` |
| ESP-GMF (`gmf_core`, `gmf_elements`, `gmf_io`) | generic graph, ports, pools | fold into the same pipeline (rusty-ESP-arduino §3.1) | same |
| `esp_codec_dev` (ES8311, ES7210, ES8388, ES8374, ES7243, ZL38063 drivers; `audio_codec_ctrl_if`, `data_if`) | codec chips | REMAKE as register data + a `CodecChip` trait | `chip::{es8311, es7210, es8388}` over `embedded-hal` I²C |
| `esp_audio_codec` (encoders/decoders: PCM, G.711, ADPCM, AAC, Opus, ALAC, AMR) | framing | REMAKE the framing; codecs from the org | `codec::{pcm, adpcm_ima, wav, flac (feature), opus (later)}` |
| ESP-SR AFE (AEC, NS, VAD, AGC), wake word | front end | **lite** REMAKE: energy VAD, AGC, DC block, simple NS later | `elements::{EnergyVad, Agc, DcBlock}` — no wake word, no AEC in v1 |
| ESP-DSP FFT / IIR / FIR | kernels | REMAKE, scalar first | `dsp::{Biquad, Fir}` here until `rusty_esp_dsp` exists |
| I2S std / TDM / PDM RX, DAC, PWM audio | transport | **WRAP** | `-esp` backends |

## 2. Crate surface

### `rusty_esp_audio-core` (`no_std`, `forbid(unsafe)`)

```rust
pub trait Element {
    fn format(&self) -> PcmFormat;
    /// Process one block INTO `out`; returns the bytes produced (may be 0 or > in.len()).
    fn process(&mut self, input: PcmBlock, out: &mut [u8]) -> Result<usize>;
}
pub struct Pipeline<'e, const N: usize> { elements: [&'e mut dyn Element; N], /* scratch over caller memory */ }

pub struct RingBuffer<'m> { /* SPSC over &'m mut [u8]; push/pop whole frames */ }
pub trait AudioSource { fn format(&self) -> PcmFormat; fn read<'b>(&mut self, out: &'b mut [u8]) -> Result<PcmBlock<'b>>; }
pub trait AudioSink   { fn format(&self) -> PcmFormat; fn write(&mut self, block: PcmBlock) -> Result<()>; }

pub mod elements { Gain, DcBlock, Biquad(HighPass|LowPass|Peak), Agc, EnergyVad, Mixer, LinearResampler, Mono<->Stereo }
pub mod codec    { pcm (format conversion i16<->i24in32<->f32), adpcm_ima (encode/decode), wav (header writer),
                   flac (feature "flac": rusty_flac encoder over caller buffers) }
pub mod chip     { pub struct Register { addr: u8, value: u8 }
                   pub trait CodecChip { fn init(&mut self, cfg: &ChipConfig) -> Result<()>; fn set_volume(&mut self, db: i8) -> Result<()>; fn mute(&mut self, bool) -> Result<()>; }
                   pub mod es8311; pub mod es7210; pub mod es8388; }
```

Rules: blocks are fixed size and caller-owned; no element allocates; every
element has a host test against a reference (numpy/scipy for filters with a
stated tolerance, byte-identical for ADPCM/FLAC/format conversion).

### `rusty_esp_audio-esp`

| Feature | Backend |
|---|---|
| `esp-idf` | `I2sIn`/`I2sOut` (`esp-idf-hal` i2s std/TDM), `PdmIn` (S3 PDM RX), DAC out (classic ESP32) |
| `esp-hal` | `esp_hal::i2s` (behind `unstable`), PDM via I2S on S3, `embassy` async wrappers |

### `rusty_esp_audio` (facade)

`prelude` = core prelude + `Element`, `Pipeline`, `RingBuffer`, `AudioSource`, `AudioSink`, the elements.

## 3. House crates

| Need | Use | Status / work item |
|---|---|---|
| FLAC | `rusty_flac` 0.1.2 | zero deps, zero features, slice-in/caller-buffer-out API, no threads — **the closest codec to `no_std` in the portfolio**. Upstream PR: feature ladder, `core::fmt` errors, gate 13 `target_arch` sites. First codec on the chip. |
| Opus | `rusty-opus` 0.9.1 | 245 `unsafe`, 248 `target_arch`, thread pool: a libopus transpile. **Not viable on-chip short term.** Decision gate at A4: port, or transcode on the host. |
| Host playback | `rff` (`wav`, `flac`, `udp://` TS) | PCM/WAV over UDP first; the Pi hub records it |
| Never | ADF's C codecs, `esp_audio_codec`'s AAC | the host has `rff` |

## 4. Milestones and kill tests

| # | Deliverable | Kill test |
|---|---|---|
| **A0** | elements, `RingBuffer`, `Pipeline`, `adpcm_ima`, `wav`, format conversions, host tests with synthetic + recorded PCM fixtures | biquad/AGC/VAD within stated tolerance of a scipy reference; ADPCM round-trip byte-identical to a reference encoder; riscv32 checks green |
| **A1** (J2) | PDM mic on XIAO S3 Sense (Track A) → `EnergyVad` → PCM/WAV over UDP to a laptop or the Pi | the laptop plays camera + mic together; 10 minutes with the drop counter recorded |
| **A2** | ES8311 + ES7210 on Korvo-2 / S3-EYE; speaker out; loopback | mic → gain → speaker loopback with a measured latency; register tables re-derived and attributed |
| **A3** | `rusty_flac` `no_std` → FLAC blocks on-chip | on-chip FLAC bytes identical to the host encoder for the same PCM; decodes in `rff` |
| **A4** | Opus decision gate: measure `rusty-opus` scalar CELT on S3 vs budget | a ledger row with cycles/frame; go/no-go recorded |
| **A5** | Track B I2S/PDM via esp-hal; PIE twins for `Biquad`/`dot_i16` via `rusty_esp_dsp` | byte-identical to scalar; ceiling probe first |

## 5. Measurement

- Block counters, dropped-block counters and ring high-water marks before any
  clock.
- Quality: an external oracle (PEAQ on the host for any lossy path,
  byte-identity for lossless) — `codec-tune-quality` discipline; self-metrics
  never headline.
- `docs/LEDGER.md` from the first number.

## 6. Risks

| Risk | Mitigation |
|---|---|
| PDM filter on S3 gives I24In32 at odd rates | `PcmFormat` carries it; conversion elements are byte-exact and tested |
| Opus never fits | audio ships PCM/ADPCM/FLAC; the host transcodes; the gate is explicit at A4 |
| ADF users expect MP3/AAC on-device | non-goal; documented in the README |

## 7. Decision log

| Date | Decision |
|---|---|
| 2026-09-01 | Fixed-block, caller-owned pipeline; no per-block heap. |
| 2026-09-01 | First on-chip codecs: PCM, IMA-ADPCM, FLAC. Opus is a measured decision at A4. |
| 2026-09-01 | ESP-SR is remade as a **lite** front end (VAD/AGC/DC block); wake word and AEC are non-goals for v1; ASR/TTS live in FFAI on the host. |
