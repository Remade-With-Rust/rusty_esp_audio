# rusty_esp_audio — mission plan

**One sentence:** ESP-ADF, ESP-GMF, `esp_codec_dev` and `esp_audio_codec`
remade in Rust — a fixed-block audio pipeline with elements that never
allocate, I2S/PDM capture and playback, codec-chip drivers as register data,
and PCM/ADPCM/FLAC framing through the Remade codecs — so a microphone or a
speaker on an ESP32 is a `PcmBlock` source or sink like any other.

Family plan: Janus `docs/plans/janus-mission.md`. Layer 1 · media. Depends on
`rusty_esp_core` only.

Written 2026-09-01. Status: **A0 shipped on the host; A1 host half done
(transport, PDM backend, firmware project) — the board is next.** Numbers in
`docs/LEDGER.md`.

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
// As shipped at A0 (2026-09-01):
pub trait Element {
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat>;      // may change rate/channels/encoding
    fn max_output_bytes(&self, input: PcmFormat, input_bytes: usize) -> usize; // default = input_bytes
    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize>; // whole frames, may be 0
    fn reset(&mut self) {}
}
pub struct Pipeline<'e, const N: usize> { /* [&mut dyn Element; N]; ping-pongs the two halves of ONE caller scratch */ }
//   process(input, scratch) -> Result<Option<PcmBlock<'scratch>>>; scratch_bytes(); output_format()

pub struct RingBuffer<'m> { /* whole-frame ring over &'m mut [u8]; push (Busy + drop count when full), push_overwrite, pop, pop_exact, high_water */ }
pub trait AudioSource { fn format(&self) -> PcmFormat; fn read<'b>(&mut self, out: &'b mut [u8]) -> Result<PcmBlock<'b>>; }
pub trait AudioSink   { fn format(&self) -> PcmFormat; fn write(&mut self, block: PcmBlock<'_>) -> Result<()>; }
pub mod source   { SineSource (test tone), CountingSink }

pub mod elements { Gain (Q15), DcBlock, Biquad (LowPass|HighPass|Peak|Notch, RBJ, DF1 f32), Agc, EnergyVad,
                   MonoToStereo, StereoToMono, mix_i16, LinearResampler (exact rational phase), Convert }
pub mod codec    { pcm (I16<->I24In32<->I32<->F32, ffmpeg rules), adpcm_ima (Encoder/Decoder, WAV layout),
                   wav (WavHeader write/parse: PCM, float, IMA) }
// Still to come: chip::{Register, CodecChip, es8311, es7210, es8388} at A2; flac at A3.
```

Every element takes interleaved i16 (the stateful ones up to 2 channels);
`Convert` and `codec::pcm` move between encodings at the edges.

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
| **A0** ✅ 2026-09-01 | elements, `RingBuffer`, `Pipeline`, `adpcm_ima`, `wav`, format conversions, host tests with synthetic PCM (recorded-speech fixtures still to add) | **passed:** biquads within 1 LSB of ffmpeg (f64 DF1) and scipy `lfilter`; IMA ADPCM **byte-identical to ffmpeg in both directions** (mono and stereo), PCM conversions byte-identical to swresample; AGC/VAD/DC-block/resampler property tests; riscv32 both rungs green; 45 + 3 + 6 tests. `docs/LEDGER.md` |
| **A1** (J2) ◐ host half 2026-09-01 | PDM mic on XIAO S3 Sense (Track A) → `DcBlock` → `EnergyVad` → raw s16le PCM over UDP to a laptop or the Pi. **Done on the host:** `net::{UdpPcmSender, UdpPcmReceiver}` (ffmpeg reads the datagrams straight off the socket), `wavfile::WavWriter` + `pcm_record` (ffprobe-verified), `tone_send`, `idf::PdmIn` over esp-idf-hal, the firmware project `firmware/xiao-s3-sense-idf-pdm-udp` **builds** for xtensa-esp32s3-espidf (ESP-IDF v5.5.1; 986,688 B image, 64 % of the 1.5 MiB factory partition) | the laptop plays camera + mic together; 10 minutes with the drop counter recorded — **needs the board** |
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
| 2026-09-01 | IMA ADPCM: the decoder is the IMA reference expansion (what ffmpeg and every player decode with); the encoder tracks the decoder by default (closed loop) and offers `ffmpeg_compatible()` for byte parity with `adpcm_ima_wav`, whose encoder uses a different prediction rule than its own decoder. Found by the oracle, not by reading. |
| 2026-09-01 | A1 transport is **raw s16le datagrams, one 20 ms block each, no header** — so `ffplay`/ffmpeg/`rff` play a device with no receiver code. Sequence numbers and timestamps come with `janus/media/1` over iroh (N-track), not here. |
| 2026-09-01 | `Element::output_format` replaces a static `format()`: elements may change rate, channels or encoding, and the pipeline sizes its two scratch halves from `max_output_bytes`. `RingBuffer` is single-owner (a mutex or a task split wraps it per track). |
