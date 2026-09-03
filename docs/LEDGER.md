# rusty_esp_audio — ledger

Every number a README or plan claims lives here first, with how it was taken.
Discipline: external oracle before self-metric; byte-identity where the maths
is exact, a stated tolerance where it is not; nothing on a chip is claimed
until a chip ran it.

## A0 — the pipeline on the host (2026-09-01)

Machine: Windows 11, Rust 1.98.0 stable, ffmpeg/ffprobe 8.1, scipy 1.17.1 on
numpy 2.4.6 (`python` on `PATH`). Tests: `cargo test --workspace` and
`cargo test -p rusty_esp_audio-esp --features std`.

| gate | result |
|---|---|
| unit tests, `rusty_esp_audio-core` | **45** pass |
| unit tests, `rusty_esp_audio-esp` (`std`: UDP loopback, WAV file) | **3** pass |
| external-oracle tests, `tests/audio_oracle.rs` | **6** pass (below) |
| clippy `--all-targets -D warnings`, `cargo fmt --check` | clean |
| `riscv32imac-unknown-none-elf` core-only, `riscv32imafc-unknown-none-elf` + `alloc` | check green |

### IMA ADPCM against ffmpeg (`adpcm_ima_wav`)

Signal: two tones plus LCG noise, 16 kHz, 8 blocks of ffmpeg's 1024-byte
blocks (2041 frames mono, 1017 frames stereo).

| claim | mono | stereo |
|---|---|---|
| `Encoder::ffmpeg_compatible` output == `ffmpeg -c:a adpcm_ima_wav` data chunk | **byte-identical**, 16 328 frames | **byte-identical**, 8 136 frames |
| `Decoder` over ffmpeg's blocks == ffmpeg's own decode | **byte-identical** | **byte-identical** |
| ffmpeg decodes our WAV (our header + default `Encoder` blocks) == our `Decoder` | **byte-identical** | **byte-identical** |
| decoded SNR, default closed-loop encoder vs ffmpeg's encoder | 27.52 dB vs 27.48 dB | 21.84 dB vs 21.78 dB |

What the probe found: ffmpeg's *decoder* expands with the IMA reference
shift-sum (`step>>3 + …`) while its *encoder* predicts with
`((2n+1)·step)>>3`, so ffmpeg's encoder drifts a unit or two from every
decoder. Our default encoder tracks the decoder's rule (closed loop); the
ffmpeg rule is one call away for byte parity. The SNR gain is small and
positive on this signal (+0.04 / +0.06 dB) — recorded, not headlined.

### PCM conversions against ffmpeg (`swresample`)

s16→s32, s32→s16, s16→f32, f32→s16 over 16 000 samples, plus 8 000 random
floats in ±1.3 (rounding ties-even and clipping): **byte-identical**.

### Biquads against ffmpeg and scipy

Signal: 1 s of the tone-plus-noise mix at 16 kHz. Ours: RBJ design in f64
rounded to f32, direct form I in f32, output rounded ties-away. ffmpeg:
`highpass`/`lowpass`/`equalizer` with `a=di:r=f64`. scipy: the same RBJ
coefficients in f64 through `signal.lfilter`, rounded and clipped the same way.

| filter | vs ffmpeg (f64 DF1) | vs scipy (f64) |
|---|---|---|
| high-pass 300 Hz, Q 0.7071 | max 1 LSB, 269 / 16 000 samples differ | max 1 LSB, 269 differ |
| low-pass 3400 Hz, Q 0.7071 | max 1 LSB, 5 differ | max 1 LSB, 5 differ |
| peak +6 dB at 1 kHz, Q 1 | max 1 LSB, 45 differ | max 1 LSB, 45 differ |

The f32 state costs at most one LSB against a double reference on this
material. Frequency-response checks (unit tests): Butterworth corner at
−3.01 ± 0.15 dB, two octaves out at least −23 dB, peak +6 ± 0.1 dB, notch
below −40 dB.

### Transport

| claim | result |
|---|---|
| `ffmpeg -f s16le -i udp://…` captures 0.5 s from `UdpPcmSender` | **pass**: the 8 000 captured samples are a contiguous run of the stream |
| `UdpPcmReceiver` → `WavWriter` → `ffprobe` | **pass**: `pcm_s16le,16000,1,8000` for 25 datagrams / 16 000 bytes |
| loopback: 640-byte block → 3 datagrams at a 301-byte ceiling; 3 840-byte stereo block → 4 datagrams at 1 200 | pass |

### VAD and AGC on recorded speech, against ffmpeg `silencedetect`

`tests/audio_oracle.rs::recorded_speech_vad_agrees_with_ffmpeg_silencedetect`
takes any recording through `JANUS_SPEECH_WAV` (ffmpeg normalises it to
16 kHz mono first; nothing is vendored). ffmpeg's `silencedetect=noise=-45dB:d=0.2`
gives the reference speech fraction; ours is `EnergyVad` at −45 dBFS with a
200 ms hangover over 20 ms blocks; the AGC runs the voice defaults from 0 dB.

| clip (local, FFai bench fixtures) | ffmpeg speech fraction | `EnergyVad` | AGC speech-block level, median / p90 |
|---|---:|---:|---:|
| real speech, 9.1 s (mistral.rs `sample_speech.wav`, 44.1 k) | 0.973 | 0.998 (452/453) | −25.9 / **−20.0** dBFS |
| espeak Harvard sentence `hvd-01-01`, 2.2 s (22.05 k) | 1.000 | 0.972 (106/109) | −28.2 / **−20.0** dBFS |
| espeak Harvard sentence `hvd-02-06`, 2.0 s (22.05 k) | 1.000 | 0.970 (98/101) | −28.4 / **−20.0** dBFS |

The VAD agrees with ffmpeg within 3 points on every clip. The AGC holds the
loud blocks (p90) at the target to within 0.05 dB on every clip; the median
sits 6–8 dB under it because per-block speech levels swing 20 dB and the
release is a deliberate 6 dB/s — on the 2 s clips it has not finished
settling, which is why the test judges p90 only on clips of 6 s or more.

### Elements measured by property tests

| element | measured |
|---|---|
| `Gain` | Q15 rounding: half rounds up, unity byte-exact, +20 dB saturates |
| `DcBlock` (20 Hz) | 8 000 DC → residual under 400 after 250 ms; 1 kHz tone level change < 0.09 dB |
| `Agc` (voice defaults) | −43 dBFS tone reaches −20 ± 0.6 dBFS in 6 s (6 dB/s release); −3.7 dBFS tone reaches −20 ± 0.6 in 1 s (60 dB/s attack); −73 dBFS input freezes the gain; +30 dB cap holds |
| `EnergyVad` | −24 dBFS tone → speech; 3-block hangover exact; −53 dBFS → silence |
| `LinearResampler` | 16 k→48 k ramp interpolated exactly; 48 k→16 k of 480 frames gives exactly 160; 44.1 k→16 k over 100 blocks gives 15 999–16 000 (no drift) |
| `StereoToMono` / `MonoToStereo` / `mix_i16` | floor average, byte-exact duplication, saturating sum |
| `RingBuffer` | wrap, drop counting on full (`Busy`), overwrite of oldest, high-water mark |

### FLAC through `rusty_flac` (feature `flac`), against ffmpeg

`rusty_flac` on its `no-std` branch (PR #8: `no_std` + `alloc`, `libm`
feature for deterministic math), through `codec::flac::FlacEncoder`: 2 s
chunks of the tone-plus-noise signal, 20 ms blocks pushed one at a time.

| chunk | PCM → FLAC | ffmpeg decode == source | our decode == source | two encodes identical | ffprobe |
|---|---:|---|---|---|---|
| mono, level 5 | 64 000 B → 52 503 B (82.0 %) | **byte-identical** | byte-identical | yes | `flac,16000,1,32000` |
| stereo, level 8 | 128 000 B → 110 348 B (86.2 %) | **byte-identical** | byte-identical | yes | `flac,16000,2,32000` |

The ratios are poor because the test signal is a third noise by amplitude;
FLAC is lossless and this is what noise costs. `riscv32imac` check with
`--features alloc,flac`: green. `rff` was not run (not built on this machine);
ffmpeg is the stronger oracle anyway. This is the host half of the J2 FLAC
kill test; the chip half (a chunk from the board byte-identical to the host
encoder's, both with `libm`) needs the board.

## First Track A firmware build (2026-09-01)

`firmware/xiao-s3-sense-idf-pdm-udp` (`PdmIn` → `Pipeline[DcBlock]` →
`EnergyVad` → `UdpPcmSender`, Wi-Fi via esp-idf-svc 0.52) builds for
`xtensa-esp32s3-espidf` with ESP-IDF **v5.5.1**, `cargo build --release` on
this Windows box (esp toolchain from `espup`, cargo 1.97 nightly):

| quantity | value |
|---|---:|
| app image (`espflash save-image --chip esp32s3`) | **986,688 B** |
| factory partition, `partitions_singleapp_large.csv` (8 MB flash) | 1,536,000 B → **64.2 %** used |
| `.flash.text` / `.flash.rodata` | 738,900 B / 141,320 B |
| `.iram0.text` | 84,323 B |
| static DRAM (`.dram0.data` + `.dram0.bss`) | 20,744 B + 17,000 B |
| rebuild after the IDF is configured (Rust crates + link) | 27 s |

No PSRAM configured (audio does not need it). Not measured: RAM at run time,
blocks per second, dropped datagrams, the VAD ratio — all need the board.
The first build attempt exposed two more Windows walls (mission plan §8):
the fresh project had no `Cargo.lock` yet, so esp-idf-sys fell back to its
defaults and cloned a second IDF into the project; and git hit the 260-char
path limit checking out IDF submodules under `~/.espressif`.

## Not yet measured

- **Anything running on a chip.** Blocks per second, dropped datagrams and
  the VAD ratio over ten minutes need the board (A1).
- `LinearResampler` aliasing (it is linear interpolation; a sinc resampler is
  a later element when a rate change matters for quality).
- `Agc`/`EnergyVad` on a *corpus* of speech (three clips so far, above).
- FLAC on the chip (A3, waits on `rusty_flac` `no_std`); Opus (A4 gate).

## A2 host half: codec chips as data (host, 2026-09-02)

| Gate | Result |
|---|---|
| ES8311 bring-up (`Config::slave(16_000)`) against a fake I²C bus: the 28 register writes of Espressif's `es8311_codec_init` in its order with its values, including the 16 kHz / 4.096 MHz clock row expanded through the driver's arithmetic | **pass** |
| ES8311 master mode, MCLK from BCLK, both clocks inverted, PDM mic at 8 kHz: the bits the driver sets (`REG00` 0xC0, `REG01` 0xFF, `REG02` multiplier ×4, `REG06` bit 5) | pass |
| ES8311 format / width / start / stop / mute / mic gain / volume: `SDPIN`/`SDPOUT` bit fields, the start's power-up writes, the suspend sequence, `DAC_REG31` mute bits, `ADC_REG16`, the half-dB volume map (`0xBF` = 0 dB, `0x5B` = −50 dB as the vendor comment says) | pass |
| ES8311 unsupported (MCLK, rate) pair refused before any write; chip id read from `REGFD/REGFE` | pass |
| ES7210 bring-up (`Config::slave(16_000)`): the vendor `es7210_adc_init` head verbatim, then mic select (inputs 1 + 2 powered, PGA on at 24 dB, ADC12 clocks on, no TDM) | pass |
| ES7210 four mics in master mode from the doubler: TDM on, every PGA at 30 dB, both ADC pairs powered | pass |
| ES7210 format / width / start / stop / mute; `Module::Dac` refused (no DAC); the clock register the stop replaces with `0x7F` restored by the next start | pass |
| Clock tables: 75 ES8311 rows and 25 ES7210 rows, every row's dividers in range | pass |
| ES8388 bring-up (`Config::slave()`): the 30 register writes of Espressif's `es8388_init` in its order with its values, ADC volume 0 dB included | **pass** |
| ES8388 master mode, output pair 2, microphone 1: `MASTERMODE`, `DACPOWER`, `ADCCONTROL2` | pass |
| ES8388 start / stop: the `CHIPPOWER` state-machine pulse only when `DACCONTROL21` changes (none after init, one after a stop), ADC and DAC power, mute bit; the DAC-only start leaves the ADC down | pass |
| ES8388 format and width on both converters, the MCLK : LRCK ratio codes (256 → 2, 384 → 3, 1500 → 27, 300 refused before any write), volume (`0x64` = −50 dB as the vendor comment says), mic PGA (24 dB → `0x88`, 10 dB refused), output and input selects, line bypass on and off with its pulse | pass |

Unit tests: **58 pass** in the core (14 of them for `chip`), esp 3 + 7 oracle
unchanged; clippy `-D warnings`, `riscv32imac` no_std check, `cargo deny`.
No latency number: the loopback needs the board.

## The no-panic gate (host, 2026-09-02)

Every parser that takes bytes from a wire, a store or a bus must return an
error on bad input, never panic — the house rule made a test:
`tests/no_panic.rs` feeds each one random inputs from an LCG (the same corpus
on every machine) and mutations of a valid encoding (bit flips, overwrites,
truncation, extension, insertion, removal), under `catch_unwind` so a failure
names the parser and prints the input.

| covered | result |
|---|---|
| `WavHeader::parse` (40 000: random and mutations of a valid PCM16 header), `adpcm_ima::Decoder::decode_block` (20 000 blocks across channel and block-size combinations), `flac::decode_pcm16` (4 000, `--features flac`) | **one finding, guarded and filed:** a 42-byte FLAC header claiming billions of samples made `rusty_flac::decode` reserve ~80 GB from `STREAMINFO.total_samples` and abort the process (not a panic — an allocation failure); `decode_pcm16` now refuses a count no stream of that length could hold before calling the decoder, and the decoder's own allocation is [rusty_flac#9](https://github.com/Remade-With-Rust/rusty_flac/issues/9) |
