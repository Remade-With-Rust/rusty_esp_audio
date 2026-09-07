# xiao-s3-sense-idf-pdm-udp

Janus **J2 / A1** firmware, Track A (std on ESP-IDF): the XIAO ESP32-S3 Sense
microphone → `rusty_esp_audio` (`PdmIn` → `DcBlock` → `EnergyVad`) → raw
s16le PCM over UDP, 20 ms per datagram, to a laptop or the Pi hub.

## Prerequisites (one-time, this machine)

- `espup install` (the `esp` Xtensa toolchain), `cargo install ldproxy espflash`
- The esp env in the shell: `LIBCLANG_PATH` to esp-clang's `libclang.dll`, the
  esp-clang and xtensa-esp-elf `bin` dirs on `PATH`
- Python 3 on `PATH` that is **not** a virtual environment (the IDF installer
  refuses to run from one)

On Windows the ESP-IDF build needs a very short target directory (esp-idf-sys
refuses long output paths), so set `CARGO_TARGET_DIR` to something like
`C:\janus-a`. The IDF tools install globally under `~/.espressif`. Because
the target dir then sits outside the project, `.cargo/config.toml` pins
`CARGO_WORKSPACE_DIR` so esp-idf-sys still reads this manifest.

## Two modes, chosen at build time

With a destination compiled in it streams; with none it measures. The arm is
picked by `option_env!`, so it is the same source either way and the first
line of the log says which ran.

```sh
export CARGO_TARGET_DIR=C:/janus-s3                # Windows only
```

**Bench (no network needed).** Nothing to set. No Wi-Fi is started at all,
which is what lets this row be answered on a bench with no access point, and
also means nothing contends for the CPU while the microphone is timed.

```sh
export JANUS_MIC_SECS=600                          # optional, default 10
cargo build --release
espino flash   --board xiao-esp32s3-sense --port COM4 --app $CARGO_TARGET_DIR/xtensa-esp32s3-espidf/release/xiao-s3-sense-idf-pdm-udp
espino monitor --board xiao-esp32s3-sense --port COM4 --expect "== DONE ==" --timeout 800 > soak.txt
python tools/decode-wav-dump.py soak.txt mic.wav   # ffprobe and ffmpeg judge it
```

**Stream (the J2 demo).**

```sh
export JANUS_WIFI_SSID=yournet JANUS_WIFI_PASS=yourpass
export JANUS_AUDIO_DEST=192.168.1.20:5004          # the laptop or Pi
cargo build --release
cargo run --release                                # espflash flash --monitor
```

On the laptop:

```sh
ffplay -f s16le -ar 16000 -ch_layout mono -i udp://0.0.0.0:5004
# or record it:
cargo run -p rusty_esp_audio-esp --features std --example pcm_record -- 0.0.0.0:5004 mic.wav 600
```

## The kill test (from the plan, A1)

The laptop plays the camera (J1, `http://<ip>/stream`) and the microphone
together; ten minutes on serial with the block count, the dropped-datagram
count and the VAD speech ratio. Numbers go to `rusty_esp_audio/docs/LEDGER.md`.

## Status: run on the board, 2026-09-06

Ten minutes on a XIAO ESP32-S3 Sense: **30 000 blocks, 50.000 per second,
16 000.0 samples per second against a nominal 16 000**, and zero short reads,
read errors or dropped blocks. The audio clock and the system clock agree to
**0.52 ppm** over the ten minutes. ffprobe reads the dumped capture back as
16 kHz mono s16le, and ffmpeg's level agrees with the decoder script's to
0.03 dB.

The VAD speech ratio was 0.469 over ten seconds and 0.235 over ten minutes in
the same room: a short capture measures the minute it was taken in, not the
room. Rows and method lines in `rusty_esp_audio/docs/LEDGER.md`.

Still open on A1: dropped datagrams, which needs a network.

Builds on 2026-09-01 against ESP-IDF v5.5.1: a 986,688-byte app image, 64 %
of the 1.5 MiB factory partition `partitions_singleapp_large.csv` gives an
8 MB part; 27 s to rebuild once the IDF is configured.

Two things the first build taught (mission plan §8): before building run
`cargo metadata --filter-platform=xtensa-esp32s3-espidf --format-version 1 >/dev/null`
once in this directory (unused umbrella `[patch]` rows otherwise make
esp-idf-sys's `cargo metadata --locked` fail and it silently ignores the
manifest; `.cargo/config.toml` pins the tools dir so it can at least never
clone an IDF into the project); and `git config --global core.longpaths true`
before the IDF submodules land under `~/.espressif`.

## Notes

- Raw datagrams carry no header on purpose: `ffplay`, ffmpeg and `rff` read
  them directly. Loss shows as a gap. The sequence-numbered transport comes
  with `janus/media/1` over iroh.
- Credentials and destination are compile-time here. `espino adopt` replaces
  them with the signed adoption record.
- The PDM front end is ESP-IDF's I2S PDM RX (esp-idf-hal 0.46); the
  application layer is Rust. The Track B version uses esp-hal's I2S.
