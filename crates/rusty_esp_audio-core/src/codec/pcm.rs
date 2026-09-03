//! Sample-format conversion with fixed, documented rules — the ones ffmpeg's
//! `swresample` uses, so the oracle test in `rusty_esp_audio-esp` can demand
//! byte identity. The conversion lives in `rusty_esp_dsp::sample::pcm` (moved
//! verbatim in D0, 2026-09-02, with its rule table and its tests); this
//! module keeps the path this crate always had.

pub use rusty_esp_dsp::sample::pcm::{convert, output_bytes};
