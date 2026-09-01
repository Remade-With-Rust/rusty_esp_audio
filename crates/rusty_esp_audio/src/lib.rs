#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
//! `rusty_esp_audio` — ESP-ADF / ESP-GMF / esp_codec_dev remade in Rust: I2S/PDM capture and playback, a fixed-block audio pipeline, codec-chip drivers, Opus/FLAC/ADPCM framing via the Remade codecs. Memory safe, no_std core.
//!
//! This is the facade: it re-exports the `no_std` core and exposes the
//! chip backends under [`esp`]. Depend on this crate; reach into the
//! sub-crates only when you are building a backend.
//!
//! Part of Janus (Remade With Rust). Plan: `docs/plans/rusty_esp_audio.md`.

pub use rusty_esp_audio_core::*;

/// Chip backends (`esp-hal` for Track B, `esp-idf` for Track A).
pub mod esp {
    pub use rusty_esp_audio_esp::*;
}

/// The names a sketch or firmware wants in scope.
pub mod prelude {
    pub use rusty_esp_audio_core::prelude::*;
}
