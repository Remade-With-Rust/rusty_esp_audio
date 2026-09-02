//! Framing and conversion: `esp_audio_codec`'s PCM/ADPCM remade, plus WAV
//! headers, plus FLAC through the house `rusty_flac` behind the `flac`
//! feature (the one allocating module). Lossy codecs are a measured decision
//! (A4).

pub mod adpcm_ima;
#[cfg(feature = "flac")]
pub mod flac;
pub mod pcm;
pub mod wav;
