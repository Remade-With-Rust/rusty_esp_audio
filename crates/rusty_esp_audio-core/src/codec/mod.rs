//! Framing and conversion: `esp_audio_codec`'s PCM/ADPCM remade, plus WAV
//! headers. Lossless codecs (FLAC) come from the Remade crates once they
//! are `no_std` (milestone A3); lossy ones are a measured decision (A4).

pub mod adpcm_ima;
pub mod pcm;
pub mod wav;
