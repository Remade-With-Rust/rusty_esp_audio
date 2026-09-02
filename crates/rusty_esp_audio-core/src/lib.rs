#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
//! `rusty_esp_audio-core` — the pure heart of `rusty_esp_audio`.
//!
//! ESP-ADF's `audio_pipeline` / `audio_element` / `ringbuf`, `esp_audio_codec`'s
//! PCM/ADPCM framing and the lite front end of ESP-SR, remade as fixed-block
//! elements over caller-owned memory.
//!
//! Rules this crate lives by (from the Janus mission plan):
//!
//! 1. `no_std` by default; `alloc` is a feature, never an assumption.
//! 2. No drivers, no HAL types, no `esp-*` crate, no allocator. Backends live
//!    in `rusty_esp_audio-esp`.
//! 3. Every type that crosses to another Janus package comes from
//!    `rusty_esp_core`, so packages compose without conversions.
//! 4. Blocks and buffers are **borrowed over caller-owned memory**; no element
//!    allocates, ever — not even at construction.
//! 5. `forbid(unsafe)`. The scalar path is the oracle; any faster path is gated
//!    byte-identical against it.
//!
//! Layout:
//!
//! | module | contents |
//! |---|---|
//! | [`source`] | `AudioSource` / `AudioSink`, a test tone, a counting sink |
//! | [`ring`] | `RingBuffer`: whole-frame SPSC ring over a caller slice |
//! | [`pipeline`] | `Element` and `Pipeline<N>`: the fixed-block graph |
//! | [`elements`] | gain, DC block, RBJ biquads, AGC, energy VAD, channel ops, linear resampler, format conversion |
//! | [`codec`] | `pcm` conversions, `adpcm_ima` (IMA ADPCM, WAV layout), `wav` headers |

#[cfg(feature = "alloc")]
extern crate alloc;

pub use rusty_esp_core as esp_core;

pub mod codec;
pub mod elements;
pub mod pipeline;
pub mod ring;
pub mod source;

pub use pipeline::{Element, Pipeline};
pub use ring::RingBuffer;
pub use source::{AudioSink, AudioSource};

/// The names a sketch or firmware wants in scope.
pub mod prelude {
    pub use rusty_esp_core::prelude::*;

    pub use crate::codec::pcm::convert as convert_pcm;
    pub use crate::elements::{
        Agc, AgcConfig, Biquad, BiquadKind, Convert, DcBlock, EnergyVad, Gain, LinearResampler,
        MonoToStereo, StereoToMono, VadConfig,
    };
    pub use crate::pipeline::{Element, Pipeline};
    pub use crate::ring::RingBuffer;
    pub use crate::source::{AudioSink, AudioSource, CountingSink, SineSource};
}

/// Crate version, for capability manifests and logs.
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Level of an interleaved i16 block in dBFS (RMS over all channels).
/// Digital silence returns `-120.0`.
#[must_use]
pub fn rms_dbfs_i16(samples: &[u8]) -> f32 {
    let mut acc: i64 = 0;
    let mut n: i64 = 0;
    for s in samples.chunks_exact(2) {
        let v = i64::from(i16::from_le_bytes([s[0], s[1]]));
        acc += v * v;
        n += 1;
    }
    if n == 0 || acc == 0 {
        return -120.0;
    }
    // Exact in f64 up to 2^53; the final division and log are the only rounding.
    let mean = acc as f64 / n as f64;
    let rms = libm::sqrt(mean) / 32768.0;
    (20.0 * libm::log10(rms)) as f32
}

/// Write an `i16` sample as two little-endian bytes.
#[inline]
pub(crate) fn put_i16(out: &mut [u8], v: i16) {
    let b = v.to_le_bytes();
    out[0] = b[0];
    out[1] = b[1];
}

/// Read a little-endian `i16`.
#[inline]
pub(crate) fn get_i16(b: &[u8]) -> i16 {
    i16::from_le_bytes([b[0], b[1]])
}

/// Saturate an `i32` into `i16`.
#[inline]
pub(crate) fn sat16(v: i32) -> i16 {
    v.clamp(i32::from(i16::MIN), i32::from(i16::MAX)) as i16
}

/// Round an `f32` to the nearest `i16`, ties away from zero, saturating.
#[inline]
pub(crate) fn round_sat16(v: f32) -> i16 {
    let r = libm::roundf(v);
    if r >= 32767.0 {
        i16::MAX
    } else if r <= -32768.0 {
        i16::MIN
    } else {
        r as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn dbfs_of_full_scale_square_is_zero() {
        let mut buf = [0u8; 64];
        for (i, s) in buf.chunks_exact_mut(2).enumerate() {
            put_i16(s, if i % 2 == 0 { 32767 } else { -32767 });
        }
        let db = rms_dbfs_i16(&buf);
        assert!(db.abs() < 0.001, "{db}");
        assert_eq!(rms_dbfs_i16(&[0u8; 8]), -120.0);
        assert_eq!(rms_dbfs_i16(&[]), -120.0);
    }

    #[test]
    fn rounding_and_saturation() {
        assert_eq!(round_sat16(0.4), 0);
        assert_eq!(round_sat16(0.5), 1);
        assert_eq!(round_sat16(-0.5), -1);
        assert_eq!(round_sat16(40000.0), i16::MAX);
        assert_eq!(round_sat16(-40000.0), i16::MIN);
        assert_eq!(sat16(70000), i16::MAX);
        assert_eq!(sat16(-70000), i16::MIN);
    }
}
