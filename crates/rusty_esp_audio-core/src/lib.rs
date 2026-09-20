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
//! | [`chip`] | codec chips as register data over `embedded-hal` I²C: `es8311` (ADC + DAC), `es7210` (4-channel ADC) |

#[cfg(feature = "alloc")]
extern crate alloc;

pub use rusty_esp_core as esp_core;

pub mod chip;
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

/// Level of an interleaved i16 block in dBFS — `rusty_esp_dsp`'s reduction
/// (moved there in D0, 2026-09-02), at the path this crate always had.
#[cfg(not(feature = "pie-s3"))]
pub use rusty_esp_dsp::sample::{peak_abs_i16, rms_dbfs_i16};

#[cfg(feature = "pie-s3")]
pub use pie::{peak_abs_i16, rms_dbfs_i16};

/// The chip twins of the reductions this crate re-exports.
///
/// `rms_dbfs_i16` is the one that matters: the shipping PDM firmware calls
/// it once per captured block, right after `Pipeline::process`, and until
/// this module existed it ran the scalar while a measured −79.6% twin sat
/// unreachable in `rusty_esp_dsp-esp`.
///
/// Each function is `cfg`-switched on the TARGET, not on the feature, so a
/// host build of this crate with `pie-s3` on still runs the oracle and the
/// tests still mean what they say.
#[cfg(feature = "pie-s3")]
pub(crate) mod pie {
    /// Level of an interleaved i16 block in dBFS. See
    /// [`rusty_esp_dsp::sample::rms_dbfs_i16`], which this is gated against.
    #[must_use]
    pub fn rms_dbfs_i16(samples: &[u8]) -> f32 {
        #[cfg(target_arch = "xtensa")]
        {
            rusty_esp_dsp_esp::pie_s3::rms_dbfs_i16(samples)
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            rusty_esp_dsp::sample::rms_dbfs_i16(samples)
        }
    }

    /// Largest magnitude in an i16 block. See
    /// [`rusty_esp_dsp::sample::peak_abs_i16`], which this is gated against.
    #[must_use]
    pub fn peak_abs_i16(a: &[i16]) -> u16 {
        #[cfg(target_arch = "xtensa")]
        {
            rusty_esp_dsp_esp::pie_s3::peak_abs_i16(a)
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            rusty_esp_dsp::sample::peak_abs_i16(a)
        }
    }

    // ---- the element helpers -----------------------------------------
    //
    // Each returns `true` when it handled the block. The `pie-s3`-off twin
    // of each returns `false` from a `const`-foldable body, so the caller's
    // scalar arm is never dead code and no `unreachable_code` lint fires.

    /// Mono to stereo. [`crate::elements::MonoToStereo`] is the oracle.
    pub fn mono_to_stereo(src: &[i16], dst: &mut [i16]) -> bool {
        #[cfg(target_arch = "xtensa")]
        {
            rusty_esp_dsp_esp::pie_s3::mono_to_stereo_i16(src, dst);
            true
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            let _ = (src, dst);
            false
        }
    }

    /// Stereo to mono. [`crate::elements::StereoToMono`] is the oracle.
    pub fn stereo_to_mono(src: &[i16], dst: &mut [i16]) -> bool {
        #[cfg(target_arch = "xtensa")]
        {
            rusty_esp_dsp_esp::pie_s3::stereo_to_mono_i16(src, dst);
            true
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            let _ = (src, dst);
            false
        }
    }

    /// Saturating sum of two i16 streams. [`crate::elements::mix_i16`] is
    /// the oracle.
    pub fn mix(a: &[i16], b: &[i16], out: &mut [i16]) -> bool {
        #[cfg(target_arch = "xtensa")]
        {
            rusty_esp_dsp_esp::pie_s3::mix_i16(a, b, out);
            true
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            let _ = (a, b, out);
            false
        }
    }

    /// `(x * q15 + (1 << 14)) >> 15`, clamped. The twin restricts itself to
    /// `|q15| <= 32767` and hands anything louder back, so the caller must
    /// keep its own wide path.
    pub fn gain(src: &[i16], q15: i32, dst: &mut [i16]) -> bool {
        #[cfg(target_arch = "xtensa")]
        {
            if q15.unsigned_abs() <= 32767 {
                rusty_esp_dsp_esp::pie_s3::gain_i16(src, q15, dst);
                return true;
            }
            false
        }
        #[cfg(not(target_arch = "xtensa"))]
        {
            let _ = (src, q15, dst);
            false
        }
    }
}

/// The `pie-s3`-off twin of [`pie`]: every helper declines, so each element
/// takes the scalar arm it already had.
#[cfg(not(feature = "pie-s3"))]
pub(crate) mod pie {
    pub fn mono_to_stereo(_: &[i16], _: &mut [i16]) -> bool {
        false
    }
    pub fn stereo_to_mono(_: &[i16], _: &mut [i16]) -> bool {
        false
    }
    pub fn mix(_: &[i16], _: &[i16], _: &mut [i16]) -> bool {
        false
    }
    pub fn gain(_: &[i16], _: i32, _: &mut [i16]) -> bool {
        false
    }
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
    // One definition, in the crate every element already speaks. See
    // `rusty_esp_core::pcm::round_sat_i16` for why it truncates once and why
    // its final conversion is unchecked.
    rusty_esp_core::pcm::round_sat_i16(v)
}

#[cfg(test)]
mod tests {
    use super::*;

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
