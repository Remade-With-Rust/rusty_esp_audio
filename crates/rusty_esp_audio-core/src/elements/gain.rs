//! `Gain`: fixed-point Q15 gain with rounding and saturation.

use rusty_esp_core::error::Result;
use rusty_esp_core::pcm::{PcmBlock, PcmFormat};

use super::{require_i16_any, require_room};
use crate::pipeline::Element;
use rusty_esp_core::pcm::{as_i16, as_i16_mut};

use crate::{get_i16, put_i16};

/// Multiply every sample by a fixed gain.
///
/// The gain is a Q15 integer, so `y = (x * g + 2^14) >> 15` (round half up)
/// then saturate. Unity is `32768` and is byte-identical passthrough. The
/// largest representable gain is about +30 dB (`i32::MAX >> 15`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Gain {
    q15: i32,
}

impl Gain {
    /// Unity gain.
    pub const UNITY: Gain = Gain { q15: 1 << 15 };

    /// A gain from a linear factor (negative factors invert).
    #[must_use]
    pub fn linear(factor: f32) -> Self {
        let q = libm::roundf(factor * 32768.0);
        let q15 = if q >= 65536.0 * 512.0 {
            65536 * 512
        } else if q <= -65536.0 * 512.0 {
            -65536 * 512
        } else {
            q as i32
        };
        Gain { q15 }
    }

    /// A gain from a raw Q15 factor, clamped to the representable range.
    ///
    /// The oracle test needs to reach an exact `q15` -- including the values
    /// either side of where the product stops fitting `i32` -- which going
    /// through `linear` cannot promise.
    #[must_use]
    pub const fn from_q15(q15: i32) -> Self {
        Gain {
            q15: if q15 > 65536 * 512 {
                65536 * 512
            } else if q15 < -65536 * 512 {
                -65536 * 512
            } else {
                q15
            },
        }
    }

    /// A gain from decibels.
    #[must_use]
    pub fn from_db(db: f32) -> Self {
        Self::linear(libm::powf(10.0, db / 20.0))
    }

    /// The Q15 factor in use.
    #[must_use]
    pub const fn q15(&self) -> i32 {
        self.q15
    }

    /// Apply this gain to one sample.
    #[inline]
    #[must_use]
    pub fn apply(&self, x: i16) -> i16 {
        let y = (i64::from(x) * i64::from(self.q15) + (1 << 14)) >> 15;
        // ONE clamp, not two. The old form clamped to the `i32` range and then
        // `sat16` clamped that to `i16` -- and `i16` is a subset of `i32`, so
        // the first clamp could never change an outcome the second did not.
        // Same value for every input; two fewer 64-bit compares per sample.
        y.clamp(-32768, 32767) as i16
    }
}

/// `(x * g + 2^14) >> 15` over aligned samples, sixteen a trip.
///
/// The product is exact in `i32` here: the caller has checked
/// `|g| <= 65 535`, and `|x| <= 32 768`, so `|x * g| <= 2^31 - 2^15` and the
/// rounding term still fits.
fn gain_narrow(src: &[i16], g: i32, dst: &mut [i16]) {
    // SIXTEEN. Thirty-two measured +100.5% against a 0.0% null arm on an
    // ESP32-S3 (2026-09-19) -- the SAME cliff the byte arm hits at the same
    // width, so the register window, not the access method, is what sets it.
    let mut ci = src.chunks_exact(16);
    let mut co = dst.chunks_exact_mut(16);
    for (i, o) in ci.by_ref().zip(co.by_ref()) {
        for k in 0..16 {
            let y = (i32::from(i[k]) * g + (1 << 14)) >> 15;
            o[k] = y.clamp(-32768, 32767) as i16;
        }
    }
    for (i, o) in ci.remainder().iter().zip(co.into_remainder().iter_mut()) {
        let y = (i32::from(*i) * g + (1 << 14)) >> 15;
        *o = y.clamp(-32768, 32767) as i16;
    }
}

/// The same over the gains whose product does NOT fit `i32`.
fn gain_wide(src: &[i16], g: i32, dst: &mut [i16]) {
    for (i, o) in src.iter().zip(dst.iter_mut()) {
        let y = (i64::from(*i) * i64::from(g) + (1 << 14)) >> 15;
        *o = y.clamp(-32768, 32767) as i16;
    }
}

impl Element for Gain {
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
        require_i16_any(input)?;
        Ok(input)
    }

    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
        require_i16_any(input.format)?;
        require_room(out, input.data.len())?;
        if self.q15 == 1 << 15 {
            out[..input.data.len()].copy_from_slice(input.data);
            return Ok(input.data.len());
        }
        // FAST ARM: native halfword loads. `as_i16` settles the alignment
        // once per call; `None` sends a misaligned or big-endian buffer down
        // the byte path below, which stays the oracle.
        let n = input.data.len();
        if let (Some(src), Some(dst)) = (as_i16(input.data), as_i16_mut(&mut out[..n])) {
            if self.q15.unsigned_abs() < 65536 {
                gain_narrow(src, self.q15, dst);
            } else {
                gain_wide(src, self.q15, dst);
            }
            return Ok(n);
        }

        // How wide the product has to be is a property of the GAIN, which is
        // fixed for the whole call -- not of the sample. With |x| <= 32768 and
        // |g| <= 65535 the product is at most 2^31 - 2^15, so the rounding term
        // still fits and `i32` is EXACT: same bytes out, one `mull` in place of
        // the 64x64 sequence. Above that the wide path is the only correct one,
        // so the test is hoisted out of the loop rather than run per sample.
        let g = self.q15;
        if g.unsigned_abs() < 65536 {
            let n = input.data.len();
            // SIXTEEN. Thirty-two measured +115.3% against a 1.4% null arm on
            // an ESP32-S3 (2026-09-19) -- the working set stops fitting the
            // register window and every lane starts spilling. Unroll width has
            // a cliff, not a plateau, and this is where this body's sits.
            let mut ci = input.data.chunks_exact(32);
            let mut co = out[..n].chunks_exact_mut(32);
            for (i, o) in ci.by_ref().zip(co.by_ref()) {
                for k in 0..16 {
                    let y = (i32::from(get_i16(&i[k * 2..])) * g + (1 << 14)) >> 15;
                    put_i16(&mut o[k * 2..], y.clamp(-32768, 32767) as i16);
                }
            }
            for (i, o) in ci
                .remainder()
                .chunks_exact(2)
                .zip(co.into_remainder().chunks_exact_mut(2))
            {
                let y = (i32::from(get_i16(i)) * g + (1 << 14)) >> 15;
                put_i16(o, y.clamp(-32768, 32767) as i16);
            }
        } else {
            for (i, o) in input.data.chunks_exact(2).zip(out.chunks_exact_mut(2)) {
                put_i16(o, self.apply(get_i16(i)));
            }
        }
        Ok(input.data.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn unity_is_exact_and_half_rounds() {
        assert_eq!(Gain::linear(1.0), Gain::UNITY);
        assert_eq!(Gain::UNITY.apply(-12345), -12345);
        let half = Gain::from_db(-6.0206);
        assert_eq!(half.q15(), 16384);
        assert_eq!(half.apply(3), 2); // 1.5 rounds up
        assert_eq!(half.apply(-3), -1); // -1.5 rounds up (toward +inf)
        assert_eq!(half.apply(i16::MIN), -16384);
    }

    #[test]
    fn saturates() {
        let loud = Gain::from_db(20.0);
        assert_eq!(loud.q15(), 327_680);
        assert_eq!(loud.apply(20_000), i16::MAX);
        assert_eq!(loud.apply(-20_000), i16::MIN);
        assert_eq!(loud.apply(100), 1000);
        let inv = Gain::linear(-1.0);
        assert_eq!(inv.apply(5), -5);
    }

    #[test]
    fn processes_blocks() {
        let mut g = Gain::from_db(6.0206);
        let f = PcmFormat::PCM16_48K_STEREO;
        let mut input = [0u8; 8];
        put_i16(&mut input[0..], 100);
        put_i16(&mut input[2..], -100);
        put_i16(&mut input[4..], 16384);
        put_i16(&mut input[6..], -16385);
        let blk = PcmBlock::new(f, rusty_esp_core::time::Micros::ZERO, &input).unwrap();
        let mut out = [0u8; 8];
        assert_eq!(g.process(blk, &mut out).unwrap(), 8);
        assert_eq!(get_i16(&out[0..]), 200);
        assert_eq!(get_i16(&out[2..]), -200);
        assert_eq!(get_i16(&out[4..]), i16::MAX);
        assert_eq!(get_i16(&out[6..]), i16::MIN);
    }
}
