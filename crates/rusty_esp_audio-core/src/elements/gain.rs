//! `Gain`: fixed-point Q15 gain with rounding and saturation.

use rusty_esp_core::error::Result;
use rusty_esp_core::pcm::{PcmBlock, PcmFormat};

use super::{require_i16_any, require_room};
use crate::pipeline::Element;
use crate::{get_i16, put_i16, sat16};

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
        sat16(y.clamp(i64::from(i32::MIN), i64::from(i32::MAX)) as i32)
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
        for (i, o) in input.data.chunks_exact(2).zip(out.chunks_exact_mut(2)) {
            put_i16(o, self.apply(get_i16(i)));
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
