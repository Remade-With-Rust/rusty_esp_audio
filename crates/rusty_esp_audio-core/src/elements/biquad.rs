//! `Biquad`: RBJ cookbook second-order sections in direct form I.
//!
//! The coefficient formulas are the Audio EQ Cookbook's (Robert
//! Bristow-Johnson), the same ones ffmpeg's `lowpass`/`highpass`/`equalizer`
//! filters and scipy users compute, which is what makes an external oracle
//! possible: `rusty_esp_audio-esp`'s oracle test runs the same filter in
//! ffmpeg and in scipy over the same samples and records the largest
//! difference in LSBs.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::pcm::{PcmBlock, PcmFormat};

use super::{MAX_CHANNELS, require_i16, require_room};
use crate::pipeline::Element;
use crate::{get_i16, put_i16, round_sat16};

/// Which response to design.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BiquadKind {
    /// Second-order low-pass at `f0` with quality `q`.
    LowPass {
        /// Corner frequency in Hz.
        f0: f32,
        /// Quality factor (0.7071 is Butterworth).
        q: f32,
    },
    /// Second-order high-pass at `f0` with quality `q`.
    HighPass {
        /// Corner frequency in Hz.
        f0: f32,
        /// Quality factor (0.7071 is Butterworth).
        q: f32,
    },
    /// Peaking EQ: `gain_db` at `f0`, width set by `q`.
    Peak {
        /// Centre frequency in Hz.
        f0: f32,
        /// Quality factor.
        q: f32,
        /// Boost (positive) or cut (negative) in dB.
        gain_db: f32,
    },
    /// Notch at `f0` with quality `q`.
    Notch {
        /// Centre frequency in Hz.
        f0: f32,
        /// Quality factor.
        q: f32,
    },
}

/// Normalised coefficients (`a0 = 1`).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Coefficients {
    /// Feed-forward.
    pub b0: f32,
    /// Feed-forward, one sample back.
    pub b1: f32,
    /// Feed-forward, two samples back.
    pub b2: f32,
    /// Feedback, one sample back.
    pub a1: f32,
    /// Feedback, two samples back.
    pub a2: f32,
}

impl Coefficients {
    /// Unity passthrough.
    pub const IDENTITY: Coefficients = Coefficients {
        b0: 1.0,
        b1: 0.0,
        b2: 0.0,
        a1: 0.0,
        a2: 0.0,
    };

    /// Design `kind` for sample rate `rate`. Computed in `f64` and rounded
    /// to `f32` once, so the design matches a double-precision reference
    /// as closely as `f32` allows. `f0` must lie in `(0, rate/2)`.
    pub fn design(kind: BiquadKind, rate: u32) -> Result<Self> {
        let fs = f64::from(rate);
        let (f0, q) = match kind {
            BiquadKind::LowPass { f0, q }
            | BiquadKind::HighPass { f0, q }
            | BiquadKind::Peak { f0, q, .. }
            | BiquadKind::Notch { f0, q } => (f64::from(f0), f64::from(q)),
        };
        if f0.is_nan() || q.is_nan() || f0 <= 0.0 || f0 >= fs / 2.0 || q <= 0.0 {
            return Err(Error::InvalidFormat);
        }
        let w0 = core::f64::consts::TAU * f0 / fs;
        let cos_w0 = libm::cos(w0);
        let alpha = libm::sin(w0) / (2.0 * q);
        let (b0, b1, b2, a0, a1, a2) = match kind {
            BiquadKind::LowPass { .. } => (
                (1.0 - cos_w0) / 2.0,
                1.0 - cos_w0,
                (1.0 - cos_w0) / 2.0,
                1.0 + alpha,
                -2.0 * cos_w0,
                1.0 - alpha,
            ),
            BiquadKind::HighPass { .. } => (
                (1.0 + cos_w0) / 2.0,
                -(1.0 + cos_w0),
                (1.0 + cos_w0) / 2.0,
                1.0 + alpha,
                -2.0 * cos_w0,
                1.0 - alpha,
            ),
            BiquadKind::Peak { gain_db, .. } => {
                let a = libm::pow(10.0, f64::from(gain_db) / 40.0);
                (
                    1.0 + alpha * a,
                    -2.0 * cos_w0,
                    1.0 - alpha * a,
                    1.0 + alpha / a,
                    -2.0 * cos_w0,
                    1.0 - alpha / a,
                )
            }
            BiquadKind::Notch { .. } => (
                1.0,
                -2.0 * cos_w0,
                1.0,
                1.0 + alpha,
                -2.0 * cos_w0,
                1.0 - alpha,
            ),
        };
        Ok(Coefficients {
            b0: (b0 / a0) as f32,
            b1: (b1 / a0) as f32,
            b2: (b2 / a0) as f32,
            a1: (a1 / a0) as f32,
            a2: (a2 / a0) as f32,
        })
    }
}

/// A biquad section over up to [`MAX_CHANNELS`] channels of i16.
///
/// Direct form I in `f32`: `y = b0·x + b1·x1 + b2·x2 − a1·y1 − a2·y2`; output
/// rounded ties-away and saturated. The design is redone when the input rate
/// changes.
#[derive(Debug, Clone)]
pub struct Biquad {
    kind: BiquadKind,
    rate: u32,
    coef: Coefficients,
    x1: [f32; MAX_CHANNELS],
    x2: [f32; MAX_CHANNELS],
    y1: [f32; MAX_CHANNELS],
    y2: [f32; MAX_CHANNELS],
}

impl Biquad {
    /// A section of `kind`; designed on the first block.
    #[must_use]
    pub fn new(kind: BiquadKind) -> Self {
        Biquad {
            kind,
            rate: 0,
            coef: Coefficients::IDENTITY,
            x1: [0.0; MAX_CHANNELS],
            x2: [0.0; MAX_CHANNELS],
            y1: [0.0; MAX_CHANNELS],
            y2: [0.0; MAX_CHANNELS],
        }
    }

    /// The coefficients in use (identity until the first block).
    #[must_use]
    pub fn coefficients(&self) -> Coefficients {
        self.coef
    }

    /// Filter one sample of channel `c`.
    #[inline]
    fn tick(&mut self, c: usize, x: f32) -> f32 {
        let k = &self.coef;
        let y = k.b0 * x + k.b1 * self.x1[c] + k.b2 * self.x2[c]
            - k.a1 * self.y1[c]
            - k.a2 * self.y2[c];
        self.x2[c] = self.x1[c];
        self.x1[c] = x;
        self.y2[c] = self.y1[c];
        self.y1[c] = y;
        y
    }

    fn retune(&mut self, rate: u32) -> Result<()> {
        if self.rate != rate {
            self.coef = Coefficients::design(self.kind, rate)?;
            self.rate = rate;
        }
        Ok(())
    }
}

impl Element for Biquad {
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
        require_i16(input)?;
        Coefficients::design(self.kind, input.sample_rate_hz)?;
        Ok(input)
    }

    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
        require_i16(input.format)?;
        require_room(out, input.data.len())?;
        self.retune(input.format.sample_rate_hz)?;
        let ch = input.format.channels as usize;
        let k = self.coef;
        // `tick` carries FOUR state words per channel through `self`, so the
        // generic form is eight memory round trips a sample plus five
        // coefficient loads. Resolving `ch` once holds all of it in registers.
        if ch == 1 {
            let (mut x1, mut x2) = (self.x1[0], self.x2[0]);
            let (mut y1, mut y2) = (self.y1[0], self.y2[0]);
            // Two samples a trip. The second sample's state is the first's
            // written out by hand -- x1 becomes `a`, x2 becomes the old x1,
            // and likewise for y -- so the arithmetic per sample is unchanged
            // and only the counter and the pointer bumps are amortised. Same
            // shape that paid on `DcBlock`.
            let n = input.data.len();
            let mut ci = input.data.chunks_exact(4);
            let mut co = out[..n].chunks_exact_mut(4);
            for (fi, fo) in ci.by_ref().zip(co.by_ref()) {
                let a = f32::from(get_i16(fi));
                let ya = k.b0 * a + k.b1 * x1 + k.b2 * x2 - k.a1 * y1 - k.a2 * y2;
                put_i16(fo, round_sat16(ya));
                let b = f32::from(get_i16(&fi[2..]));
                let yb = k.b0 * b + k.b1 * a + k.b2 * x1 - k.a1 * ya - k.a2 * y1;
                put_i16(&mut fo[2..], round_sat16(yb));
                x2 = a;
                x1 = b;
                y2 = ya;
                y1 = yb;
            }
            for (fi, fo) in ci
                .remainder()
                .chunks_exact(2)
                .zip(co.into_remainder().chunks_exact_mut(2))
            {
                let x = f32::from(get_i16(fi));
                let y = k.b0 * x + k.b1 * x1 + k.b2 * x2 - k.a1 * y1 - k.a2 * y2;
                x2 = x1;
                x1 = x;
                y2 = y1;
                y1 = y;
                put_i16(fo, round_sat16(y));
            }
            self.x1[0] = x1;
            self.x2[0] = x2;
            self.y1[0] = y1;
            self.y2[0] = y2;
            return Ok(input.data.len());
        }
        if ch == 2 {
            let (mut lx1, mut lx2) = (self.x1[0], self.x2[0]);
            let (mut ly1, mut ly2) = (self.y1[0], self.y2[0]);
            let (mut rx1, mut rx2) = (self.x1[1], self.x2[1]);
            let (mut ry1, mut ry2) = (self.y1[1], self.y2[1]);
            for (fi, fo) in input.data.chunks_exact(4).zip(out.chunks_exact_mut(4)) {
                let x = f32::from(get_i16(fi));
                let y = k.b0 * x + k.b1 * lx1 + k.b2 * lx2 - k.a1 * ly1 - k.a2 * ly2;
                lx2 = lx1;
                lx1 = x;
                ly2 = ly1;
                ly1 = y;
                put_i16(fo, round_sat16(y));
                let x = f32::from(get_i16(&fi[2..]));
                let y = k.b0 * x + k.b1 * rx1 + k.b2 * rx2 - k.a1 * ry1 - k.a2 * ry2;
                rx2 = rx1;
                rx1 = x;
                ry2 = ry1;
                ry1 = y;
                put_i16(&mut fo[2..], round_sat16(y));
            }
            self.x1[0] = lx1;
            self.x2[0] = lx2;
            self.y1[0] = ly1;
            self.y2[0] = ly2;
            self.x1[1] = rx1;
            self.x2[1] = rx2;
            self.y1[1] = ry1;
            self.y2[1] = ry2;
            return Ok(input.data.len());
        }
        // Unreachable today (MAX_CHANNELS is 2); kept as the oracle.
        for (fi, fo) in input
            .data
            .chunks_exact(ch * 2)
            .zip(out.chunks_exact_mut(ch * 2))
        {
            for c in 0..ch {
                let y = self.tick(c, f32::from(get_i16(&fi[c * 2..])));
                put_i16(&mut fo[c * 2..], round_sat16(y));
            }
        }
        Ok(input.data.len())
    }

    fn reset(&mut self) {
        self.x1 = [0.0; MAX_CHANNELS];
        self.x2 = [0.0; MAX_CHANNELS];
        self.y1 = [0.0; MAX_CHANNELS];
        self.y2 = [0.0; MAX_CHANNELS];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rms_dbfs_i16;
    use crate::source::{AudioSource, SineSource};

    fn steady_level(kind: BiquadKind, tone_hz: f32) -> f32 {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut bq = Biquad::new(kind);
        let mut src = SineSource::new(f, tone_hz, 10_000).unwrap();
        let mut buf = [0u8; 640];
        let mut out = [0u8; 640];
        let mut level = 0.0;
        for _ in 0..50 {
            let blk = src.read(&mut buf).unwrap();
            bq.process(blk, &mut out).unwrap();
            level = rms_dbfs_i16(&out);
        }
        level - rms_dbfs_i16(&buf)
    }

    #[test]
    fn butterworth_corner_is_minus_three_db() {
        let lp = BiquadKind::LowPass {
            f0: 1000.0,
            q: core::f32::consts::FRAC_1_SQRT_2,
        };
        let at_corner = steady_level(lp, 1000.0);
        assert!((at_corner + 3.01).abs() < 0.15, "{at_corner}");
        let far_below = steady_level(lp, 100.0);
        assert!(far_below.abs() < 0.05, "{far_below}");
        let far_above = steady_level(lp, 4000.0);
        assert!(far_above < -23.0, "{far_above}"); // 2 octaves up: -24 dB
        let hp = BiquadKind::HighPass {
            f0: 300.0,
            q: core::f32::consts::FRAC_1_SQRT_2,
        };
        assert!(steady_level(hp, 75.0) < -23.0);
        assert!(steady_level(hp, 3000.0).abs() < 0.05);
    }

    #[test]
    fn peak_and_notch_do_what_they_say() {
        let peak = BiquadKind::Peak {
            f0: 1000.0,
            q: 1.0,
            gain_db: 6.0,
        };
        let g = steady_level(peak, 1000.0);
        assert!((g - 6.0).abs() < 0.1, "{g}");
        let notch = BiquadKind::Notch { f0: 1000.0, q: 2.0 };
        assert!(steady_level(notch, 1000.0) < -40.0);
        assert!(steady_level(notch, 250.0).abs() < 0.2);
    }

    #[test]
    fn design_rejects_out_of_band() {
        assert_eq!(
            Coefficients::design(BiquadKind::LowPass { f0: 9000.0, q: 1.0 }, 16_000).err(),
            Some(Error::InvalidFormat)
        );
        assert_eq!(
            Coefficients::design(BiquadKind::LowPass { f0: 100.0, q: 0.0 }, 16_000).err(),
            Some(Error::InvalidFormat)
        );
        let bq = Biquad::new(BiquadKind::LowPass { f0: 9000.0, q: 1.0 });
        assert_eq!(
            bq.output_format(PcmFormat::PCM16_16K_MONO).err(),
            Some(Error::InvalidFormat)
        );
    }
}
