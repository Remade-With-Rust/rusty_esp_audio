//! `DcBlock`: a one-pole high-pass that removes the offset a PDM or I2S
//! front end leaves on the signal.

use rusty_esp_core::error::Result;
use rusty_esp_core::pcm::{PcmBlock, PcmFormat};

use super::{MAX_CHANNELS, require_i16, require_room};
use crate::pipeline::Element;
use crate::{get_i16, put_i16, round_sat16};

/// `y[n] = x[n] - x[n-1] + r·y[n-1]`, per channel, in `f32`.
///
/// `r = 1 - 2π·fc/fs`; the default corner is 20 Hz. The response at fc is
/// −3 dB; above ~5·fc the gain is within 1 % of unity. State is two floats
/// per channel. Output is rounded (ties away from zero) and saturated.
#[derive(Debug, Clone)]
pub struct DcBlock {
    corner_hz: f32,
    r: f32,
    rate: u32,
    x1: [f32; MAX_CHANNELS],
    y1: [f32; MAX_CHANNELS],
}

impl DcBlock {
    /// A blocker with a 20 Hz corner; the pole is set from the first block's rate.
    #[must_use]
    pub fn new() -> Self {
        Self::with_corner(20.0)
    }

    /// A blocker with the given corner frequency.
    #[must_use]
    pub fn with_corner(corner_hz: f32) -> Self {
        DcBlock {
            corner_hz,
            r: 0.0,
            rate: 0,
            x1: [0.0; MAX_CHANNELS],
            y1: [0.0; MAX_CHANNELS],
        }
    }

    /// The pole radius `r` for `rate`.
    #[must_use]
    pub fn pole(corner_hz: f32, rate: u32) -> f32 {
        1.0 - core::f32::consts::TAU * corner_hz / rate as f32
    }

    fn retune(&mut self, rate: u32) {
        if self.rate != rate {
            self.rate = rate;
            self.r = Self::pole(self.corner_hz, rate);
        }
    }
}

impl Default for DcBlock {
    fn default() -> Self {
        Self::new()
    }
}

impl Element for DcBlock {
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
        require_i16(input)?;
        Ok(input)
    }

    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
        require_i16(input.format)?;
        require_room(out, input.data.len())?;
        self.retune(input.format.sample_rate_hz);
        let ch = input.format.channels as usize;
        let r = self.r;
        // `ch` is fixed for the whole call, so resolving it once lets the state
        // live in REGISTERS for the block. The generic form re-reads `x1[c]`
        // and `y1[c]` out of the struct and writes them back every frame --
        // four memory round trips a sample to carry two floats.
        if ch == 1 {
            let (mut x1, mut y1) = (self.x1[0], self.y1[0]);
            // Two samples a trip. The recurrence is serial and stays serial --
            // nothing here breaks the dependency -- so this only amortises the
            // counter and the two pointer bumps over twice the work.
            let n = input.data.len();
            let mut ci = input.data.chunks_exact(4);
            let mut co = out[..n].chunks_exact_mut(4);
            for (fi, fo) in ci.by_ref().zip(co.by_ref()) {
                let a = f32::from(get_i16(fi));
                let ya = a - x1 + r * y1;
                put_i16(fo, round_sat16(ya));
                let b = f32::from(get_i16(&fi[2..]));
                let yb = b - a + r * ya;
                put_i16(&mut fo[2..], round_sat16(yb));
                x1 = b;
                y1 = yb;
            }
            for (fi, fo) in ci
                .remainder()
                .chunks_exact(2)
                .zip(co.into_remainder().chunks_exact_mut(2))
            {
                let x = f32::from(get_i16(fi));
                let y = x - x1 + r * y1;
                x1 = x;
                y1 = y;
                put_i16(fo, round_sat16(y));
            }
            self.x1[0] = x1;
            self.y1[0] = y1;
            return Ok(input.data.len());
        }
        if ch == 2 {
            let (mut xl, mut yl) = (self.x1[0], self.y1[0]);
            let (mut xr, mut yr) = (self.x1[1], self.y1[1]);
            for (fi, fo) in input.data.chunks_exact(4).zip(out.chunks_exact_mut(4)) {
                let l = f32::from(get_i16(fi));
                let ly = l - xl + r * yl;
                xl = l;
                yl = ly;
                put_i16(fo, round_sat16(ly));
                let rr = f32::from(get_i16(&fi[2..]));
                let ry = rr - xr + r * yr;
                xr = rr;
                yr = ry;
                put_i16(&mut fo[2..], round_sat16(ry));
            }
            self.x1[0] = xl;
            self.y1[0] = yl;
            self.x1[1] = xr;
            self.y1[1] = yr;
            return Ok(input.data.len());
        }
        // MAX_CHANNELS is 2 and `require_i16` rejects more, so nothing reaches
        // here today; it stays as the oracle the two arms above must match.
        for (fi, fo) in input
            .data
            .chunks_exact(ch * 2)
            .zip(out.chunks_exact_mut(ch * 2))
        {
            for c in 0..ch {
                let x = f32::from(get_i16(&fi[c * 2..]));
                let y = x - self.x1[c] + r * self.y1[c];
                self.x1[c] = x;
                self.y1[c] = y;
                put_i16(&mut fo[c * 2..], round_sat16(y));
            }
        }
        Ok(input.data.len())
    }

    fn reset(&mut self) {
        self.x1 = [0.0; MAX_CHANNELS];
        self.y1 = [0.0; MAX_CHANNELS];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::rms_dbfs_i16;
    use crate::source::{AudioSource, SineSource};
    use rusty_esp_core::time::Micros;

    #[test]
    fn removes_dc_within_a_quarter_second() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut dc = DcBlock::new();
        let mut input = [0u8; 320];
        for s in input.chunks_exact_mut(2) {
            put_i16(s, 8000);
        }
        let mut out = [0u8; 320];
        let mut last = 0i16;
        for _ in 0..25 {
            // 25 × 10 ms
            let blk = PcmBlock::new(f, Micros::ZERO, &input).unwrap();
            dc.process(blk, &mut out).unwrap();
            last = get_i16(&out[318..]);
        }
        assert!(last.abs() < 400, "residual {last}");
    }

    #[test]
    fn passes_a_1khz_tone_within_one_percent() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut dc = DcBlock::new();
        let mut src = SineSource::new(f, 1000.0, 10_000).unwrap();
        let mut buf = [0u8; 640];
        let mut out = [0u8; 640];
        let mut level_in = 0.0;
        let mut level_out = 0.0;
        for _ in 0..20 {
            let blk = src.read(&mut buf).unwrap();
            dc.process(blk, &mut out).unwrap();
            level_in = rms_dbfs_i16(&buf);
            level_out = rms_dbfs_i16(&out);
        }
        assert!(
            (level_in - level_out).abs() < 0.09,
            "{level_in} vs {level_out}"
        );
    }
}
