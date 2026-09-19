//! `LinearResampler`: exact-rational-phase linear interpolation between
//! sample rates. Deterministic on every target; the cheap option for
//! control-rate changes (16 k → 48 k for a codec chip, 48 k → 16 k for a voice
//! pipeline). It is not a windowed-sinc resampler; the LEDGER says what its
//! aliasing is.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::pcm::{PcmBlock, PcmFormat};

use super::{MAX_CHANNELS, require_i16, require_room};
use crate::pipeline::Element;
use crate::{get_i16, put_i16};

/// Linear interpolation from `in_rate` to `out_rate` over up to
/// [`MAX_CHANNELS`] channels of i16.
///
/// The phase is a rational number: with the rates reduced by their gcd to
/// `in_r : out_r`, output frame `k` sits at input position `k·in_r/out_r`
/// exactly, so 16 k → 48 k lands every third output on an input sample and
/// the long-run rate error is zero. The interpolation is
/// `y = x0 + ((x1 − x0)·frac_q32 + 2^31) >> 32` with `frac_q32 =
/// (rem << 32) / out_r`. The last frame of each block is kept so blocks join
/// seamlessly; output frames that need the next block wait for it. Same-rate
/// setups are a byte-exact copy.
#[derive(Debug, Clone)]
pub struct LinearResampler {
    in_rate: u32,
    out_rate: u32,
    /// Reduced ratio.
    in_r: u64,
    out_r: u64,
    /// Position of the next output frame in units of `1/out_r` input frames,
    /// in the shifted coordinate where 0 is `last` and `k` is `frame(k − 1)`.
    num: u64,
    last: [i16; MAX_CHANNELS],
    primed: bool,
}

const fn gcd(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let t = a % b;
        a = b;
        b = t;
    }
    a
}

impl LinearResampler {
    /// A resampler from `in_rate` to `out_rate` (both non-zero).
    pub fn new(in_rate: u32, out_rate: u32) -> Result<Self> {
        if in_rate == 0 || out_rate == 0 {
            return Err(Error::InvalidFormat);
        }
        let g = gcd(u64::from(in_rate), u64::from(out_rate));
        Ok(LinearResampler {
            in_rate,
            out_rate,
            in_r: u64::from(in_rate) / g,
            out_r: u64::from(out_rate) / g,
            num: 0,
            last: [0; MAX_CHANNELS],
            primed: false,
        })
    }

    /// Output frames this element can emit for `in_frames` input frames.
    #[must_use]
    pub fn max_output_frames(&self, in_frames: usize) -> usize {
        // frames · out/in, rounded up, plus one for the phase carried in.
        ((in_frames as u64 * u64::from(self.out_rate)).div_ceil(u64::from(self.in_rate)) + 1)
            as usize
    }
}

impl Element for LinearResampler {
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
        require_i16(input)?;
        if input.sample_rate_hz != self.in_rate {
            return Err(Error::Unsupported);
        }
        PcmFormat::new(self.out_rate, input.channels, input.sample)
    }

    fn max_output_bytes(&self, input: PcmFormat, input_bytes: usize) -> usize {
        let fb = input.frame_bytes().max(1);
        self.max_output_frames(input_bytes / fb) * fb
    }

    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
        self.output_format(input.format)?;
        let ch = input.format.channels as usize;
        let fb = ch * 2;
        let in_frames = input.frames();
        if self.in_rate == self.out_rate {
            require_room(out, input.data.len())?;
            out[..input.data.len()].copy_from_slice(input.data);
            return Ok(input.data.len());
        }
        require_room(out, self.max_output_frames(in_frames) * fb)?;

        let frame = |i: usize, c: usize| -> i16 { get_i16(&input.data[i * fb + c * 2..]) };
        if !self.primed {
            // The stream starts at the first frame: emit it at phase zero.
            for c in 0..ch {
                self.last[c] = frame(0, c);
            }
            self.primed = true;
            self.num = self.out_r; // shifted position 1 = frame(0)
        }
        // Interpolation needs frame(idx) to exist, i.e. shifted idx < in_frames.
        let end = in_frames as u64 * self.out_r;
        let mut written = 0usize;
        // `num` only ever advances by `in_r`, so its quotient and remainder by
        // `out_r` can be CARRIED instead of recomputed. That retires two
        // 64-bit divisions per output frame -- and a 64-bit division on this
        // core is a libcall, not an instruction -- for a compare and a
        // subtract per whole input frame crossed.
        let mut idx = (self.num / self.out_r) as usize;
        let mut rem = self.num % self.out_r;
        while self.num < end {
            let frac = (rem << 32) / self.out_r;
            for c in 0..ch {
                let x0 = i64::from(if idx == 0 {
                    self.last[c]
                } else {
                    frame(idx - 1, c)
                });
                let x1 = i64::from(frame(idx, c));
                let y = x0 + (((x1 - x0) * frac as i64 + (1 << 31)) >> 32);
                put_i16(&mut out[written * fb + c * 2..], y as i16);
            }
            written += 1;
            self.num += self.in_r;
            rem += self.in_r;
            while rem >= self.out_r {
                rem -= self.out_r;
                idx += 1;
            }
        }
        // Carry the last frame and rebase the phase onto it.
        for c in 0..ch {
            self.last[c] = frame(in_frames - 1, c);
        }
        self.num -= end;
        Ok(written * fb)
    }

    fn reset(&mut self) {
        self.num = 0;
        self.primed = false;
        self.last = [0; MAX_CHANNELS];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rusty_esp_core::time::Micros;

    fn run(rs: &mut LinearResampler, fmt: PcmFormat, samples: &[i16]) -> Vec<i16> {
        let mut bytes = vec![0u8; samples.len() * 2];
        for (b, s) in bytes.chunks_exact_mut(2).zip(samples) {
            put_i16(b, *s);
        }
        let blk = PcmBlock::new(fmt, Micros::ZERO, &bytes).unwrap();
        let mut out = vec![0u8; rs.max_output_bytes(fmt, bytes.len())];
        let n = rs.process(blk, &mut out).unwrap();
        out[..n].chunks_exact(2).map(get_i16).collect()
    }

    #[test]
    fn upsampling_a_ramp_interpolates_exactly() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut rs = LinearResampler::new(16_000, 48_000).unwrap();
        // 0, 300, 600, 900 → every third output lands on an input; the two
        // outputs after 900 need the next block.
        let out = run(&mut rs, f, &[0, 300, 600, 900]);
        assert_eq!(out, [0, 100, 200, 300, 400, 500, 600, 700, 800]);
        // The next block continues from 900 without a seam.
        let out2 = run(&mut rs, f, &[1200, 1500]);
        assert_eq!(out2, [900, 1000, 1100, 1200, 1300, 1400]);
    }

    #[test]
    fn awkward_ratio_has_no_long_run_drift() {
        // 44.1 k → 16 k reduces to 441 : 160.
        let f = PcmFormat::new(44_100, 1, rusty_esp_core::pcm::SampleFormat::I16).unwrap();
        let mut rs = LinearResampler::new(44_100, 16_000).unwrap();
        let mut total = 0usize;
        let block: Vec<i16> = (0..441).map(|i| (i % 7) as i16).collect();
        for _ in 0..100 {
            total += run(&mut rs, f, &block).len();
        }
        // 44 100 frames in → 16 000 out, minus the tail waiting for more input.
        assert!((15_999..=16_000).contains(&total), "{total}");
    }

    #[test]
    fn downsampling_a_constant_is_constant_and_counts_frames() {
        let f = PcmFormat::PCM16_48K_STEREO;
        let mut rs = LinearResampler::new(48_000, 16_000).unwrap();
        let mut total = 0usize;
        for _ in 0..10 {
            let block: Vec<i16> = (0..96)
                .map(|i| if i % 2 == 0 { 1000 } else { -1000 })
                .collect();
            let out = run(&mut rs, f, &block);
            assert!(out.chunks_exact(2).all(|p| p == [1000, -1000]), "{out:?}");
            total += out.len() / 2;
        }
        assert_eq!(total, 160); // 480 stereo frames in → 160 out, exactly
        assert_eq!(rs.output_format(f).unwrap().sample_rate_hz, 16_000);
    }

    #[test]
    fn same_rate_is_a_copy_and_wrong_rate_is_refused() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut rs = LinearResampler::new(16_000, 16_000).unwrap();
        assert_eq!(run(&mut rs, f, &[1, -1, 7]), [1, -1, 7]);
        let bad = LinearResampler::new(8_000, 16_000).unwrap();
        assert_eq!(bad.output_format(f).err(), Some(Error::Unsupported));
        assert_eq!(LinearResampler::new(0, 1).err(), Some(Error::InvalidFormat));
    }
}
