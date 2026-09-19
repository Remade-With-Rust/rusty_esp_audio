//! Channel operations: mono ↔ stereo and a saturating mix.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::pcm::{PcmBlock, PcmFormat};

use super::{require_i16_any, require_room};
use crate::pipeline::Element;
use crate::{get_i16, put_i16, sat16};

/// Duplicate a mono i16 stream onto two channels. Byte-exact.
#[derive(Debug, Clone, Copy, Default)]
pub struct MonoToStereo;

impl Element for MonoToStereo {
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
        require_i16_any(input)?;
        if input.channels != 1 {
            return Err(Error::Unsupported);
        }
        PcmFormat::new(input.sample_rate_hz, 2, input.sample)
    }

    fn max_output_bytes(&self, _input: PcmFormat, input_bytes: usize) -> usize {
        input_bytes * 2
    }

    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
        self.output_format(input.format)?;
        let n = input.data.len() * 2;
        require_room(out, n)?;
        // Four frames a trip: the body is six byte moves, so one-at-a-time
        // spends most of the loop on the counter and the two pointer bumps.
        let mut ci = input.data.chunks_exact(16);
        let mut co = out[..n].chunks_exact_mut(32);
        for (i, o) in ci.by_ref().zip(co.by_ref()) {
            for k in 0..8 {
                o[k * 4] = i[k * 2];
                o[k * 4 + 1] = i[k * 2 + 1];
                o[k * 4 + 2] = i[k * 2];
                o[k * 4 + 3] = i[k * 2 + 1];
            }
        }
        for (i, o) in ci
            .remainder()
            .chunks_exact(2)
            .zip(co.into_remainder().chunks_exact_mut(4))
        {
            o[0] = i[0];
            o[1] = i[1];
            o[2] = i[0];
            o[3] = i[1];
        }
        Ok(n)
    }
}

/// Average a stereo i16 stream to mono: `(l + r) >> 1` (floor). Byte-exact.
#[derive(Debug, Clone, Copy, Default)]
pub struct StereoToMono;

impl Element for StereoToMono {
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
        require_i16_any(input)?;
        if input.channels != 2 {
            return Err(Error::Unsupported);
        }
        PcmFormat::new(input.sample_rate_hz, 1, input.sample)
    }

    fn max_output_bytes(&self, _input: PcmFormat, input_bytes: usize) -> usize {
        input_bytes / 2
    }

    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
        self.output_format(input.format)?;
        let n = input.data.len() / 2;
        require_room(out, n)?;
        // Four frames a trip, for the same reason as `MonoToStereo`.
        let mut ci = input.data.chunks_exact(32);
        let mut co = out[..n].chunks_exact_mut(16);
        for (i, o) in ci.by_ref().zip(co.by_ref()) {
            for k in 0..8 {
                let l = i32::from(get_i16(&i[k * 4..]));
                let r = i32::from(get_i16(&i[k * 4 + 2..]));
                put_i16(&mut o[k * 2..], ((l + r) >> 1) as i16);
            }
        }
        for (i, o) in ci
            .remainder()
            .chunks_exact(4)
            .zip(co.into_remainder().chunks_exact_mut(2))
        {
            let l = i32::from(get_i16(&i[0..]));
            let r = i32::from(get_i16(&i[2..]));
            put_i16(o, ((l + r) >> 1) as i16);
        }
        Ok(n)
    }
}

/// Sum two equal-length i16 byte streams into `out`, saturating.
/// `a`, `b` and `out` must have the same even length.
pub fn mix_i16(a: &[u8], b: &[u8], out: &mut [u8]) -> Result<()> {
    if a.len() != b.len() || a.len() % 2 != 0 {
        return Err(Error::InvalidGeometry);
    }
    require_room(out, a.len())?;
    // Four samples a trip: the body is a load, a load, an add, a clamp and a
    // store, which is small enough that the loop overhead is a real share.
    let n = a.len();
    // FOUR, not eight: eight measured +23.4% against a 0.01% null arm on an
    // ESP32-S3 (2026-09-19). This body is the largest of the three movers --
    // two loads, an add, a clamp and a store per sample -- so it reaches its
    // stopping point one step earlier than `MonoToStereo` does.
    let mut ca = a.chunks_exact(8);
    let mut cb = b.chunks_exact(8);
    let mut co = out[..n].chunks_exact_mut(8);
    for ((x, y), o) in ca.by_ref().zip(cb.by_ref()).zip(co.by_ref()) {
        for k in 0..4 {
            let s = i32::from(get_i16(&x[k * 2..])) + i32::from(get_i16(&y[k * 2..]));
            put_i16(&mut o[k * 2..], sat16(s));
        }
    }
    for ((x, y), o) in ca
        .remainder()
        .chunks_exact(2)
        .zip(cb.remainder().chunks_exact(2))
        .zip(co.into_remainder().chunks_exact_mut(2))
    {
        put_i16(o, sat16(i32::from(get_i16(x)) + i32::from(get_i16(y))));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use rusty_esp_core::time::Micros;

    #[test]
    fn up_and_down_round_trip() {
        let mono = PcmFormat::PCM16_16K_MONO;
        let input = [1u8, 0, 0xFF, 0x7F, 0x00, 0x80];
        let blk = PcmBlock::new(mono, Micros::ZERO, &input).unwrap();
        let mut st = [0u8; 12];
        assert_eq!(MonoToStereo.process(blk, &mut st).unwrap(), 12);
        assert_eq!(st, [1, 0, 1, 0, 0xFF, 0x7F, 0xFF, 0x7F, 0, 0x80, 0, 0x80]);
        let stereo = PcmFormat::new(16_000, 2, mono.sample).unwrap();
        let blk2 = PcmBlock::new(stereo, Micros::ZERO, &st).unwrap();
        let mut back = [0u8; 6];
        assert_eq!(StereoToMono.process(blk2, &mut back).unwrap(), 6);
        assert_eq!(back, input);
    }

    #[test]
    fn downmix_floors_and_mix_saturates() {
        let stereo = PcmFormat::PCM16_48K_STEREO;
        let mut input = [0u8; 8];
        put_i16(&mut input[0..], 3);
        put_i16(&mut input[2..], -2); // (3 + -2) >> 1 = 0
        put_i16(&mut input[4..], -3);
        put_i16(&mut input[6..], 2); // (-3 + 2) >> 1 = -1 (floor)
        let blk = PcmBlock::new(stereo, Micros::ZERO, &input).unwrap();
        let mut out = [0u8; 4];
        StereoToMono.process(blk, &mut out).unwrap();
        assert_eq!(get_i16(&out[0..]), 0);
        assert_eq!(get_i16(&out[2..]), -1);

        let mut a = [0u8; 4];
        let mut b = [0u8; 4];
        put_i16(&mut a[0..], 30_000);
        put_i16(&mut b[0..], 10_000);
        put_i16(&mut a[2..], -30_000);
        put_i16(&mut b[2..], -10_000);
        let mut m = [0u8; 4];
        mix_i16(&a, &b, &mut m).unwrap();
        assert_eq!(get_i16(&m[0..]), i16::MAX);
        assert_eq!(get_i16(&m[2..]), i16::MIN);
        assert_eq!(
            mix_i16(&a, &b[..2], &mut m).err(),
            Some(Error::InvalidGeometry)
        );
    }

    #[test]
    fn wrong_channel_counts_are_refused() {
        assert_eq!(
            MonoToStereo
                .output_format(PcmFormat::PCM16_48K_STEREO)
                .err(),
            Some(Error::Unsupported)
        );
        assert_eq!(
            StereoToMono.output_format(PcmFormat::PCM16_16K_MONO).err(),
            Some(Error::Unsupported)
        );
    }
}
