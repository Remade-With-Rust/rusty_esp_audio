//! Sample-format conversion with fixed, documented rules — the ones ffmpeg's
//! `swresample` uses, so the oracle test in `rusty_esp_audio-esp` can demand
//! byte identity.
//!
//! | from → to | rule |
//! |---|---|
//! | I16 → I32 / I24In32 | `x << 16` |
//! | I32 / I24In32 → I16 | `x >> 16` (truncate toward −∞) |
//! | I32 → I24In32 | clear the low 8 bits |
//! | I24In32 → I32 | bit copy |
//! | I16 → F32 | `x / 32768` |
//! | I32 / I24In32 → F32 | `x / 2^31` |
//! | F32 → I16 | `round_ties_even(x · 32768)` clamped to the i16 range |
//! | F32 → I32 | `round_ties_even(x · 2^31)` clamped (in f64) to the i32 range |
//! | F32 → I24In32 | as I32, then clear the low 8 bits |

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::pcm::{PcmBlock, SampleFormat};

/// Bytes `convert` writes for `input_bytes` of `from` samples going to `to`.
#[must_use]
pub const fn output_bytes(from: SampleFormat, to: SampleFormat, input_bytes: usize) -> usize {
    input_bytes / from.bytes() * to.bytes()
}

#[inline]
fn f32_to_i16(x: f32) -> i16 {
    let v = libm::rintf(x * 32768.0);
    if v >= 32767.0 {
        i16::MAX
    } else if v <= -32768.0 {
        i16::MIN
    } else {
        v as i16
    }
}

#[inline]
fn f32_to_i32(x: f32) -> i32 {
    let v = libm::rint(f64::from(x) * 2_147_483_648.0);
    if v >= 2_147_483_647.0 {
        i32::MAX
    } else if v <= -2_147_483_648.0 {
        i32::MIN
    } else {
        v as i32
    }
}

/// Convert one sample given as its little-endian bytes.
#[inline]
fn convert_sample(from: SampleFormat, to: SampleFormat, i: &[u8], o: &mut [u8]) {
    use SampleFormat::{F32, I16, I24In32, I32};
    match (from, to) {
        (I16, I32) | (I16, I24In32) => {
            let v = i32::from(i16::from_le_bytes([i[0], i[1]])) << 16;
            o.copy_from_slice(&v.to_le_bytes());
        }
        (I32, I16) | (I24In32, I16) => {
            let v = i32::from_le_bytes([i[0], i[1], i[2], i[3]]) >> 16;
            o.copy_from_slice(&(v as i16).to_le_bytes());
        }
        (I32, I24In32) => {
            o.copy_from_slice(&[0, i[1], i[2], i[3]]);
        }
        (I24In32, I32) => {
            o.copy_from_slice(i);
        }
        (I16, F32) => {
            let v = f32::from(i16::from_le_bytes([i[0], i[1]])) / 32768.0;
            o.copy_from_slice(&v.to_le_bytes());
        }
        (I32, F32) | (I24In32, F32) => {
            let v = (i32::from_le_bytes([i[0], i[1], i[2], i[3]]) as f32) / 2_147_483_648.0;
            o.copy_from_slice(&v.to_le_bytes());
        }
        (F32, I16) => {
            let v = f32_to_i16(f32::from_le_bytes([i[0], i[1], i[2], i[3]]));
            o.copy_from_slice(&v.to_le_bytes());
        }
        (F32, I32) => {
            let v = f32_to_i32(f32::from_le_bytes([i[0], i[1], i[2], i[3]]));
            o.copy_from_slice(&v.to_le_bytes());
        }
        (F32, I24In32) => {
            let v = f32_to_i32(f32::from_le_bytes([i[0], i[1], i[2], i[3]])) & !0xFF;
            o.copy_from_slice(&v.to_le_bytes());
        }
        // Same format: a copy. `from`/`to` are `#[non_exhaustive]`, so a new
        // variant arriving in core lands here too and stays a copy only when
        // the sizes agree (checked by the caller).
        _ => o.copy_from_slice(i),
    }
}

/// Re-encode every sample of `input` as `to` into `out`; returns bytes
/// written. Same-format input is a byte copy.
pub fn convert(input: PcmBlock<'_>, to: SampleFormat, out: &mut [u8]) -> Result<usize> {
    let from = input.format.sample;
    let n = output_bytes(from, to, input.data.len());
    if out.len() < n {
        return Err(Error::BufferTooSmall { needed: n });
    }
    if from == to {
        out[..n].copy_from_slice(input.data);
        return Ok(n);
    }
    if from.bytes() != to.bytes() && !is_known_pair(from, to) {
        return Err(Error::Unsupported);
    }
    let (fb, tb) = (from.bytes(), to.bytes());
    for (i, o) in input
        .data
        .chunks_exact(fb)
        .zip(out[..n].chunks_exact_mut(tb))
    {
        convert_sample(from, to, i, o);
    }
    Ok(n)
}

/// Every pair the table above lists.
const fn is_known_pair(from: SampleFormat, to: SampleFormat) -> bool {
    use SampleFormat::{F32, I16, I24In32, I32};
    matches!(
        (from, to),
        (I16, I32)
            | (I16, I24In32)
            | (I16, F32)
            | (I32, I16)
            | (I32, I24In32)
            | (I32, F32)
            | (I24In32, I16)
            | (I24In32, I32)
            | (I24In32, F32)
            | (F32, I16)
            | (F32, I32)
            | (F32, I24In32)
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use rusty_esp_core::pcm::PcmFormat;
    use rusty_esp_core::time::Micros;

    fn block<'a>(sample: SampleFormat, data: &'a [u8]) -> PcmBlock<'a> {
        PcmBlock::new(
            PcmFormat::new(16_000, 1, sample).unwrap(),
            Micros::ZERO,
            data,
        )
        .unwrap()
    }

    #[test]
    fn integer_widths() {
        let mut out = [0u8; 8];
        let i16s = [0x01, 0x00, 0x00, 0x80]; // 1, -32768
        assert_eq!(
            convert(block(SampleFormat::I16, &i16s), SampleFormat::I32, &mut out).unwrap(),
            8
        );
        assert_eq!(out, [0, 0, 1, 0, 0, 0, 0, 0x80]);
        let mut back = [0u8; 4];
        convert(block(SampleFormat::I32, &out), SampleFormat::I16, &mut back).unwrap();
        assert_eq!(back, i16s);
        // Truncation: 0x0000_FFFF >> 16 = 0; 0xFFFF_0001 (negative) >> 16 = -1.
        let odd = [0xFF, 0xFF, 0x00, 0x00, 0x01, 0x00, 0xFF, 0xFF];
        convert(block(SampleFormat::I32, &odd), SampleFormat::I16, &mut back).unwrap();
        assert_eq!(back, [0, 0, 0xFF, 0xFF]);
        // I32 → I24In32 clears the low byte; I24In32 → I32 is a copy.
        let mut o24 = [0u8; 8];
        convert(
            block(SampleFormat::I32, &odd),
            SampleFormat::I24In32,
            &mut o24,
        )
        .unwrap();
        assert_eq!(o24, [0, 0xFF, 0, 0, 0, 0, 0xFF, 0xFF]);
        let mut o32 = [0u8; 8];
        convert(
            block(SampleFormat::I24In32, &o24),
            SampleFormat::I32,
            &mut o32,
        )
        .unwrap();
        assert_eq!(o32, o24);
    }

    #[test]
    fn float_rounds_ties_even_and_clamps() {
        let f = [1.0f32, -1.0, 0.5, 1.5 / 32768.0, 2.5 / 32768.0, -0.00001];
        let mut bytes = [0u8; 24];
        for (b, v) in bytes.chunks_exact_mut(4).zip(f) {
            b.copy_from_slice(&v.to_le_bytes());
        }
        let mut out = [0u8; 12];
        convert(
            block(SampleFormat::F32, &bytes),
            SampleFormat::I16,
            &mut out,
        )
        .unwrap();
        let v: [i16; 6] =
            core::array::from_fn(|i| i16::from_le_bytes([out[i * 2], out[i * 2 + 1]]));
        assert_eq!(v, [32767, -32768, 16384, 2, 2, 0]);
        // And back: exact for values that came from integers.
        let mut f_out = [0u8; 24];
        convert(
            block(SampleFormat::I16, &out),
            SampleFormat::F32,
            &mut f_out,
        )
        .unwrap();
        let back = f32::from_le_bytes([f_out[8], f_out[9], f_out[10], f_out[11]]);
        assert_eq!(back, 0.5);
        // F32 → I32 clamps at both ends; 0.5 is exactly 2^30.
        let mut o32 = [0u8; 24];
        convert(
            block(SampleFormat::F32, &bytes),
            SampleFormat::I32,
            &mut o32,
        )
        .unwrap();
        assert_eq!(
            i32::from_le_bytes([o32[0], o32[1], o32[2], o32[3]]),
            i32::MAX
        );
        assert_eq!(
            i32::from_le_bytes([o32[4], o32[5], o32[6], o32[7]]),
            i32::MIN
        );
        assert_eq!(
            i32::from_le_bytes([o32[8], o32[9], o32[10], o32[11]]),
            1 << 30
        );
    }

    #[test]
    fn same_format_copies_and_small_out_is_reported() {
        let data = [1u8, 2, 3, 4];
        let mut out = [0u8; 4];
        assert_eq!(
            convert(block(SampleFormat::I16, &data), SampleFormat::I16, &mut out).unwrap(),
            4
        );
        assert_eq!(out, data);
        let mut tiny = [0u8; 2];
        assert_eq!(
            convert(
                block(SampleFormat::I16, &data),
                SampleFormat::I32,
                &mut tiny
            )
            .err(),
            Some(Error::BufferTooSmall { needed: 8 })
        );
    }
}
