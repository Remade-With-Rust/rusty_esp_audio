//! `Element` and `Pipeline`: ESP-ADF's `audio_element` / `audio_pipeline`
//! remade as a fixed-block chain over two halves of one caller scratch buffer.
//!
//! Every element takes one block and writes its result into caller memory.
//! The pipeline ping-pongs between the two halves of `scratch`, so a chain
//! of any length needs exactly two blocks of scratch and no heap. An element
//! may change the format (rate, channels, encoding) and may produce zero
//! bytes for a block (a resampler priming, a gate closed); the pipeline
//! reports `Ok(None)` for that block and the caller moves on.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::pcm::{PcmBlock, PcmFormat};

/// One processing stage.
pub trait Element {
    /// The format this element emits for blocks in `input`, or
    /// `Err(Unsupported)` if it cannot take `input` at all.
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat>;

    /// Upper bound on the bytes [`process`](Self::process) writes for an input
    /// block of `input_bytes` bytes in `input`. Defaults to `input_bytes`
    /// (same-size elements); resamplers and channel converters override it.
    fn max_output_bytes(&self, input: PcmFormat, input_bytes: usize) -> usize {
        let _ = input;
        input_bytes
    }

    /// Process one block into `out`, returning the bytes written — a whole
    /// number of output frames, possibly zero.
    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize>;

    /// Forget all state (filter memories, gains, phases).
    fn reset(&mut self) {}
}

/// A fixed chain of `N` elements.
pub struct Pipeline<'e, const N: usize> {
    stages: [&'e mut dyn Element; N],
    /// Blocks processed.
    pub blocks: u64,
    /// Blocks that produced no output.
    pub empty_blocks: u64,
}

impl<'e, const N: usize> core::fmt::Debug for Pipeline<'e, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("Pipeline")
            .field("stages", &N)
            .field("blocks", &self.blocks)
            .field("empty_blocks", &self.empty_blocks)
            .finish()
    }
}

impl<'e, const N: usize> Pipeline<'e, N> {
    /// Chain `stages` in order.
    pub fn new(stages: [&'e mut dyn Element; N]) -> Self {
        Pipeline {
            stages,
            blocks: 0,
            empty_blocks: 0,
        }
    }

    /// The format that comes out for blocks in `input`.
    pub fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
        self.stages
            .iter()
            .try_fold(input, |f, s| s.output_format(f))
    }

    /// Bytes of `scratch` needed for an input of `input_bytes` bytes in
    /// `input`: twice the largest intermediate block.
    pub fn scratch_bytes(&self, input: PcmFormat, input_bytes: usize) -> Result<usize> {
        let mut fmt = input;
        let mut bytes = input_bytes;
        let mut largest = input_bytes;
        for s in &self.stages {
            bytes = s.max_output_bytes(fmt, bytes);
            fmt = s.output_format(fmt)?;
            largest = largest.max(bytes);
        }
        Ok(largest * 2)
    }

    /// Run one block through every stage. The result borrows `scratch`;
    /// `Ok(None)` means a stage produced nothing for this block.
    pub fn process<'b>(
        &mut self,
        input: PcmBlock<'_>,
        scratch: &'b mut [u8],
    ) -> Result<Option<PcmBlock<'b>>> {
        self.blocks += 1;
        let half = scratch.len() / 2;
        let (a, b) = scratch.split_at_mut(half);
        let ts = input.timestamp;
        let mut fmt = input.format;

        if N == 0 {
            if a.len() < input.data.len() {
                return Err(Error::BufferTooSmall {
                    needed: input.data.len() * 2,
                });
            }
            a[..input.data.len()].copy_from_slice(input.data);
            return Ok(Some(PcmBlock::new(fmt, ts, &a[..input.data.len()])?));
        }

        // Where the current data lives: `true` = in `a`.
        let mut in_a = false;
        let mut len = 0usize;
        for (i, stage) in self.stages.iter_mut().enumerate() {
            let out_fmt = stage.output_format(fmt)?;
            let in_len = if i == 0 { input.data.len() } else { len };
            let need = stage.max_output_bytes(fmt, in_len);
            if half < need {
                return Err(Error::BufferTooSmall { needed: need * 2 });
            }
            let n = if i == 0 {
                stage.process(input, a)?
            } else if in_a {
                let blk = PcmBlock::new(fmt, ts, &a[..len])?;
                stage.process(blk, b)?
            } else {
                let blk = PcmBlock::new(fmt, ts, &b[..len])?;
                stage.process(blk, a)?
            };
            if n % out_fmt.frame_bytes() != 0 || n > half {
                return Err(Error::InvalidGeometry);
            }
            in_a = if i == 0 { true } else { !in_a };
            fmt = out_fmt;
            len = n;
            if n == 0 {
                self.empty_blocks += 1;
                return Ok(None);
            }
        }
        let data: &'b [u8] = if in_a { &a[..len] } else { &b[..len] };
        Ok(Some(PcmBlock::new(fmt, ts, data)?))
    }

    /// Reset every stage.
    pub fn reset(&mut self) {
        for s in self.stages.iter_mut() {
            s.reset();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::elements::{Gain, MonoToStereo};
    use crate::{get_i16, put_i16};
    use rusty_esp_core::time::Micros;

    /// Doubles every sample; the simplest stateful-looking stage.
    struct Double;
    impl Element for Double {
        fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
            Ok(input)
        }
        fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
            for (i, o) in input.data.chunks_exact(2).zip(out.chunks_exact_mut(2)) {
                put_i16(o, get_i16(i).saturating_mul(2));
            }
            Ok(input.data.len())
        }
    }

    /// Emits nothing on odd blocks.
    struct Gate(u64);
    impl Element for Gate {
        fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
            Ok(input)
        }
        fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
            self.0 += 1;
            if self.0 % 2 == 0 {
                return Ok(0);
            }
            out[..input.data.len()].copy_from_slice(input.data);
            Ok(input.data.len())
        }
    }

    #[test]
    fn three_stages_ping_pong_and_change_format() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut d1 = Double;
        let mut d2 = Double;
        let mut up = MonoToStereo;
        let mut p = Pipeline::new([&mut d1 as &mut dyn Element, &mut d2, &mut up]);
        let out_fmt = p.output_format(f).unwrap();
        assert_eq!(out_fmt.channels, 2);
        let mut input = [0u8; 8];
        for (i, s) in input.chunks_exact_mut(2).enumerate() {
            put_i16(s, i as i16 + 1);
        }
        let need = p.scratch_bytes(f, input.len()).unwrap();
        assert_eq!(need, 32);
        let mut scratch = [0u8; 32];
        let blk = PcmBlock::new(f, Micros(5), &input).unwrap();
        let out = p.process(blk, &mut scratch).unwrap().unwrap();
        assert_eq!(out.format, out_fmt);
        assert_eq!(out.timestamp, Micros(5));
        let v: [i16; 8] = core::array::from_fn(|i| get_i16(&out.data[i * 2..]));
        assert_eq!(v, [4, 4, 8, 8, 12, 12, 16, 16]);
        assert_eq!(p.blocks, 1);
    }

    #[test]
    fn empty_output_and_small_scratch_are_reported() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut g = Gate(0);
        let mut unity = Gain::linear(1.0);
        let mut p = Pipeline::new([&mut g as &mut dyn Element, &mut unity]);
        let input = [1u8, 0, 2, 0];
        let mut scratch = [0u8; 8];
        let blk = PcmBlock::new(f, Micros::ZERO, &input).unwrap();
        assert!(p.process(blk, &mut scratch).unwrap().is_some());
        assert!(p.process(blk, &mut scratch).unwrap().is_none());
        assert_eq!(p.empty_blocks, 1);
        let mut tiny = [0u8; 6];
        assert_eq!(
            p.process(blk, &mut tiny).err(),
            Some(Error::BufferTooSmall { needed: 8 })
        );
    }

    #[test]
    fn zero_stages_copies() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut p: Pipeline<'_, 0> = Pipeline::new([]);
        let input = [1u8, 0, 2, 0];
        let mut scratch = [0u8; 8];
        let blk = PcmBlock::new(f, Micros::ZERO, &input).unwrap();
        let out = p.process(blk, &mut scratch).unwrap().unwrap();
        assert_eq!(out.data, &input);
    }
}
