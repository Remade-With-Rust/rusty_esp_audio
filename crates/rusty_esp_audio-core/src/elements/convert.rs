//! `Convert`: sample-format conversion as an element.

use rusty_esp_core::error::Result;
use rusty_esp_core::pcm::{PcmBlock, PcmFormat, SampleFormat};

use crate::codec::pcm;
use crate::pipeline::Element;

/// Re-encode every sample to `to` (see [`codec::pcm::convert`](crate::codec::pcm::convert)
/// for the exact rules). Same-format input is a byte copy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Convert {
    to: SampleFormat,
}

impl Convert {
    /// Convert to `to`.
    #[must_use]
    pub const fn to(to: SampleFormat) -> Self {
        Convert { to }
    }
}

impl Element for Convert {
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
        PcmFormat::new(input.sample_rate_hz, input.channels, self.to)
    }

    fn max_output_bytes(&self, input: PcmFormat, input_bytes: usize) -> usize {
        let per_in = input.sample.bytes();
        input_bytes / per_in * self.to.bytes()
    }

    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
        // CHIP ARM: the three INTEGER pairs. It declines the float pairs --
        // PIE is an integer unit -- and anything that does not view as
        // samples, which then takes the table below.
        if let Some(n) = crate::pie::convert(&input, self.to, out) {
            return Ok(n);
        }
        pcm::convert(input, self.to, out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::Pipeline;
    use crate::put_i16;
    use rusty_esp_core::time::Micros;

    #[test]
    fn converts_inside_a_pipeline() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut c = Convert::to(SampleFormat::I32);
        let mut p = Pipeline::new([&mut c as &mut dyn Element]);
        let mut input = [0u8; 4];
        put_i16(&mut input[0..], 1);
        put_i16(&mut input[2..], -1);
        let need = p.scratch_bytes(f, 4).unwrap();
        assert_eq!(need, 16);
        let mut scratch = [0u8; 16];
        let out = p
            .process(
                PcmBlock::new(f, Micros::ZERO, &input).unwrap(),
                &mut scratch,
            )
            .unwrap()
            .unwrap();
        assert_eq!(out.format.sample, SampleFormat::I32);
        assert_eq!(out.data, [0, 0, 1, 0, 0, 0, 0xFF, 0xFF]);
    }
}
