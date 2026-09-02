//! The elements: ESP-ADF's filter/resample/mixer elements and ESP-SR's lite
//! front end, each an [`Element`](crate::Element) over interleaved i16.
//!
//! Every element states its contract in its own docs and has a host test
//! against a reference: byte-identity where the maths is exact (gain,
//! channel ops, conversion, resampler), a stated tolerance against an
//! external oracle where it is not (`Biquad` against ffmpeg and scipy, in
//! `rusty_esp_audio-esp`'s oracle tests).

mod agc;
mod biquad;
mod channels;
mod convert;
mod dc_block;
mod gain;
mod resample;
mod vad;

pub use agc::{Agc, AgcConfig};
pub use biquad::{Biquad, BiquadKind, Coefficients};
pub use channels::{MonoToStereo, StereoToMono, mix_i16};
pub use convert::Convert;
pub use dc_block::DcBlock;
pub use gain::Gain;
pub use resample::LinearResampler;
pub use vad::{EnergyVad, VadConfig};

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::pcm::{PcmFormat, SampleFormat};

/// Channels the stateful filters keep memory for.
pub const MAX_CHANNELS: usize = 2;

/// Accept interleaved i16 with up to [`MAX_CHANNELS`] channels.
fn require_i16(input: PcmFormat) -> Result<()> {
    if input.sample != SampleFormat::I16 || input.channels as usize > MAX_CHANNELS {
        return Err(Error::Unsupported);
    }
    Ok(())
}

/// Accept interleaved i16 with any channel count.
fn require_i16_any(input: PcmFormat) -> Result<()> {
    if input.sample != SampleFormat::I16 {
        return Err(Error::Unsupported);
    }
    Ok(())
}

/// Check `out` can take `needed` bytes.
fn require_room(out: &[u8], needed: usize) -> Result<()> {
    if out.len() < needed {
        return Err(Error::BufferTooSmall { needed });
    }
    Ok(())
}
