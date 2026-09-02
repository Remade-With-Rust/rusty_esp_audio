//! Where blocks come from and where they go.
//!
//! An [`AudioSource`] fills a caller buffer with exactly one block; an
//! [`AudioSink`] consumes one. A PDM microphone, an I2S codec, a UDP socket
//! and a WAV file are all one of these, so a pipeline never knows which.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::pcm::{PcmBlock, PcmFormat, SampleFormat};
use rusty_esp_core::time::Micros;

use crate::put_i16;

/// Produces fixed-size blocks of PCM into caller memory.
pub trait AudioSource {
    /// Rate, channels and encoding of every block this source produces.
    fn format(&self) -> PcmFormat;

    /// Fill `out` completely (it must be a whole number of frames) and return
    /// the block over it, stamped with the capture time of its first frame.
    fn read<'b>(&mut self, out: &'b mut [u8]) -> Result<PcmBlock<'b>>;
}

/// Consumes blocks of PCM.
pub trait AudioSink {
    /// The format this sink accepts.
    fn format(&self) -> PcmFormat;

    /// Take one block. `Err(Busy)` means "dropped, try the next one".
    fn write(&mut self, block: PcmBlock<'_>) -> Result<()>;
}

/// A deterministic test tone: a sine at `freq_hz` and peak `amplitude`, the
/// same on every channel, with timestamps that advance by block duration.
#[derive(Debug, Clone)]
pub struct SineSource {
    format: PcmFormat,
    freq_hz: f32,
    amplitude: i16,
    /// Phase in cycles, kept in `[0, 1)` so precision does not drift.
    phase: f32,
    next: Micros,
}

impl SineSource {
    /// A tone in `format`, which must be [`SampleFormat::I16`].
    pub fn new(format: PcmFormat, freq_hz: f32, amplitude: i16) -> Result<Self> {
        if format.sample != SampleFormat::I16 {
            return Err(Error::Unsupported);
        }
        if freq_hz.is_nan() || freq_hz <= 0.0 || freq_hz * 2.0 > format.sample_rate_hz as f32 {
            return Err(Error::InvalidFormat);
        }
        Ok(SineSource {
            format,
            freq_hz,
            amplitude,
            phase: 0.0,
            next: Micros::ZERO,
        })
    }

    /// Restart the tone at phase zero and time zero.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.next = Micros::ZERO;
    }
}

impl AudioSource for SineSource {
    fn format(&self) -> PcmFormat {
        self.format
    }

    fn read<'b>(&mut self, out: &'b mut [u8]) -> Result<PcmBlock<'b>> {
        let fb = self.format.frame_bytes();
        if out.is_empty() || out.len() % fb != 0 {
            return Err(Error::InvalidGeometry);
        }
        let step = self.freq_hz / self.format.sample_rate_hz as f32;
        let amp = f32::from(self.amplitude);
        for frame in out.chunks_exact_mut(fb) {
            let v = libm::roundf(amp * libm::sinf(core::f32::consts::TAU * self.phase)) as i16;
            for ch in frame.chunks_exact_mut(2) {
                put_i16(ch, v);
            }
            self.phase += step;
            if self.phase >= 1.0 {
                self.phase -= 1.0;
            }
        }
        let ts = self.next;
        let block = PcmBlock::new(self.format, ts, out)?;
        self.next = block.end();
        Ok(block)
    }
}

/// Counts what it is given and remembers where the stream has got to.
#[derive(Debug, Clone, Default)]
pub struct CountingSink {
    format: Option<PcmFormat>,
    /// Blocks accepted.
    pub blocks: u64,
    /// Sample frames accepted.
    pub frames: u64,
    /// Bytes accepted.
    pub bytes: u64,
    /// End timestamp of the last block.
    pub last_end: Micros,
    /// Blocks refused because their format did not match the first one.
    pub rejected: u64,
}

impl CountingSink {
    /// A sink that accepts the format of its first block and holds it to that.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
}

impl AudioSink for CountingSink {
    fn format(&self) -> PcmFormat {
        self.format.unwrap_or(PcmFormat::PCM16_16K_MONO)
    }

    fn write(&mut self, block: PcmBlock<'_>) -> Result<()> {
        match self.format {
            None => self.format = Some(block.format),
            Some(f) if f != block.format => {
                self.rejected += 1;
                return Err(Error::InvalidFormat);
            }
            Some(_) => {}
        }
        self.blocks += 1;
        self.frames += block.frames() as u64;
        self.bytes += block.data.len() as u64;
        self.last_end = block.end();
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::get_i16;

    #[test]
    fn sine_is_periodic_and_timestamped() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut src = SineSource::new(f, 1000.0, 10_000).unwrap();
        let mut buf = [0u8; 640]; // 20 ms
        let b = src.read(&mut buf).unwrap();
        assert_eq!(b.timestamp, Micros::ZERO);
        assert_eq!(b.end(), Micros(20_000));
        // 1 kHz at 16 kHz: 16 samples per cycle; sample 4 is the peak.
        assert_eq!(get_i16(&buf[8..]), 10_000);
        assert_eq!(get_i16(&buf[24..]), -10_000);
        assert_eq!(get_i16(&buf[0..]), 0);
        let b2 = src.read(&mut buf).unwrap();
        assert_eq!(b2.timestamp, Micros(20_000));
        // 20 ms is a whole number of cycles, so the block repeats exactly.
        assert_eq!(get_i16(&buf[8..]), 10_000);
    }

    #[test]
    fn sine_rejects_bad_setups() {
        let f32fmt = PcmFormat::new(16_000, 1, SampleFormat::F32).unwrap();
        assert_eq!(
            SineSource::new(f32fmt, 440.0, 1).err(),
            Some(Error::Unsupported)
        );
        assert_eq!(
            SineSource::new(PcmFormat::PCM16_16K_MONO, 9000.0, 1).err(),
            Some(Error::InvalidFormat)
        );
        let mut s = SineSource::new(PcmFormat::PCM16_16K_MONO, 440.0, 1).unwrap();
        assert_eq!(s.read(&mut [0u8; 3]).err(), Some(Error::InvalidGeometry));
    }

    #[test]
    fn counting_sink_holds_its_format() {
        let mut sink = CountingSink::new();
        let f = PcmFormat::PCM16_16K_MONO;
        let data = [0u8; 64];
        sink.write(PcmBlock::new(f, Micros::ZERO, &data).unwrap())
            .unwrap();
        let other = PcmFormat::PCM16_48K_STEREO;
        assert_eq!(
            sink.write(PcmBlock::new(other, Micros::ZERO, &data).unwrap())
                .err(),
            Some(Error::InvalidFormat)
        );
        assert_eq!((sink.blocks, sink.frames, sink.rejected), (1, 32, 1));
        assert_eq!(sink.last_end, Micros(2_000));
    }
}
