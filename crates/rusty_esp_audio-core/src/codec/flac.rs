//! FLAC through the house codec, `rusty_flac`, for i16 blocks.
//!
//! **This is the one module in the core that allocates.** `rusty_flac`
//! buffers the stream it is building and returns it as a `Vec<u8>`, so this
//! module lives behind the `flac` feature (which implies `alloc`) and is
//! honest about it: a Track B firmware budgets the encoder's heap use; the
//! borrowed-frame law holds for everything else in the crate.
//!
//! The shape for a chip is **chunked streams**: push blocks for a second or
//! two, then [`FlacEncoder::finish`] — each chunk is a complete, standalone
//! FLAC stream (42-byte header plus frames) that any player, `ffmpeg` and
//! `rff` decode. The oracle test in `rusty_esp_audio-esp` decodes them with
//! ffmpeg back to the exact source PCM and checks that two encodes of the
//! same samples are byte-identical (with `rusty_flac`'s `libm` feature, that
//! identity extends across host and chip).

use alloc::vec::Vec;

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::pcm::{PcmBlock, PcmFormat, SampleFormat};

/// Bytes of `fLaC` marker plus the STREAMINFO block every chunk starts with.
pub const STREAM_HEADER_LEN: usize = 4 + 4 + 34;

/// Encodes i16 blocks into complete FLAC streams, one per chunk.
pub struct FlacEncoder {
    format: PcmFormat,
    level: u32,
    inner: Option<rusty_flac::Encoder>,
    /// Frames pushed into the current chunk.
    pub pending_frames: usize,
    /// Chunks finished.
    pub chunks: u64,
    /// Bytes of FLAC produced so far.
    pub bytes_out: u64,
    /// Bytes of PCM consumed so far.
    pub bytes_in: u64,
}

impl core::fmt::Debug for FlacEncoder {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("FlacEncoder")
            .field("format", &self.format)
            .field("level", &self.level)
            .field("pending_frames", &self.pending_frames)
            .field("chunks", &self.chunks)
            .finish_non_exhaustive()
    }
}

impl FlacEncoder {
    /// An encoder for `format` (i16, 1–8 channels) at `rusty_flac`
    /// compression `level` (0–8; 5 is a sensible chip default).
    pub fn new(format: PcmFormat, level: u32) -> Result<Self> {
        if format.sample != SampleFormat::I16 || format.channels > 8 {
            return Err(Error::Unsupported);
        }
        // Validate the parameters once, up front.
        rusty_flac::Encoder::new(format.sample_rate_hz, u32::from(format.channels), 16)
            .map_err(|_| Error::InvalidFormat)?;
        Ok(FlacEncoder {
            format,
            level: level.min(8),
            inner: None,
            pending_frames: 0,
            chunks: 0,
            bytes_out: 0,
            bytes_in: 0,
        })
    }

    /// The PCM format this encoder takes.
    #[must_use]
    pub fn format(&self) -> PcmFormat {
        self.format
    }

    fn encoder(&mut self) -> Result<&mut rusty_flac::Encoder> {
        if self.inner.is_none() {
            let mut e = rusty_flac::Encoder::new(
                self.format.sample_rate_hz,
                u32::from(self.format.channels),
                16,
            )
            .map_err(|_| Error::InvalidFormat)?;
            e.set_compression_level(self.level);
            self.inner = Some(e);
        }
        Ok(self.inner.as_mut().expect("just set"))
    }

    /// Add one block to the current chunk.
    pub fn push(&mut self, block: PcmBlock<'_>) -> Result<()> {
        if block.format != self.format {
            return Err(Error::InvalidFormat);
        }
        let frames = block.frames();
        self.encoder()?
            .push_s16le_bytes(block.data)
            .map_err(|_| Error::InvalidGeometry)?;
        self.pending_frames += frames;
        self.bytes_in += block.data.len() as u64;
        Ok(())
    }

    /// Close the current chunk and return it as a complete FLAC stream.
    /// Returns `None` when nothing was pushed since the last chunk.
    pub fn finish(&mut self) -> Option<Vec<u8>> {
        let e = self.inner.take()?;
        let stream = e.finish();
        self.pending_frames = 0;
        self.chunks += 1;
        self.bytes_out += stream.len() as u64;
        Some(stream)
    }
}

/// Encode one i16 block as a standalone FLAC stream.
pub fn encode_pcm16(block: PcmBlock<'_>, level: u32) -> Result<Vec<u8>> {
    let mut e = FlacEncoder::new(block.format, level)?;
    e.push(block)?;
    e.finish().ok_or(Error::InvalidGeometry)
}

/// The most samples a FLAC byte can carry: a frame header is at least six
/// bytes and a block at most 65 535 samples, so a stream of `n` bytes cannot
/// hold more than about `n × 11 000` samples per channel. `STREAMINFO`
/// claiming more is corrupt, and is refused before anyone allocates for it.
const MAX_SAMPLES_PER_BYTE: u64 = 16 * 1024;

/// The `total_samples` field of a stream's `STREAMINFO`, if the stream is
/// long enough to have one: `fLaC`, the block header, then 34 bytes of which
/// the low nibble of byte 13 and the four bytes after it are the 36-bit
/// count (RFC 9639 §8.2).
fn streaminfo_total_samples(flac: &[u8]) -> Option<u64> {
    let info = flac.get(8..42)?;
    if &flac[..4] != b"fLaC" || flac[4] & 0x7F != 0 {
        return None;
    }
    let hi = u64::from(info[13] & 0x0F);
    let lo = u64::from(u32::from_be_bytes([info[14], info[15], info[16], info[17]]));
    Some((hi << 32) | lo)
}

/// Decode a FLAC stream to interleaved i16 little-endian PCM. Streams that
/// are not 16-bit are refused (`Unsupported`); corrupt ones are `Corrupt`,
/// including a header whose sample count no stream of that length could
/// hold (the decoder reserves memory from that count; a lie there must not
/// become an allocation).
pub fn decode_pcm16(flac: &[u8]) -> Result<(PcmFormat, Vec<u8>)> {
    let claimed = streaminfo_total_samples(flac).ok_or(Error::Corrupt)?;
    if claimed > flac.len() as u64 * MAX_SAMPLES_PER_BYTE {
        return Err(Error::Corrupt);
    }
    let (info, planes) = rusty_flac::decode(flac).map_err(|_| Error::Corrupt)?;
    if info.bits_per_sample != 16 || info.channels == 0 || info.channels > 8 {
        return Err(Error::Unsupported);
    }
    let format = PcmFormat::new(info.sample_rate, info.channels as u8, SampleFormat::I16)?;
    let frames = planes.first().map_or(0, Vec::len);
    let ch = planes.len();
    let mut out = Vec::with_capacity(frames * ch * 2);
    for i in 0..frames {
        for plane in &planes {
            let v = *plane.get(i).ok_or(Error::Corrupt)?;
            out.extend_from_slice(&(v as i16).to_le_bytes());
        }
    }
    Ok((format, out))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::{AudioSource, SineSource};
    use rusty_esp_core::time::Micros;

    #[test]
    fn round_trips_and_is_deterministic() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut src = SineSource::new(f, 440.0, 12_000).unwrap();
        let mut buf = [0u8; 640];
        let mut pcm = Vec::new();
        let mut enc = FlacEncoder::new(f, 5).unwrap();
        for _ in 0..50 {
            let blk = src.read(&mut buf).unwrap();
            enc.push(blk).unwrap();
            pcm.extend_from_slice(&buf);
        }
        assert_eq!(enc.pending_frames, 16_000);
        let stream = enc.finish().unwrap();
        assert!(enc.finish().is_none());
        assert_eq!(&stream[..4], b"fLaC");
        assert!(
            stream.len() > STREAM_HEADER_LEN && stream.len() < pcm.len() / 2,
            "{}",
            stream.len()
        );
        let (back_f, back) = decode_pcm16(&stream).unwrap();
        assert_eq!(back_f, f);
        assert_eq!(back, pcm);
        // Same samples, same bytes.
        let again = encode_pcm16(PcmBlock::new(f, Micros::ZERO, &pcm).unwrap(), 5).unwrap();
        assert_eq!(again, stream);
        assert_eq!(enc.chunks, 1);
        assert_eq!(enc.bytes_out, stream.len() as u64);
    }

    #[test]
    fn refuses_the_wrong_format() {
        let f32fmt = PcmFormat::new(16_000, 1, SampleFormat::F32).unwrap();
        assert_eq!(FlacEncoder::new(f32fmt, 5).err(), Some(Error::Unsupported));
        let mut e = FlacEncoder::new(PcmFormat::PCM16_16K_MONO, 5).unwrap();
        let stereo = PcmBlock::new(PcmFormat::PCM16_48K_STEREO, Micros::ZERO, &[0u8; 8]).unwrap();
        assert_eq!(e.push(stereo).err(), Some(Error::InvalidFormat));
        assert_eq!(decode_pcm16(b"not flac at all").err(), Some(Error::Corrupt));
    }
}
