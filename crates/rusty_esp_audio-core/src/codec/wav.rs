//! RIFF/WAVE headers for PCM, IEEE float and IMA ADPCM, written into and
//! parsed from caller buffers. Enough for a recorder to write a file any
//! player opens, and for a reader to find the data.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::pcm::{PcmFormat, SampleFormat};

/// `WAVE_FORMAT_PCM`.
pub const TAG_PCM: u16 = 0x0001;
/// `WAVE_FORMAT_IEEE_FLOAT`.
pub const TAG_FLOAT: u16 = 0x0003;
/// `WAVE_FORMAT_IMA_ADPCM` (also `DVI_ADPCM`).
pub const TAG_IMA_ADPCM: u16 = 0x0011;

/// Longest header this module writes (IMA: 12 + 28 + 12 + 8).
pub const MAX_HEADER_LEN: usize = 60;

/// What the data chunk holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WavCodec {
    /// Integer or float PCM in `format`. `I24In32` is written as 32-bit PCM
    /// (it is one).
    Pcm(PcmFormat),
    /// IMA ADPCM blocks.
    ImaAdpcm {
        /// Frames per second.
        sample_rate_hz: u32,
        /// Channels.
        channels: u8,
        /// Bytes per block (`nBlockAlign`).
        block_align: u16,
        /// Frames per block (`wSamplesPerBlock`).
        frames_per_block: u16,
    },
}

/// A header ready to write or freshly parsed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct WavHeader {
    /// The codec and its parameters.
    pub codec: WavCodec,
    /// Bytes in the data chunk. Use `u32::MAX` while still recording; a
    /// recorder patches the two length fields when it closes the file.
    pub data_len: u32,
    /// Sample frames in the file (the `fact` chunk; written for non-PCM).
    pub total_frames: u32,
}

impl WavHeader {
    /// A PCM header for `data_len` bytes in `format`.
    #[must_use]
    pub fn pcm(format: PcmFormat, data_len: u32) -> Self {
        WavHeader {
            codec: WavCodec::Pcm(format),
            data_len,
            total_frames: data_len / format.frame_bytes() as u32,
        }
    }

    /// An IMA ADPCM header.
    #[must_use]
    pub fn ima_adpcm(
        sample_rate_hz: u32,
        channels: u8,
        block_align: u16,
        frames_per_block: u16,
        data_len: u32,
        total_frames: u32,
    ) -> Self {
        WavHeader {
            codec: WavCodec::ImaAdpcm {
                sample_rate_hz,
                channels,
                block_align,
                frames_per_block,
            },
            data_len,
            total_frames,
        }
    }

    /// Bytes [`write`](Self::write) produces for this header.
    #[must_use]
    pub const fn len(&self) -> usize {
        match self.codec {
            WavCodec::Pcm(f) => match f.sample {
                SampleFormat::F32 => 12 + 26 + 12 + 8,
                _ => 44,
            },
            WavCodec::ImaAdpcm { .. } => 12 + 28 + 12 + 8,
        }
    }

    /// Never empty; here for clippy's `len_without_is_empty`.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        false
    }

    /// Offsets (from the file start) of the RIFF size and data size fields,
    /// for patching after a recording of unknown length.
    #[must_use]
    pub const fn size_field_offsets(&self) -> (usize, usize) {
        (4, self.len() - 4)
    }

    /// Write the header into `out`; returns [`len`](Self::len).
    pub fn write(&self, out: &mut [u8]) -> Result<usize> {
        let len = self.len();
        if out.len() < len {
            return Err(Error::BufferTooSmall { needed: len });
        }
        let riff_len = if self.data_len == u32::MAX {
            u32::MAX
        } else {
            (len as u32 - 8).saturating_add(self.data_len)
        };
        let mut w = Writer { out, pos: 0 };
        w.bytes(b"RIFF");
        w.u32(riff_len);
        w.bytes(b"WAVE");
        w.bytes(b"fmt ");
        match self.codec {
            WavCodec::Pcm(f) => {
                let (tag, bits) = match f.sample {
                    SampleFormat::I16 => (TAG_PCM, 16u16),
                    SampleFormat::I24In32 | SampleFormat::I32 => (TAG_PCM, 32),
                    SampleFormat::F32 => (TAG_FLOAT, 32),
                    _ => return Err(Error::Unsupported),
                };
                let is_float = tag == TAG_FLOAT;
                w.u32(if is_float { 18 } else { 16 });
                w.u16(tag);
                w.u16(u16::from(f.channels));
                w.u32(f.sample_rate_hz);
                w.u32(f.sample_rate_hz.saturating_mul(f.frame_bytes() as u32));
                w.u16(f.frame_bytes() as u16);
                w.u16(bits);
                if is_float {
                    w.u16(0); // cbSize
                    w.bytes(b"fact");
                    w.u32(4);
                    w.u32(self.total_frames);
                }
            }
            WavCodec::ImaAdpcm {
                sample_rate_hz,
                channels,
                block_align,
                frames_per_block,
            } => {
                w.u32(20);
                w.u16(TAG_IMA_ADPCM);
                w.u16(u16::from(channels));
                w.u32(sample_rate_hz);
                // Average bytes per second: rate · block_align / frames_per_block.
                let avg = (u64::from(sample_rate_hz) * u64::from(block_align)
                    / u64::from(frames_per_block.max(1))) as u32;
                w.u32(avg);
                w.u16(block_align);
                w.u16(4);
                w.u16(2); // cbSize
                w.u16(frames_per_block);
                w.bytes(b"fact");
                w.u32(4);
                w.u32(self.total_frames);
            }
        }
        w.bytes(b"data");
        w.u32(self.data_len);
        debug_assert_eq!(w.pos, len);
        Ok(len)
    }

    /// Parse a header from the start of a file. Returns the header and the
    /// offset of the first data byte. Unknown chunks before `data` are
    /// skipped; `data_len` is taken as written (may be `u32::MAX` or 0 for a
    /// stream still being recorded).
    pub fn parse(bytes: &[u8]) -> Result<(Self, usize)> {
        if bytes.len() < 12 || &bytes[..4] != b"RIFF" || &bytes[8..12] != b"WAVE" {
            return Err(Error::InvalidFormat);
        }
        let mut pos = 12;
        let mut fmt: Option<(u16, u16, u32, u16, u16, u16)> = None; // tag, ch, rate, align, bits, spb
        let mut total_frames = 0u32;
        while pos + 8 <= bytes.len() {
            let id = &bytes[pos..pos + 4];
            let size = u32::from_le_bytes([
                bytes[pos + 4],
                bytes[pos + 5],
                bytes[pos + 6],
                bytes[pos + 7],
            ]);
            let body = pos + 8;
            match id {
                b"fmt " => {
                    if size < 16 || body + size as usize > bytes.len() {
                        return Err(Error::InvalidFormat);
                    }
                    let r = |o: usize| u16::from_le_bytes([bytes[body + o], bytes[body + o + 1]]);
                    let r32 = |o: usize| {
                        u32::from_le_bytes([
                            bytes[body + o],
                            bytes[body + o + 1],
                            bytes[body + o + 2],
                            bytes[body + o + 3],
                        ])
                    };
                    let spb = if size >= 20 && r(16) >= 2 { r(18) } else { 0 };
                    fmt = Some((r(0), r(2), r32(4), r(12), r(14), spb));
                }
                b"fact" => {
                    if size >= 4 && body + 4 <= bytes.len() {
                        total_frames = u32::from_le_bytes([
                            bytes[body],
                            bytes[body + 1],
                            bytes[body + 2],
                            bytes[body + 3],
                        ]);
                    }
                }
                b"data" => {
                    let (tag, ch, rate, align, bits, spb) = fmt.ok_or(Error::InvalidFormat)?;
                    if ch == 0 || ch > 255 || rate == 0 {
                        return Err(Error::InvalidFormat);
                    }
                    let codec = match (tag, bits) {
                        (TAG_PCM, 16) => {
                            WavCodec::Pcm(PcmFormat::new(rate, ch as u8, SampleFormat::I16)?)
                        }
                        (TAG_PCM, 32) => {
                            WavCodec::Pcm(PcmFormat::new(rate, ch as u8, SampleFormat::I32)?)
                        }
                        (TAG_FLOAT, 32) => {
                            WavCodec::Pcm(PcmFormat::new(rate, ch as u8, SampleFormat::F32)?)
                        }
                        (TAG_IMA_ADPCM, 4) => {
                            if spb == 0 {
                                return Err(Error::InvalidFormat);
                            }
                            WavCodec::ImaAdpcm {
                                sample_rate_hz: rate,
                                channels: ch as u8,
                                block_align: align,
                                frames_per_block: spb,
                            }
                        }
                        _ => return Err(Error::Unsupported),
                    };
                    let total = if total_frames != 0 {
                        total_frames
                    } else if let WavCodec::Pcm(f) = codec {
                        if size == u32::MAX {
                            0
                        } else {
                            size / f.frame_bytes() as u32
                        }
                    } else {
                        0
                    };
                    return Ok((
                        WavHeader {
                            codec,
                            data_len: size,
                            total_frames: total,
                        },
                        body,
                    ));
                }
                _ => {}
            }
            // Chunks are word-aligned.
            pos = body
                .saturating_add(size as usize)
                .saturating_add(size as usize & 1);
        }
        Err(Error::InvalidFormat)
    }
}

struct Writer<'a> {
    out: &'a mut [u8],
    pos: usize,
}

impl Writer<'_> {
    fn bytes(&mut self, b: &[u8]) {
        self.out[self.pos..self.pos + b.len()].copy_from_slice(b);
        self.pos += b.len();
    }
    fn u16(&mut self, v: u16) {
        self.bytes(&v.to_le_bytes());
    }
    fn u32(&mut self, v: u32) {
        self.bytes(&v.to_le_bytes());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pcm16_header_is_the_canonical_44_bytes() {
        let h = WavHeader::pcm(PcmFormat::PCM16_16K_MONO, 32_000);
        let mut buf = [0u8; 44];
        assert_eq!(h.write(&mut buf).unwrap(), 44);
        assert_eq!(&buf[..4], b"RIFF");
        assert_eq!(
            u32::from_le_bytes([buf[4], buf[5], buf[6], buf[7]]),
            36 + 32_000
        );
        assert_eq!(&buf[8..16], b"WAVEfmt ");
        assert_eq!(buf[16], 16);
        assert_eq!(&buf[20..24], &[1, 0, 1, 0]); // PCM, mono
        assert_eq!(
            u32::from_le_bytes([buf[24], buf[25], buf[26], buf[27]]),
            16_000
        );
        assert_eq!(
            u32::from_le_bytes([buf[28], buf[29], buf[30], buf[31]]),
            32_000
        );
        assert_eq!(&buf[32..36], &[2, 0, 16, 0]);
        assert_eq!(&buf[36..40], b"data");
        assert_eq!(h.size_field_offsets(), (4, 40));
        let (back, off) = WavHeader::parse(&buf).unwrap();
        assert_eq!(back, h);
        assert_eq!(off, 44);
    }

    #[test]
    fn float_and_ima_headers_round_trip() {
        let f = PcmFormat::new(48_000, 2, SampleFormat::F32).unwrap();
        let h = WavHeader::pcm(f, 800);
        let mut buf = [0u8; MAX_HEADER_LEN];
        let n = h.write(&mut buf).unwrap();
        assert_eq!(n, 58);
        let (back, off) = WavHeader::parse(&buf[..n]).unwrap();
        assert_eq!(back, h);
        assert_eq!(off, n);
        assert_eq!(back.total_frames, 100);

        let ima = WavHeader::ima_adpcm(16_000, 1, 1024, 2041, 8192, 16_328);
        let n = ima.write(&mut buf).unwrap();
        assert_eq!(n, 60);
        // avg bytes/s = 16000·1024/2041 = 8027
        assert_eq!(
            u32::from_le_bytes([buf[28], buf[29], buf[30], buf[31]]),
            8027
        );
        let (back, off) = WavHeader::parse(&buf[..n]).unwrap();
        assert_eq!(back, ima);
        assert_eq!(off, 60);
    }

    #[test]
    fn parse_skips_unknown_chunks_and_rejects_garbage() {
        let h = WavHeader::pcm(PcmFormat::PCM16_48K_STEREO, 4);
        let mut buf = [0u8; 44];
        h.write(&mut buf).unwrap();
        // Splice a LIST chunk (odd size, padded) between fmt and data.
        let mut with_list = Vec::new();
        with_list.extend_from_slice(&buf[..36]);
        with_list.extend_from_slice(b"LIST");
        with_list.extend_from_slice(&3u32.to_le_bytes());
        with_list.extend_from_slice(&[1, 2, 3, 0]);
        with_list.extend_from_slice(&buf[36..]);
        let (back, off) = WavHeader::parse(&with_list).unwrap();
        assert_eq!(back, h);
        assert_eq!(off, 56);
        assert_eq!(WavHeader::parse(b"RIFX").err(), Some(Error::InvalidFormat));
        let mut tiny = [0u8; 10];
        assert_eq!(
            h.write(&mut tiny).err(),
            Some(Error::BufferTooSmall { needed: 44 })
        );
    }
}
