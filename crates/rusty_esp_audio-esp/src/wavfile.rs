//! WAV files over `std::fs`: a writer that is an [`AudioSink`] and patches
//! its sizes on close, and a whole-file reader for tools and tests.

use std::fs::File;
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::Path;

use rusty_esp_audio_core::AudioSink;
use rusty_esp_audio_core::codec::wav::{MAX_HEADER_LEN, WavCodec, WavHeader};
use rusty_esp_audio_core::esp_core::error::{Error, Result};
use rusty_esp_audio_core::esp_core::pcm::{PcmBlock, PcmFormat};

/// Writes PCM blocks to a WAV file. Call [`finish`](Self::finish) (or drop)
/// to patch the length fields; until then the header says "unknown length".
#[derive(Debug)]
pub struct WavWriter {
    file: Option<File>,
    header: WavHeader,
    /// Payload bytes written so far.
    pub data_len: u64,
    /// Blocks written.
    pub blocks: u64,
}

impl WavWriter {
    /// Create (or truncate) `path` for PCM in `format`.
    pub fn create(path: impl AsRef<Path>, format: PcmFormat) -> Result<Self> {
        let mut file = File::create(path).map_err(|_| Error::Hardware)?;
        let header = WavHeader::pcm(format, u32::MAX);
        let mut buf = [0u8; MAX_HEADER_LEN];
        let n = header.write(&mut buf)?;
        file.write_all(&buf[..n]).map_err(|_| Error::Hardware)?;
        Ok(WavWriter {
            file: Some(file),
            header,
            data_len: 0,
            blocks: 0,
        })
    }

    /// Append raw frames in the file's format.
    pub fn write_bytes(&mut self, data: &[u8]) -> Result<()> {
        let file = self.file.as_mut().ok_or(Error::Busy)?;
        file.write_all(data).map_err(|_| Error::Hardware)?;
        self.data_len += data.len() as u64;
        Ok(())
    }

    /// Patch the RIFF and data sizes and close the file. Returns the total
    /// file length. Payloads over 4 GiB are saturated (WAV cannot say more).
    pub fn finish(&mut self) -> Result<u64> {
        let mut file = self.file.take().ok_or(Error::Busy)?;
        let data = u32::try_from(self.data_len).unwrap_or(u32::MAX - 1);
        let header_len = self.header.len() as u32;
        let (riff_off, data_off) = self.header.size_field_offsets();
        file.seek(SeekFrom::Start(riff_off as u64))
            .map_err(|_| Error::Hardware)?;
        file.write_all(&(header_len - 8 + data).to_le_bytes())
            .map_err(|_| Error::Hardware)?;
        file.seek(SeekFrom::Start(data_off as u64))
            .map_err(|_| Error::Hardware)?;
        file.write_all(&data.to_le_bytes())
            .map_err(|_| Error::Hardware)?;
        file.flush().map_err(|_| Error::Hardware)?;
        Ok(u64::from(header_len) + self.data_len)
    }
}

impl Drop for WavWriter {
    fn drop(&mut self) {
        if self.file.is_some() {
            let _ = self.finish();
        }
    }
}

impl AudioSink for WavWriter {
    fn format(&self) -> PcmFormat {
        match self.header.codec {
            WavCodec::Pcm(f) => f,
            WavCodec::ImaAdpcm { .. } => PcmFormat::PCM16_16K_MONO,
        }
    }

    fn write(&mut self, block: PcmBlock<'_>) -> Result<()> {
        if block.format != self.format() {
            return Err(Error::InvalidFormat);
        }
        self.write_bytes(block.data)?;
        self.blocks += 1;
        Ok(())
    }
}

/// Read a whole WAV file: its header and the data chunk bytes (truncated to
/// the declared length when the file is longer, the whole rest when the
/// header says "unknown").
pub fn read_all(path: impl AsRef<Path>) -> Result<(WavHeader, Vec<u8>)> {
    let mut bytes = Vec::new();
    File::open(path)
        .and_then(|mut f| f.read_to_end(&mut bytes))
        .map_err(|_| Error::Hardware)?;
    let (header, off) = WavHeader::parse(&bytes)?;
    let rest = &bytes[off.min(bytes.len())..];
    let take = if header.data_len == u32::MAX {
        rest.len()
    } else {
        (header.data_len as usize).min(rest.len())
    };
    Ok((header, rest[..take].to_vec()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use rusty_esp_audio_core::esp_core::time::Micros;

    #[test]
    fn writes_patches_and_reads_back() {
        let dir = std::env::temp_dir().join(format!("janus-wav-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("t.wav");
        let f = PcmFormat::PCM16_16K_MONO;
        let data: Vec<u8> = (0..640u32).map(|i| (i * 7) as u8).collect();
        {
            let mut w = WavWriter::create(&path, f).unwrap();
            w.write(PcmBlock::new(f, Micros::ZERO, &data).unwrap())
                .unwrap();
            w.write(PcmBlock::new(f, Micros::ZERO, &data).unwrap())
                .unwrap();
            assert_eq!(w.finish().unwrap(), 44 + 1280);
            assert_eq!(w.finish().err(), Some(Error::Busy));
        }
        let (h, d) = read_all(&path).unwrap();
        assert_eq!(h, WavHeader::pcm(f, 1280));
        assert_eq!(d.len(), 1280);
        assert_eq!(&d[..640], &data[..]);
        let _ = std::fs::remove_dir_all(&dir);
    }
}
