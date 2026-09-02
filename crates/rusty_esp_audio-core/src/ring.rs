//! `RingBuffer`: ESP-ADF's `ringbuf` remade as a whole-frame ring over a
//! caller-owned slice.
//!
//! The ring never splits a sample frame, counts every frame it had to drop,
//! and remembers its high-water mark, so a firmware can print the three
//! numbers that matter (capacity, peak fill, drops) before anyone reaches for
//! a clock. It is single-owner: a Track A firmware that needs a DMA task and a
//! network task shares it behind a `Mutex`; Track B hands it to one Embassy
//! task and signals the other.

use rusty_esp_core::error::{Error, Result};
use rusty_esp_core::pcm::PcmFormat;

/// A whole-frame byte ring over borrowed memory.
#[derive(Debug)]
pub struct RingBuffer<'m> {
    buf: &'m mut [u8],
    frame_bytes: usize,
    /// Usable bytes: a whole number of frames.
    cap: usize,
    /// Read position in bytes.
    head: usize,
    /// Bytes stored.
    len: usize,
    /// Frames refused or overwritten because the ring was full.
    pub dropped_frames: u64,
    /// Most frames ever stored at once.
    pub high_water_frames: usize,
}

impl<'m> RingBuffer<'m> {
    /// A ring over `buf` for frames of `format`. `buf` must hold at least one
    /// frame; a trailing partial frame is unused.
    pub fn new(buf: &'m mut [u8], format: PcmFormat) -> Result<Self> {
        let frame_bytes = format.frame_bytes();
        let cap = (buf.len() / frame_bytes) * frame_bytes;
        if cap == 0 {
            return Err(Error::BufferTooSmall {
                needed: frame_bytes,
            });
        }
        Ok(RingBuffer {
            buf,
            frame_bytes,
            cap,
            head: 0,
            len: 0,
            dropped_frames: 0,
            high_water_frames: 0,
        })
    }

    /// Frames the ring can hold.
    #[must_use]
    pub fn capacity_frames(&self) -> usize {
        self.cap / self.frame_bytes
    }

    /// Frames stored right now.
    #[must_use]
    pub fn available_frames(&self) -> usize {
        self.len / self.frame_bytes
    }

    /// Frames that can still be pushed without dropping.
    #[must_use]
    pub fn free_frames(&self) -> usize {
        (self.cap - self.len) / self.frame_bytes
    }

    /// True when nothing is stored.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Forget the contents (counters are kept).
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }

    fn check_frames(&self, data: &[u8]) -> Result<()> {
        if data.len() % self.frame_bytes != 0 {
            return Err(Error::InvalidGeometry);
        }
        if data.len() > self.cap {
            return Err(Error::BufferTooSmall { needed: data.len() });
        }
        Ok(())
    }

    fn write_at_tail(&mut self, data: &[u8]) {
        let tail = (self.head + self.len) % self.cap;
        let first = (self.cap - tail).min(data.len());
        self.buf[tail..tail + first].copy_from_slice(&data[..first]);
        let rest = data.len() - first;
        if rest > 0 {
            self.buf[..rest].copy_from_slice(&data[first..]);
        }
        self.len += data.len();
        let frames = self.len / self.frame_bytes;
        if frames > self.high_water_frames {
            self.high_water_frames = frames;
        }
    }

    /// Append whole frames. When they do not all fit, nothing is stored, the
    /// frames count as dropped and `Err(Busy)` comes back — the producer
    /// keeps running and the drop is on the record.
    pub fn push(&mut self, data: &[u8]) -> Result<()> {
        self.check_frames(data)?;
        if data.len() > self.cap - self.len {
            self.dropped_frames += (data.len() / self.frame_bytes) as u64;
            return Err(Error::Busy);
        }
        self.write_at_tail(data);
        Ok(())
    }

    /// Append whole frames, discarding the oldest stored frames to make room.
    /// Returns how many frames were discarded (also added to
    /// `dropped_frames`). Use it when latency matters more than continuity.
    pub fn push_overwrite(&mut self, data: &[u8]) -> Result<usize> {
        self.check_frames(data)?;
        let need = data.len().saturating_sub(self.cap - self.len);
        if need > 0 {
            self.head = (self.head + need) % self.cap;
            self.len -= need;
            let frames = need / self.frame_bytes;
            self.dropped_frames += frames as u64;
            self.write_at_tail(data);
            return Ok(frames);
        }
        self.write_at_tail(data);
        Ok(0)
    }

    /// Move up to `out.len()` bytes of whole frames out; returns bytes moved
    /// (0 when the ring is empty).
    pub fn pop(&mut self, out: &mut [u8]) -> usize {
        let want = (out.len() / self.frame_bytes) * self.frame_bytes;
        let n = want.min(self.len);
        if n == 0 {
            return 0;
        }
        let first = (self.cap - self.head).min(n);
        out[..first].copy_from_slice(&self.buf[self.head..self.head + first]);
        if n > first {
            out[first..n].copy_from_slice(&self.buf[..n - first]);
        }
        self.head = (self.head + n) % self.cap;
        self.len -= n;
        n
    }

    /// Like [`pop`](Self::pop) but only when `out.len()` bytes are stored;
    /// otherwise nothing moves and `false` comes back. This is how a consumer
    /// waits for one whole block.
    pub fn pop_exact(&mut self, out: &mut [u8]) -> bool {
        if out.len() % self.frame_bytes != 0 || out.len() > self.len || out.is_empty() {
            return false;
        }
        self.pop(out);
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fmt() -> PcmFormat {
        PcmFormat::PCM16_48K_STEREO // 4-byte frames
    }

    #[test]
    fn wraps_and_counts() {
        let mut mem = [0u8; 18]; // 4 frames usable, 2 bytes spare
        let mut r = RingBuffer::new(&mut mem, fmt()).unwrap();
        assert_eq!(r.capacity_frames(), 4);
        r.push(&[1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3]).unwrap();
        let mut out = [0u8; 8];
        assert_eq!(r.pop(&mut out), 8);
        assert_eq!(out, [1, 1, 1, 1, 2, 2, 2, 2]);
        // Now head is at frame 2; pushing 3 frames wraps around the end.
        r.push(&[4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6]).unwrap();
        assert_eq!(r.available_frames(), 4);
        assert_eq!(r.free_frames(), 0);
        assert_eq!(r.high_water_frames, 4);
        // Full: a push drops and reports Busy without storing anything.
        assert_eq!(r.push(&[9, 9, 9, 9]).err(), Some(Error::Busy));
        assert_eq!(r.dropped_frames, 1);
        let mut all = [0u8; 16];
        assert_eq!(r.pop(&mut all), 16);
        assert_eq!(all, [3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6]);
        assert!(r.is_empty());
        assert_eq!(r.pop(&mut all), 0);
    }

    #[test]
    fn overwrite_discards_oldest() {
        let mut mem = [0u8; 12];
        let mut r = RingBuffer::new(&mut mem, fmt()).unwrap();
        r.push(&[1, 1, 1, 1, 2, 2, 2, 2]).unwrap();
        assert_eq!(r.push_overwrite(&[3, 3, 3, 3, 4, 4, 4, 4]).unwrap(), 1);
        assert_eq!(r.dropped_frames, 1);
        let mut out = [0u8; 12];
        assert_eq!(r.pop(&mut out), 12);
        assert_eq!(out, [2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4]);
    }

    #[test]
    fn pop_exact_waits_for_a_block() {
        let mut mem = [0u8; 16];
        let mut r = RingBuffer::new(&mut mem, fmt()).unwrap();
        r.push(&[7; 4]).unwrap();
        let mut out = [0u8; 8];
        assert!(!r.pop_exact(&mut out));
        r.push(&[8; 4]).unwrap();
        assert!(r.pop_exact(&mut out));
        assert_eq!(out, [7, 7, 7, 7, 8, 8, 8, 8]);
    }

    #[test]
    fn rejects_misaligned_and_oversize() {
        let mut mem = [0u8; 16];
        let mut r = RingBuffer::new(&mut mem, fmt()).unwrap();
        assert_eq!(r.push(&[0; 6]).err(), Some(Error::InvalidGeometry));
        assert_eq!(
            r.push(&[0; 20]).err(),
            Some(Error::BufferTooSmall { needed: 20 })
        );
        let mut tiny = [0u8; 3];
        assert_eq!(
            RingBuffer::new(&mut tiny, fmt()).err(),
            Some(Error::BufferTooSmall { needed: 4 })
        );
    }
}
