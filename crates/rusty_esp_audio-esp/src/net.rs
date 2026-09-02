//! Raw PCM over UDP: the simplest transport that a player already understands.
//!
//! Each datagram is a whole number of interleaved sample frames in the
//! stream's format, nothing else — so `ffplay -f s16le -ar 16000 -ch_layout
//! mono -i udp://0.0.0.0:5004` plays a device live, and `rff`/ffmpeg record
//! it. There is no header, so the receiver must know the format (the
//! device's capability manifest carries it) and loss shows up as a gap, not
//! a glitch report. A sequence-numbered variant comes with `janus/media/1`
//! over iroh; this one is for the LAN and the kill test.

use std::net::{SocketAddr, ToSocketAddrs, UdpSocket};
use std::time::{Duration, Instant};

use rusty_esp_audio_core::esp_core::error::{Error, Result};
use rusty_esp_audio_core::esp_core::pcm::{PcmBlock, PcmFormat};
use rusty_esp_audio_core::esp_core::time::Micros;
use rusty_esp_audio_core::{AudioSink, AudioSource};

/// Default payload ceiling per datagram: fits an Ethernet MTU with room for
/// IPv6 and a VPN header. 1200 bytes is 600 mono i16 frames (37.5 ms at 16 kHz).
pub const DEFAULT_DATAGRAM_BYTES: usize = 1200;

/// Sends PCM blocks as raw datagrams.
#[derive(Debug)]
pub struct UdpPcmSender {
    socket: UdpSocket,
    dest: SocketAddr,
    format: PcmFormat,
    datagram_bytes: usize,
    /// Datagrams sent.
    pub datagrams: u64,
    /// Payload bytes sent.
    pub bytes: u64,
    /// `send_to` failures (the block, or the rest of it, was dropped).
    pub failed: u64,
}

impl UdpPcmSender {
    /// Bind `bind` (use `0.0.0.0:0` for any port) and address `dest`.
    pub fn new(
        bind: impl ToSocketAddrs,
        dest: impl ToSocketAddrs,
        format: PcmFormat,
    ) -> Result<Self> {
        let socket = UdpSocket::bind(bind).map_err(|_| Error::Hardware)?;
        let dest = dest
            .to_socket_addrs()
            .map_err(|_| Error::InvalidFormat)?
            .next()
            .ok_or(Error::InvalidFormat)?;
        Ok(UdpPcmSender {
            socket,
            dest,
            format,
            datagram_bytes: DEFAULT_DATAGRAM_BYTES,
            datagrams: 0,
            bytes: 0,
            failed: 0,
        })
    }

    /// Change the payload ceiling (rounded down to whole frames; at least one).
    #[must_use]
    pub fn with_datagram_bytes(mut self, bytes: usize) -> Self {
        self.datagram_bytes = bytes.max(self.format.frame_bytes());
        self
    }

    /// Frames per datagram.
    #[must_use]
    pub fn datagram_frames(&self) -> usize {
        (self.datagram_bytes / self.format.frame_bytes()).max(1)
    }

    /// The bound local address.
    pub fn local_addr(&self) -> Result<SocketAddr> {
        self.socket.local_addr().map_err(|_| Error::Hardware)
    }

    /// Where datagrams go.
    #[must_use]
    pub fn dest(&self) -> SocketAddr {
        self.dest
    }
}

impl AudioSink for UdpPcmSender {
    fn format(&self) -> PcmFormat {
        self.format
    }

    fn write(&mut self, block: PcmBlock<'_>) -> Result<()> {
        if block.format != self.format {
            return Err(Error::InvalidFormat);
        }
        let chunk = self.datagram_frames() * self.format.frame_bytes();
        for part in block.data.chunks(chunk) {
            match self.socket.send_to(part, self.dest) {
                Ok(n) if n == part.len() => {
                    self.datagrams += 1;
                    self.bytes += n as u64;
                }
                _ => {
                    self.failed += 1;
                    return Err(Error::Busy);
                }
            }
        }
        Ok(())
    }
}

/// Receives raw PCM datagrams as blocks, stamped with the receiver's own
/// monotonic clock (the sender's timestamps do not travel in this transport).
#[derive(Debug)]
pub struct UdpPcmReceiver {
    socket: UdpSocket,
    format: PcmFormat,
    epoch: Instant,
    /// Datagrams received.
    pub datagrams: u64,
    /// Payload bytes received.
    pub bytes: u64,
    /// Datagrams discarded because their length was not a whole number of frames.
    pub misaligned: u64,
    /// Address of the last sender.
    pub last_peer: Option<SocketAddr>,
}

impl UdpPcmReceiver {
    /// Bind `addr` and expect `format`. Blocks on `recv` until the timeout,
    /// 2 s by default.
    pub fn bind(addr: impl ToSocketAddrs, format: PcmFormat) -> Result<Self> {
        let socket = UdpSocket::bind(addr).map_err(|_| Error::Hardware)?;
        socket
            .set_read_timeout(Some(Duration::from_secs(2)))
            .map_err(|_| Error::Hardware)?;
        Ok(UdpPcmReceiver {
            socket,
            format,
            epoch: Instant::now(),
            datagrams: 0,
            bytes: 0,
            misaligned: 0,
            last_peer: None,
        })
    }

    /// How long `recv` waits before returning `Err(Timeout)`.
    pub fn set_timeout(&mut self, timeout: Duration) -> Result<()> {
        self.socket
            .set_read_timeout(Some(timeout))
            .map_err(|_| Error::Hardware)
    }

    /// The bound local address.
    pub fn local_addr(&self) -> Result<SocketAddr> {
        self.socket.local_addr().map_err(|_| Error::Hardware)
    }

    /// Receive one datagram into `buf` (which must hold the largest datagram
    /// expected) and return it as a block. Misaligned datagrams are counted
    /// and skipped; `Err(Timeout)` when nothing arrived.
    pub fn recv<'b>(&mut self, buf: &'b mut [u8]) -> Result<PcmBlock<'b>> {
        loop {
            let (n, peer) = match self.socket.recv_from(buf) {
                Ok(x) => x,
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    return Err(Error::Timeout);
                }
                Err(_) => return Err(Error::Hardware),
            };
            self.last_peer = Some(peer);
            if n == 0 || n % self.format.frame_bytes() != 0 {
                self.misaligned += 1;
                continue;
            }
            self.datagrams += 1;
            self.bytes += n as u64;
            let ts = Micros(self.epoch.elapsed().as_micros() as u64);
            return PcmBlock::new(self.format, ts, &buf[..n]);
        }
    }
}

impl AudioSource for UdpPcmReceiver {
    fn format(&self) -> PcmFormat {
        self.format
    }

    /// Fill `out` exactly, concatenating datagrams; the block is stamped with
    /// the arrival time of its first datagram.
    fn read<'b>(&mut self, out: &'b mut [u8]) -> Result<PcmBlock<'b>> {
        let fb = self.format.frame_bytes();
        if out.is_empty() || out.len() % fb != 0 {
            return Err(Error::InvalidGeometry);
        }
        let mut filled = 0usize;
        let mut first_ts = None;
        while filled < out.len() {
            let room = out.len() - filled;
            // Receive into the remaining space; a datagram larger than the room
            // would be truncated by the OS, so refuse that geometry up front.
            let (n, peer) = match self.socket.recv_from(&mut out[filled..]) {
                Ok(x) => x,
                Err(e)
                    if matches!(
                        e.kind(),
                        std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut
                    ) =>
                {
                    return Err(Error::Timeout);
                }
                Err(_) => return Err(Error::Hardware),
            };
            self.last_peer = Some(peer);
            if n == 0 || n % fb != 0 || n > room {
                self.misaligned += 1;
                continue;
            }
            if first_ts.is_none() {
                first_ts = Some(Micros(self.epoch.elapsed().as_micros() as u64));
            }
            self.datagrams += 1;
            self.bytes += n as u64;
            filled += n;
        }
        PcmBlock::new(self.format, first_ts.unwrap_or(Micros::ZERO), out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rusty_esp_audio_core::source::SineSource;

    #[test]
    fn loopback_splits_into_frame_aligned_datagrams() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut rx = UdpPcmReceiver::bind("127.0.0.1:0", f).unwrap();
        let dest = rx.local_addr().unwrap();
        let mut tx = UdpPcmSender::new("127.0.0.1:0", dest, f)
            .unwrap()
            .with_datagram_bytes(301); // → 150 frames = 300 bytes
        assert_eq!(tx.datagram_frames(), 150);
        let mut src = SineSource::new(f, 440.0, 1000).unwrap();
        let mut buf = [0u8; 640];
        let blk = src.read(&mut buf).unwrap();
        tx.write(blk).unwrap();
        assert_eq!(tx.datagrams, 3); // 300 + 300 + 40
        let mut got = Vec::new();
        let mut d = [0u8; 1500];
        for _ in 0..3 {
            let b = rx.recv(&mut d).unwrap();
            got.extend_from_slice(b.data);
        }
        assert_eq!(got, buf);
        assert_eq!(rx.datagrams, 3);
        rx.set_timeout(Duration::from_millis(50)).unwrap();
        assert_eq!(rx.recv(&mut d).err(), Some(Error::Timeout));
    }

    #[test]
    fn read_reassembles_a_whole_block() {
        let f = PcmFormat::PCM16_48K_STEREO;
        let mut rx = UdpPcmReceiver::bind("127.0.0.1:0", f).unwrap();
        let dest = rx.local_addr().unwrap();
        let mut tx = UdpPcmSender::new("127.0.0.1:0", dest, f).unwrap();
        let data: Vec<u8> = (0..3840u32).map(|i| i as u8).collect(); // 20 ms
        tx.write(PcmBlock::new(f, Micros::ZERO, &data).unwrap())
            .unwrap();
        assert_eq!(tx.datagrams, 4); // 1200 × 3 + 240
        let mut out = vec![0u8; 3840];
        let b = rx.read(&mut out).unwrap();
        assert_eq!(b.data, &data[..]);
        assert_eq!(b.frames(), 960);
        assert_eq!(
            tx.write(PcmBlock::new(PcmFormat::PCM16_16K_MONO, Micros::ZERO, &data).unwrap())
                .err(),
            Some(Error::InvalidFormat)
        );
    }
}
