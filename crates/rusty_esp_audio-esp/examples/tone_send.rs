//! Send a test tone as raw s16le datagrams, paced to real time — the
//! stand-in for a device while there is no microphone on the desk.
//!
//! ```sh
//! cargo run -p rusty_esp_audio-esp --features std --example tone_send -- 127.0.0.1:5004 5 440
//! ffplay -f s16le -ar 16000 -ch_layout mono -i udp://0.0.0.0:5004
//! ```

use std::time::{Duration, Instant};

use rusty_esp_audio_core::esp_core::pcm::PcmFormat;
use rusty_esp_audio_core::source::SineSource;
use rusty_esp_audio_core::{AudioSink, AudioSource};
use rusty_esp_audio_esp::net::UdpPcmSender;

fn main() {
    let mut args = std::env::args().skip(1);
    let dest = args.next().unwrap_or_else(|| "127.0.0.1:5004".to_string());
    let seconds: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(5);
    let freq: f32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(440.0);

    let format = PcmFormat::PCM16_16K_MONO;
    let mut src = SineSource::new(format, freq, 8000).expect("tone");
    let mut tx = UdpPcmSender::new("0.0.0.0:0", dest.as_str(), format).expect("socket");
    let block_ms = 20u64;
    let mut buf = vec![0u8; format.bytes_for_micros(block_ms * 1000)];
    let blocks = seconds * 1000 / block_ms;
    let start = Instant::now();
    eprintln!(
        "tone_send: {freq} Hz s16le {} Hz mono → {} for {seconds}s ({} B datagrams)",
        format.sample_rate_hz,
        tx.dest(),
        tx.datagram_frames() * format.frame_bytes()
    );
    for i in 0..blocks {
        let block = src.read(&mut buf).expect("block");
        if tx.write(block).is_err() {
            eprintln!("send failed at block {i}");
        }
        let due = start + Duration::from_millis((i + 1) * block_ms);
        if let Some(wait) = due.checked_duration_since(Instant::now()) {
            std::thread::sleep(wait);
        }
    }
    eprintln!(
        "tone_send: {} datagrams, {} bytes, {} failed, {:.2}s",
        tx.datagrams,
        tx.bytes,
        tx.failed,
        start.elapsed().as_secs_f32()
    );
}
