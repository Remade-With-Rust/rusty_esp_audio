//! Record raw s16le datagrams from a device into a WAV file — the Pi hub's
//! record path for J2, no `rff` needed.
//!
//! ```sh
//! cargo run -p rusty_esp_audio-esp --features std --example pcm_record -- 0.0.0.0:5004 mic.wav 10
//! ffprobe mic.wav
//! ```

use std::time::{Duration, Instant};

use rusty_esp_audio_core::AudioSink;
use rusty_esp_audio_core::esp_core::pcm::PcmFormat;
use rusty_esp_audio_esp::net::UdpPcmReceiver;
use rusty_esp_audio_esp::wavfile::WavWriter;

fn main() {
    let mut args = std::env::args().skip(1);
    let bind = args.next().unwrap_or_else(|| "0.0.0.0:5004".to_string());
    let path = args.next().unwrap_or_else(|| "mic.wav".to_string());
    let seconds: u64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(10);
    let rate: u32 = args.next().and_then(|s| s.parse().ok()).unwrap_or(16_000);

    let format = PcmFormat::new(
        rate,
        1,
        rusty_esp_audio_core::esp_core::pcm::SampleFormat::I16,
    )
    .expect("format");
    let mut rx = UdpPcmReceiver::bind(bind.as_str(), format).expect("bind");
    rx.set_timeout(Duration::from_secs(5)).expect("timeout");
    let mut wav = WavWriter::create(&path, format).expect("create");
    let mut buf = vec![0u8; 4096];
    let deadline = Instant::now() + Duration::from_secs(seconds);
    eprintln!(
        "pcm_record: listening on {} for {seconds}s → {path}",
        rx.local_addr().expect("addr")
    );
    let mut first = None;
    while Instant::now() < deadline {
        match rx.recv(&mut buf) {
            Ok(block) => {
                if first.is_none() {
                    first = Some(Instant::now());
                    eprintln!("pcm_record: first datagram from {:?}", rx.last_peer);
                }
                wav.write(block).expect("write");
            }
            Err(e) => {
                eprintln!("pcm_record: {e}");
                if first.is_some() {
                    break;
                }
            }
        }
    }
    let total = wav.finish().expect("finish");
    let secs = rx.bytes as f64 / f64::from(format.sample_rate_hz * format.frame_bytes() as u32);
    eprintln!(
        "pcm_record: {} datagrams, {} bytes ({secs:.2}s of audio), {} misaligned → {path} ({total} B)",
        rx.datagrams, rx.bytes, rx.misaligned
    );
}
