//! Janus J2 on the XIAO ESP32-S3 Sense: PDM microphone → raw PCM over UDP.
//!
//! The kill test: on the laptop,
//! `ffplay -f s16le -ar 16000 -ch_layout mono -i udp://0.0.0.0:5004` plays
//! the room, next to the J1 camera stream; this log prints blocks, dropped
//! datagrams and the VAD's speech ratio every second. Credentials and the
//! destination are compile-time:
//!
//! ```sh
//! JANUS_WIFI_SSID=mynet JANUS_WIFI_PASS=secret JANUS_AUDIO_DEST=192.168.1.20:5004 cargo run --release
//! ```

use std::time::Instant;

use anyhow::{Context, Result};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::link_patches;
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use rusty_esp_audio_core::elements::{DcBlock, EnergyVad};
use rusty_esp_audio_core::esp_core::pcm::PcmFormat;
use rusty_esp_audio_core::{rms_dbfs_i16, AudioSink, AudioSource, Element, Pipeline};
use rusty_esp_audio_esp::idf::PdmIn;
use rusty_esp_audio_esp::net::UdpPcmSender;

const SSID: &str = env!("JANUS_WIFI_SSID");
const PASS: &str = env!("JANUS_WIFI_PASS");
/// `host:port` of the laptop or Pi that plays or records the stream.
const DEST: &str = env!("JANUS_AUDIO_DEST");

/// 16 kHz mono i16: the voice format.
const FORMAT: PcmFormat = PcmFormat::PCM16_16K_MONO;
/// One block is 20 ms = 320 frames = 640 bytes: one datagram.
const BLOCK_BYTES: usize = 640;

fn main() -> Result<()> {
    link_patches();
    EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;
    let sysloop = EspSystemEventLoop::take()?;
    let nvs = EspDefaultNvsPartition::take()?;

    let mut wifi = BlockingWifi::wrap(
        EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?,
        sysloop,
    )?;
    wifi.set_configuration(&Configuration::Client(ClientConfiguration {
        ssid: SSID
            .try_into()
            .map_err(|_| anyhow::anyhow!("SSID too long"))?,
        password: PASS
            .try_into()
            .map_err(|_| anyhow::anyhow!("password too long"))?,
        ..Default::default()
    }))?;
    wifi.start()?;
    wifi.connect()?;
    wifi.wait_netif_up()?;
    let ip = wifi.wifi().sta_netif().get_ip_info()?.ip;
    log::info!("janus j2: {ip} → raw s16le 16 kHz mono to udp://{DEST}");

    // XIAO ESP32-S3 Sense microphone: PDM CLK on GPIO42, DATA on GPIO41.
    let mut mic = PdmIn::new(
        peripherals.i2s0,
        peripherals.pins.gpio42,
        peripherals.pins.gpio41,
        FORMAT.sample_rate_hz,
    )
    .context("pdm mic")?;
    let mut tx = UdpPcmSender::new("0.0.0.0:0", DEST, FORMAT).context("udp")?;

    let mut dc = DcBlock::new();
    let mut pipeline = Pipeline::new([&mut dc as &mut dyn Element]);
    let mut vad = EnergyVad::default();
    let mut buf = vec![0u8; BLOCK_BYTES];
    let mut scratch = vec![
        0u8;
        pipeline
            .scratch_bytes(FORMAT, BLOCK_BYTES)
            .context("scratch")?
    ];

    let started = Instant::now();
    let mut blocks: u64 = 0;
    let mut dropped: u64 = 0;
    loop {
        let block = match mic.read(&mut buf) {
            Ok(b) => b,
            Err(e) => {
                log::warn!("mic read: {e}");
                continue;
            }
        };
        let Some(out) = pipeline.process(block, &mut scratch).context("pipeline")? else {
            continue;
        };
        let level = rms_dbfs_i16(out.data);
        vad.judge(level);
        if tx.write(out).is_err() {
            dropped += 1;
        }
        blocks += 1;
        if blocks % 50 == 0 {
            let secs = started.elapsed().as_secs_f32().max(0.001);
            log::info!(
                "blocks={blocks} ({:.1}/s) dropped={dropped} level={level:.1} dBFS speech={} vad={}/{} short_reads={}",
                blocks as f32 / secs,
                vad.is_speech(),
                vad.speech_blocks,
                vad.blocks,
                mic.short_reads
            );
        }
    }
}
