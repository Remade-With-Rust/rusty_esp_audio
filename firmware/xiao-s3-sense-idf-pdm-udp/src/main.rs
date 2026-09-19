//! Janus J2 / A1 on the XIAO ESP32-S3 Sense: the PDM microphone, two ways.
//!
//! **With a destination compiled in** it is the J2 demo: microphone through a
//! voice front end and out as PCM over UDP, which `ffplay` reads directly.
//!
//! The front end is three stages, in the order a voice front end goes:
//! [`DcBlock`] takes out the PDM converter's offset, a [`Biquad`] high-pass
//! at 80 Hz takes out the rumble below speech, and [`Agc`] brings the level
//! up to something a far end can use. Each block is then reported with both
//! its RMS level and its PEAK: an RMS figure hides clipping, and make-up gain
//! is exactly where clipping appears.
//!
//! ```sh
//! JANUS_WIFI_SSID=mynet JANUS_WIFI_PASS=secret JANUS_AUDIO_DEST=192.168.1.20:5004 cargo run --release
//! ```
//!
//! **With none compiled in** it is the A1 bench: no Wi-Fi is started at all,
//! and the microphone is measured against the chip's own clock, then dumped
//! over the serial link for an oracle on the laptop. That path exists because
//! a bench without a 2.4 GHz access point can still answer most of the A1
//! row, and because the answer is better without a radio competing for the
//! CPU. `option_env!` rather than a feature: the same source, and which arm
//! runs is visible in the first line of the log.
//!
//! The bench runs **two passes, never one loop**. The first is timed and
//! writes nothing, so the block rate is the microphone's and not the serial
//! link's; the second captures a fixed number of bytes and dumps them, and
//! its timing is irrelevant. Fusing them would drag the rate down to whatever
//! the console can carry (codec-measurement 13).
//!
//! Lines are prefixed `MIC` and `WAV` so a monitor can parse them.

use std::fmt::Write as _;
use std::time::Instant;

use anyhow::{Context, Result};
use esp_idf_svc::eventloop::EspSystemEventLoop;
use esp_idf_svc::hal::peripherals::Peripherals;
use esp_idf_svc::log::EspLogger;
use esp_idf_svc::nvs::EspDefaultNvsPartition;
use esp_idf_svc::sys::link_patches;
use esp_idf_svc::wifi::{BlockingWifi, ClientConfiguration, Configuration, EspWifi};
use rusty_esp_audio_core::codec::wav::WavHeader;
use rusty_esp_audio_core::elements::{Agc, Biquad, BiquadKind, DcBlock, EnergyVad};
use rusty_esp_audio_core::esp_core::pcm::{as_i16, PcmFormat};
use rusty_esp_audio_core::{peak_abs_i16, rms_dbfs_i16, AudioSink, AudioSource, Element, Pipeline};
use rusty_esp_audio_esp::idf::PdmIn;
use rusty_esp_audio_esp::net::UdpPcmSender;

/// All three must be present for the network arm; any missing one is the
/// bench.
const SSID: Option<&str> = option_env!("JANUS_WIFI_SSID");
const PASS: Option<&str> = option_env!("JANUS_WIFI_PASS");
const DEST: Option<&str> = option_env!("JANUS_AUDIO_DEST");

/// 16 kHz mono i16: the voice format.
const FORMAT: PcmFormat = PcmFormat::PCM16_16K_MONO;
/// One block is 20 ms = 320 frames = 640 bytes: one datagram.
const BLOCK_BYTES: usize = 640;

/// Bench pass 1: how long the block-rate measurement runs, settable at build
/// time with `JANUS_MIC_SECS`. Ten seconds is enough to size the rate; the
/// A1 row asks for ten minutes, which is a soak and catches what a short run
/// cannot -- drift, a slow leak, an I2S channel that stalls once an hour.
const MEASURE_SECS_ENV: Option<&str> = option_env!("JANUS_MIC_SECS");
/// Default when nothing is set.
const MEASURE_SECS_DEFAULT: u64 = 10;
/// Bench pass 2: seconds of audio dumped for the laptop to judge.
const DUMP_SECS: usize = 2;

fn main() -> Result<()> {
    link_patches();
    EspLogger::initialize_default();

    let peripherals = Peripherals::take()?;

    // The radio is brought up only when there is somewhere to send. A bench
    // run never starts it, so nothing else is contending for the CPU while
    // the microphone is being timed.
    // The driver must outlive the send loop, so it is bound here rather
    // than inside the arm that builds it.
    let (dest, _wifi) = match (SSID, PASS, DEST) {
        (Some(ssid), Some(pass), Some(dest)) => {
            let sysloop = EspSystemEventLoop::take()?;
            let nvs = EspDefaultNvsPartition::take()?;
            let mut wifi = BlockingWifi::wrap(
                EspWifi::new(peripherals.modem, sysloop.clone(), Some(nvs))?,
                sysloop,
            )?;
            wifi.set_configuration(&Configuration::Client(ClientConfiguration {
                ssid: ssid
                    .try_into()
                    .map_err(|_| anyhow::anyhow!("SSID too long"))?,
                password: pass
                    .try_into()
                    .map_err(|_| anyhow::anyhow!("password too long"))?,
                ..Default::default()
            }))?;
            wifi.start()?;
            wifi.connect()?;
            wifi.wait_netif_up()?;
            let ip = wifi.wifi().sta_netif().get_ip_info()?.ip;
            println!("MIC mode=stream ip={ip} dest={dest}");
            (Some(dest), Some(wifi))
        }
        _ => {
            println!("MIC mode=bench radio=off reason=no-destination-compiled-in");
            (None, None)
        }
    };

    // XIAO ESP32-S3 Sense microphone: PDM CLK on GPIO42, DATA on GPIO41.
    let mut mic = PdmIn::new(
        peripherals.i2s0,
        peripherals.pins.gpio42,
        peripherals.pins.gpio41,
        FORMAT.sample_rate_hz,
    )
    .context("pdm mic")?;

    // A voice front end, in the order a voice front end goes: remove the
    // PDM converter's DC offset, then the rumble below speech, then bring the
    // level up to something a far end can use.
    //
    // It was one stage (DcBlock) until the reachability census: a microphone
    // that ships raw is not what this crate's elements are for, and every one
    // of them but DcBlock had no caller outside a test.
    let mut dc = DcBlock::new();
    let mut hp = Biquad::new(BiquadKind::HighPass {
        f0: 80.0,
        q: core::f32::consts::FRAC_1_SQRT_2,
    });
    let mut agc = Agc::default();
    let mut pipeline = Pipeline::new([
        &mut dc as &mut dyn Element,
        &mut hp as &mut dyn Element,
        &mut agc as &mut dyn Element,
    ]);
    let mut vad = EnergyVad::default();
    let mut buf = vec![0u8; BLOCK_BYTES];
    let mut scratch = vec![
        0u8;
        pipeline
            .scratch_bytes(FORMAT, BLOCK_BYTES)
            .context("scratch")?
    ];

    match dest {
        Some(dest) => stream(
            &mut mic,
            &mut pipeline,
            &mut vad,
            &mut buf,
            &mut scratch,
            dest,
        ),
        None => bench(&mut mic, &mut pipeline, &mut vad, &mut buf, &mut scratch),
    }
}

/// The J2 demo: every block out as a datagram, counters once a second.
fn stream(
    mic: &mut PdmIn<'_>,
    pipeline: &mut Pipeline<'_, 3>,
    vad: &mut EnergyVad,
    buf: &mut [u8],
    scratch: &mut [u8],
    dest: &str,
) -> Result<()> {
    let mut tx = UdpPcmSender::new("0.0.0.0:0", dest, FORMAT).context("udp")?;
    let started = Instant::now();
    let mut blocks: u64 = 0;
    let mut dropped: u64 = 0;
    loop {
        let block = match mic.read(buf) {
            Ok(b) => b,
            Err(e) => {
                log::warn!("mic read: {e}");
                continue;
            }
        };
        let Some(out) = pipeline.process(block, scratch).context("pipeline")? else {
            continue;
        };
        let level = rms_dbfs_i16(out.data);
        // RMS hides clipping, and a front end with make-up gain is exactly
        // where clipping appears. `peak_abs_i16` is 32 768 at full scale.
        let peak = as_i16(out.data).map_or(0, peak_abs_i16);
        vad.judge(level);
        if tx.write(out).is_err() {
            dropped += 1;
        }
        blocks += 1;
        if blocks % 50 == 0 {
            let secs = started.elapsed().as_secs_f32().max(0.001);
            println!(
                "MIC blocks={blocks} rate={:.2}/s dropped={dropped} level={level:.1}dBFS peak={peak} speech={}/{} short_reads={}",
                blocks as f32 / secs,
                vad.speech_blocks,
                vad.blocks,
                mic.short_reads
            );
        }
    }
}

/// The A1 bench: rate first with nothing written, then a fixed dump.
fn bench(
    mic: &mut PdmIn<'_>,
    pipeline: &mut Pipeline<'_, 3>,
    vad: &mut EnergyVad,
    buf: &mut [u8],
    scratch: &mut [u8],
) -> Result<()> {
    let measure_secs = MEASURE_SECS_ENV
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(MEASURE_SECS_DEFAULT);
    println!(
        "MIC pass=rate secs={measure_secs} rate_hz={} channels={} block_bytes={BLOCK_BYTES}",
        FORMAT.sample_rate_hz, FORMAT.channels
    );

    // One untimed block: the first read pays for the channel starting up.
    let _ = mic.read(buf);

    let started = Instant::now();
    let mut blocks: u64 = 0;
    let mut errors: u64 = 0;
    let mut empty: u64 = 0;
    let mut level_min = f32::INFINITY;
    let mut level_max = f32::NEG_INFINITY;
    let mut peak_max: u16 = 0;
    while started.elapsed().as_secs() < measure_secs {
        let block = match mic.read(buf) {
            Ok(b) => b,
            Err(_) => {
                errors += 1;
                continue;
            }
        };
        let Some(out) = pipeline.process(block, scratch).context("pipeline")? else {
            empty += 1;
            continue;
        };
        let level = rms_dbfs_i16(out.data);
        level_min = level_min.min(level);
        level_max = level_max.max(level);
        // RMS hides clipping, and a front end with make-up gain is where
        // clipping appears. 32 768 is full scale.
        peak_max = peak_max.max(as_i16(out.data).map_or(0, peak_abs_i16));
        vad.judge(level);
        blocks += 1;
    }
    let elapsed = started.elapsed();
    let secs = elapsed.as_secs_f64().max(1e-9);

    // Work first, clock second: the block and sample counts are exact, the
    // rate is derived from them.
    let samples = blocks * (BLOCK_BYTES as u64) / FORMAT.frame_bytes() as u64;
    println!(
        "MIC work blocks={blocks} samples={samples} bytes={} driver_blocks={} short_reads={} read_errors={errors} pipeline_empty={empty} peak_max={peak_max}",
        blocks * BLOCK_BYTES as u64,
        mic.blocks,
        mic.short_reads
    );
    println!(
        "MIC rate us={} blocks_per_s={:.3} samples_per_s={:.1} nominal_hz={} ratio={:.5}",
        elapsed.as_micros(),
        blocks as f64 / secs,
        samples as f64 / secs,
        FORMAT.sample_rate_hz,
        (samples as f64 / secs) / f64::from(FORMAT.sample_rate_hz)
    );
    println!(
        "MIC level min={level_min:.1}dBFS max={level_max:.1}dBFS speech={}/{} ratio={:.3} threshold={:.1}dBFS",
        vad.speech_blocks,
        vad.blocks,
        vad.speech_blocks as f64 / (vad.blocks.max(1)) as f64,
        vad.config().threshold_dbfs
    );

    // Pass 2, deterministic: a fixed number of bytes, and the clock does not
    // matter because nothing here is a rate.
    let want = DUMP_SECS * FORMAT.sample_rate_hz as usize * FORMAT.frame_bytes();
    println!("MIC pass=dump bytes={want} secs={DUMP_SECS}");
    let mut pcm: Vec<u8> = Vec::with_capacity(want);
    while pcm.len() < want {
        let Ok(block) = mic.read(buf) else { continue };
        let Some(out) = pipeline.process(block, scratch).context("pipeline")? else {
            continue;
        };
        let take = (want - pcm.len()).min(out.data.len());
        pcm.extend_from_slice(&out.data[..take]);
    }

    // A WAV rather than raw: the header carries the rate and channel count,
    // so the laptop reads back what this firmware claims instead of being
    // told it on the command line.
    let header = WavHeader::pcm(FORMAT, pcm.len() as u32);
    let mut hdr = [0u8; rusty_esp_audio_core::codec::wav::MAX_HEADER_LEN];
    let n = header.write(&mut hdr).context("wav header")?;
    println!("WAV begin total={} header={n}", n + pcm.len());
    dump_hex(&hdr[..n]);
    dump_hex(&pcm);
    println!("WAV end");
    println!("== DONE ==");
    loop {
        std::thread::sleep(std::time::Duration::from_secs(1));
    }
}

/// Emit `bytes` as lines of hex, 64 bytes each, prefixed so a monitor can
/// pick them out of the log.
fn dump_hex(bytes: &[u8]) {
    const PER_LINE: usize = 64;
    let mut line = String::with_capacity(PER_LINE * 2);
    for chunk in bytes.chunks(PER_LINE) {
        line.clear();
        for b in chunk {
            let _ = write!(line, "{b:02x}");
        }
        println!("WAVDATA {line}");
    }
}
