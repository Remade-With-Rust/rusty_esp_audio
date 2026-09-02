//! External oracles for the audio core, run on the host:
//!
//! - **ffmpeg** (`adpcm_ima_wav`, `swresample`, `highpass`/`lowpass`) and
//!   **ffprobe** for IMA ADPCM byte-identity in both directions, PCM
//!   conversion byte-identity, the biquad tolerance, raw PCM over UDP, and
//!   the WAV files the recorder writes;
//! - **scipy** (`signal.lfilter` in double) for the biquad tolerance.
//!
//! A missing tool skips its test with a loud line instead of failing, so the
//! host matrix stays green on runners without ffmpeg; the ledger records the
//! run on a machine that has it.

use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::time::Duration;

use rusty_esp_audio_core::codec::adpcm_ima::{
    Decoder, Encoder, ffmpeg_block_align, frames_per_block,
};
use rusty_esp_audio_core::codec::pcm::convert;
use rusty_esp_audio_core::codec::wav::{MAX_HEADER_LEN, WavCodec, WavHeader};
use rusty_esp_audio_core::elements::{Biquad, BiquadKind};
use rusty_esp_audio_core::esp_core::pcm::{PcmBlock, PcmFormat, SampleFormat};
use rusty_esp_audio_core::esp_core::time::Micros;
use rusty_esp_audio_core::source::SineSource;
use rusty_esp_audio_core::{AudioSink, AudioSource, Element};
use rusty_esp_audio_esp::net::{UdpPcmReceiver, UdpPcmSender};
use rusty_esp_audio_esp::wavfile::{WavWriter, read_all};

fn have(tool: &str, probe: &[&str]) -> bool {
    let ok = Command::new(tool)
        .args(probe)
        .stdout(Stdio::null())
        .stderr(Stdio::null())
        .status()
        .map(|s| s.success())
        .unwrap_or(false);
    if !ok {
        eprintln!("{tool} not usable on this machine; external oracle skipped");
    }
    ok
}

fn tmp(name: &str) -> PathBuf {
    let dir = std::env::temp_dir().join(format!("janus-audio-oracle-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    dir.join(name)
}

fn ffmpeg(args: &[&str]) {
    let out = Command::new("ffmpeg")
        .args(["-hide_banner", "-loglevel", "error", "-y"])
        .args(args)
        .output()
        .expect("run ffmpeg");
    assert!(
        out.status.success(),
        "ffmpeg {:?} failed: {}",
        args,
        String::from_utf8_lossy(&out.stderr)
    );
}

/// A deterministic test signal: two tones plus LCG noise, per channel different.
fn signal(frames: usize, channels: usize) -> Vec<u8> {
    let mut lcg: u32 = 0x1234_5678;
    let mut out = Vec::with_capacity(frames * channels * 2);
    for i in 0..frames {
        let t = i as f32 / 16_000.0;
        for c in 0..channels {
            lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            let noise = ((lcg >> 16) as i32 - 32_768) as f32 / 32_768.0;
            let v = if c == 0 {
                12_000.0 * (core::f32::consts::TAU * 440.0 * t).sin()
                    + 3_000.0 * (core::f32::consts::TAU * 1_234.0 * t).sin()
                    + 2_000.0 * noise
            } else {
                6_000.0 * (core::f32::consts::TAU * 200.0 * t).sin() + 8_000.0 * noise
            };
            out.extend_from_slice(&(v.round() as i16).to_le_bytes());
        }
    }
    out
}

fn first_mismatch(a: &[u8], b: &[u8]) -> Option<usize> {
    if a.len() != b.len() {
        return Some(a.len().min(b.len()));
    }
    a.iter().zip(b).position(|(x, y)| x != y)
}

/// Encode `pcm` with ffmpeg's `adpcm_ima_wav`; return the block geometry it
/// chose and the data chunk.
fn ffmpeg_ima(pcm: &[u8], channels: u8, name: &str) -> (PathBuf, usize, usize, Vec<u8>) {
    let raw = tmp(&format!("{name}{channels}.raw"));
    std::fs::write(&raw, pcm).unwrap();
    let ff_wav = tmp(&format!("{name}{channels}.wav"));
    let ch = channels.to_string();
    ffmpeg(&[
        "-f",
        "s16le",
        "-ar",
        "16000",
        "-ac",
        &ch,
        "-i",
        raw.to_str().unwrap(),
        "-c:a",
        "adpcm_ima_wav",
        ff_wav.to_str().unwrap(),
    ]);
    let (header, ff_data) = read_all(&ff_wav).unwrap();
    let (block_align, fpb) = match header.codec {
        WavCodec::ImaAdpcm {
            block_align,
            frames_per_block,
            ..
        } => (block_align as usize, frames_per_block as usize),
        other => panic!("ffmpeg wrote {other:?}"),
    };
    (ff_wav, block_align, fpb, ff_data)
}

fn adpcm_case(channels: u8) {
    // Learn ffmpeg's block geometry for this channel count (1024-byte blocks:
    // 2041 frames mono, 1017 stereo), then build a signal of exactly 8 blocks.
    let (_, _, fpb, _) = ffmpeg_ima(&signal(4096, channels as usize), channels, "probe");
    assert_eq!(
        fpb,
        frames_per_block(channels, ffmpeg_block_align()),
        "ffmpeg's frames per block"
    );
    let frames = fpb * 8;
    let pcm = signal(frames, channels as usize);
    let (ff_wav, block_align, fpb2, ff_data) = ffmpeg_ima(&pcm, channels, "in");
    assert_eq!(fpb2, fpb);
    assert_eq!(ff_data.len(), block_align * 8);

    let pcm_block = fpb * channels as usize * 2;
    let encode_all = |enc: &mut Encoder| -> Vec<u8> {
        let mut ours = vec![0u8; block_align * 8];
        for (i, (src, dst)) in pcm
            .chunks_exact(pcm_block)
            .zip(ours.chunks_exact_mut(block_align))
            .enumerate()
        {
            assert_eq!(
                enc.encode_block(src, dst).unwrap(),
                block_align,
                "block {i}"
            );
        }
        ours
    };
    let decode_all = |data: &[u8]| -> Vec<u8> {
        let mut dec = Decoder::new(channels, fpb).unwrap();
        let mut out = vec![0u8; pcm.len()];
        for (src, dst) in data
            .chunks_exact(block_align)
            .zip(out.chunks_exact_mut(pcm_block))
        {
            dec.decode_block(src, dst).unwrap();
        }
        out
    };

    // 1. In ffmpeg-compatible mode our encoder is byte-identical to ffmpeg's.
    let ff_mode = encode_all(&mut Encoder::ffmpeg_compatible(channels, fpb).unwrap());
    assert_eq!(
        first_mismatch(&ff_mode, &ff_data),
        None,
        "ffmpeg-compatible encoder differs from ffmpeg adpcm_ima_wav ({channels} ch)"
    );

    // 2. Our decoder over ffmpeg's blocks is byte-identical to ffmpeg's decoder.
    let ff_dec = tmp(&format!("ffdec{channels}.raw"));
    ffmpeg(&[
        "-i",
        ff_wav.to_str().unwrap(),
        "-f",
        "s16le",
        ff_dec.to_str().unwrap(),
    ]);
    let ff_pcm = std::fs::read(&ff_dec).unwrap();
    let our_pcm = decode_all(&ff_data);
    assert_eq!(
        first_mismatch(&our_pcm, &ff_pcm),
        None,
        "decoder differs from ffmpeg ({channels} ch)"
    );

    // 3. ffmpeg decodes a WAV we wrote (our header, the default closed-loop
    //    encoder's blocks) exactly as our decoder does.
    let ours = encode_all(&mut Encoder::new(channels, fpb).unwrap());
    let ours_wav = tmp(&format!("ours{channels}.wav"));
    let h = WavHeader::ima_adpcm(
        16_000,
        channels,
        block_align as u16,
        fpb as u16,
        ours.len() as u32,
        frames as u32,
    );
    let mut hb = [0u8; MAX_HEADER_LEN];
    let n = h.write(&mut hb).unwrap();
    let mut file = hb[..n].to_vec();
    file.extend_from_slice(&ours);
    std::fs::write(&ours_wav, &file).unwrap();
    let ours_dec = tmp(&format!("oursdec{channels}.raw"));
    ffmpeg(&[
        "-i",
        ours_wav.to_str().unwrap(),
        "-f",
        "s16le",
        ours_dec.to_str().unwrap(),
    ]);
    let ours_pcm = decode_all(&ours);
    assert_eq!(
        std::fs::read(&ours_dec).unwrap(),
        ours_pcm,
        "ffmpeg reads our IMA WAV differently"
    );

    // 4. The closed-loop rule reconstructs at least as well as ffmpeg's encoder.
    let snr = |decoded: &[u8]| -> f64 {
        let (mut sig, mut err) = (0f64, 0f64);
        for (a, b) in pcm.chunks_exact(2).zip(decoded.chunks_exact(2)) {
            let x = f64::from(i16::from_le_bytes([a[0], a[1]]));
            let y = f64::from(i16::from_le_bytes([b[0], b[1]]));
            sig += x * x;
            err += (x - y) * (x - y);
        }
        10.0 * (sig / err.max(1e-9)).log10()
    };
    let snr_ff = snr(&ff_pcm);
    let snr_ours = snr(&ours_pcm);
    eprintln!(
        "adpcm_ima ({channels} ch): ffmpeg-mode encoder, decoder and WAV byte-identical to ffmpeg over {frames} frames; \
         decoded SNR: closed-loop {snr_ours:.2} dB vs ffmpeg encoder {snr_ff:.2} dB"
    );
    assert!(
        snr_ours >= snr_ff - 0.05,
        "closed-loop prediction should not be worse"
    );
}

#[test]
fn adpcm_ima_is_byte_identical_to_ffmpeg_both_ways() {
    if !have("ffmpeg", &["-version"]) {
        return;
    }
    adpcm_case(1);
    adpcm_case(2);
}

#[test]
fn pcm_conversions_are_byte_identical_to_ffmpeg() {
    if !have("ffmpeg", &["-version"]) {
        return;
    }
    let pcm = signal(16_000, 1);
    let raw = tmp("conv_in.raw");
    std::fs::write(&raw, &pcm).unwrap();
    let f16 = PcmFormat::PCM16_16K_MONO;
    let blk = PcmBlock::new(f16, Micros::ZERO, &pcm).unwrap();

    for (fmt, sample) in [("s32le", SampleFormat::I32), ("f32le", SampleFormat::F32)] {
        let out = tmp(&format!("conv_{fmt}.raw"));
        ffmpeg(&[
            "-f",
            "s16le",
            "-ar",
            "16000",
            "-ac",
            "1",
            "-i",
            raw.to_str().unwrap(),
            "-f",
            fmt,
            out.to_str().unwrap(),
        ]);
        let ff = std::fs::read(&out).unwrap();
        let mut ours = vec![0u8; pcm.len() * 2];
        assert_eq!(convert(blk, sample, &mut ours).unwrap(), ff.len());
        assert_eq!(first_mismatch(&ours, &ff), None, "s16 → {fmt}");

        // And back down to s16 from ffmpeg's wide output.
        let back = tmp(&format!("conv_{fmt}_back.raw"));
        ffmpeg(&[
            "-f",
            fmt,
            "-ar",
            "16000",
            "-ac",
            "1",
            "-i",
            out.to_str().unwrap(),
            "-f",
            "s16le",
            back.to_str().unwrap(),
        ]);
        let ff_back = std::fs::read(&back).unwrap();
        let wide = PcmBlock::new(
            PcmFormat::new(16_000, 1, sample).unwrap(),
            Micros::ZERO,
            &ff,
        )
        .unwrap();
        let mut ours_back = vec![0u8; pcm.len()];
        convert(wide, SampleFormat::I16, &mut ours_back).unwrap();
        assert_eq!(first_mismatch(&ours_back, &ff_back), None, "{fmt} → s16");
    }

    // Floats that do not come from integers: exercise rounding and clipping.
    let mut floats = Vec::new();
    let mut lcg: u32 = 7;
    for _ in 0..8000 {
        lcg = lcg.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let v = ((lcg >> 8) as f32 / 16_777_216.0) * 2.6 - 1.3; // ±1.3, clips
        floats.extend_from_slice(&v.to_le_bytes());
    }
    let fraw = tmp("floats.raw");
    std::fs::write(&fraw, &floats).unwrap();
    let f_out = tmp("floats_s16.raw");
    ffmpeg(&[
        "-f",
        "f32le",
        "-ar",
        "16000",
        "-ac",
        "1",
        "-i",
        fraw.to_str().unwrap(),
        "-f",
        "s16le",
        f_out.to_str().unwrap(),
    ]);
    let ff = std::fs::read(&f_out).unwrap();
    let fblk = PcmBlock::new(
        PcmFormat::new(16_000, 1, SampleFormat::F32).unwrap(),
        Micros::ZERO,
        &floats,
    )
    .unwrap();
    let mut ours = vec![0u8; 16_000];
    convert(fblk, SampleFormat::I16, &mut ours).unwrap();
    assert_eq!(
        first_mismatch(&ours, &ff),
        None,
        "random f32 → s16 (rounding/clipping)"
    );
    eprintln!("pcm conversions: s16↔s32, s16↔f32 and clipped f32→s16 byte-identical to ffmpeg");
}

fn run_biquad(kind: BiquadKind, pcm: &[u8]) -> Vec<u8> {
    let f = PcmFormat::PCM16_16K_MONO;
    let mut bq = Biquad::new(kind);
    let mut out = vec![0u8; pcm.len()];
    for (i, o) in pcm.chunks_exact(640).zip(out.chunks_exact_mut(640)) {
        bq.process(PcmBlock::new(f, Micros::ZERO, i).unwrap(), o)
            .unwrap();
    }
    out
}

fn max_abs_diff(a: &[u8], b: &[u8]) -> (i32, usize) {
    assert_eq!(a.len(), b.len());
    let mut max = 0;
    let mut differing = 0;
    for (x, y) in a.chunks_exact(2).zip(b.chunks_exact(2)) {
        let d = (i32::from(i16::from_le_bytes([x[0], x[1]]))
            - i32::from(i16::from_le_bytes([y[0], y[1]])))
        .abs();
        if d > 0 {
            differing += 1;
        }
        max = max.max(d);
    }
    (max, differing)
}

#[test]
fn biquads_match_ffmpeg_within_two_lsb() {
    if !have("ffmpeg", &["-version"]) {
        return;
    }
    let pcm = signal(16_000, 1);
    let raw = tmp("bq_in.raw");
    std::fs::write(&raw, &pcm).unwrap();
    let q = core::f32::consts::FRAC_1_SQRT_2;
    let cases = [
        (
            "highpass=f=300:t=q:w=0.70710677:a=di:r=f64",
            BiquadKind::HighPass { f0: 300.0, q },
        ),
        (
            "lowpass=f=3400:t=q:w=0.70710677:a=di:r=f64",
            BiquadKind::LowPass { f0: 3400.0, q },
        ),
        (
            "equalizer=f=1000:t=q:w=1.0:g=6:a=di:r=f64",
            BiquadKind::Peak {
                f0: 1000.0,
                q: 1.0,
                gain_db: 6.0,
            },
        ),
    ];
    for (i, (filter, kind)) in cases.iter().enumerate() {
        let out = tmp(&format!("bq_ff{i}.raw"));
        ffmpeg(&[
            "-f",
            "s16le",
            "-ar",
            "16000",
            "-ac",
            "1",
            "-i",
            raw.to_str().unwrap(),
            "-af",
            filter,
            "-f",
            "s16le",
            out.to_str().unwrap(),
        ]);
        let ff = std::fs::read(&out).unwrap();
        let ours = run_biquad(*kind, &pcm);
        let (max, differing) = max_abs_diff(&ours, &ff);
        eprintln!(
            "biquad vs ffmpeg `{filter}`: max |diff| = {max} LSB over {} samples, {differing} differ",
            ff.len() / 2
        );
        assert!(max <= 2, "{filter}: {max} LSB");
    }
}

#[test]
fn biquad_matches_scipy_within_one_lsb() {
    if !have("python", &["-c", "import numpy, scipy"]) {
        return;
    }
    let pcm = signal(16_000, 1);
    let raw = tmp("sp_in.raw");
    std::fs::write(&raw, &pcm).unwrap();
    let script = tmp("rbj.py");
    std::fs::write(
        &script,
        r#"
import sys, math, numpy as np
from scipy.signal import lfilter
kind, f0, q, gain_db, inp, outp = sys.argv[1], float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]), sys.argv[5], sys.argv[6]
fs = 16000.0
w0 = 2 * math.pi * f0 / fs
c = math.cos(w0); alpha = math.sin(w0) / (2 * q)
if kind == "hp":
    b = [(1 + c) / 2, -(1 + c), (1 + c) / 2]; a = [1 + alpha, -2 * c, 1 - alpha]
elif kind == "lp":
    b = [(1 - c) / 2, 1 - c, (1 - c) / 2]; a = [1 + alpha, -2 * c, 1 - alpha]
else:
    A = 10 ** (gain_db / 40)
    b = [1 + alpha * A, -2 * c, 1 - alpha * A]; a = [1 + alpha / A, -2 * c, 1 - alpha / A]
b = np.array(b) / a[0]; a = np.array(a) / a[0]
x = np.fromfile(inp, dtype="<i2").astype(np.float64)
y = lfilter(b, a, x)
# round half away from zero, then saturate, like the Rust side
y = np.sign(y) * np.floor(np.abs(y) + 0.5)
y = np.clip(y, -32768, 32767).astype("<i2")
y.tofile(outp)
"#,
    )
    .unwrap();
    let q = core::f32::consts::FRAC_1_SQRT_2;
    let cases = [
        (
            "hp",
            300.0f32,
            q,
            0.0f32,
            BiquadKind::HighPass { f0: 300.0, q },
        ),
        ("lp", 3400.0, q, 0.0, BiquadKind::LowPass { f0: 3400.0, q }),
        (
            "pk",
            1000.0,
            1.0,
            6.0,
            BiquadKind::Peak {
                f0: 1000.0,
                q: 1.0,
                gain_db: 6.0,
            },
        ),
    ];
    for (name, f0, qq, g, kind) in cases {
        let out = tmp(&format!("sp_{name}.raw"));
        let status = Command::new("python")
            .arg(&script)
            .args([name, &f0.to_string(), &qq.to_string(), &g.to_string()])
            .arg(&raw)
            .arg(&out)
            .status()
            .unwrap();
        assert!(status.success(), "scipy reference failed for {name}");
        let reference = std::fs::read(&out).unwrap();
        let ours = run_biquad(kind, &pcm);
        let (max, differing) = max_abs_diff(&ours, &reference);
        eprintln!(
            "biquad vs scipy lfilter (f64) {name} f0={f0} q={qq}: max |diff| = {max} LSB, {differing} differ"
        );
        assert!(max <= 1, "{name}: {max} LSB");
    }
}

fn free_udp_port() -> u16 {
    std::net::UdpSocket::bind("127.0.0.1:0")
        .unwrap()
        .local_addr()
        .unwrap()
        .port()
}

fn send_tone(dest: &str, blocks: usize) -> Vec<u8> {
    let f = PcmFormat::PCM16_16K_MONO;
    let mut src = SineSource::new(f, 440.0, 8000).unwrap();
    let mut tx = UdpPcmSender::new("127.0.0.1:0", dest, f).unwrap();
    let mut buf = [0u8; 640];
    let mut sent = Vec::new();
    for _ in 0..blocks {
        let blk = src.read(&mut buf).unwrap();
        tx.write(blk).unwrap();
        sent.extend_from_slice(&buf);
        std::thread::sleep(Duration::from_millis(20));
    }
    sent
}

#[test]
fn raw_pcm_over_udp_is_read_by_ffmpeg() {
    if !have("ffmpeg", &["-version"]) {
        return;
    }
    let port = free_udp_port();
    let out = tmp("udp_ff.wav");
    let url = format!("udp://127.0.0.1:{port}?timeout=5000000&fifo_size=100000&overrun_nonfatal=1");
    let mut child = Command::new("ffmpeg")
        .args([
            "-hide_banner",
            "-loglevel",
            "error",
            "-y",
            "-f",
            "s16le",
            "-ar",
            "16000",
            "-ac",
            "1",
            "-i",
        ])
        .arg(&url)
        .args(["-t", "0.5", "-c:a", "pcm_s16le"])
        .arg(&out)
        .stdin(Stdio::null())
        .spawn()
        .expect("spawn ffmpeg");
    std::thread::sleep(Duration::from_millis(800));
    let sent = send_tone(&format!("127.0.0.1:{port}"), 60); // 1.2 s
    let status = child.wait().unwrap();
    assert!(status.success(), "ffmpeg udp capture failed");
    let (header, data) = read_all(&out).unwrap();
    assert_eq!(header.codec, WavCodec::Pcm(PcmFormat::PCM16_16K_MONO));
    assert_eq!(data.len(), 16_000, "0.5 s of s16le");
    // ffmpeg started somewhere on a datagram boundary of our stream (one
    // 20 ms block, 640 bytes, per datagram).
    let datagram = 640;
    let found = (0..sent.len() - data.len())
        .step_by(datagram)
        .any(|k| sent[k..k + data.len()] == data[..]);
    assert!(
        found,
        "ffmpeg's capture is not a contiguous run of what we sent"
    );
    eprintln!("raw PCM over UDP: ffmpeg captured 0.5 s that matches the stream sample for sample");
}

#[test]
fn recorder_writes_a_wav_ffprobe_reads() {
    if !have("ffprobe", &["-version"]) {
        return;
    }
    let f = PcmFormat::PCM16_16K_MONO;
    let mut rx = UdpPcmReceiver::bind("127.0.0.1:0", f).unwrap();
    let dest = rx.local_addr().unwrap().to_string();
    let path = tmp("rec.wav");
    let path2 = path.clone();
    let recorder = std::thread::spawn(move || {
        let mut wav = WavWriter::create(&path2, f).unwrap();
        let mut buf = [0u8; 2048];
        let mut got = 0usize;
        while got < 16_000 {
            let blk = rx.recv(&mut buf).unwrap();
            got += blk.data.len();
            wav.write(blk).unwrap();
        }
        (wav.finish().unwrap(), rx.datagrams)
    });
    let sent = send_tone(&dest, 25);
    let (total, datagrams) = recorder.join().unwrap();
    assert_eq!(total, 44 + 16_000);
    assert_eq!(datagrams, 25, "one 640-byte block per datagram");
    let (_, data) = read_all(&path).unwrap();
    assert_eq!(data, sent);
    let probe = Command::new("ffprobe")
        .args([
            "-v",
            "error",
            "-show_entries",
            "stream=codec_name,sample_rate,channels,duration_ts",
            "-of",
            "csv=p=0",
        ])
        .arg(&path)
        .output()
        .unwrap();
    let text = String::from_utf8_lossy(&probe.stdout);
    let line = text.lines().next().unwrap_or("").trim();
    assert_eq!(line, "pcm_s16le,16000,1,8000", "ffprobe: {text}");
    eprintln!("recorder: ffprobe reads the WAV as {line}");
}

/// Recorded speech through the VAD and AGC, against ffmpeg's `silencedetect`.
///
/// Set `JANUS_SPEECH_WAV` to any speech recording (it is normalised to 16 kHz
/// mono s16le by ffmpeg first). Nothing is vendored; the ledger records the
/// run on this machine. Skips when the variable is unset.
#[test]
fn recorded_speech_vad_agrees_with_ffmpeg_silencedetect() {
    use rusty_esp_audio_core::elements::{Agc, EnergyVad, VadConfig};
    use rusty_esp_audio_core::rms_dbfs_i16;
    let Some(src) = std::env::var_os("JANUS_SPEECH_WAV") else {
        eprintln!("JANUS_SPEECH_WAV not set; recorded-speech oracle skipped");
        return;
    };
    if !have("ffmpeg", &["-version"]) {
        return;
    }
    let raw = tmp("speech16k.raw");
    ffmpeg(&[
        "-i",
        src.to_str().unwrap(),
        "-ar",
        "16000",
        "-ac",
        "1",
        "-f",
        "s16le",
        raw.to_str().unwrap(),
    ]);
    let pcm = std::fs::read(&raw).unwrap();
    let f = PcmFormat::PCM16_16K_MONO;
    let total_secs = pcm.len() as f64 / 32_000.0;

    // ffmpeg's view: silence below -45 dB lasting at least 200 ms.
    let out = Command::new("ffmpeg")
        .args([
            "-hide_banner",
            "-nostats",
            "-f",
            "s16le",
            "-ar",
            "16000",
            "-ac",
            "1",
            "-i",
        ])
        .arg(&raw)
        .args(["-af", "silencedetect=noise=-45dB:d=0.2", "-f", "null", "-"])
        .output()
        .unwrap();
    let log = String::from_utf8_lossy(&out.stderr);
    let mut silence_secs = 0.0f64;
    let mut segments = 0usize;
    for line in log.lines() {
        if let Some(pos) = line.find("silence_duration: ") {
            let v: f64 = line[pos + 18..].trim().parse().unwrap_or(0.0);
            silence_secs += v;
            segments += 1;
        }
    }
    let ff_speech_fraction = 1.0 - silence_secs / total_secs;

    // Ours: the same threshold, 200 ms of hangover, 20 ms blocks.
    let mut vad = EnergyVad::new(VadConfig {
        threshold_dbfs: -45.0,
        hangover_blocks: 10,
        gate: false,
    });
    let mut agc = Agc::default();
    let mut out_buf = [0u8; 640];
    let mut speech_levels = Vec::new();
    for block in pcm.chunks_exact(640) {
        let blk = PcmBlock::new(f, Micros::ZERO, block).unwrap();
        vad.process(blk, &mut out_buf).unwrap();
        agc.process(blk, &mut out_buf).unwrap();
        if vad.is_speech() {
            speech_levels.push(rms_dbfs_i16(&out_buf));
        }
    }
    let our_speech_fraction = vad.speech_blocks as f64 / vad.blocks as f64;
    speech_levels.sort_by(|a, b| a.partial_cmp(b).unwrap());
    let median = speech_levels[speech_levels.len() / 2];
    let p90 = speech_levels[speech_levels.len() * 9 / 10];
    eprintln!(
        "recorded speech ({total_secs:.1} s): ffmpeg silencedetect {segments} silences, speech fraction {ff_speech_fraction:.3}; \
         EnergyVad speech fraction {our_speech_fraction:.3} ({}/{} blocks); AGC speech-block level median {median:.1} / p90 {p90:.1} dBFS (target -20)",
        vad.speech_blocks, vad.blocks
    );
    assert!(
        (our_speech_fraction - ff_speech_fraction).abs() <= 0.10,
        "VAD disagrees with ffmpeg by more than 10 points"
    );
    // The AGC's slow release (6 dB/s) needs a few seconds to settle, and it
    // holds the LOUD blocks at the target: per-block speech levels swing
    // 20 dB, so the median sits under it by design. Judge the p90 on clips
    // long enough to have settled.
    if total_secs >= 6.0 {
        assert!((-24.0..=-14.0).contains(&p90), "AGC p90 {p90} dBFS");
    }
}

/// The FLAC half of the J2 kill test, on the host: chunks from
/// `codec::flac` decode in ffmpeg to the exact source PCM, our own decoder
/// agrees, and the same samples give the same bytes twice.
#[cfg(feature = "flac")]
#[test]
fn flac_chunks_decode_in_ffmpeg_to_the_source_pcm() {
    use rusty_esp_audio_core::codec::flac::{FlacEncoder, decode_pcm16};
    if !have("ffmpeg", &["-version"]) {
        return;
    }
    for (channels, level) in [(1u8, 5u32), (2, 8)] {
        let f = PcmFormat::new(16_000, channels, SampleFormat::I16).unwrap();
        let pcm = signal(32_000, channels as usize); // 2 s
        let mut enc = FlacEncoder::new(f, level).unwrap();
        let block = 640 * channels as usize;
        for chunk in pcm.chunks_exact(block) {
            enc.push(PcmBlock::new(f, Micros::ZERO, chunk).unwrap())
                .unwrap();
        }
        let stream = enc.finish().unwrap();
        let path = tmp(&format!("chunk{channels}.flac"));
        std::fs::write(&path, &stream).unwrap();
        let raw = tmp(&format!("chunk{channels}.raw"));
        ffmpeg(&[
            "-i",
            path.to_str().unwrap(),
            "-f",
            "s16le",
            raw.to_str().unwrap(),
        ]);
        let ff = std::fs::read(&raw).unwrap();
        assert_eq!(
            first_mismatch(&ff, &pcm),
            None,
            "ffmpeg decode of our FLAC ({channels} ch) is not the source PCM"
        );
        let (back_f, ours) = decode_pcm16(&stream).unwrap();
        assert_eq!(back_f, f);
        assert_eq!(ours, pcm);
        let mut again = FlacEncoder::new(f, level).unwrap();
        for chunk in pcm.chunks_exact(block) {
            again
                .push(PcmBlock::new(f, Micros::ZERO, chunk).unwrap())
                .unwrap();
        }
        assert_eq!(
            again.finish().unwrap(),
            stream,
            "encoder is not deterministic"
        );
        let probe = Command::new("ffprobe")
            .args([
                "-v",
                "error",
                "-show_entries",
                "stream=codec_name,sample_rate,channels,duration_ts",
                "-of",
                "csv=p=0",
            ])
            .arg(&path)
            .output()
            .unwrap();
        let text = String::from_utf8_lossy(&probe.stdout);
        let line = text.lines().next().unwrap_or("").trim().to_string();
        assert_eq!(
            line,
            format!("flac,16000,{channels},32000"),
            "ffprobe: {text}"
        );
        eprintln!(
            "flac ({channels} ch, level {level}): {} B of PCM → {} B FLAC ({:.1} %), ffmpeg decodes it to the source exactly, deterministic; ffprobe: {line}",
            pcm.len(),
            stream.len(),
            100.0 * stream.len() as f64 / pcm.len() as f64
        );
    }
}
