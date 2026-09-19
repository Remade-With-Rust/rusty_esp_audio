// 0.7071 is a truncation of FRAC_1_SQRT_2, and clippy is right that it is.
// It stays: the frozen digests below were taken with THAT literal, and
// tightening it changes the coefficients and therefore every output byte.
// (Tried on 2026-09-19 -- the gate caught it immediately, which is the gate
// doing its job. A digest is evidence only while the input that made it is
// unchanged.)
#![allow(clippy::approx_constant)]

//! The byte-identity gate for the element optimisation pass (I9).
//!
//! Every kernel here is being restructured for instruction count, never for
//! arithmetic. So the gate is the strongest one available: a digest of the
//! exact output bytes over a swept corpus, frozen from the code as it stood
//! BEFORE the pass. A change that moves one byte fails, and the failure names
//! the kernel.
//!
//! The digests are not magic numbers to be re-blessed when they break. If one
//! moves, the change altered the signal — revert it, or prove the old bytes
//! were wrong and say so in the ledger.
//!
//! The stateful elements carry a SECOND, independent gate: feeding a block in
//! one call must equal feeding it one frame at a time. That property is a
//! statement about the recurrence rather than about a recorded value, so it
//! keeps holding if a corpus is ever regenerated.

use rusty_esp_audio_core::elements::{
    Agc, Biquad, BiquadKind, DcBlock, Gain, LinearResampler, MonoToStereo, StereoToMono,
    mix_i16,
};
use rusty_esp_audio_core::pipeline::Element;
use rusty_esp_core::pcm::{PcmBlock, PcmFormat, SampleFormat};
use rusty_esp_core::time::Micros;

/// The gate. `CAPTURE=1 cargo test -- --nocapture` PRINTS the digests instead
/// of checking them. That mode exists to freeze a baseline from code already
/// known correct -- never to make a failing gate pass.
fn gate(name: &str, bytes: &[u8], want: u64) {
    let got = digest(bytes);
    if std::env::var_os("CAPTURE").is_some() {
        println!("CAPTURE {name} = {got}");
    } else {
        assert_eq!(got, want, "{name} is no longer byte-identical");
    }
}

/// FNV-1a, so the gate needs no dependency and reads the same everywhere.
fn digest(bytes: &[u8]) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for &b in bytes {
        h ^= u64::from(b);
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// A deterministic corpus that reaches the saturating and sign-change paths,
/// not just the quiet middle a sine tone would stay in.
fn corpus(frames: usize, ch: usize) -> Vec<u8> {
    let mut v = Vec::with_capacity(frames * ch * 2);
    for i in 0..frames * ch {
        let x: i16 = match i % 7 {
            0 => i16::MIN,
            1 => i16::MAX,
            2 => 0,
            3 => -1,
            _ => (((i as i32) * 5779) % 65536 - 32768) as i16,
        };
        v.extend_from_slice(&x.to_le_bytes());
    }
    v
}

fn fmt(ch: u8) -> PcmFormat {
    PcmFormat::new(16_000, ch, SampleFormat::I16).unwrap()
}

fn run<E: Element>(e: &mut E, f: PcmFormat, data: &[u8], out: &mut [u8]) -> usize {
    let b = PcmBlock::new(f, Micros(0), data).unwrap();
    e.process(b, out).unwrap()
}

/// One call over the whole block must equal one call per frame.
fn split_matches_whole<E: Element + Clone>(e: &E, f: PcmFormat, data: &[u8], cap: usize) {
    let fb = f.frame_bytes();
    let mut whole = vec![0u8; cap];
    let n = run(&mut e.clone(), f, data, &mut whole);

    let mut piece = Vec::new();
    let mut one = e.clone();
    let mut buf = vec![0u8; cap];
    for fr in data.chunks_exact(fb) {
        let m = run(&mut one, f, fr, &mut buf);
        piece.extend_from_slice(&buf[..m]);
    }
    assert_eq!(
        &whole[..n],
        &piece[..],
        "block-at-once and frame-at-a-time disagree"
    );
}

#[test]
fn stateless_elements_are_byte_identical() {
    let mono = corpus(512, 1);
    let stereo = corpus(512, 2);

    // Gain: every arm of the Q15 range, including the ones whose product does
    // NOT fit i32 and must keep the wide path.
    let mut acc = Vec::new();
    for &q in &[
        0i32, 1, -1, 16384, 32768, 65535, 65536, 65537, 131_072, -131_072, 1 << 20, 65536 * 512,
        -65536 * 512,
    ] {
        let g = Gain::from_q15(q);
        for fr in mono.chunks_exact(2) {
            let x = i16::from_le_bytes([fr[0], fr[1]]);
            acc.extend_from_slice(&g.apply(x).to_le_bytes());
        }
    }
    gate("GAIN_APPLY", &acc, GAIN_APPLY);

    let mut out = vec![0u8; 512 * 2];
    let n = run(&mut Gain::from_q15(16384), fmt(1), &mono, &mut out);
    gate("GAIN_PROCESS", &out[..n], GAIN_PROCESS);

    let mut st = vec![0u8; 512 * 4];
    let n = run(&mut MonoToStereo, fmt(1), &mono, &mut st);
    gate("MONO_TO_STEREO", &st[..n], MONO_TO_STEREO);

    let mut mo = vec![0u8; 512 * 2];
    let n = run(&mut StereoToMono, fmt(2), &stereo, &mut mo);
    gate("STEREO_TO_MONO", &mo[..n], STEREO_TO_MONO);

    let b: Vec<u8> = mono.iter().rev().copied().collect();
    let mut mx = vec![0u8; 512 * 2];
    mix_i16(&mono, &b, &mut mx).unwrap();
    gate("MIX_I16", &mx, MIX_I16);
}

#[test]
fn stateful_elements_are_byte_identical() {
    for ch in [1u8, 2] {
        let data = corpus(512, ch as usize);
        let f = fmt(ch);
        let cap = data.len();

        let mut dc = DcBlock::new();
        let mut out = vec![0u8; cap];
        let n = run(&mut dc, f, &data, &mut out);
        gate(&format!("DC_BLOCK ch={ch}"), &out[..n], DC_BLOCK[usize::from(ch) - 1]);
        split_matches_whole(&DcBlock::new(), f, &data, cap);

        let kind = BiquadKind::LowPass {
            f0: 3000.0,
            q: 0.7071,
        };
        let mut bq = Biquad::new(kind);
        let mut out = vec![0u8; cap];
        let n = run(&mut bq, f, &data, &mut out);
        gate(&format!("BIQUAD_LP ch={ch}"), &out[..n], BIQUAD_LP[usize::from(ch) - 1]);
        split_matches_whole(&Biquad::new(kind), f, &data, cap);
    }
}

#[test]
fn resampler_is_byte_identical() {
    // Up, down, and a ratio whose reduction is not small (44.1k <-> 48k).
    let pairs: [(u32, u32); 6] = [
        (16_000, 48_000),
        (48_000, 16_000),
        (44_100, 48_000),
        (48_000, 44_100),
        (16_000, 16_000),
        (8_000, 44_100),
    ];
    for (i, &(a, b)) in pairs.iter().enumerate() {
        let data = corpus(400, 1);
        let f = PcmFormat::new(a, 1, SampleFormat::I16).unwrap();
        let mut rs = LinearResampler::new(a, b).unwrap();
        let cap = rs.max_output_frames(400) * 2;
        let mut out = vec![0u8; cap];
        // three blocks, so the phase carried across a boundary is exercised
        let mut acc = Vec::new();
        for chunk in data.chunks(400 / 3 * 2) {
            let blk = PcmBlock::new(f, Micros(0), chunk).unwrap();
            let n = rs.process(blk, &mut out).unwrap();
            acc.extend_from_slice(&out[..n]);
        }
        gate(&format!("RESAMPLE[{i}] {a}->{b}"), &acc, RESAMPLE[i]);
    }

    // Stereo, and the split gate.
    let data = corpus(300, 2);
    let f = PcmFormat::new(16_000, 2, SampleFormat::I16).unwrap();
    let rs = LinearResampler::new(16_000, 48_000).unwrap();
    let cap = rs.max_output_frames(300) * 4;
    let mut out = vec![0u8; cap];
    let n = run(&mut rs.clone(), f, &data, &mut out);
    gate("RESAMPLE_STEREO", &out[..n], RESAMPLE_STEREO);
    split_matches_whole(&rs, f, &data, cap);
}

// ---- the frozen digests --------------------------------------------------
// Captured 2026-09-19 from the code as it stood before the I9 element pass.
const GAIN_APPLY: u64 = 11_873_871_104_420_170_391;
const GAIN_PROCESS: u64 = 2_992_203_796_148_523_917;
const MONO_TO_STEREO: u64 = 5_149_472_300_421_449_861;
const STEREO_TO_MONO: u64 = 13_470_942_495_081_143_834;
const MIX_I16: u64 = 3_410_800_590_168_634_747;
const DC_BLOCK: [u64; 2] = [1_089_841_316_401_847_127, 18_142_877_616_869_531_782];
const BIQUAD_LP: [u64; 2] = [4_042_443_379_142_208_225, 14_718_782_710_653_446_052];
const RESAMPLE: [u64; 6] = [
    16_911_818_136_423_541_981,
    9_027_358_275_306_514_462,
    11_766_731_195_925_026_033,
    7_487_350_096_454_747_003,
    4_113_759_242_397_977_576,
    3_280_085_387_383_701_394,
];
const RESAMPLE_STEREO: u64 = 4_688_573_486_506_082_247;

/// Every element that reads i16 now has TWO arms: a fast one over an aligned
/// `&[i16]` view and the original byte loop, which is both the oracle and
/// what a misaligned or big-endian buffer gets.
///
/// Every ordinary buffer is aligned, so the digests above only ever exercise
/// the FAST arm. This drives BOTH, by sliding the input and the output one
/// byte into larger buffers -- which makes the ADDRESS wrong while leaving
/// the length valid, the reason `as_i16` refuses in practice -- and requires
/// the two to produce identical bytes.
fn arms_agree<E: Element + Clone>(name: &str, e: &E, f: PcmFormat, src: &[u8], outlen: usize) {
    let mut fast = vec![0u8; outlen];
    let a = run(&mut e.clone(), f, src, &mut fast);

    let mut off_src = vec![0u8; src.len() + 1];
    off_src[1..].copy_from_slice(src);
    let mut off_dst = vec![0u8; outlen + 1];
    let b = run(&mut e.clone(), f, &off_src[1..], &mut off_dst[1..]);

    assert_eq!(a, b, "{name}: the arms returned different byte counts");
    assert_eq!(
        &fast[..a],
        &off_dst[1..1 + b],
        "{name}: the aligned and byte arms disagree"
    );
}

#[test]
fn every_two_armed_element_agrees_with_itself() {
    let kind = BiquadKind::LowPass {
        f0: 3000.0,
        q: 0.7071,
    };
    for frames in [1usize, 2, 3, 7, 8, 9, 15, 16, 17, 31, 64, 257] {
        let mono = corpus(frames, 1);
        let stereo = corpus(frames, 2);

        arms_agree("StereoToMono", &StereoToMono, fmt(2), &stereo, stereo.len() / 2);
        arms_agree("MonoToStereo", &MonoToStereo, fmt(1), &mono, mono.len() * 2);
        // both sides of the Q15 range: the narrow arm whose product fits i32
        // and the wide one that must stay 64-bit
        for &q in &[1i32, 16384, 65535, 65536, 1 << 20, 65536 * 512] {
            arms_agree("Gain", &Gain::from_q15(q), fmt(1), &mono, mono.len());
        }
        for ch in [1u8, 2] {
            let d = corpus(frames, ch as usize);
            arms_agree("DcBlock", &DcBlock::new(), fmt(ch), &d, d.len());
            arms_agree("Biquad", &Biquad::new(kind), fmt(ch), &d, d.len());
            arms_agree("Agc", &Agc::default(), fmt(ch), &d, d.len());
        }

        // mix_i16 is a free function, and needs all THREE buffers to view
        let other: Vec<u8> = mono.iter().rev().copied().collect();
        let mut fast = vec![0u8; mono.len()];
        mix_i16(&mono, &other, &mut fast).unwrap();
        let mut oa = vec![0u8; mono.len() + 1];
        let mut ob = vec![0u8; mono.len() + 1];
        let mut oc = vec![0u8; mono.len() + 1];
        oa[1..].copy_from_slice(&mono);
        ob[1..].copy_from_slice(&other);
        mix_i16(&oa[1..], &ob[1..], &mut oc[1..]).unwrap();
        assert_eq!(fast, oc[1..], "mix_i16: the arms disagree at {frames} frames");
    }
}

/// `LinearResampler`'s mono path has an aligned arm too, and its output
/// length is not the input's -- so it needs its own two-arm check rather
/// than `arms_agree`.
#[test]
fn resampler_arms_agree_including_the_misaligned_one() {
    for (a, b) in [
        (16_000u32, 48_000u32),
        (48_000, 16_000),
        (44_100, 48_000),
        (8_000, 44_100),
    ] {
        let data = corpus(400, 1);
        let f = PcmFormat::new(a, 1, SampleFormat::I16).unwrap();
        let cap = LinearResampler::new(a, b).unwrap().max_output_frames(400) * 2;

        // aligned: fresh Vecs
        let mut rs = LinearResampler::new(a, b).unwrap();
        let mut fast = vec![0u8; cap];
        let mut got = Vec::new();
        for chunk in data.chunks(400 / 3 * 2) {
            let n = rs.process(PcmBlock::new(f, Micros(0), chunk).unwrap(), &mut fast).unwrap();
            got.extend_from_slice(&fast[..n]);
        }

        // misaligned: slide input and output one byte
        let mut rs2 = LinearResampler::new(a, b).unwrap();
        let mut off_out = vec![0u8; cap + 1];
        let mut slow = Vec::new();
        for chunk in data.chunks(400 / 3 * 2) {
            let mut off_in = vec![0u8; chunk.len() + 1];
            off_in[1..].copy_from_slice(chunk);
            let n = rs2
                .process(
                    PcmBlock::new(f, Micros(0), &off_in[1..]).unwrap(),
                    &mut off_out[1..],
                )
                .unwrap();
            slow.extend_from_slice(&off_out[1..1 + n]);
        }
        assert_eq!(got, slow, "LinearResampler arms disagree at {a} -> {b}");
    }
}
