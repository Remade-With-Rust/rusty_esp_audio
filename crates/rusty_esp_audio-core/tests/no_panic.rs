//! The robustness gate: every decoder fed from a file or a stream returns an
//! error on bad input; it never panics. Random bytes from an LCG and
//! mutations of a valid header, under `catch_unwind` so a failure names the
//! decoder and prints the input.

use std::panic::{AssertUnwindSafe, catch_unwind};

use rusty_esp_audio_core::codec::adpcm_ima::Decoder;
use rusty_esp_audio_core::codec::wav::WavHeader;

struct Lcg(u64);

impl Lcg {
    fn next(&mut self) -> u64 {
        self.0 = self
            .0
            .wrapping_mul(6_364_136_223_846_793_005)
            .wrapping_add(1_442_695_040_888_963_407);
        self.0
    }

    fn below(&mut self, n: usize) -> usize {
        (self.next() % n.max(1) as u64) as usize
    }

    fn bytes(&mut self, max_len: usize) -> Vec<u8> {
        let n = self.below(max_len + 1);
        (0..n).map(|_| (self.next() >> 56) as u8).collect()
    }

    fn mutate(&mut self, base: &[u8]) -> Vec<u8> {
        let mut v = base.to_vec();
        match self.below(6) {
            0 if !v.is_empty() => {
                let i = self.below(v.len());
                v[i] ^= 1 << self.below(8);
            }
            1 if !v.is_empty() => {
                let i = self.below(v.len());
                v[i] = (self.next() >> 56) as u8;
            }
            2 => v.truncate(self.below(v.len() + 1)),
            3 => {
                let extra = self.bytes(16);
                v.extend_from_slice(&extra);
            }
            4 => {
                let i = self.below(v.len() + 1);
                v.insert(i, (self.next() >> 56) as u8);
            }
            _ if !v.is_empty() => {
                let i = self.below(v.len());
                v.remove(i);
            }
            _ => {}
        }
        v
    }
}

fn check<R>(name: &str, input: &[u8], f: impl FnOnce() -> R) {
    if catch_unwind(AssertUnwindSafe(f)).is_err() {
        let hex: String = input.iter().take(256).map(|b| format!("{b:02x}")).collect();
        panic!("{name} panicked on {} bytes: {hex}", input.len());
    }
}

/// A 44-byte PCM16 mono 16 kHz WAV header with one sample of data.
fn wav_pcm16() -> Vec<u8> {
    let mut v = Vec::new();
    v.extend_from_slice(b"RIFF");
    v.extend_from_slice(&38u32.to_le_bytes());
    v.extend_from_slice(b"WAVE");
    v.extend_from_slice(b"fmt ");
    v.extend_from_slice(&16u32.to_le_bytes());
    v.extend_from_slice(&1u16.to_le_bytes()); // PCM
    v.extend_from_slice(&1u16.to_le_bytes()); // mono
    v.extend_from_slice(&16_000u32.to_le_bytes());
    v.extend_from_slice(&32_000u32.to_le_bytes());
    v.extend_from_slice(&2u16.to_le_bytes());
    v.extend_from_slice(&16u16.to_le_bytes());
    v.extend_from_slice(b"data");
    v.extend_from_slice(&2u32.to_le_bytes());
    v.extend_from_slice(&[0x34, 0x12]);
    v
}

#[test]
fn wav_header_parse_never_panics() {
    let mut rng = Lcg(0xA0D1_0001);
    let base = wav_pcm16();
    assert!(
        WavHeader::parse(&base).is_ok(),
        "the fixture is a WAV the crate reads"
    );
    for i in 0..40_000 {
        let input = match i % 3 {
            0 => rng.bytes(120),
            _ => rng.mutate(&base),
        };
        check("WavHeader::parse", &input, || {
            WavHeader::parse(&input).map(|(h, n)| {
                let mut out = [0u8; 128];
                let _ = h.write(&mut out);
                n
            })
        });
    }
}

#[test]
fn adpcm_decoder_never_panics() {
    let mut rng = Lcg(0xA0D1_0002);
    let mut out = vec![0u8; 16 * 1024];
    for i in 0..20_000 {
        let channels = 1 + (i % 2) as u8;
        let frames = [1usize, 8, 505, 1017, 2041][i % 5];
        let block = rng.bytes(1100);
        check("adpcm_ima::Decoder::decode_block", &block, || {
            if let Ok(mut d) = Decoder::new(channels, frames) {
                let _ = d.decode_block(&block, &mut out);
                let _ = d.decode_block(&block, &mut out[..17]);
            }
        });
    }
}

#[cfg(feature = "flac")]
#[test]
fn flac_decoder_never_panics() {
    use rusty_esp_audio_core::codec::flac::decode_pcm16;
    let mut rng = Lcg(0xA0D1_0003);
    let mut base = b"fLaC".to_vec();
    base.extend_from_slice(&[0x80, 0x00, 0x00, 0x22]);
    base.extend_from_slice(&[0u8; 34]);
    for i in 0..4_000 {
        let input = if i % 2 == 0 {
            rng.bytes(400)
        } else {
            rng.mutate(&base)
        };
        check("flac::decode_pcm16", &input, || {
            decode_pcm16(&input).map(|(f, pcm)| (f, pcm.len()))
        });
    }
}
