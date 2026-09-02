//! IMA ADPCM in the WAV (`WAVE_FORMAT_IMA_ADPCM`, tag `0x11`) block layout:
//! 4:1 compression of 16-bit PCM with a 4-byte header per channel per block.
//!
//! This is the encoder/decoder pair `esp_audio_codec` ships as `adpcm`,
//! proven against the real ffmpeg on the host (the oracle test in
//! `rusty_esp_audio-esp`, results in the ledger):
//!
//! - the **decoder** reconstructs exactly as ffmpeg's `adpcm_ima_wav`
//!   decoder — the IMA reference expansion
//!   `diff = step>>3 + [step if b2] + [step>>1 if b1] + [step>>2 if b0]`;
//! - the **encoder** quantises as ffmpeg's does (`n = min(7, |Δ|·4/step)`)
//!   and, by default, tracks its prediction with the *decoder's* rule, so the
//!   encoder never drifts from what players reconstruct. ffmpeg's own encoder
//!   instead predicts with `((2n+1)·step) >> 3`, which is off by a unit or two
//!   from its decoder; [`PredictionRule::FfmpegEncoder`] reproduces that
//!   byte-for-byte for anyone who needs ffmpeg parity.
//!
//! Block layout, per block of `frames_per_block` sample frames:
//!
//! ```text
//! for each channel: i16 first sample, u8 step index, u8 reserved (0)
//! then, for every following 8 frames: 4 bytes per channel, channels
//! interleaved by 4-byte word; each byte holds two nibbles, low first.
//! ```
//!
//! `block_align = 4·ch + (frames_per_block − 1)·ch / 2`; ffmpeg uses
//! `block_align = 1024·ch`, i.e. 2041 frames per block.

use rusty_esp_core::error::{Error, Result};

/// The 89-entry step table (IMA / Microsoft / ffmpeg).
pub const STEP_TABLE: [i32; 89] = [
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17, 19, 21, 23, 25, 28, 31, 34, 37, 41, 45, 50, 55, 60, 66,
    73, 80, 88, 97, 107, 118, 130, 143, 157, 173, 190, 209, 230, 253, 279, 307, 337, 371, 408, 449,
    494, 544, 598, 658, 724, 796, 876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066, 2272,
    2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358, 5894, 6484, 7132, 7845, 8630, 9493,
    10442, 11487, 12635, 13899, 15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767,
];

/// Step-index adjustment per nibble.
pub const INDEX_TABLE: [i8; 16] = [-1, -1, -1, -1, 2, 4, 6, 8, -1, -1, -1, -1, 2, 4, 6, 8];

/// How the encoder updates its prediction after each nibble.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum PredictionRule {
    /// The decoder's own expansion (IMA reference): closed-loop, no drift.
    #[default]
    DecoderExact,
    /// ffmpeg's encoder: `((2n+1)·step) >> 3`. Byte-identical to
    /// `ffmpeg -c:a adpcm_ima_wav`; drifts a unit or two from decoders.
    FfmpegEncoder,
}

/// Per-channel coder state: predictor and step index.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ChannelState {
    /// Current prediction (an i16 value held in i32).
    pub predictor: i32,
    /// Index into [`STEP_TABLE`], 0..=88.
    pub step_index: u8,
}

impl ChannelState {
    /// IMA reference expansion magnitude for `nibble` at `step`.
    #[inline]
    #[must_use]
    pub const fn diff_reference(step: i32, nibble: u8) -> i32 {
        let mut d = step >> 3;
        if nibble & 4 != 0 {
            d += step;
        }
        if nibble & 2 != 0 {
            d += step >> 1;
        }
        if nibble & 1 != 0 {
            d += step >> 2;
        }
        d
    }

    /// ffmpeg's encoder-side expansion magnitude.
    #[inline]
    #[must_use]
    pub const fn diff_ffmpeg_encoder(step: i32, nibble: u8) -> i32 {
        ((2 * (nibble & 7) as i32 + 1) * step) >> 3
    }

    #[inline]
    fn advance(&mut self, nibble: u8) {
        let idx = i32::from(self.step_index) + i32::from(INDEX_TABLE[usize::from(nibble & 15)]);
        self.step_index = idx.clamp(0, 88) as u8;
    }

    #[inline]
    fn apply(&mut self, d: i32, negative: bool) {
        self.predictor += if negative { -d } else { d };
        self.predictor = self
            .predictor
            .clamp(i32::from(i16::MIN), i32::from(i16::MAX));
    }

    /// Quantise one sample against the state and update it.
    #[inline]
    pub fn encode(&mut self, sample: i16, rule: PredictionRule) -> u8 {
        let step = STEP_TABLE[usize::from(self.step_index)];
        let delta = i32::from(sample) - self.predictor;
        let mag = (delta.unsigned_abs() * 4 / step as u32).min(7) as u8;
        let nibble = mag | if delta < 0 { 8 } else { 0 };
        let d = match rule {
            PredictionRule::DecoderExact => Self::diff_reference(step, nibble),
            PredictionRule::FfmpegEncoder => Self::diff_ffmpeg_encoder(step, nibble),
        };
        self.apply(d, delta < 0);
        self.advance(nibble);
        nibble
    }

    /// Expand one nibble (the IMA reference / ffmpeg decoder rule).
    #[inline]
    pub fn decode(&mut self, nibble: u8) -> i16 {
        let step = STEP_TABLE[usize::from(self.step_index)];
        self.advance(nibble);
        self.apply(Self::diff_reference(step, nibble), nibble & 8 != 0);
        self.predictor as i16
    }
}

/// Channels this codec handles.
pub const MAX_CHANNELS: usize = 2;

/// Bytes of one encoded block for `channels` and `frames_per_block`.
pub const fn block_align(channels: u8, frames_per_block: usize) -> usize {
    4 * channels as usize + (frames_per_block - 1) * channels as usize / 2
}

/// Frames per block for an encoded `block_align` (the WAV `nBlockAlign`).
pub const fn frames_per_block(channels: u8, block_align: usize) -> usize {
    (block_align - 4 * channels as usize) * 2 / channels as usize + 1
}

/// ffmpeg's block size, whatever the channel count: 1024 bytes, so 2041
/// frames per block in mono and 1017 in stereo.
pub const fn ffmpeg_block_align() -> usize {
    1024
}

fn check(channels: u8, frames_per_block: usize) -> Result<()> {
    if channels == 0 || channels as usize > MAX_CHANNELS {
        return Err(Error::Unsupported);
    }
    // After the header sample, frames come in groups of 8.
    if frames_per_block < 9 || (frames_per_block - 1) % 8 != 0 {
        return Err(Error::InvalidFormat);
    }
    Ok(())
}

/// Encoder for one stream; the step index carries across blocks, as ffmpeg's does.
#[derive(Debug, Clone)]
pub struct Encoder {
    channels: u8,
    frames_per_block: usize,
    rule: PredictionRule,
    state: [ChannelState; MAX_CHANNELS],
    /// Blocks encoded.
    pub blocks: u64,
}

impl Encoder {
    /// An encoder for `channels` (1 or 2) and `frames_per_block`
    /// (`8k + 1`, e.g. 2041), predicting with [`PredictionRule::DecoderExact`].
    pub fn new(channels: u8, frames_per_block: usize) -> Result<Self> {
        Self::with_rule(channels, frames_per_block, PredictionRule::DecoderExact)
    }

    /// An encoder byte-identical to `ffmpeg -c:a adpcm_ima_wav`.
    pub fn ffmpeg_compatible(channels: u8, frames_per_block: usize) -> Result<Self> {
        Self::with_rule(channels, frames_per_block, PredictionRule::FfmpegEncoder)
    }

    /// An encoder with an explicit prediction rule.
    pub fn with_rule(channels: u8, frames_per_block: usize, rule: PredictionRule) -> Result<Self> {
        check(channels, frames_per_block)?;
        Ok(Encoder {
            channels,
            frames_per_block,
            rule,
            state: [ChannelState::default(); MAX_CHANNELS],
            blocks: 0,
        })
    }

    /// Frames each block consumes.
    #[must_use]
    pub const fn frames_per_block(&self) -> usize {
        self.frames_per_block
    }

    /// Bytes each block produces.
    #[must_use]
    pub const fn block_align(&self) -> usize {
        block_align(self.channels, self.frames_per_block)
    }

    /// The prediction rule in use.
    #[must_use]
    pub const fn rule(&self) -> PredictionRule {
        self.rule
    }

    /// Encode exactly one block of interleaved i16 little-endian PCM
    /// (`frames_per_block · channels · 2` bytes) into `out`; returns
    /// [`block_align`](Self::block_align).
    pub fn encode_block(&mut self, pcm: &[u8], out: &mut [u8]) -> Result<usize> {
        let ch = self.channels as usize;
        if pcm.len() != self.frames_per_block * ch * 2 {
            return Err(Error::InvalidGeometry);
        }
        let need = self.block_align();
        if out.len() < need {
            return Err(Error::BufferTooSmall { needed: need });
        }
        let sample = |frame: usize, c: usize| -> i16 {
            let i = (frame * ch + c) * 2;
            i16::from_le_bytes([pcm[i], pcm[i + 1]])
        };
        let mut w = 0usize;
        for c in 0..ch {
            let first = sample(0, c);
            self.state[c].predictor = i32::from(first);
            out[w..w + 2].copy_from_slice(&first.to_le_bytes());
            out[w + 2] = self.state[c].step_index;
            out[w + 3] = 0;
            w += 4;
        }
        let groups = (self.frames_per_block - 1) / 8;
        for g in 0..groups {
            for c in 0..ch {
                let base = 1 + g * 8;
                for j in 0..4 {
                    let lo = self.state[c].encode(sample(base + j * 2, c), self.rule);
                    let hi = self.state[c].encode(sample(base + j * 2 + 1, c), self.rule);
                    out[w] = lo | (hi << 4);
                    w += 1;
                }
            }
        }
        self.blocks += 1;
        Ok(w)
    }
}

/// Decoder for one stream. Stateless across blocks (every block carries its
/// own header), so it can start anywhere.
#[derive(Debug, Clone)]
pub struct Decoder {
    channels: u8,
    frames_per_block: usize,
    /// Blocks decoded.
    pub blocks: u64,
}

impl Decoder {
    /// A decoder for `channels` (1 or 2) and `frames_per_block`.
    pub fn new(channels: u8, frames_per_block: usize) -> Result<Self> {
        check(channels, frames_per_block)?;
        Ok(Decoder {
            channels,
            frames_per_block,
            blocks: 0,
        })
    }

    /// Bytes each block consumes.
    #[must_use]
    pub const fn block_align(&self) -> usize {
        block_align(self.channels, self.frames_per_block)
    }

    /// Bytes each block produces.
    #[must_use]
    pub const fn pcm_bytes_per_block(&self) -> usize {
        self.frames_per_block * self.channels as usize * 2
    }

    /// Decode one block (`block_align` bytes) to interleaved i16 PCM;
    /// returns [`pcm_bytes_per_block`](Self::pcm_bytes_per_block).
    pub fn decode_block(&mut self, block: &[u8], out: &mut [u8]) -> Result<usize> {
        let ch = self.channels as usize;
        if block.len() != self.block_align() {
            return Err(Error::InvalidGeometry);
        }
        let need = self.pcm_bytes_per_block();
        if out.len() < need {
            return Err(Error::BufferTooSmall { needed: need });
        }
        let mut state = [ChannelState::default(); MAX_CHANNELS];
        let mut r = 0usize;
        let put = |out: &mut [u8], frame: usize, c: usize, v: i16| {
            let i = (frame * ch + c) * 2;
            out[i..i + 2].copy_from_slice(&v.to_le_bytes());
        };
        for (c, st) in state.iter_mut().enumerate().take(ch) {
            let first = i16::from_le_bytes([block[r], block[r + 1]]);
            let idx = block[r + 2];
            if idx > 88 {
                return Err(Error::Corrupt);
            }
            *st = ChannelState {
                predictor: i32::from(first),
                step_index: idx,
            };
            put(out, 0, c, first);
            r += 4;
        }
        let groups = (self.frames_per_block - 1) / 8;
        for g in 0..groups {
            for (c, st) in state.iter_mut().enumerate().take(ch) {
                let base = 1 + g * 8;
                for j in 0..4 {
                    let byte = block[r];
                    r += 1;
                    let a = st.decode(byte & 0x0F);
                    let b = st.decode(byte >> 4);
                    put(out, base + j * 2, c, a);
                    put(out, base + j * 2 + 1, c, b);
                }
            }
        }
        self.blocks += 1;
        Ok(need)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn pcm_of(samples: &[i16]) -> Vec<u8> {
        samples.iter().flat_map(|s| s.to_le_bytes()).collect()
    }

    fn i16s(bytes: &[u8]) -> Vec<i16> {
        bytes
            .chunks_exact(2)
            .map(|b| i16::from_le_bytes([b[0], b[1]]))
            .collect()
    }

    #[test]
    fn sizes_match_the_wav_convention() {
        assert_eq!(block_align(1, 2041), 1024);
        assert_eq!(block_align(2, 2041), 2048);
        assert_eq!(frames_per_block(1, 1024), 2041);
        assert_eq!(frames_per_block(2, 2048), 2041);
        assert_eq!(frames_per_block(2, ffmpeg_block_align()), 1017);
        assert_eq!(block_align(2, 1017), 1024);
        assert_eq!(Encoder::new(3, 2041).err(), Some(Error::Unsupported));
        assert_eq!(Encoder::new(1, 2040).err(), Some(Error::InvalidFormat));
        assert_eq!(Decoder::new(1, 8).err(), Some(Error::InvalidFormat));
    }

    #[test]
    fn the_two_expansions_differ_by_truncation_only() {
        // step 7, nibble 1: reference 0 + (7>>2) = 1; ffmpeg encoder (3·7)>>3 = 2.
        assert_eq!(ChannelState::diff_reference(7, 1), 1);
        assert_eq!(ChannelState::diff_ffmpeg_encoder(7, 1), 2);
        // Multiples of 8 agree exactly.
        for n in 0..16u8 {
            assert_eq!(
                ChannelState::diff_reference(1024, n),
                ChannelState::diff_ffmpeg_encoder(1024, n)
            );
        }
        for step in STEP_TABLE {
            for n in 0..16u8 {
                let d = (ChannelState::diff_reference(step, n)
                    - ChannelState::diff_ffmpeg_encoder(step, n))
                .abs();
                assert!(d <= 2, "step {step} nibble {n}: {d}");
            }
        }
    }

    #[test]
    fn round_trip_tracks_a_ramp_closely() {
        // 17 frames per block (header + 16), mono.
        let mut enc = Encoder::new(1, 17).unwrap();
        let mut dec = Decoder::new(1, 17).unwrap();
        let samples: Vec<i16> = (0..17).map(|i| (i * 100 - 800) as i16).collect();
        let pcm = pcm_of(&samples);
        let mut blk = [0u8; 12];
        assert_eq!(enc.encode_block(&pcm, &mut blk).unwrap(), 12);
        assert_eq!(&blk[..2], &(-800i16).to_le_bytes());
        assert_eq!(blk[2], 0);
        let mut out = [0u8; 34];
        assert_eq!(dec.decode_block(&blk, &mut out).unwrap(), 34);
        let got = i16s(&out);
        assert_eq!(got[0], -800);
        // ADPCM from step index 0 needs a few samples to catch a 100/sample
        // ramp; by the end it is within one step.
        for (i, (g, s)) in got.iter().zip(&samples).enumerate().skip(8) {
            assert!(
                (i32::from(*g) - i32::from(*s)).abs() <= 60,
                "sample {i}: {g} vs {s}"
            );
        }
        assert_eq!(enc.blocks, 1);
        assert_eq!(dec.blocks, 1);
    }

    #[test]
    fn closed_loop_encoder_predictor_equals_decoder_output() {
        let mut enc = Encoder::new(1, 41).unwrap();
        let mut dec = Decoder::new(1, 41).unwrap();
        let samples: Vec<i16> = (0..41)
            .map(|i| ((i * 977) % 20_000 - 10_000) as i16)
            .collect();
        let pcm = pcm_of(&samples);
        let mut blk = [0u8; 24];
        enc.encode_block(&pcm, &mut blk).unwrap();
        let mut out = [0u8; 82];
        dec.decode_block(&blk, &mut out).unwrap();
        // The encoder's final predictor is exactly the decoder's last sample.
        assert_eq!(enc.state[0].predictor, i32::from(i16s(&out)[40]));
    }

    #[test]
    fn stereo_interleaves_by_word_and_step_index_carries() {
        let mut enc = Encoder::new(2, 9).unwrap();
        let mut dec = Decoder::new(2, 9).unwrap();
        // Left: a loud alternating signal; right: silence.
        let mut frames = Vec::new();
        for i in 0..9 {
            frames.push(if i % 2 == 0 { 20_000 } else { -20_000 });
            frames.push(0);
        }
        let pcm = pcm_of(&frames);
        let mut blk = [0u8; 16];
        assert_eq!(enc.encode_block(&pcm, &mut blk).unwrap(), 16);
        // Header: L first sample, idx 0; R first sample 0, idx 0.
        assert_eq!(&blk[..4], &[0x20, 0x4E, 0, 0]);
        assert_eq!(&blk[4..8], &[0, 0, 0, 0]);
        // Right channel word is all zero nibbles (silence codes as 0 each).
        assert_eq!(&blk[12..16], &[0, 0, 0, 0]);
        let mut out = [0u8; 36];
        dec.decode_block(&blk, &mut out).unwrap();
        let got = i16s(&out);
        assert!(got.iter().skip(1).step_by(2).all(|&r| r == 0));
        // A second block starts from the carried step index, which grew.
        let mut blk2 = [0u8; 16];
        enc.encode_block(&pcm, &mut blk2).unwrap();
        assert!(blk2[2] > 0, "left step index should have climbed");
        assert_eq!(blk2[6], 0, "right stayed at 0 (nibble 0 is -1 clamped)");
    }

    #[test]
    fn decoder_rejects_bad_headers_and_sizes() {
        let mut dec = Decoder::new(1, 9).unwrap();
        let mut out = [0u8; 18];
        let mut blk = [0u8; 8];
        blk[2] = 89;
        assert_eq!(dec.decode_block(&blk, &mut out).err(), Some(Error::Corrupt));
        assert_eq!(
            dec.decode_block(&blk[..7], &mut out).err(),
            Some(Error::InvalidGeometry)
        );
        blk[2] = 0;
        assert_eq!(
            dec.decode_block(&blk, &mut out[..10]).err(),
            Some(Error::BufferTooSmall { needed: 18 })
        );
    }
}
