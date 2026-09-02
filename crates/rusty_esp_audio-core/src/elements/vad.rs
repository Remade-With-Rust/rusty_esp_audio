//! `EnergyVad`: a block-energy voice activity detector with hangover — the
//! lite remake of ESP-SR's VAD stage.

use rusty_esp_core::error::Result;
use rusty_esp_core::pcm::{PcmBlock, PcmFormat};

use super::{require_i16, require_room};
use crate::pipeline::Element;
use crate::rms_dbfs_i16;

/// How the detector decides.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VadConfig {
    /// A block at or above this RMS level (dBFS) is speech.
    pub threshold_dbfs: f32,
    /// After the last speech block, stay "speaking" for this many more blocks.
    pub hangover_blocks: u32,
    /// When true, blocks judged silent are emitted as digital silence;
    /// otherwise every block passes through unchanged.
    pub gate: bool,
}

impl VadConfig {
    /// −45 dBFS, 10 blocks of hangover (200 ms at 20 ms), no gating.
    pub const VOICE: VadConfig = VadConfig {
        threshold_dbfs: -45.0,
        hangover_blocks: 10,
        gate: false,
    };
}

impl Default for VadConfig {
    fn default() -> Self {
        Self::VOICE
    }
}

/// The detector. Read [`is_speech`](Self::is_speech) after each block.
#[derive(Debug, Clone)]
pub struct EnergyVad {
    cfg: VadConfig,
    hang: u32,
    speech: bool,
    /// Level of the last block in dBFS.
    pub last_level_dbfs: f32,
    /// Blocks seen.
    pub blocks: u64,
    /// Blocks reported as speech (including hangover).
    pub speech_blocks: u64,
}

impl EnergyVad {
    /// A detector with `cfg`.
    #[must_use]
    pub fn new(cfg: VadConfig) -> Self {
        EnergyVad {
            cfg,
            hang: 0,
            speech: false,
            last_level_dbfs: -120.0,
            blocks: 0,
            speech_blocks: 0,
        }
    }

    /// Whether the last block was judged speech (or inside the hangover).
    #[must_use]
    pub fn is_speech(&self) -> bool {
        self.speech
    }

    /// The configuration.
    #[must_use]
    pub fn config(&self) -> &VadConfig {
        &self.cfg
    }

    /// Judge one block's level; returns the decision.
    pub fn judge(&mut self, level_dbfs: f32) -> bool {
        self.blocks += 1;
        self.last_level_dbfs = level_dbfs;
        if level_dbfs >= self.cfg.threshold_dbfs {
            self.hang = self.cfg.hangover_blocks;
            self.speech = true;
        } else if self.hang > 0 {
            self.hang -= 1;
            self.speech = true;
        } else {
            self.speech = false;
        }
        if self.speech {
            self.speech_blocks += 1;
        }
        self.speech
    }
}

impl Default for EnergyVad {
    fn default() -> Self {
        Self::new(VadConfig::VOICE)
    }
}

impl Element for EnergyVad {
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
        require_i16(input)?;
        Ok(input)
    }

    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
        require_i16(input.format)?;
        require_room(out, input.data.len())?;
        let speech = self.judge(rms_dbfs_i16(input.data));
        let n = input.data.len();
        if self.cfg.gate && !speech {
            out[..n].fill(0);
        } else {
            out[..n].copy_from_slice(input.data);
        }
        Ok(n)
    }

    fn reset(&mut self) {
        self.hang = 0;
        self.speech = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::{AudioSource, SineSource};
    use rusty_esp_core::time::Micros;

    #[test]
    fn tone_then_silence_with_hangover() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut vad = EnergyVad::new(VadConfig {
            hangover_blocks: 3,
            gate: true,
            ..VadConfig::VOICE
        });
        let mut src = SineSource::new(f, 440.0, 3000).unwrap(); // ≈ -24 dBFS
        let mut buf = [0u8; 640];
        let mut out = [0u8; 640];
        let silence = [0u8; 640];
        for _ in 0..5 {
            let blk = src.read(&mut buf).unwrap();
            vad.process(blk, &mut out).unwrap();
            assert!(vad.is_speech());
            assert_eq!(out, buf);
        }
        let mut decisions = [false; 6];
        for d in decisions.iter_mut() {
            let blk = PcmBlock::new(f, Micros::ZERO, &silence).unwrap();
            vad.process(blk, &mut out).unwrap();
            *d = vad.is_speech();
        }
        assert_eq!(decisions, [true, true, true, false, false, false]);
        assert_eq!(out, silence);
        assert_eq!(vad.blocks, 11);
        assert_eq!(vad.speech_blocks, 8);
    }

    #[test]
    fn quiet_noise_stays_below_threshold() {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut vad = EnergyVad::default();
        let mut src = SineSource::new(f, 440.0, 100).unwrap(); // ≈ -53 dBFS
        let mut buf = [0u8; 640];
        let mut out = [0u8; 640];
        let blk = src.read(&mut buf).unwrap();
        vad.process(blk, &mut out).unwrap();
        assert!(!vad.is_speech());
        assert!(vad.last_level_dbfs < -50.0 && vad.last_level_dbfs > -56.0);
        // Ungated: the block passes through untouched.
        assert_eq!(out, buf);
    }
}
