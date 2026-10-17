//! `Agc`: block-wise automatic gain control, the lite stand-in for ESP-SR's.

use rusty_esp_core::error::Result;
use rusty_esp_core::pcm::{PcmBlock, PcmFormat, as_i16, as_i16_mut};

use super::{require_i16, require_room};
use crate::pipeline::Element;
use crate::{get_i16, put_i16, rms_dbfs_i16, round_sat16};

/// How the AGC behaves.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AgcConfig {
    /// Level the output settles to, in dBFS (RMS).
    pub target_dbfs: f32,
    /// The gain never exceeds this many dB.
    pub max_gain_db: f32,
    /// The gain never goes below this many dB (negative = allowed to attenuate).
    pub min_gain_db: f32,
    /// How fast the gain comes DOWN when the input is too loud, in dB per second.
    pub attack_db_per_s: f32,
    /// How fast the gain goes UP when the input is too quiet, in dB per second.
    pub release_db_per_s: f32,
    /// Below this input level the gain is frozen, so silence and room noise
    /// are not pumped up to the target.
    pub noise_floor_dbfs: f32,
}

impl AgcConfig {
    /// Voice defaults: −20 dBFS target, +30/−24 dB gain range, 60 dB/s
    /// attack, 6 dB/s release, −55 dBFS floor.
    pub const VOICE: AgcConfig = AgcConfig {
        target_dbfs: -20.0,
        max_gain_db: 30.0,
        min_gain_db: -24.0,
        attack_db_per_s: 60.0,
        release_db_per_s: 6.0,
        noise_floor_dbfs: -55.0,
    };
}

impl Default for AgcConfig {
    fn default() -> Self {
        Self::VOICE
    }
}

/// One gain per block, moved toward the target at the attack or release
/// rate, applied in `f32` with rounding and saturation.
///
/// The block level is its RMS in dBFS; the correction is
/// `target − (level + gain)`, limited to what the rate allows in one block
/// duration and to the gain range. Because the whole block gets one gain,
/// blocks should be 10–30 ms.
#[derive(Debug, Clone)]
pub struct Agc {
    cfg: AgcConfig,
    gain_db: f32,
    /// Blocks processed.
    pub blocks: u64,
    /// Blocks whose level was below the noise floor (gain frozen).
    pub frozen_blocks: u64,
    /// Level of the last input block in dBFS.
    pub last_level_dbfs: f32,
}

impl Agc {
    /// An AGC starting at 0 dB gain.
    #[must_use]
    pub fn new(cfg: AgcConfig) -> Self {
        Agc {
            cfg,
            gain_db: 0.0,
            blocks: 0,
            frozen_blocks: 0,
            last_level_dbfs: -120.0,
        }
    }

    /// The gain being applied, in dB.
    #[must_use]
    pub fn gain_db(&self) -> f32 {
        self.gain_db
    }

    /// The configuration.
    #[must_use]
    pub fn config(&self) -> &AgcConfig {
        &self.cfg
    }

    /// Update the gain for a block of `level_dbfs` lasting `block_secs`.
    fn update(&mut self, level_dbfs: f32, block_secs: f32) {
        self.last_level_dbfs = level_dbfs;
        if level_dbfs < self.cfg.noise_floor_dbfs {
            self.frozen_blocks += 1;
            return;
        }
        let want = self.cfg.target_dbfs - (level_dbfs + self.gain_db);
        let step = if want < 0.0 {
            -(self.cfg.attack_db_per_s * block_secs).min(-want)
        } else {
            (self.cfg.release_db_per_s * block_secs).min(want)
        };
        self.gain_db = (self.gain_db + step).clamp(self.cfg.min_gain_db, self.cfg.max_gain_db);
    }
}

impl Default for Agc {
    fn default() -> Self {
        Self::new(AgcConfig::VOICE)
    }
}

impl Element for Agc {
    fn output_format(&self, input: PcmFormat) -> Result<PcmFormat> {
        require_i16(input)?;
        Ok(input)
    }

    fn process(&mut self, input: PcmBlock<'_>, out: &mut [u8]) -> Result<usize> {
        require_i16(input.format)?;
        require_room(out, input.data.len())?;
        self.blocks += 1;
        let level = rms_dbfs_i16(input.data);
        let secs = input.duration_micros() as f32 / 1_000_000.0;
        self.update(level, secs);
        let lin = libm::powf(10.0, self.gain_db / 20.0);
        // FAST ARM: native halfword loads; the byte path below is the oracle.
        let n = input.data.len();
        if let (Some(src), Some(dst)) = (as_i16(input.data), as_i16_mut(&mut out[..n])) {
            let mut ci = src.chunks_exact(8);
            let mut co = dst.chunks_exact_mut(8);
            for (i, o) in ci.by_ref().zip(co.by_ref()) {
                for k in 0..8 {
                    o[k] = round_sat16(f32::from(i[k]) * lin);
                }
            }
            for (i, o) in ci.remainder().iter().zip(co.into_remainder().iter_mut()) {
                *o = round_sat16(f32::from(*i) * lin);
            }
            return Ok(n);
        }

        // Four samples a trip. The gain is resolved once per block already;
        // what was left per sample was a dependent load-store pair wrapped in
        // loop overhead, which an in-order core cannot overlap.
        let n = input.data.len();
        let mut ci = input.data.chunks_exact(16);
        let mut co = out[..n].chunks_exact_mut(16);
        for (i, o) in ci.by_ref().zip(co.by_ref()) {
            for k in 0..8 {
                let y = round_sat16(f32::from(get_i16(&i[k * 2..])) * lin);
                put_i16(&mut o[k * 2..], y);
            }
        }
        for (i, o) in ci
            .remainder()
            .chunks_exact(2)
            .zip(co.into_remainder().chunks_exact_mut(2))
        {
            put_i16(o, round_sat16(f32::from(get_i16(i)) * lin));
        }
        Ok(n)
    }

    fn reset(&mut self) {
        self.gain_db = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::source::{AudioSource, SineSource};

    /// Run `secs` seconds of a tone at `amplitude` through a voice AGC and
    /// return the output level of the last block.
    fn settle(amplitude: i16, secs: u32) -> (f32, Agc) {
        let f = PcmFormat::PCM16_16K_MONO;
        let mut agc = Agc::default();
        let mut src = SineSource::new(f, 440.0, amplitude).unwrap();
        let mut buf = [0u8; 640]; // 20 ms
        let mut out = [0u8; 640];
        let mut level = 0.0;
        for _ in 0..secs * 50 {
            let blk = src.read(&mut buf).unwrap();
            agc.process(blk, &mut out).unwrap();
            level = rms_dbfs_i16(&out);
        }
        (level, agc)
    }

    #[test]
    fn quiet_input_is_raised_to_target() {
        // 328 peak ≈ -43 dBFS RMS: 23 dB below target, 6 dB/s release → ~4 s.
        let (level, agc) = settle(328, 6);
        assert!(
            (level + 20.0).abs() < 0.6,
            "{level} at gain {}",
            agc.gain_db()
        );
    }

    #[test]
    fn loud_input_is_pulled_down_fast() {
        // 30000 peak ≈ -3.7 dBFS RMS: 16 dB above target at 60 dB/s → <0.5 s.
        let (level, _) = settle(30_000, 1);
        assert!((level + 20.0).abs() < 0.6, "{level}");
    }

    #[test]
    fn gain_is_capped_and_silence_freezes_it() {
        // 10 peak ≈ -73 dBFS: below the noise floor, gain must stay at 0 dB.
        let (_, agc) = settle(10, 1);
        assert_eq!(agc.gain_db(), 0.0);
        assert_eq!(agc.frozen_blocks, 50);
        // 60 peak ≈ -57 dBFS: still under the floor; 120 peak ≈ -51 dBFS: over
        // it but 31 dB below target, so the gain saturates at +30 dB.
        let (_, agc) = settle(120, 8);
        assert!((agc.gain_db() - 30.0).abs() < 1e-3, "{}", agc.gain_db());
    }
}
