//! ES8311 — Everest's mono low-power ADC + DAC, the speaker-and-mic codec on
//! the ESP32-S3-BOX, Korvo-2 and S3-EYE.
//!
//! Register map, clock-coefficient table and the bring-up / start / stop
//! sequences are re-derived from Espressif's `esp-adf`
//! `components/audio_hal/driver/es8311/{es8311.h, es8311.c}` (ESPRESSIF MIT
//! License, © 2019 Espressif Systems (Shanghai) Co., Ltd): the same registers
//! in the same order with the same values, as data, with the arithmetic the
//! driver performs on the coefficient rows spelled out in [`Es8311::set_sample_rate`].
//! The volume register's meaning is the datasheet's: `0xBF` is 0 dB, one
//! step is 0.5 dB, `0x00` is −95.5 dB, `0xFF` is +32 dB.
//!
//! Not here: the PA-enable GPIO (a board pin the firmware drives) and the
//! I²C bus itself (any `embedded-hal` 1.0 `I2c`).

use embedded_hal::i2c::I2c;
use rusty_esp_core::error::{Error, Result};

use super::{Bits, CodecChip, I2cRegs, I2sFormat, Mode, Module};

/// 7-bit I²C address (the vendor driver's 8-bit `0x30`).
pub const ADDR: u8 = 0x18;

/// The chip-id bytes `CHD1_REGFD` / `CHD2_REGFE` read back on a real part.
pub const CHIP_ID: (u8, u8) = (0x83, 0x11);

/// The register map (`es8311.h`).
#[allow(missing_docs)]
pub mod reg {
    pub const RESET_REG00: u8 = 0x00;
    pub const CLK_MANAGER_REG01: u8 = 0x01;
    pub const CLK_MANAGER_REG02: u8 = 0x02;
    pub const CLK_MANAGER_REG03: u8 = 0x03;
    pub const CLK_MANAGER_REG04: u8 = 0x04;
    pub const CLK_MANAGER_REG05: u8 = 0x05;
    pub const CLK_MANAGER_REG06: u8 = 0x06;
    pub const CLK_MANAGER_REG07: u8 = 0x07;
    pub const CLK_MANAGER_REG08: u8 = 0x08;
    pub const SDPIN_REG09: u8 = 0x09;
    pub const SDPOUT_REG0A: u8 = 0x0A;
    pub const SYSTEM_REG0B: u8 = 0x0B;
    pub const SYSTEM_REG0C: u8 = 0x0C;
    pub const SYSTEM_REG0D: u8 = 0x0D;
    pub const SYSTEM_REG0E: u8 = 0x0E;
    pub const SYSTEM_REG0F: u8 = 0x0F;
    pub const SYSTEM_REG10: u8 = 0x10;
    pub const SYSTEM_REG11: u8 = 0x11;
    pub const SYSTEM_REG12: u8 = 0x12;
    pub const SYSTEM_REG13: u8 = 0x13;
    pub const SYSTEM_REG14: u8 = 0x14;
    pub const ADC_REG15: u8 = 0x15;
    pub const ADC_REG16: u8 = 0x16;
    pub const ADC_REG17: u8 = 0x17;
    pub const ADC_REG18: u8 = 0x18;
    pub const ADC_REG19: u8 = 0x19;
    pub const ADC_REG1A: u8 = 0x1A;
    pub const ADC_REG1B: u8 = 0x1B;
    pub const ADC_REG1C: u8 = 0x1C;
    pub const DAC_REG31: u8 = 0x31;
    pub const DAC_REG32: u8 = 0x32;
    pub const DAC_REG33: u8 = 0x33;
    pub const DAC_REG34: u8 = 0x34;
    pub const DAC_REG35: u8 = 0x35;
    pub const DAC_REG37: u8 = 0x37;
    pub const GPIO_REG44: u8 = 0x44;
    pub const GP_REG45: u8 = 0x45;
    pub const CHD1_REGFD: u8 = 0xFD;
    pub const CHD2_REGFE: u8 = 0xFE;
    pub const CHVER_REGFF: u8 = 0xFF;
}

/// One row of the clock-coefficient table: how to derive ADC/DAC clocks,
/// LRCK and BCLK for a (MCLK, sample rate) pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Coeff {
    /// Master clock, Hz.
    pub mclk: u32,
    /// Sample rate, Hz.
    pub rate: u32,
    /// Pre-divider, 1..=8.
    pub pre_div: u8,
    /// Pre-multiplier: 1, 2, 4 or 8.
    pub pre_multi: u8,
    /// ADC clock divider.
    pub adc_div: u8,
    /// DAC clock divider.
    pub dac_div: u8,
    /// 0 = single speed, 1 = double speed.
    pub fs_mode: u8,
    /// LRCK divider, high byte.
    pub lrck_h: u8,
    /// LRCK divider, low byte.
    pub lrck_l: u8,
    /// BCLK divider.
    pub bclk_div: u8,
    /// ADC oversampling.
    pub adc_osr: u8,
    /// DAC oversampling.
    pub dac_osr: u8,
}

/// A table row; the argument list is the vendor table's column order.
#[allow(clippy::too_many_arguments)]
const fn c(
    mclk: u32,
    rate: u32,
    pre_div: u8,
    pre_multi: u8,
    adc_div: u8,
    dac_div: u8,
    fs_mode: u8,
    lrck_h: u8,
    lrck_l: u8,
    bclk_div: u8,
    adc_osr: u8,
    dac_osr: u8,
) -> Coeff {
    Coeff {
        mclk,
        rate,
        pre_div,
        pre_multi,
        adc_div,
        dac_div,
        fs_mode,
        lrck_h,
        lrck_l,
        bclk_div,
        adc_osr,
        dac_osr,
    }
}

/// The vendor driver's `coeff_div[]`, verbatim.
pub const COEFF: &[Coeff] = &[
    // 8k
    c(
        12_288_000, 8000, 0x06, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        18_432_000, 8000, 0x03, 0x02, 0x03, 0x03, 0x00, 0x05, 0xff, 0x18, 0x10, 0x20,
    ),
    c(
        16_384_000, 8000, 0x08, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        8_192_000, 8000, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        6_144_000, 8000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        4_096_000, 8000, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        3_072_000, 8000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        2_048_000, 8000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        1_536_000, 8000, 0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        1_024_000, 8000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    // 11.025k
    c(
        11_289_600, 11025, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        5_644_800, 11025, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        2_822_400, 11025, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        1_411_200, 11025, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    // 12k
    c(
        12_288_000, 12000, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        6_144_000, 12000, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        3_072_000, 12000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        1_536_000, 12000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    // 16k
    c(
        12_288_000, 16000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        18_432_000, 16000, 0x03, 0x02, 0x03, 0x03, 0x00, 0x02, 0xff, 0x0c, 0x10, 0x20,
    ),
    c(
        16_384_000, 16000, 0x04, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        8_192_000, 16000, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        6_144_000, 16000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        4_096_000, 16000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        3_072_000, 16000, 0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        2_048_000, 16000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        1_536_000, 16000, 0x03, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    c(
        1_024_000, 16000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x20,
    ),
    // 22.05k
    c(
        11_289_600, 22050, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        5_644_800, 22050, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        2_822_400, 22050, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        1_411_200, 22050, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    // 24k
    c(
        12_288_000, 24000, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        18_432_000, 24000, 0x03, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        6_144_000, 24000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        3_072_000, 24000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        1_536_000, 24000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    // 32k
    c(
        12_288_000, 32000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        18_432_000, 32000, 0x03, 0x04, 0x03, 0x03, 0x00, 0x02, 0xff, 0x0c, 0x10, 0x10,
    ),
    c(
        16_384_000, 32000, 0x02, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        8_192_000, 32000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        6_144_000, 32000, 0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        4_096_000, 32000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        3_072_000, 32000, 0x03, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        2_048_000, 32000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        1_536_000, 32000, 0x03, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10,
    ),
    c(
        1_024_000, 32000, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    // 44.1k
    c(
        11_289_600, 44100, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        5_644_800, 44100, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        2_822_400, 44100, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        1_411_200, 44100, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    // 48k
    c(
        12_288_000, 48000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        18_432_000, 48000, 0x03, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        6_144_000, 48000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        3_072_000, 48000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        1_536_000, 48000, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    // 64k
    c(
        12_288_000, 64000, 0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        18_432_000, 64000, 0x03, 0x04, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10,
    ),
    c(
        16_384_000, 64000, 0x01, 0x01, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        8_192_000, 64000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        6_144_000, 64000, 0x01, 0x04, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10,
    ),
    c(
        4_096_000, 64000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        3_072_000, 64000, 0x01, 0x08, 0x03, 0x03, 0x01, 0x01, 0x7f, 0x06, 0x10, 0x10,
    ),
    c(
        2_048_000, 64000, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        1_536_000, 64000, 0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0xbf, 0x03, 0x18, 0x18,
    ),
    c(
        1_024_000, 64000, 0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10,
    ),
    // 88.2k
    c(
        11_289_600, 88200, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        5_644_800, 88200, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        2_822_400, 88200, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        1_411_200, 88200, 0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10,
    ),
    // 96k
    c(
        12_288_000, 96000, 0x01, 0x02, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        18_432_000, 96000, 0x03, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        6_144_000, 96000, 0x01, 0x04, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        3_072_000, 96000, 0x01, 0x08, 0x01, 0x01, 0x00, 0x00, 0xff, 0x04, 0x10, 0x10,
    ),
    c(
        1_536_000, 96000, 0x01, 0x08, 0x01, 0x01, 0x01, 0x00, 0x7f, 0x02, 0x10, 0x10,
    ),
];

/// The coefficient row for a (MCLK, rate) pair, if the chip supports it.
#[must_use]
pub fn coeff(mclk_hz: u32, rate_hz: u32) -> Option<&'static Coeff> {
    COEFF
        .iter()
        .find(|c| c.mclk == mclk_hz && c.rate == rate_hz)
}

/// The vendor driver's default master clock for a rate: 256 × Fs.
#[must_use]
pub const fn mclk_for(rate_hz: u32) -> u32 {
    rate_hz * 256
}

/// Where the chip's internal MCLK comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MclkSource {
    /// The MCLK pin (the ESP32 drives a 256 × Fs clock).
    Pin,
    /// Multiplied up from BCLK (no MCLK wire: `DIG_MCLK = LRCK × 256 = BCLK × 8`).
    Sclk,
}

/// Analog microphone PGA gain (`ADC_REG16`), in 6 dB steps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MicGain {
    /// 0 dB.
    Db0 = 0,
    /// 6 dB.
    Db6 = 1,
    /// 12 dB.
    Db12 = 2,
    /// 18 dB.
    Db18 = 3,
    /// 24 dB.
    Db24 = 4,
    /// 30 dB.
    Db30 = 5,
    /// 36 dB.
    Db36 = 6,
    /// 42 dB.
    Db42 = 7,
}

/// Bring-up parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Who drives BCLK / LRCK.
    pub mode: Mode,
    /// Internal MCLK source.
    pub mclk: MclkSource,
    /// Invert the MCLK input.
    pub invert_mclk: bool,
    /// Invert BCLK.
    pub invert_sclk: bool,
    /// The microphone is a PDM digital mic, not analog.
    pub dmic: bool,
    /// Sample rate, Hz.
    pub sample_rate_hz: u32,
    /// Master clock, Hz (the coefficient table must have the pair).
    pub mclk_hz: u32,
}

impl Config {
    /// The vendor driver's defaults: slave, MCLK from the pin, nothing
    /// inverted, analog mic, 256 × Fs.
    #[must_use]
    pub const fn slave(sample_rate_hz: u32) -> Self {
        Config {
            mode: Mode::Slave,
            mclk: MclkSource::Pin,
            invert_mclk: false,
            invert_sclk: false,
            dmic: false,
            sample_rate_hz,
            mclk_hz: mclk_for(sample_rate_hz),
        }
    }
}

/// The DAC volume register for a level in half-decibels: `0` → `0xBF`
/// (0 dB), `+64` → `0xFF` (+32 dB), `-191` → `0x00` (−95.5 dB); clamped.
#[must_use]
pub const fn dac_volume_reg(half_db: i16) -> u8 {
    let v = 0xBF + half_db as i32;
    if v < 0 {
        0
    } else if v > 0xFF {
        0xFF
    } else {
        v as u8
    }
}

/// The driver.
#[derive(Debug)]
pub struct Es8311<I> {
    regs: I2cRegs<I>,
    dmic: bool,
    mclk: MclkSource,
}

impl<I: I2c> Es8311<I> {
    /// Over `i2c` at the chip's 7-bit `addr` (normally [`ADDR`]).
    pub fn new(i2c: I, addr: u8) -> Self {
        Es8311 {
            regs: I2cRegs::new(i2c, addr),
            dmic: false,
            mclk: MclkSource::Pin,
        }
    }

    /// Give the bus back.
    pub fn release(self) -> I {
        self.regs.release()
    }

    /// The register map, for anything the driver does not wrap.
    pub fn regs(&mut self) -> &mut I2cRegs<I> {
        &mut self.regs
    }

    /// Read the chip id bytes; [`CHIP_ID`] on a real part.
    pub fn chip_id(&mut self) -> Result<(u8, u8)> {
        Ok((
            self.regs.read(reg::CHD1_REGFD)?,
            self.regs.read(reg::CHD2_REGFE)?,
        ))
    }

    /// The vendor driver's `es8311_codec_init`: the fixed bring-up writes,
    /// master/slave, MCLK source, the clock coefficients for `cfg`'s rate,
    /// the inversions, and the ADC front-end defaults.
    pub fn init(&mut self, cfg: &Config) -> Result<()> {
        self.dmic = cfg.dmic;
        self.mclk = cfg.mclk;
        let r = &mut self.regs;
        // "Enhance ES8311 I2C noise immunity"; written twice because the
        // first write to the chip occasionally fails.
        r.write(reg::GPIO_REG44, 0x08)?;
        r.write(reg::GPIO_REG44, 0x08)?;
        r.write(reg::CLK_MANAGER_REG01, 0x30)?;
        r.write(reg::CLK_MANAGER_REG02, 0x00)?;
        r.write(reg::CLK_MANAGER_REG03, 0x10)?;
        r.write(reg::ADC_REG16, 0x24)?;
        r.write(reg::CLK_MANAGER_REG04, 0x10)?;
        r.write(reg::CLK_MANAGER_REG05, 0x00)?;
        r.write(reg::SYSTEM_REG0B, 0x00)?;
        r.write(reg::SYSTEM_REG0C, 0x00)?;
        r.write(reg::SYSTEM_REG10, 0x1F)?;
        r.write(reg::SYSTEM_REG11, 0x7F)?;
        r.write(reg::RESET_REG00, 0x80)?;
        let mut regv = r.read(reg::RESET_REG00)?;
        match cfg.mode {
            Mode::Master => regv |= 0x40,
            Mode::Slave => regv &= 0xBF,
        }
        r.write(reg::RESET_REG00, regv)?;
        r.write(reg::CLK_MANAGER_REG01, 0x3F)?;
        let mut regv = r.read(reg::CLK_MANAGER_REG01)?;
        match cfg.mclk {
            MclkSource::Pin => regv &= 0x7F,
            MclkSource::Sclk => regv |= 0x80,
        }
        r.write(reg::CLK_MANAGER_REG01, regv)?;
        self.set_sample_rate(cfg.sample_rate_hz, cfg.mclk_hz)?;
        let r = &mut self.regs;
        let mut regv = r.read(reg::CLK_MANAGER_REG01)?;
        if cfg.invert_mclk {
            regv |= 0x40;
        } else {
            regv &= !0x40;
        }
        r.write(reg::CLK_MANAGER_REG01, regv)?;
        let mut regv = r.read(reg::CLK_MANAGER_REG06)?;
        if cfg.invert_sclk {
            regv |= 0x20;
        } else {
            regv &= !0x20;
        }
        r.write(reg::CLK_MANAGER_REG06, regv)?;
        r.write(reg::SYSTEM_REG13, 0x10)?;
        r.write(reg::ADC_REG1B, 0x0A)?;
        r.write(reg::ADC_REG1C, 0x6A)?;
        Ok(())
    }

    /// DAC volume register, straight ([`dac_volume_reg`] maps half-dB to it).
    pub fn set_dac_volume_reg(&mut self, value: u8) -> Result<()> {
        self.regs.write(reg::DAC_REG32, value)
    }

    /// Analog mic gain (`ADC_REG16`).
    pub fn set_mic_gain(&mut self, gain: MicGain) -> Result<()> {
        self.regs.write(reg::ADC_REG16, gain as u8)
    }
}

impl<I: I2c> CodecChip for Es8311<I> {
    /// `es8311_config_sample`: the coefficient row's fields into
    /// `CLK_MANAGER_REG02..REG08`, read-modify-write where the driver keeps
    /// bits.
    fn set_sample_rate(&mut self, sample_rate_hz: u32, mclk_hz: u32) -> Result<()> {
        let c = coeff(mclk_hz, sample_rate_hz).ok_or(Error::Unsupported)?;
        let r = &mut self.regs;
        let mut regv = r.read(reg::CLK_MANAGER_REG02)? & 0x07;
        regv |= (c.pre_div - 1) << 5;
        let mut datmp: u8 = match c.pre_multi {
            1 => 0,
            2 => 1,
            4 => 2,
            8 => 3,
            _ => 0,
        };
        if self.mclk == MclkSource::Sclk {
            // DIG_MCLK = LRCK × 256 = BCLK × 8; at 8 kHz BCLK needs 512 kHz
            // (32-bit slots) and the multiplier is ×4.
            datmp = if sample_rate_hz == 8000 { 2 } else { 3 };
        }
        regv |= datmp << 3;
        r.write(reg::CLK_MANAGER_REG02, regv)?;
        let regv = ((c.adc_div - 1) << 4) | (c.dac_div - 1);
        r.write(reg::CLK_MANAGER_REG05, regv)?;
        let mut regv = r.read(reg::CLK_MANAGER_REG03)? & 0x80;
        regv |= c.fs_mode << 6;
        regv |= c.adc_osr;
        r.write(reg::CLK_MANAGER_REG03, regv)?;
        let mut regv = r.read(reg::CLK_MANAGER_REG04)? & 0x80;
        regv |= c.dac_osr;
        r.write(reg::CLK_MANAGER_REG04, regv)?;
        let mut regv = r.read(reg::CLK_MANAGER_REG07)? & 0xC0;
        regv |= c.lrck_h;
        r.write(reg::CLK_MANAGER_REG07, regv)?;
        r.write(reg::CLK_MANAGER_REG08, c.lrck_l)?;
        let mut regv = r.read(reg::CLK_MANAGER_REG06)? & 0xE0;
        regv |= if c.bclk_div < 19 {
            c.bclk_div - 1
        } else {
            c.bclk_div
        };
        r.write(reg::CLK_MANAGER_REG06, regv)
    }

    /// `es8311_config_fmt` then `es8311_set_bits_per_sample`, on the DAC
    /// (`SDPIN_REG09`) and ADC (`SDPOUT_REG0A`) serial ports.
    fn set_format(&mut self, fmt: I2sFormat, bits: Bits) -> Result<()> {
        let r = &mut self.regs;
        let mut dac = r.read(reg::SDPIN_REG09)?;
        let mut adc = r.read(reg::SDPOUT_REG0A)?;
        match fmt {
            I2sFormat::Standard => {
                dac &= 0xFC;
                adc &= 0xFC;
            }
            I2sFormat::LeftJustified => {
                dac = (dac & 0xFC) | 0x01;
                adc = (adc & 0xFC) | 0x01;
            }
            I2sFormat::Dsp => {
                dac = (dac & 0xDC) | 0x03;
                adc = (adc & 0xDC) | 0x03;
            }
        }
        r.write(reg::SDPIN_REG09, dac)?;
        r.write(reg::SDPOUT_REG0A, adc)?;
        let mut dac = r.read(reg::SDPIN_REG09)?;
        let mut adc = r.read(reg::SDPOUT_REG0A)?;
        match bits {
            Bits::B16 => {
                dac |= 0x0C;
                adc |= 0x0C;
            }
            Bits::B24 => {}
            Bits::B32 => {
                dac |= 0x10;
                adc |= 0x10;
            }
        }
        r.write(reg::SDPIN_REG09, dac)?;
        r.write(reg::SDPOUT_REG0A, adc)
    }

    /// `es8311_start`: unmute the chosen serial ports, power the analog
    /// blocks, choose the analog or PDM mic, set the ADC/DAC ramps and the
    /// internal reference.
    fn start(&mut self, module: Module) -> Result<()> {
        let r = &mut self.regs;
        let mut dac = (r.read(reg::SDPIN_REG09)? & 0xBF) | 0x40;
        let mut adc = (r.read(reg::SDPOUT_REG0A)? & 0xBF) | 0x40;
        if matches!(module, Module::Adc | Module::AdcDac) {
            adc &= !0x40;
        }
        if matches!(module, Module::Dac | Module::AdcDac) {
            dac &= !0x40;
        }
        r.write(reg::SDPIN_REG09, dac)?;
        r.write(reg::SDPOUT_REG0A, adc)?;
        r.write(reg::ADC_REG17, 0xBF)?;
        r.write(reg::SYSTEM_REG0E, 0x02)?;
        r.write(reg::SYSTEM_REG12, 0x00)?;
        r.write(reg::SYSTEM_REG14, 0x1A)?;
        let mut regv = r.read(reg::SYSTEM_REG14)?;
        if self.dmic {
            regv |= 0x40;
        } else {
            regv &= !0x40;
        }
        r.write(reg::SYSTEM_REG14, regv)?;
        r.write(reg::SYSTEM_REG0D, 0x01)?;
        r.write(reg::ADC_REG15, 0x40)?;
        r.write(reg::DAC_REG37, 0x08)?;
        r.write(reg::GP_REG45, 0x00)?;
        // internal reference signal (ADCL + DACR)
        r.write(reg::GPIO_REG44, 0x58)
    }

    /// `es8311_suspend`: volumes to minimum, analog blocks off.
    fn stop(&mut self) -> Result<()> {
        let r = &mut self.regs;
        r.write(reg::DAC_REG32, 0x00)?;
        r.write(reg::ADC_REG17, 0x00)?;
        r.write(reg::SYSTEM_REG0E, 0xFF)?;
        r.write(reg::SYSTEM_REG12, 0x02)?;
        r.write(reg::SYSTEM_REG14, 0x00)?;
        r.write(reg::SYSTEM_REG0D, 0xFA)?;
        r.write(reg::ADC_REG15, 0x00)?;
        r.write(reg::GP_REG45, 0x01)
    }

    /// `es8311_mute`: DAC mute bits in `DAC_REG31`.
    fn set_mute(&mut self, mute: bool) -> Result<()> {
        let regv = self.regs.read(reg::DAC_REG31)? & 0x9F;
        self.regs
            .write(reg::DAC_REG31, if mute { regv | 0x60 } else { regv })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chip::fake::FakeCodec;

    #[test]
    fn coefficient_table_is_complete_and_findable() {
        assert_eq!(COEFF.len(), 75, "the vendor table's 75 rows");
        assert!(coeff(4_096_000, 16000).is_some());
        assert!(coeff(12_288_000, 48000).is_some());
        assert!(coeff(4_096_000, 48000).is_none());
        assert_eq!(mclk_for(16000), 4_096_000);
        for c in COEFF {
            assert!((1..=8).contains(&c.pre_div), "{c:?}");
            assert!(matches!(c.pre_multi, 1 | 2 | 4 | 8), "{c:?}");
            assert!(c.adc_div >= 1 && c.dac_div >= 1, "{c:?}");
        }
    }

    #[test]
    fn volume_register_is_half_db_from_zero_at_bf() {
        assert_eq!(dac_volume_reg(0), 0xBF);
        assert_eq!(dac_volume_reg(64), 0xFF);
        assert_eq!(dac_volume_reg(65), 0xFF);
        assert_eq!(dac_volume_reg(-191), 0x00);
        assert_eq!(dac_volume_reg(-200), 0x00);
        assert_eq!(dac_volume_reg(-100), 0x5B, "the driver's -50 dB point");
    }

    /// The bring-up writes the vendor driver makes, in its order, for the
    /// default slave configuration at 16 kHz with a 4.096 MHz MCLK.
    #[test]
    fn init_reproduces_the_vendor_sequence() {
        let mut chip = Es8311::new(FakeCodec::default(), ADDR);
        chip.init(&Config::slave(16_000)).unwrap();
        let bus = chip.release();
        assert_eq!(bus.addr_seen, Some(0x18));
        let expected: &[(u8, u8)] = &[
            (0x44, 0x08),
            (0x44, 0x08),
            (0x01, 0x30),
            (0x02, 0x00),
            (0x03, 0x10),
            (0x16, 0x24),
            (0x04, 0x10),
            (0x05, 0x00),
            (0x0B, 0x00),
            (0x0C, 0x00),
            (0x10, 0x1F),
            (0x11, 0x7F),
            (0x00, 0x80),
            (0x00, 0x80), // slave: bit 6 cleared
            (0x01, 0x3F),
            (0x01, 0x3F), // MCLK from the pin: bit 7 cleared
            // the 16 kHz / 4.096 MHz row: pre_div 1, ×1, dividers 1, ss, lrck 0x00ff, bclk 4
            (0x02, 0x00),
            (0x05, 0x00),
            (0x03, 0x10),
            (0x04, 0x20), // DAC OSR 0x20 in the 8 to 16 kHz rows
            (0x07, 0x00),
            (0x08, 0xFF),
            (0x06, 0x03),
            (0x01, 0x3F), // MCLK not inverted
            (0x06, 0x03), // BCLK not inverted
            (0x13, 0x10),
            (0x1B, 0x0A),
            (0x1C, 0x6A),
        ];
        assert_eq!(bus.writes, expected);
    }

    #[test]
    fn master_mode_and_sclk_derived_mclk_set_their_bits() {
        let mut chip = Es8311::new(FakeCodec::default(), ADDR);
        chip.init(&Config {
            mode: Mode::Master,
            mclk: MclkSource::Sclk,
            invert_mclk: true,
            invert_sclk: true,
            dmic: true,
            sample_rate_hz: 8000,
            mclk_hz: mclk_for(8000),
        })
        .unwrap();
        let bus = chip.release();
        assert_eq!(bus.reg(0x00), 0xC0, "master bit on top of the reset value");
        assert_eq!(bus.reg(0x01), 0xFF, "MCLK from BCLK, inverted");
        // 8 kHz / 2.048 MHz over BCLK: pre_div 1 (bits 7..5 clear), multiplier x4 (datmp 2) in bits 4..3
        assert_eq!(bus.reg(0x02), 2 << 3);
        assert!(bus.reg(0x06) & 0x20 != 0, "BCLK inverted");
        assert_eq!(bus.reg(0x1C), 0x6A);
    }

    #[test]
    fn format_start_stop_mute_and_gain() {
        let mut chip = Es8311::new(FakeCodec::default(), ADDR);
        chip.init(&Config::slave(48_000)).unwrap();
        chip.set_format(I2sFormat::Standard, Bits::B16).unwrap();
        chip.start(Module::AdcDac).unwrap();
        {
            let bus = chip.regs().bus();
            assert_eq!(bus.reg(0x09) & 0x4F, 0x0C, "DAC port: 16-bit, unmuted");
            assert_eq!(bus.reg(0x0A) & 0x4F, 0x0C, "ADC port: 16-bit, unmuted");
            assert_eq!(bus.reg(0x14), 0x1A, "analog mic, PGA on");
            assert_eq!(bus.reg(0x44), 0x58);
            assert_eq!(bus.reg(0x45), 0x00);
        }
        chip.set_format(I2sFormat::Dsp, Bits::B32).unwrap();
        chip.start(Module::Dac).unwrap();
        {
            let bus = chip.regs().bus();
            assert_eq!(bus.reg(0x09) & 0x53, 0x13, "DAC: DSP, 32-bit, on");
            assert_eq!(
                bus.reg(0x0A) & 0x40,
                0x40,
                "ADC port muted when only the DAC starts"
            );
        }
        chip.set_mute(true).unwrap();
        assert_eq!(chip.regs().bus().reg(0x31) & 0x60, 0x60);
        chip.set_mute(false).unwrap();
        assert_eq!(chip.regs().bus().reg(0x31) & 0x60, 0x00);
        chip.set_mic_gain(MicGain::Db24).unwrap();
        chip.set_dac_volume_reg(dac_volume_reg(-12)).unwrap();
        chip.stop().unwrap();
        let bus = chip.release();
        assert_eq!(bus.reg(0x16), 4);
        assert_eq!(
            bus.history(0x32),
            [0xB3, 0x00],
            "volume then the suspend's zero"
        );
        assert_eq!(bus.reg(0x0D), 0xFA);
        assert_eq!(bus.reg(0x45), 0x01);
        assert_eq!(bus.reg(0x0E), 0xFF);
    }

    #[test]
    fn unsupported_clock_pair_is_refused_before_any_write() {
        let mut chip = Es8311::new(FakeCodec::default(), ADDR);
        assert_eq!(
            chip.set_sample_rate(44_100, 4_096_000),
            Err(Error::Unsupported)
        );
        assert!(chip.release().writes.is_empty());
    }

    #[test]
    fn chip_id_reads_the_id_registers() {
        let mut chip = Es8311::new(
            FakeCodec::with_defaults(&[(0xFD, 0x83), (0xFE, 0x11)]),
            ADDR,
        );
        assert_eq!(chip.chip_id().unwrap(), CHIP_ID);
    }
}
