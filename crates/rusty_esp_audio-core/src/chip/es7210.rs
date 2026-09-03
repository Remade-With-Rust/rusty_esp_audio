//! ES7210 — Everest's four-channel ADC, the microphone-array front end on
//! the Korvo-2 and S3-EYE (two of its four inputs feed the S3-BOX's array).
//!
//! Register map, clock-coefficient table and every sequence are re-derived
//! from Espressif's `esp-adf` `components/audio_hal/driver/es7210/{es7210.h,
//! es7210.c}` (ESPRESSIF MIT License, © 2019 Espressif Systems (Shanghai) Co.,
//! Ltd): the same registers in the same order with the same values, as
//! data. The one behavioural choice the vendor driver makes at compile time
//! — TDM on the serial port once three or more mics are enabled — is a field
//! of [`Config`].

use embedded_hal::i2c::I2c;
use rusty_esp_core::error::{Error, Result};

use super::{Bits, CodecChip, I2cRegs, I2sFormat, Mode, Module};

/// 7-bit I²C addresses by the AD1/AD0 strap (the vendor's `0x80/0x82/0x84/0x86`).
pub const ADDR_00: u8 = 0x40;
/// AD1 = 0, AD0 = 1.
pub const ADDR_01: u8 = 0x41;
/// AD1 = 1, AD0 = 0.
pub const ADDR_10: u8 = 0x42;
/// AD1 = 1, AD0 = 1.
pub const ADDR_11: u8 = 0x43;

/// The register map (`es7210.h`).
#[allow(missing_docs)]
pub mod reg {
    pub const RESET_REG00: u8 = 0x00;
    pub const CLOCK_OFF_REG01: u8 = 0x01;
    pub const MAINCLK_REG02: u8 = 0x02;
    pub const MASTER_CLK_REG03: u8 = 0x03;
    pub const LRCK_DIVH_REG04: u8 = 0x04;
    pub const LRCK_DIVL_REG05: u8 = 0x05;
    pub const POWER_DOWN_REG06: u8 = 0x06;
    pub const OSR_REG07: u8 = 0x07;
    pub const MODE_CONFIG_REG08: u8 = 0x08;
    pub const TIME_CONTROL0_REG09: u8 = 0x09;
    pub const TIME_CONTROL1_REG0A: u8 = 0x0A;
    pub const SDP_INTERFACE1_REG11: u8 = 0x11;
    pub const SDP_INTERFACE2_REG12: u8 = 0x12;
    pub const ADC_AUTOMUTE_REG13: u8 = 0x13;
    pub const ADC34_MUTERANGE_REG14: u8 = 0x14;
    pub const ADC12_MUTERANGE_REG15: u8 = 0x15;
    pub const ADC34_HPF2_REG20: u8 = 0x20;
    pub const ADC34_HPF1_REG21: u8 = 0x21;
    pub const ADC12_HPF1_REG22: u8 = 0x22;
    pub const ADC12_HPF2_REG23: u8 = 0x23;
    pub const ANALOG_REG40: u8 = 0x40;
    pub const MIC12_BIAS_REG41: u8 = 0x41;
    pub const MIC34_BIAS_REG42: u8 = 0x42;
    pub const MIC1_GAIN_REG43: u8 = 0x43;
    pub const MIC2_GAIN_REG44: u8 = 0x44;
    pub const MIC3_GAIN_REG45: u8 = 0x45;
    pub const MIC4_GAIN_REG46: u8 = 0x46;
    pub const MIC1_POWER_REG47: u8 = 0x47;
    pub const MIC2_POWER_REG48: u8 = 0x48;
    pub const MIC3_POWER_REG49: u8 = 0x49;
    pub const MIC4_POWER_REG4A: u8 = 0x4A;
    pub const MIC12_POWER_REG4B: u8 = 0x4B;
    pub const MIC34_POWER_REG4C: u8 = 0x4C;
}

/// Which of the four inputs are on, as a bit set (`MIC1` = bit 0).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mics(pub u8);

impl Mics {
    /// Input 1.
    pub const MIC1: Mics = Mics(0x01);
    /// Input 2.
    pub const MIC2: Mics = Mics(0x02);
    /// Input 3.
    pub const MIC3: Mics = Mics(0x04);
    /// Input 4.
    pub const MIC4: Mics = Mics(0x08);
    /// The vendor driver's default: inputs 1 and 2.
    pub const MIC12: Mics = Mics(0x03);
    /// All four.
    pub const ALL: Mics = Mics(0x0F);

    /// `true` when input `n` (1..=4) is in the set.
    #[must_use]
    pub const fn has(self, n: u8) -> bool {
        n >= 1 && n <= 4 && self.0 & (1 << (n - 1)) != 0
    }

    /// How many inputs are on.
    #[must_use]
    pub const fn count(self) -> u8 {
        (self.0 & 0x0F).count_ones() as u8
    }
}

/// Microphone PGA gain (`MICn_GAIN_REG43..46`, low nibble).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Gain {
    /// 0 dB.
    Db0 = 0,
    /// 3 dB.
    Db3 = 1,
    /// 6 dB.
    Db6 = 2,
    /// 9 dB.
    Db9 = 3,
    /// 12 dB.
    Db12 = 4,
    /// 15 dB.
    Db15 = 5,
    /// 18 dB.
    Db18 = 6,
    /// 21 dB.
    Db21 = 7,
    /// 24 dB (the vendor driver's default).
    Db24 = 8,
    /// 27 dB.
    Db27 = 9,
    /// 30 dB.
    Db30 = 10,
    /// 33 dB.
    Db33 = 11,
    /// 34.5 dB.
    Db34_5 = 12,
    /// 36 dB.
    Db36 = 13,
    /// 37.5 dB.
    Db37_5 = 14,
}

/// One row of the clock-coefficient table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Coeff {
    /// Master clock, Hz.
    pub mclk: u32,
    /// LRCK (sample rate), Hz.
    pub lrck: u32,
    /// Single / double speed.
    pub ss_ds: u8,
    /// ADC clock divider (`MAINCLK_REG02` low bits).
    pub adc_div: u8,
    /// DLL bypass (`MAINCLK_REG02` bit 7).
    pub dll: u8,
    /// Clock doubler (`MAINCLK_REG02` bit 6).
    pub doubler: u8,
    /// Oversampling (`OSR_REG07`).
    pub osr: u8,
    /// MCLK source select.
    pub mclk_src: u8,
    /// LRCK divider, high byte (`LRCK_DIVH_REG04`).
    pub lrck_h: u8,
    /// LRCK divider, low byte (`LRCK_DIVL_REG05`).
    pub lrck_l: u8,
}

/// A table row; the argument list is the vendor table's column order.
#[allow(clippy::too_many_arguments)]
const fn c(
    mclk: u32,
    lrck: u32,
    ss_ds: u8,
    adc_div: u8,
    dll: u8,
    doubler: u8,
    osr: u8,
    mclk_src: u8,
    lrck_h: u8,
    lrck_l: u8,
) -> Coeff {
    Coeff {
        mclk,
        lrck,
        ss_ds,
        adc_div,
        dll,
        doubler,
        osr,
        mclk_src,
        lrck_h,
        lrck_l,
    }
}

/// The vendor driver's `coeff_div[]`, verbatim.
pub const COEFF: &[Coeff] = &[
    // 8k
    c(
        12_288_000, 8000, 0x00, 0x03, 0x01, 0x00, 0x20, 0x00, 0x06, 0x00,
    ),
    c(
        16_384_000, 8000, 0x00, 0x04, 0x01, 0x00, 0x20, 0x00, 0x08, 0x00,
    ),
    c(
        19_200_000, 8000, 0x00, 0x1e, 0x00, 0x01, 0x28, 0x00, 0x09, 0x60,
    ),
    c(
        4_096_000, 8000, 0x00, 0x01, 0x01, 0x00, 0x20, 0x00, 0x02, 0x00,
    ),
    // 11.025k
    c(
        11_289_600, 11025, 0x00, 0x02, 0x01, 0x00, 0x20, 0x00, 0x01, 0x00,
    ),
    // 12k
    c(
        12_288_000, 12000, 0x00, 0x02, 0x01, 0x00, 0x20, 0x00, 0x04, 0x00,
    ),
    c(
        19_200_000, 12000, 0x00, 0x14, 0x00, 0x01, 0x28, 0x00, 0x06, 0x40,
    ),
    // 16k
    c(
        4_096_000, 16000, 0x00, 0x01, 0x01, 0x01, 0x20, 0x00, 0x01, 0x00,
    ),
    c(
        19_200_000, 16000, 0x00, 0x0a, 0x00, 0x00, 0x1e, 0x00, 0x04, 0x80,
    ),
    c(
        16_384_000, 16000, 0x00, 0x02, 0x01, 0x00, 0x20, 0x00, 0x04, 0x00,
    ),
    c(
        12_288_000, 16000, 0x00, 0x03, 0x01, 0x01, 0x20, 0x00, 0x03, 0x00,
    ),
    // 22.05k
    c(
        11_289_600, 22050, 0x00, 0x01, 0x01, 0x00, 0x20, 0x00, 0x02, 0x00,
    ),
    // 24k
    c(
        12_288_000, 24000, 0x00, 0x01, 0x01, 0x00, 0x20, 0x00, 0x02, 0x00,
    ),
    c(
        19_200_000, 24000, 0x00, 0x0a, 0x00, 0x01, 0x28, 0x00, 0x03, 0x20,
    ),
    // 32k
    c(
        12_288_000, 32000, 0x00, 0x03, 0x00, 0x00, 0x20, 0x00, 0x01, 0x80,
    ),
    c(
        16_384_000, 32000, 0x00, 0x01, 0x01, 0x00, 0x20, 0x00, 0x02, 0x00,
    ),
    c(
        19_200_000, 32000, 0x00, 0x05, 0x00, 0x00, 0x1e, 0x00, 0x02, 0x58,
    ),
    // 44.1k
    c(
        11_289_600, 44100, 0x00, 0x01, 0x01, 0x01, 0x20, 0x00, 0x01, 0x00,
    ),
    // 48k
    c(
        12_288_000, 48000, 0x00, 0x01, 0x01, 0x01, 0x20, 0x00, 0x01, 0x00,
    ),
    c(
        19_200_000, 48000, 0x00, 0x05, 0x00, 0x01, 0x28, 0x00, 0x01, 0x90,
    ),
    // 64k
    c(
        16_384_000, 64000, 0x01, 0x01, 0x01, 0x00, 0x20, 0x00, 0x01, 0x00,
    ),
    c(
        19_200_000, 64000, 0x00, 0x05, 0x00, 0x01, 0x1e, 0x00, 0x01, 0x2c,
    ),
    // 88.2k
    c(
        11_289_600, 88200, 0x01, 0x01, 0x01, 0x01, 0x20, 0x00, 0x00, 0x80,
    ),
    // 96k
    c(
        12_288_000, 96000, 0x01, 0x01, 0x01, 0x01, 0x20, 0x00, 0x00, 0x80,
    ),
    c(
        19_200_000, 96000, 0x01, 0x05, 0x00, 0x01, 0x28, 0x00, 0x00, 0xc8,
    ),
];

/// The coefficient row for a (MCLK, rate) pair, if the chip supports it.
#[must_use]
pub fn coeff(mclk_hz: u32, rate_hz: u32) -> Option<&'static Coeff> {
    COEFF
        .iter()
        .find(|c| c.mclk == mclk_hz && c.lrck == rate_hz)
}

/// The vendor driver's default master clock for a rate: 256 × Fs.
#[must_use]
pub const fn mclk_for(rate_hz: u32) -> u32 {
    rate_hz * 256
}

/// In master mode, where the chip's MCLK comes from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MclkSource {
    /// The MCLK pad.
    Pad,
    /// The internal clock doubler (the vendor driver's default).
    Doubler,
}

/// Bring-up parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Who drives BCLK / LRCK.
    pub mode: Mode,
    /// MCLK source in master mode.
    pub mclk: MclkSource,
    /// Inputs to enable.
    pub mics: Mics,
    /// PGA gain for the enabled inputs.
    pub gain: Gain,
    /// Sample rate, Hz.
    pub sample_rate_hz: u32,
    /// Master clock, Hz.
    pub mclk_hz: u32,
    /// Switch the serial port to TDM when at least this many inputs are on
    /// (the vendor driver's `ENABLE_TDM_MAX_NUM`, 3).
    pub tdm_from_mics: u8,
    /// Use the vendor's alternative DSP framing byte (`0x13` instead of `0x03`).
    pub dsp_mode_alt: bool,
}

impl Config {
    /// The vendor driver's defaults: slave, doubler, inputs 1 + 2 at 24 dB,
    /// TDM from three inputs, 256 × Fs.
    #[must_use]
    pub const fn slave(sample_rate_hz: u32) -> Self {
        Config {
            mode: Mode::Slave,
            mclk: MclkSource::Doubler,
            mics: Mics::MIC12,
            gain: Gain::Db24,
            sample_rate_hz,
            mclk_hz: mclk_for(sample_rate_hz),
            tdm_from_mics: 3,
            dsp_mode_alt: false,
        }
    }
}

/// The driver.
#[derive(Debug)]
pub struct Es7210<I> {
    regs: I2cRegs<I>,
    mics: Mics,
    gain: Gain,
    tdm_from_mics: u8,
    dsp_mode_alt: bool,
    /// `CLOCK_OFF_REG01` as it was before the last stop, restored by start.
    clock_reg: u8,
}

impl<I: I2c> Es7210<I> {
    /// Over `i2c` at the chip's 7-bit `addr` ([`ADDR_00`] on the reference boards).
    pub fn new(i2c: I, addr: u8) -> Self {
        Es7210 {
            regs: I2cRegs::new(i2c, addr),
            mics: Mics::MIC12,
            gain: Gain::Db24,
            tdm_from_mics: 3,
            dsp_mode_alt: false,
            clock_reg: 0x3F,
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

    /// `es7210_adc_init`: reset, clocks on, state-machine timing, the HPF
    /// quick setup, master/slave and MCLK source, analog power and mic bias,
    /// OSR, the main clock, then the rate, the inputs and their gain.
    pub fn init(&mut self, cfg: &Config) -> Result<()> {
        self.tdm_from_mics = cfg.tdm_from_mics;
        self.dsp_mode_alt = cfg.dsp_mode_alt;
        self.gain = cfg.gain;
        let r = &mut self.regs;
        r.write(reg::RESET_REG00, 0xFF)?;
        r.write(reg::RESET_REG00, 0x41)?;
        r.write(reg::CLOCK_OFF_REG01, 0x3F)?;
        r.write(reg::TIME_CONTROL0_REG09, 0x30)?;
        r.write(reg::TIME_CONTROL1_REG0A, 0x30)?;
        r.write(reg::ADC12_HPF2_REG23, 0x2A)?;
        r.write(reg::ADC12_HPF1_REG22, 0x0A)?;
        r.write(reg::ADC34_HPF2_REG20, 0x0A)?;
        r.write(reg::ADC34_HPF1_REG21, 0x2A)?;
        match cfg.mode {
            Mode::Master => {
                r.update(reg::MODE_CONFIG_REG08, 0x01, 0x01)?;
                let src = match cfg.mclk {
                    MclkSource::Pad => 0x00,
                    MclkSource::Doubler => 0x80,
                };
                r.update(reg::MASTER_CLK_REG03, 0x80, src)?;
            }
            Mode::Slave => r.update(reg::MODE_CONFIG_REG08, 0x01, 0x00)?,
        }
        // analog on, VDDA 3.3 V, VMID 5 kΩ start; mic bias 2.87 V
        r.write(reg::ANALOG_REG40, 0x43)?;
        r.write(reg::MIC12_BIAS_REG41, 0x70)?;
        r.write(reg::MIC34_BIAS_REG42, 0x70)?;
        r.write(reg::OSR_REG07, 0x20)?;
        r.write(reg::MAINCLK_REG02, 0xC1)?;
        self.set_sample_rate(cfg.sample_rate_hz, cfg.mclk_hz)?;
        self.select_mics(cfg.mics)?;
        self.set_gain(cfg.mics, cfg.gain)
    }

    /// `es7210_mic_select`: power the chosen inputs, their PGAs and the
    /// ADC pairs they sit on, and pick TDM once enough are on.
    pub fn select_mics(&mut self, mics: Mics) -> Result<()> {
        if mics.0 & 0x0F == 0 {
            return Err(Error::Unsupported);
        }
        self.mics = mics;
        let gain = self.gain as u8;
        let r = &mut self.regs;
        for i in 0..4 {
            r.update(reg::MIC1_GAIN_REG43 + i, 0x10, 0x00)?;
        }
        r.write(reg::MIC12_POWER_REG4B, 0xFF)?;
        r.write(reg::MIC34_POWER_REG4C, 0xFF)?;
        if mics.has(1) {
            r.update(reg::CLOCK_OFF_REG01, 0x0B, 0x00)?;
            r.write(reg::MIC12_POWER_REG4B, 0x00)?;
            r.update(reg::MIC1_GAIN_REG43, 0x10, 0x10)?;
            r.update(reg::MIC1_GAIN_REG43, 0x0F, gain)?;
        }
        if mics.has(2) {
            r.update(reg::CLOCK_OFF_REG01, 0x0B, 0x00)?;
            r.write(reg::MIC12_POWER_REG4B, 0x00)?;
            r.update(reg::MIC2_GAIN_REG44, 0x10, 0x10)?;
            r.update(reg::MIC2_GAIN_REG44, 0x0F, gain)?;
        }
        if mics.has(3) {
            r.update(reg::CLOCK_OFF_REG01, 0x15, 0x00)?;
            r.write(reg::MIC34_POWER_REG4C, 0x00)?;
            r.update(reg::MIC3_GAIN_REG45, 0x10, 0x10)?;
            r.update(reg::MIC3_GAIN_REG45, 0x0F, gain)?;
        }
        if mics.has(4) {
            r.update(reg::CLOCK_OFF_REG01, 0x15, 0x00)?;
            r.write(reg::MIC34_POWER_REG4C, 0x00)?;
            r.update(reg::MIC4_GAIN_REG46, 0x10, 0x10)?;
            r.update(reg::MIC4_GAIN_REG46, 0x0F, gain)?;
        }
        let tdm = mics.count() >= self.tdm_from_mics;
        r.write(reg::SDP_INTERFACE2_REG12, if tdm { 0x02 } else { 0x00 })
    }

    /// `es7210_adc_set_gain` for the inputs in `mics`.
    pub fn set_gain(&mut self, mics: Mics, gain: Gain) -> Result<()> {
        self.gain = gain;
        let r = &mut self.regs;
        for n in 1..=4u8 {
            if mics.has(n) {
                r.update(reg::MIC1_GAIN_REG43 + (n - 1), 0x0F, gain as u8)?;
            }
        }
        Ok(())
    }

    /// The inputs currently selected.
    #[must_use]
    pub fn mics(&self) -> Mics {
        self.mics
    }
}

impl<I: I2c> CodecChip for Es7210<I> {
    /// `es7210_config_sample`: the row's dividers into `MAINCLK_REG02`, OSR,
    /// and the LRCK divider pair.
    fn set_sample_rate(&mut self, sample_rate_hz: u32, mclk_hz: u32) -> Result<()> {
        let c = coeff(mclk_hz, sample_rate_hz).ok_or(Error::Unsupported)?;
        let r = &mut self.regs;
        let regv = c.adc_div | (c.doubler << 6) | (c.dll << 7);
        r.write(reg::MAINCLK_REG02, regv)?;
        r.write(reg::OSR_REG07, c.osr)?;
        r.write(reg::LRCK_DIVH_REG04, c.lrck_h)?;
        r.write(reg::LRCK_DIVL_REG05, c.lrck_l)
    }

    /// `es7210_config_fmt` then `es7210_set_bits`, both in `SDP_INTERFACE1_REG11`.
    fn set_format(&mut self, fmt: I2sFormat, bits: Bits) -> Result<()> {
        let r = &mut self.regs;
        let mut iface = r.read(reg::SDP_INTERFACE1_REG11)? & 0xFC;
        iface |= match fmt {
            I2sFormat::Standard => 0x00,
            I2sFormat::LeftJustified => 0x01,
            I2sFormat::Dsp => {
                if self.dsp_mode_alt {
                    0x13
                } else {
                    0x03
                }
            }
        };
        r.write(reg::SDP_INTERFACE1_REG11, iface)?;
        let mut iface = r.read(reg::SDP_INTERFACE1_REG11)? & 0x1F;
        iface |= match bits {
            Bits::B16 => 0x60,
            Bits::B24 => 0x00,
            Bits::B32 => 0x80,
        };
        r.write(reg::SDP_INTERFACE1_REG11, iface)
    }

    /// `es7210_start`: clocks back as they were before the stop, power up,
    /// analog on, mic powers, then the input selection again. The ES7210 has
    /// no DAC: [`Module::Dac`] is refused.
    fn start(&mut self, module: Module) -> Result<()> {
        if module == Module::Dac {
            return Err(Error::Unsupported);
        }
        let live = self.regs.read(reg::CLOCK_OFF_REG01)?;
        if live != 0x7F && live != 0xFF {
            self.clock_reg = live;
        }
        let clock = self.clock_reg;
        let r = &mut self.regs;
        r.write(reg::CLOCK_OFF_REG01, clock)?;
        r.write(reg::POWER_DOWN_REG06, 0x00)?;
        r.write(reg::ANALOG_REG40, 0x43)?;
        r.write(reg::MIC1_POWER_REG47, 0x08)?;
        r.write(reg::MIC2_POWER_REG48, 0x08)?;
        r.write(reg::MIC3_POWER_REG49, 0x08)?;
        r.write(reg::MIC4_POWER_REG4A, 0x08)?;
        let mics = self.mics;
        self.select_mics(mics)
    }

    /// `es7210_stop`: every mic and ADC pair off, analog off, clocks off,
    /// power down — remembering the clock register for the next start.
    fn stop(&mut self) -> Result<()> {
        let live = self.regs.read(reg::CLOCK_OFF_REG01)?;
        if live != 0x7F && live != 0xFF {
            self.clock_reg = live;
        }
        let r = &mut self.regs;
        r.write(reg::MIC1_POWER_REG47, 0xFF)?;
        r.write(reg::MIC2_POWER_REG48, 0xFF)?;
        r.write(reg::MIC3_POWER_REG49, 0xFF)?;
        r.write(reg::MIC4_POWER_REG4A, 0xFF)?;
        r.write(reg::MIC12_POWER_REG4B, 0xFF)?;
        r.write(reg::MIC34_POWER_REG4C, 0xFF)?;
        r.write(reg::ANALOG_REG40, 0xC0)?;
        r.write(reg::CLOCK_OFF_REG01, 0x7F)?;
        r.write(reg::POWER_DOWN_REG06, 0x07)
    }

    /// `es7210_set_mute`: the mute-range bits of both ADC pairs.
    fn set_mute(&mut self, mute: bool) -> Result<()> {
        let v = if mute { 0x03 } else { 0x00 };
        self.regs.update(reg::ADC34_MUTERANGE_REG14, 0x03, v)?;
        self.regs.update(reg::ADC12_MUTERANGE_REG15, 0x03, v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chip::fake::FakeCodec;

    #[test]
    fn coefficient_table_and_mic_sets() {
        assert_eq!(COEFF.len(), 25);
        assert!(coeff(4_096_000, 16000).is_some());
        assert!(coeff(19_200_000, 96000).is_some());
        assert!(coeff(4_096_000, 48000).is_none());
        assert_eq!(Mics::MIC12.count(), 2);
        assert!(Mics::MIC12.has(1) && Mics::MIC12.has(2) && !Mics::MIC12.has(3));
        assert!(!Mics::ALL.has(0) && !Mics::ALL.has(5));
        assert_eq!(Mics::ALL.count(), 4);
    }

    /// The vendor bring-up, register for register, in the default slave
    /// configuration at 16 kHz / 4.096 MHz with inputs 1 + 2 at 24 dB.
    #[test]
    fn init_reproduces_the_vendor_sequence() {
        let mut chip = Es7210::new(FakeCodec::default(), ADDR_00);
        chip.init(&Config::slave(16_000)).unwrap();
        let bus = chip.release();
        assert_eq!(bus.addr_seen, Some(0x40));
        let head: &[(u8, u8)] = &[
            (0x00, 0xFF),
            (0x00, 0x41),
            (0x01, 0x3F),
            (0x09, 0x30),
            (0x0A, 0x30),
            (0x23, 0x2A),
            (0x22, 0x0A),
            (0x20, 0x0A),
            (0x21, 0x2A),
            (0x08, 0x00), // slave
            (0x40, 0x43),
            (0x41, 0x70),
            (0x42, 0x70),
            (0x07, 0x20),
            (0x02, 0xC1),
            // 16 kHz / 4.096 MHz: adc_div 1, doubler, dll bypass -> 0xC1; osr 0x20; lrck 0x0100
            (0x02, 0xC1),
            (0x07, 0x20),
            (0x04, 0x01),
            (0x05, 0x00),
        ];
        assert_eq!(&bus.writes[..head.len()], head);
        // after mic select + gain: inputs 1 and 2 powered, PGA on, 24 dB; TDM off
        assert_eq!(bus.reg(0x4B), 0x00, "ADC12 pair powered");
        assert_eq!(bus.reg(0x4C), 0xFF, "ADC34 pair off");
        assert_eq!(bus.reg(0x43), 0x18, "mic 1: PGA on, 24 dB");
        assert_eq!(bus.reg(0x44), 0x18);
        assert_eq!(bus.reg(0x45) & 0x10, 0x00, "mic 3 PGA off");
        assert_eq!(bus.reg(0x01), 0x3F & !0x0B, "ADC12 clocks on");
        assert_eq!(bus.reg(0x12), 0x00, "two mics: no TDM");
    }

    #[test]
    fn four_mics_in_master_mode_turn_on_tdm_and_the_doubler() {
        let mut chip = Es7210::new(FakeCodec::default(), ADDR_11);
        chip.init(&Config {
            mode: Mode::Master,
            mclk: MclkSource::Doubler,
            mics: Mics::ALL,
            gain: Gain::Db30,
            sample_rate_hz: 48_000,
            mclk_hz: mclk_for(48_000),
            tdm_from_mics: 3,
            dsp_mode_alt: false,
        })
        .unwrap();
        let bus = chip.release();
        assert_eq!(bus.addr_seen, Some(0x43));
        assert_eq!(bus.reg(0x08) & 0x01, 0x01, "master");
        assert_eq!(bus.reg(0x03) & 0x80, 0x80, "MCLK from the doubler");
        assert_eq!(bus.reg(0x12), 0x02, "four mics: TDM");
        for r in 0x43..=0x46u8 {
            assert_eq!(bus.reg(r), 0x10 | 10, "every PGA on at 30 dB");
        }
        assert_eq!(bus.reg(0x01), 0x3F & !0x0B & !0x15);
        assert_eq!(bus.reg(0x4C), 0x00);
    }

    #[test]
    fn format_start_stop_and_mute() {
        let mut chip = Es7210::new(FakeCodec::default(), ADDR_00);
        chip.init(&Config::slave(16_000)).unwrap();
        chip.set_format(I2sFormat::Standard, Bits::B16).unwrap();
        assert_eq!(chip.regs().bus().reg(0x11), 0x60);
        chip.set_format(I2sFormat::Dsp, Bits::B32).unwrap();
        assert_eq!(chip.regs().bus().reg(0x11), 0x83);
        assert_eq!(chip.start(Module::Dac), Err(Error::Unsupported));
        chip.start(Module::Adc).unwrap();
        {
            let bus = chip.regs().bus();
            assert_eq!(bus.reg(0x06), 0x00);
            assert_eq!(bus.reg(0x47), 0x08);
            assert_eq!(bus.reg(0x4A), 0x08);
        }
        let clock_before = chip.regs().bus().reg(0x01);
        chip.stop().unwrap();
        {
            let bus = chip.regs().bus();
            assert_eq!(bus.reg(0x01), 0x7F);
            assert_eq!(bus.reg(0x06), 0x07);
            assert_eq!(bus.reg(0x40), 0xC0);
            assert_eq!(bus.reg(0x47), 0xFF);
        }
        chip.start(Module::AdcDac).unwrap();
        assert_eq!(
            chip.regs().bus().history(0x01).last().copied(),
            Some(clock_before),
            "start restores the clock register the stop replaced with 0x7F"
        );
        chip.set_mute(true).unwrap();
        assert_eq!(chip.regs().bus().reg(0x14) & 0x03, 0x03);
        assert_eq!(chip.regs().bus().reg(0x15) & 0x03, 0x03);
        chip.set_mute(false).unwrap();
        assert_eq!(chip.regs().bus().reg(0x15) & 0x03, 0x00);
        assert_eq!(chip.select_mics(Mics(0)), Err(Error::Unsupported));
    }
}
