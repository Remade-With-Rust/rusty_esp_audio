//! ES8156 — Everest's mono DAC with a headphone/line driver, the playback
//! half of the ESP32-S3-BOX-Lite (its capture half is the [`super::es7243e`]).
//!
//! Register map and every sequence are re-derived from Espressif's `esp-adf`
//! `components/audio_hal/driver/es8156/{es8156.h, es8156.c}` (ESPRESSIF MIT
//! License, © 2019 Espressif Systems (Shanghai) Co., Ltd): the same registers
//! in the same order with the same values, as data. The vendor driver never
//! reconfigures the serial port or the clocks after bring-up — in slave mode
//! the part follows the incoming LRCK — so [`CodecChip::set_sample_rate`] is
//! a no-op here and [`CodecChip::set_format`] accepts only the framing the
//! bring-up leaves (I²S, 16-bit) and refuses the rest rather than guess the
//! `DAC_SDP` bit layout.
//!
//! Volume is the datasheet's, shared with the ES8311: `0xBF` is 0 dB, one
//! step 0.5 dB, `0x00` −95.5 dB, `0xFF` +32 dB; the bring-up leaves `0xB3`
//! (−6 dB, the vendor's "70 %").

use embedded_hal::i2c::I2c;
use rusty_esp_core::error::{Error, Result};

use super::{Bits, CodecChip, I2cRegs, I2sFormat, Module};

/// 7-bit I²C address (the vendor driver's 8-bit `0x10`).
pub const ADDR: u8 = 0x08;

/// The volume register for a level in half-decibels: the ES8311's map.
pub use super::es8311::dac_volume_reg as volume_reg;

/// The register map (`es8156.h`).
#[allow(missing_docs)]
pub mod reg {
    pub const RESET_REG00: u8 = 0x00;
    pub const MAINCLOCK_CTL_REG01: u8 = 0x01;
    pub const SCLK_MODE_REG02: u8 = 0x02;
    pub const LRCLK_DIV_H_REG03: u8 = 0x03;
    pub const LRCLK_DIV_L_REG04: u8 = 0x04;
    pub const SCLK_DIV_REG05: u8 = 0x05;
    pub const NFS_CONFIG_REG06: u8 = 0x06;
    pub const MISC_CONTROL1_REG07: u8 = 0x07;
    pub const CLOCK_ON_OFF_REG08: u8 = 0x08;
    pub const MISC_CONTROL2_REG09: u8 = 0x09;
    pub const TIME_CONTROL1_REG0A: u8 = 0x0A;
    pub const TIME_CONTROL2_REG0B: u8 = 0x0B;
    pub const CHIP_STATUS_REG0C: u8 = 0x0C;
    pub const P2S_CONTROL_REG0D: u8 = 0x0D;
    pub const DAC_OSR_COUNTER_REG10: u8 = 0x10;
    pub const DAC_SDP_REG11: u8 = 0x11;
    pub const AUTOMUTE_SET_REG12: u8 = 0x12;
    pub const DAC_MUTE_REG13: u8 = 0x13;
    pub const VOLUME_CONTROL_REG14: u8 = 0x14;
    pub const ALC_CONFIG1_REG15: u8 = 0x15;
    pub const ALC_CONFIG2_REG16: u8 = 0x16;
    pub const ALC_CONFIG3_REG17: u8 = 0x17;
    pub const MISC_CONTROL3_REG18: u8 = 0x18;
    pub const EQ_CONTROL1_REG19: u8 = 0x19;
    pub const EQ_CONTROL2_REG1A: u8 = 0x1A;
    pub const ANALOG_SYS1_REG20: u8 = 0x20;
    pub const ANALOG_SYS2_REG21: u8 = 0x21;
    pub const ANALOG_SYS3_REG22: u8 = 0x22;
    pub const ANALOG_SYS4_REG23: u8 = 0x23;
    pub const ANALOG_LP_REG24: u8 = 0x24;
    pub const ANALOG_SYS5_REG25: u8 = 0x25;
    pub const I2C_PAGESEL_REGFC: u8 = 0xFC;
    pub const CHIPID1_REGFD: u8 = 0xFD;
    pub const CHIPID0_REGFE: u8 = 0xFE;
    pub const CHIP_VERSION_REGFF: u8 = 0xFF;
}

/// The vendor's `es8156_codec_init`, register for register.
pub const INIT: &[(u8, u8)] = &[
    (reg::SCLK_MODE_REG02, 0x04),
    (reg::ANALOG_SYS1_REG20, 0x2A),
    (reg::ANALOG_SYS2_REG21, 0x3C),
    (reg::ANALOG_SYS3_REG22, 0x00),
    (reg::ANALOG_LP_REG24, 0x07),
    (reg::ANALOG_SYS4_REG23, 0x00),
    (reg::TIME_CONTROL1_REG0A, 0x01),
    (reg::TIME_CONTROL2_REG0B, 0x01),
    (reg::DAC_SDP_REG11, 0x00),
    (reg::VOLUME_CONTROL_REG14, 0xB3), // 179: -6 dB, the vendor's "70 %"
    (reg::P2S_CONTROL_REG0D, 0x14),
    (reg::MISC_CONTROL3_REG18, 0x00),
    (reg::CLOCK_ON_OFF_REG08, 0x3F),
    (reg::RESET_REG00, 0x02),
    (reg::RESET_REG00, 0x03),
    (reg::ANALOG_SYS5_REG25, 0x20),
];

/// The vendor's `es8156_resume`: clocks on, analog up, volume back.
pub const RESUME: &[(u8, u8)] = &[
    (reg::CLOCK_ON_OFF_REG08, 0x3F),
    (reg::MISC_CONTROL2_REG09, 0x00),
    (reg::MISC_CONTROL3_REG18, 0x00),
    (reg::ANALOG_SYS5_REG25, 0x20),
    (reg::ANALOG_SYS3_REG22, 0x00),
    (reg::ANALOG_SYS2_REG21, 0x3C),
    (reg::EQ_CONTROL1_REG19, 0x20),
    (reg::VOLUME_CONTROL_REG14, 0xB3),
];

/// The vendor's `es8156_standby`: volume to minimum, analog down, clocks off.
pub const STANDBY: &[(u8, u8)] = &[
    (reg::VOLUME_CONTROL_REG14, 0x00),
    (reg::EQ_CONTROL1_REG19, 0x02),
    (reg::ANALOG_SYS2_REG21, 0x1F),
    (reg::ANALOG_SYS3_REG22, 0x02),
    (reg::ANALOG_SYS5_REG25, 0x21),
    (reg::ANALOG_SYS5_REG25, 0xA1),
    (reg::MISC_CONTROL3_REG18, 0x01),
    (reg::MISC_CONTROL2_REG09, 0x02),
    (reg::MISC_CONTROL2_REG09, 0x01),
    (reg::CLOCK_ON_OFF_REG08, 0x00),
];

/// The driver.
#[derive(Debug)]
pub struct Es8156<I> {
    regs: I2cRegs<I>,
}

impl<I: I2c> Es8156<I> {
    /// Over `i2c` at the chip's 7-bit `addr` (normally [`ADDR`]).
    pub fn new(i2c: I, addr: u8) -> Self {
        Es8156 {
            regs: I2cRegs::new(i2c, addr),
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

    /// Read the chip id bytes `CHIPID1_REGFD` / `CHIPID0_REGFE`.
    pub fn chip_id(&mut self) -> Result<(u8, u8)> {
        Ok((
            self.regs.read(reg::CHIPID1_REGFD)?,
            self.regs.read(reg::CHIPID0_REGFE)?,
        ))
    }

    /// The vendor bring-up ([`INIT`]).
    pub fn init(&mut self) -> Result<()> {
        self.write_all(INIT)
    }

    /// DAC volume register, straight ([`volume_reg`] maps half-dB to it).
    pub fn set_volume_reg(&mut self, value: u8) -> Result<()> {
        self.regs.write(reg::VOLUME_CONTROL_REG14, value)
    }

    fn write_all(&mut self, table: &[(u8, u8)]) -> Result<()> {
        for &(r, v) in table {
            self.regs.write(r, v)?;
        }
        Ok(())
    }
}

impl<I: I2c> CodecChip for Es8156<I> {
    /// A slave ES8156 follows the LRCK it is given; the vendor driver writes
    /// nothing here and neither does this.
    fn set_sample_rate(&mut self, _sample_rate_hz: u32, _mclk_hz: u32) -> Result<()> {
        Ok(())
    }

    /// Only the framing the bring-up leaves — I²S, 16-bit slots — is
    /// accepted (`DAC_SDP_REG11 = 0x00`); anything else is `Unsupported`
    /// until the register's layout is taken from the datasheet, not guessed.
    fn set_format(&mut self, fmt: I2sFormat, bits: Bits) -> Result<()> {
        match (fmt, bits) {
            (I2sFormat::Standard, Bits::B16) => self.regs.write(reg::DAC_SDP_REG11, 0x00),
            _ => Err(Error::Unsupported),
        }
    }

    /// `es8156_resume`; the part has no ADC, so [`Module::Adc`] is refused.
    fn start(&mut self, module: Module) -> Result<()> {
        if module == Module::Adc {
            return Err(Error::Unsupported);
        }
        self.write_all(RESUME)
    }

    /// `es8156_standby`.
    fn stop(&mut self) -> Result<()> {
        self.write_all(STANDBY)
    }

    /// `DAC_MUTE_REG13` bits 1 and 2.
    fn set_mute(&mut self, mute: bool) -> Result<()> {
        let v = self.regs.read(reg::DAC_MUTE_REG13)?;
        let v = if mute { v | 0x06 } else { v & !0x06 };
        self.regs.write(reg::DAC_MUTE_REG13, v)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chip::fake::FakeCodec;

    #[test]
    fn init_resume_standby_are_the_vendor_tables_in_order() {
        let mut chip = Es8156::new(FakeCodec::default(), ADDR);
        chip.init().unwrap();
        chip.start(Module::Dac).unwrap();
        chip.stop().unwrap();
        let bus = chip.release();
        assert_eq!(bus.addr_seen, Some(0x08));
        let expected: Vec<(u8, u8)> = INIT.iter().chain(RESUME).chain(STANDBY).copied().collect();
        assert_eq!(bus.writes, expected);
        assert_eq!(INIT.len(), 16);
        assert_eq!(bus.reg(0x14), 0x00, "standby leaves the volume at minimum");
    }

    #[test]
    fn mute_volume_format_and_the_refusals() {
        let mut chip = Es8156::new(
            FakeCodec::with_defaults(&[(0xFD, 0x81), (0xFE, 0x56)]),
            ADDR,
        );
        chip.init().unwrap();
        chip.set_mute(true).unwrap();
        assert_eq!(chip.regs().bus().reg(0x13) & 0x06, 0x06);
        chip.set_mute(false).unwrap();
        assert_eq!(chip.regs().bus().reg(0x13) & 0x06, 0x00);
        chip.set_volume_reg(volume_reg(0)).unwrap();
        assert_eq!(chip.regs().bus().reg(0x14), 0xBF);
        chip.set_volume_reg(volume_reg(-12)).unwrap();
        assert_eq!(
            chip.regs().bus().reg(0x14),
            0xB3,
            "-6 dB is what the bring-up left"
        );
        chip.set_format(I2sFormat::Standard, Bits::B16).unwrap();
        let writes = chip.regs().bus().writes.len();
        assert_eq!(
            chip.set_format(I2sFormat::Dsp, Bits::B32),
            Err(Error::Unsupported)
        );
        assert_eq!(
            chip.set_format(I2sFormat::Standard, Bits::B24),
            Err(Error::Unsupported)
        );
        assert_eq!(chip.start(Module::Adc), Err(Error::Unsupported));
        assert_eq!(
            chip.regs().bus().writes.len(),
            writes,
            "refusals write nothing"
        );
        chip.set_sample_rate(48_000, 12_288_000).unwrap();
        assert_eq!(
            chip.regs().bus().writes.len(),
            writes,
            "a slave follows LRCK; nothing written"
        );
        assert_eq!(chip.chip_id().unwrap(), (0x81, 0x56));
    }
}
