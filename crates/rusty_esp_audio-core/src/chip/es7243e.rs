//! ES7243E — Everest's stereo ADC, the capture half of the ESP32-S3-BOX-Lite
//! (its playback half is the [`super::es8156`]).
//!
//! Every sequence is re-derived from Espressif's `esp-adf`
//! `components/audio_hal/driver/es7243e/es7243e.c` (ESPRESSIF MIT License,
//! © 2019 Espressif Systems (Shanghai) Co., Ltd), which addresses the part by
//! raw register number and never names them; this module keeps the numbers
//! and the comments the driver had. Three tables — bring-up, start, stop —
//! are the whole contract: the vendor driver configures no format, no rate
//! and no volume through it (the bring-up sets SCLK = MCLK/4 and LRCK =
//! MCLK/256 for master mode and the part follows the bus as a slave), so
//! those trait methods accept only what the tables leave and refuse the rest.

use embedded_hal::i2c::I2c;
use rusty_esp_core::error::{Error, Result};

use super::{Bits, CodecChip, I2cRegs, I2sFormat, Module};

/// 7-bit I²C address (the vendor driver's 8-bit `0x20`); the AD pins move it.
pub const ADDR: u8 = 0x10;

/// PGA gain register of channel 1 (`0x20`) — the value is the part's code:
/// `0x10` after [`START`], `0x1A` (+30 dB by the vendor's comment) after
/// [`INIT`].
pub const REG_PGA1: u8 = 0x20;
/// PGA gain register of channel 2.
pub const REG_PGA2: u8 = 0x21;

/// The vendor's `es7243e_adc_init`, register for register, comments kept.
pub const INIT: &[(u8, u8)] = &[
    (0x01, 0x3A),
    (0x00, 0x80),
    (0xF9, 0x00),
    (0x04, 0x02),
    (0x04, 0x01),
    (0xF9, 0x01),
    (0x00, 0x1E),
    (0x01, 0x00),
    (0x02, 0x00),
    (0x03, 0x20),
    (0x04, 0x01),
    (0x0D, 0x00),
    (0x05, 0x00),
    (0x06, 0x03), // SCLK = MCLK/4
    (0x07, 0x00), // LRCK = MCLK/256
    (0x08, 0xFF), // LRCK = MCLK/256
    (0x09, 0xCA),
    (0x0A, 0x85),
    (0x0B, 0x00),
    (0x0E, 0xBF),
    (0x0F, 0x80),
    (0x14, 0x0C),
    (0x15, 0x0C),
    (0x17, 0x02),
    (0x18, 0x26),
    (0x19, 0x77),
    (0x1A, 0xF4),
    (0x1B, 0x66),
    (0x1C, 0x44),
    (0x1E, 0x00),
    (0x1F, 0x0C),
    (REG_PGA1, 0x1A), // PGA gain +30 dB
    (REG_PGA2, 0x1A), // PGA gain +30 dB
    (0x00, 0x80),     // slave mode
    (0x01, 0x3A),
    (0x16, 0x3F),
    (0x16, 0x00),
];

/// The vendor's start: analog up, PGAs to the running code, clocks on.
pub const START: &[(u8, u8)] = &[
    (0xF9, 0x00),
    (0x04, 0x01),
    (0x17, 0x01),
    (REG_PGA1, 0x10),
    (REG_PGA2, 0x10),
    (0x00, 0x80),
    (0x01, 0x3A),
    (0x16, 0x3F),
    (0x16, 0x00),
];

/// The vendor's stop: mute, PGAs off, analog and clocks down.
pub const STOP: &[(u8, u8)] = &[
    (0x04, 0x02),
    (0x04, 0x01),
    (0xF7, 0x30),
    (0xF9, 0x01),
    (0x16, 0xFF),
    (0x17, 0x00),
    (0x01, 0x38),
    (REG_PGA1, 0x00),
    (REG_PGA2, 0x00),
    (0x00, 0x00),
    (0x00, 0x1E),
    (0x01, 0x30),
    (0x01, 0x00),
];

/// The driver.
#[derive(Debug)]
pub struct Es7243e<I> {
    regs: I2cRegs<I>,
}

impl<I: I2c> Es7243e<I> {
    /// Over `i2c` at the chip's 7-bit `addr` (normally [`ADDR`]).
    pub fn new(i2c: I, addr: u8) -> Self {
        Es7243e {
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

    /// The vendor bring-up ([`INIT`]).
    pub fn init(&mut self) -> Result<()> {
        self.write_all(INIT)
    }

    /// Both PGA gain registers to the part's `code` (the vendor uses `0x10`
    /// running and `0x1A` for +30 dB; the code-to-dB table is the
    /// datasheet's, not guessed here).
    pub fn set_pga_code(&mut self, code: u8) -> Result<()> {
        self.regs.write(REG_PGA1, code)?;
        self.regs.write(REG_PGA2, code)
    }

    fn write_all(&mut self, table: &[(u8, u8)]) -> Result<()> {
        for &(r, v) in table {
            self.regs.write(r, v)?;
        }
        Ok(())
    }
}

impl<I: I2c> CodecChip for Es7243e<I> {
    /// A slave ES7243E follows the bus; the vendor driver writes nothing
    /// here and neither does this.
    fn set_sample_rate(&mut self, _sample_rate_hz: u32, _mclk_hz: u32) -> Result<()> {
        Ok(())
    }

    /// Only the framing the bring-up leaves (I²S, 16-bit) is accepted, and
    /// it is already in place, so nothing is written; anything else is
    /// `Unsupported`.
    fn set_format(&mut self, fmt: I2sFormat, bits: Bits) -> Result<()> {
        match (fmt, bits) {
            (I2sFormat::Standard, Bits::B16) => Ok(()),
            _ => Err(Error::Unsupported),
        }
    }

    /// [`START`]; the part has no DAC, so [`Module::Dac`] is refused.
    fn start(&mut self, module: Module) -> Result<()> {
        if module == Module::Dac {
            return Err(Error::Unsupported);
        }
        self.write_all(START)
    }

    /// [`STOP`].
    fn stop(&mut self) -> Result<()> {
        self.write_all(STOP)
    }

    /// The vendor driver has no mute for this part (its stop sequence is
    /// the mute); `Unsupported`.
    fn set_mute(&mut self, _mute: bool) -> Result<()> {
        Err(Error::Unsupported)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chip::fake::FakeCodec;

    #[test]
    fn init_start_stop_are_the_vendor_tables_in_order() {
        let mut chip = Es7243e::new(FakeCodec::default(), ADDR);
        chip.init().unwrap();
        chip.start(Module::Adc).unwrap();
        chip.stop().unwrap();
        let bus = chip.release();
        assert_eq!(bus.addr_seen, Some(0x10));
        let expected: Vec<(u8, u8)> = INIT.iter().chain(START).chain(STOP).copied().collect();
        assert_eq!(bus.writes, expected);
        assert_eq!((INIT.len(), START.len(), STOP.len()), (37, 9, 13));
        assert_eq!(bus.reg(REG_PGA1), 0x00, "stop leaves the PGAs off");
    }

    #[test]
    fn pga_and_the_refusals() {
        let mut chip = Es7243e::new(FakeCodec::default(), ADDR);
        chip.init().unwrap();
        assert_eq!(chip.regs().bus().reg(REG_PGA1), 0x1A);
        chip.set_pga_code(0x10).unwrap();
        assert_eq!(chip.regs().bus().reg(REG_PGA2), 0x10);
        let writes = chip.regs().bus().writes.len();
        assert_eq!(chip.start(Module::Dac), Err(Error::Unsupported));
        assert_eq!(chip.set_mute(true), Err(Error::Unsupported));
        assert_eq!(
            chip.set_format(I2sFormat::LeftJustified, Bits::B16),
            Err(Error::Unsupported)
        );
        chip.set_format(I2sFormat::Standard, Bits::B16).unwrap();
        chip.set_sample_rate(16_000, 4_096_000).unwrap();
        assert_eq!(
            chip.regs().bus().writes.len(),
            writes,
            "nothing written by any of those"
        );
    }
}
