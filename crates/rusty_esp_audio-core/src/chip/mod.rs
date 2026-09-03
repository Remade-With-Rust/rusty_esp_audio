//! Codec chips as register data — `esp_codec_dev`'s drivers remade.
//!
//! A codec chip is an I²C register map and a handful of sequences: bring-up,
//! clocking for a sample rate, the serial-port format, start, stop, gain and
//! mute. Each chip module here carries those as **data** (register
//! constants, a clock-coefficient table, ordered write lists) with the
//! arithmetic the vendor driver performs on them, re-derived from Espressif's
//! `esp-adf` drivers (ESPRESSIF MIT License, © 2019 Espressif Systems
//! (Shanghai) Co., Ltd) and attributed in each module. The bus is
//! `embedded-hal` 1.0 `I2c`, so the same driver runs over `esp-idf-hal`
//! (Track A), `esp-hal` (Track B) and a fake in the host tests.
//!
//! What a driver here does **not** do: touch a GPIO (the PA-enable pin is the
//! board's, handed to the firmware), own a clock (delays are the caller's
//! closure), or map "volume 0..100" to a register through a hidden curve —
//! the register value and the dB it means are the API, and a product puts its
//! own taste on top.
//!
//! | chip | role | module |
//! |---|---|---|
//! | ES8311 | mono ADC + DAC, the Korvo-2 / S3-BOX / S3-EYE speaker path | [`es8311`] |
//! | ES7210 | four-channel ADC (mic array), the Korvo-2 / S3-EYE mic path | [`es7210`] |

pub mod es7210;
pub mod es8311;

use embedded_hal::i2c::I2c;
use rusty_esp_core::error::{Error, Result};

/// One register write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Register {
    /// Register address.
    pub addr: u8,
    /// Value.
    pub value: u8,
}

impl Register {
    /// Build a register write.
    #[must_use]
    pub const fn new(addr: u8, value: u8) -> Self {
        Register { addr, value }
    }
}

/// One step of a register sequence.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RegOp {
    /// Write `value` to `addr`.
    Write {
        /// Register address.
        addr: u8,
        /// Value.
        value: u8,
    },
    /// Read `addr`, replace the bits under `mask` with `value`'s, write it back.
    Update {
        /// Register address.
        addr: u8,
        /// The bits that change.
        mask: u8,
        /// Their new value (masked).
        value: u8,
    },
    /// Wait this many milliseconds before the next step.
    DelayMs(u16),
}

/// Which converters a start or stop applies to.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Module {
    /// Capture only.
    Adc,
    /// Playback only.
    Dac,
    /// Both.
    AdcDac,
}

/// Who drives the serial clocks.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// The chip generates BCLK / LRCK.
    Master,
    /// The ESP32's I²S peripheral does.
    Slave,
}

/// Serial data port framing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum I2sFormat {
    /// Philips I²S (one BCLK delay after LRCK).
    Standard,
    /// Left-justified (the vendor drivers treat right-justified the same).
    LeftJustified,
    /// DSP / PCM mode A.
    Dsp,
}

/// Bits per sample slot on the serial port.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Bits {
    /// 16.
    B16,
    /// 24.
    B24,
    /// 32.
    B32,
}

/// The register map every codec chip exposes over I²C: one address byte,
/// one data byte, and a read that is a write of the address followed by a
/// read of the value.
#[derive(Debug)]
pub struct I2cRegs<I> {
    i2c: I,
    addr: u8,
}

impl<I: I2c> I2cRegs<I> {
    /// Talk to the chip at 7-bit address `addr`.
    pub fn new(i2c: I, addr: u8) -> Self {
        I2cRegs { i2c, addr }
    }

    /// The 7-bit bus address.
    #[must_use]
    pub fn addr(&self) -> u8 {
        self.addr
    }

    /// Give the bus back.
    pub fn release(self) -> I {
        self.i2c
    }

    /// The bus, borrowed.
    pub fn bus(&self) -> &I {
        &self.i2c
    }

    /// The bus, borrowed mutably (for a transaction the driver does not wrap).
    pub fn bus_mut(&mut self) -> &mut I {
        &mut self.i2c
    }

    /// Write one register.
    pub fn write(&mut self, reg: u8, value: u8) -> Result<()> {
        self.i2c
            .write(self.addr, &[reg, value])
            .map_err(|_| Error::Hardware)
    }

    /// Read one register.
    pub fn read(&mut self, reg: u8) -> Result<u8> {
        let mut v = [0u8; 1];
        self.i2c
            .write_read(self.addr, &[reg], &mut v)
            .map_err(|_| Error::Hardware)?;
        Ok(v[0])
    }

    /// Replace the bits under `mask` with `value`'s (the vendor drivers'
    /// `update_reg_bit`: `(old & !mask) | (value & mask)`).
    pub fn update(&mut self, reg: u8, mask: u8, value: u8) -> Result<()> {
        let old = self.read(reg)?;
        self.write(reg, (old & !mask) | (value & mask))
    }

    /// Run a register sequence. `delay_ms` is called for each
    /// [`RegOp::DelayMs`] step (the caller owns the clock). Returns the number
    /// of registers written.
    pub fn apply(&mut self, table: &[RegOp], mut delay_ms: impl FnMut(u16)) -> Result<usize> {
        let mut written = 0;
        for op in table {
            match *op {
                RegOp::Write { addr, value } => {
                    self.write(addr, value)?;
                    written += 1;
                }
                RegOp::Update { addr, mask, value } => {
                    self.update(addr, mask, value)?;
                    written += 1;
                }
                RegOp::DelayMs(ms) => delay_ms(ms),
            }
        }
        Ok(written)
    }
}

/// What a firmware asks of any codec chip, whichever one the board carries.
pub trait CodecChip {
    /// Clock the converters for `sample_rate_hz` from a master clock of
    /// `mclk_hz` (a chip supports only the pairs in its coefficient table).
    fn set_sample_rate(&mut self, sample_rate_hz: u32, mclk_hz: u32) -> Result<()>;
    /// Serial port framing and slot width.
    fn set_format(&mut self, fmt: I2sFormat, bits: Bits) -> Result<()>;
    /// Power the converters up and unmute the port.
    fn start(&mut self, module: Module) -> Result<()>;
    /// Power down.
    fn stop(&mut self) -> Result<()>;
    /// Mute without changing gain.
    fn set_mute(&mut self, mute: bool) -> Result<()>;
}

#[cfg(test)]
pub(crate) mod fake {
    //! A register-map I²C device for tests: remembers every write and
    //! answers reads from what was written (0 before).

    use core::convert::Infallible;

    use embedded_hal::i2c::{ErrorType, I2c, Operation, SevenBitAddress};

    #[derive(Debug, Default)]
    pub struct FakeCodec {
        pub addr_seen: Option<u8>,
        pub regs: std::collections::BTreeMap<u8, u8>,
        pub writes: std::vec::Vec<(u8, u8)>,
        last_reg: u8,
    }

    impl FakeCodec {
        /// Preload registers, as a chip's reset defaults would.
        pub fn with_defaults(defaults: &[(u8, u8)]) -> Self {
            let mut f = FakeCodec::default();
            for &(r, v) in defaults {
                f.regs.insert(r, v);
            }
            f
        }

        /// The value a register holds now.
        pub fn reg(&self, r: u8) -> u8 {
            *self.regs.get(&r).unwrap_or(&0)
        }

        /// Every value written to `r`, in order.
        pub fn history(&self, r: u8) -> std::vec::Vec<u8> {
            self.writes
                .iter()
                .filter(|(a, _)| *a == r)
                .map(|(_, v)| *v)
                .collect()
        }
    }

    impl ErrorType for FakeCodec {
        type Error = Infallible;
    }

    impl I2c<SevenBitAddress> for FakeCodec {
        fn transaction(
            &mut self,
            address: SevenBitAddress,
            operations: &mut [Operation<'_>],
        ) -> Result<(), Infallible> {
            self.addr_seen = Some(address);
            for op in operations {
                match op {
                    Operation::Write(bytes) => {
                        self.last_reg = bytes[0];
                        if let Some(&v) = bytes.get(1) {
                            self.regs.insert(bytes[0], v);
                            self.writes.push((bytes[0], v));
                        }
                    }
                    Operation::Read(buf) => {
                        for (i, b) in buf.iter_mut().enumerate() {
                            *b = self.reg(self.last_reg.wrapping_add(i as u8));
                        }
                    }
                }
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fake::FakeCodec;
    use super::*;

    #[test]
    fn write_read_update_apply() {
        let mut regs = I2cRegs::new(FakeCodec::default(), 0x18);
        assert_eq!(regs.addr(), 0x18);
        regs.write(0x10, 0xA5).unwrap();
        assert_eq!(regs.read(0x10).unwrap(), 0xA5);
        regs.update(0x10, 0x0F, 0x03).unwrap();
        assert_eq!(regs.read(0x10).unwrap(), 0xA3);
        let mut delays = 0;
        let n = regs
            .apply(
                &[
                    RegOp::Write {
                        addr: 0x11,
                        value: 1,
                    },
                    RegOp::DelayMs(2),
                    RegOp::Update {
                        addr: 0x11,
                        mask: 0xF0,
                        value: 0x20,
                    },
                ],
                |_| delays += 1,
            )
            .unwrap();
        assert_eq!((n, delays), (2, 1));
        let bus = regs.release();
        assert_eq!(bus.reg(0x11), 0x21);
        assert_eq!(bus.addr_seen, Some(0x18));
        assert_eq!(bus.history(0x10), [0xA5, 0xA3]);
    }
}
