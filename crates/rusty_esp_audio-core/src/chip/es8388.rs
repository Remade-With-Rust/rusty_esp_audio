//! ES8388 — Everest's stereo ADC + DAC with line-in bypass, the codec on the
//! ESP32-LyraT (v4.2 / v4.3) and the boards that copied it.
//!
//! Register map and every sequence are re-derived from Espressif's `esp-adf`
//! `components/audio_hal/driver/es8388/{es8388.h, es8388.c}` and the shared
//! `esxxx_common.h` (ESPRESSIF MIT License, © 2019 Espressif Systems
//! (Shanghai) Co., Ltd): the same registers in the same order with the same
//! values, as data. Unlike the ES8311 there is no clock-coefficient table:
//! the chip takes an MCLK/LRCK ratio (`ADCCONTROL5` / `DACCONTROL2`) and, in
//! master mode, a BCLK divider (`MASTERMODE`); [`Es8388::set_sample_rate`]
//! turns a (MCLK, rate) pair into that ratio and refuses one the chip has no
//! code for.
//!
//! Volume is the datasheet's: `0x00` is 0 dB, one step is −0.5 dB, `0xC0`
//! is −96 dB, for the DAC (`DACCONTROL4/5`) and the ADC (`ADCCONTROL8/9`)
//! alike. The PA-enable GPIO and the headphone-detect pin are the board's.

use embedded_hal::i2c::I2c;
use rusty_esp_core::error::{Error, Result};

use super::{Bits, CodecChip, I2cRegs, I2sFormat, Mode, Module};

/// 7-bit I²C address with CE low (the vendor driver's 8-bit `0x20`); `0x11`
/// with CE high.
pub const ADDR: u8 = 0x10;

/// The register map (`es8388.h`).
#[allow(missing_docs)]
pub mod reg {
    pub const CONTROL1: u8 = 0x00;
    pub const CONTROL2: u8 = 0x01;
    pub const CHIPPOWER: u8 = 0x02;
    pub const ADCPOWER: u8 = 0x03;
    pub const DACPOWER: u8 = 0x04;
    pub const CHIPLOPOW1: u8 = 0x05;
    pub const CHIPLOPOW2: u8 = 0x06;
    pub const ANAVOLMANAG: u8 = 0x07;
    pub const MASTERMODE: u8 = 0x08;
    pub const ADCCONTROL1: u8 = 0x09;
    pub const ADCCONTROL2: u8 = 0x0A;
    pub const ADCCONTROL3: u8 = 0x0B;
    pub const ADCCONTROL4: u8 = 0x0C;
    pub const ADCCONTROL5: u8 = 0x0D;
    pub const ADCCONTROL6: u8 = 0x0E;
    pub const ADCCONTROL7: u8 = 0x0F;
    pub const ADCCONTROL8: u8 = 0x10;
    pub const ADCCONTROL9: u8 = 0x11;
    pub const ADCCONTROL10: u8 = 0x12;
    pub const ADCCONTROL11: u8 = 0x13;
    pub const ADCCONTROL12: u8 = 0x14;
    pub const ADCCONTROL13: u8 = 0x15;
    pub const ADCCONTROL14: u8 = 0x16;
    pub const DACCONTROL1: u8 = 0x17;
    pub const DACCONTROL2: u8 = 0x18;
    pub const DACCONTROL3: u8 = 0x19;
    pub const DACCONTROL4: u8 = 0x1A;
    pub const DACCONTROL5: u8 = 0x1B;
    pub const DACCONTROL6: u8 = 0x1C;
    pub const DACCONTROL7: u8 = 0x1D;
    pub const DACCONTROL8: u8 = 0x1E;
    pub const DACCONTROL9: u8 = 0x1F;
    pub const DACCONTROL10: u8 = 0x20;
    pub const DACCONTROL11: u8 = 0x21;
    pub const DACCONTROL12: u8 = 0x22;
    pub const DACCONTROL13: u8 = 0x23;
    pub const DACCONTROL14: u8 = 0x24;
    pub const DACCONTROL15: u8 = 0x25;
    pub const DACCONTROL16: u8 = 0x26;
    pub const DACCONTROL17: u8 = 0x27;
    pub const DACCONTROL18: u8 = 0x28;
    pub const DACCONTROL19: u8 = 0x29;
    pub const DACCONTROL20: u8 = 0x2A;
    pub const DACCONTROL21: u8 = 0x2B;
    pub const DACCONTROL22: u8 = 0x2C;
    pub const DACCONTROL23: u8 = 0x2D;
    pub const DACCONTROL24: u8 = 0x2E;
    pub const DACCONTROL25: u8 = 0x2F;
    pub const DACCONTROL26: u8 = 0x30;
    pub const DACCONTROL27: u8 = 0x31;
    pub const DACCONTROL28: u8 = 0x32;
    pub const DACCONTROL29: u8 = 0x33;
    pub const DACCONTROL30: u8 = 0x34;
}

/// Which line outputs the DAC drives (`DACPOWER` bits; OR them).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DacOutputs(pub u8);

impl DacOutputs {
    /// Left, output 1.
    pub const LOUT1: DacOutputs = DacOutputs(0x04);
    /// Left, output 2.
    pub const LOUT2: DacOutputs = DacOutputs(0x08);
    /// Right, output 1.
    pub const ROUT1: DacOutputs = DacOutputs(0x10);
    /// Right, output 2.
    pub const ROUT2: DacOutputs = DacOutputs(0x20);
    /// Both channels of output 1 (the vendor's "line 2" board wiring).
    pub const LINE1: DacOutputs = DacOutputs(0x14);
    /// Both channels of output 2 (the vendor's "line 1" board wiring).
    pub const LINE2: DacOutputs = DacOutputs(0x28);
    /// Everything.
    pub const ALL: DacOutputs = DacOutputs(0x3C);
}

/// Where the ADC listens (`ADCCONTROL2` high nibble and the two mic codes).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum AdcInput {
    /// LINPUT1 / RINPUT1.
    Line1 = 0x00,
    /// Microphone 1.
    Mic1 = 0x05,
    /// Microphone 2.
    Mic2 = 0x06,
    /// LINPUT2 / RINPUT2.
    Line2 = 0x50,
    /// The differential pair.
    Difference = 0xF0,
}

/// Bring-up parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Who drives BCLK / LRCK.
    pub mode: Mode,
    /// The line outputs to power.
    pub dac_output: DacOutputs,
    /// The ADC input to select.
    pub adc_input: AdcInput,
}

impl Config {
    /// The vendor driver's defaults: slave, every output, line input 1.
    #[must_use]
    pub const fn slave() -> Self {
        Config {
            mode: Mode::Slave,
            dac_output: DacOutputs::ALL,
            adc_input: AdcInput::Line1,
        }
    }
}

/// The volume register for a level in half-decibels at or below zero: `0`
/// → `0x00` (0 dB), `-100` → `0x64` (−50 dB), `-192` → `0xC0` (−96 dB);
/// clamped at both ends.
#[must_use]
pub const fn volume_reg(half_db: i16) -> u8 {
    let v = -(half_db as i32);
    if v < 0 {
        0
    } else if v > 0xC0 {
        0xC0
    } else {
        v as u8
    }
}

/// The `ADCCONTROL5` / `DACCONTROL2` code for an MCLK : LRCK ratio
/// (`esxxx_common.h`'s `es_lclk_div_t`), or `None` when the chip has none.
#[must_use]
pub const fn lrck_ratio_code(ratio: u32) -> Option<u8> {
    Some(match ratio {
        128 => 0,
        192 => 1,
        256 => 2,
        384 => 3,
        512 => 4,
        576 => 5,
        768 => 6,
        1024 => 7,
        1152 => 8,
        1408 => 9,
        1536 => 10,
        2112 => 11,
        2304 => 12,
        125 => 16,
        136 => 17,
        250 => 18,
        272 => 19,
        375 => 20,
        500 => 21,
        544 => 22,
        750 => 23,
        1000 => 24,
        1088 => 25,
        1496 => 26,
        1500 => 27,
        _ => return None,
    })
}

/// The driver.
#[derive(Debug)]
pub struct Es8388<I> {
    regs: I2cRegs<I>,
}

impl<I: I2c> Es8388<I> {
    /// Over `i2c` at the chip's 7-bit `addr` (normally [`ADDR`]).
    pub fn new(i2c: I, addr: u8) -> Self {
        Es8388 {
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

    /// `es8388_init`: mute, chip power and the three undocumented analog
    /// trims, master/slave, the DAC path (16-bit I²S, single speed, ratio
    /// 256, mixers at 0 dB, outputs), the ADC path (PGA, input select,
    /// 16-bit I²S, ratio 256, 0 dB), ADC power.
    pub fn init(&mut self, cfg: &Config) -> Result<()> {
        let r = &mut self.regs;
        r.write(reg::DACCONTROL3, 0x04)?; // DAC muted while the rest comes up
        r.write(reg::CONTROL2, 0x50)?;
        r.write(reg::CHIPPOWER, 0x00)?;
        r.write(0x35, 0xA0)?;
        r.write(0x37, 0xD0)?;
        r.write(0x39, 0xD0)?;
        r.write(
            reg::MASTERMODE,
            match cfg.mode {
                Mode::Slave => 0x00,
                Mode::Master => 0x01,
            },
        )?;
        r.write(reg::DACPOWER, 0xC0)?;
        r.write(reg::CONTROL1, 0x12)?;
        r.write(reg::DACCONTROL1, 0x18)?;
        r.write(reg::DACCONTROL2, 0x02)?;
        r.write(reg::DACCONTROL16, 0x00)?;
        r.write(reg::DACCONTROL17, 0x90)?;
        r.write(reg::DACCONTROL20, 0x90)?;
        r.write(reg::DACCONTROL21, 0x80)?;
        r.write(reg::DACCONTROL23, 0x00)?;
        r.write(reg::DACCONTROL24, 0x1E)?;
        r.write(reg::DACCONTROL25, 0x1E)?;
        r.write(reg::DACCONTROL26, 0)?;
        r.write(reg::DACCONTROL27, 0)?;
        r.write(reg::DACPOWER, cfg.dac_output.0)?;
        r.write(reg::ADCPOWER, 0xFF)?;
        r.write(reg::ADCCONTROL1, 0xBB)?;
        r.write(reg::ADCCONTROL2, cfg.adc_input as u8)?;
        r.write(reg::ADCCONTROL3, 0x02)?;
        r.write(reg::ADCCONTROL4, 0x0C)?;
        r.write(reg::ADCCONTROL5, 0x02)?;
        self.set_volume(Module::Adc, 0)?;
        self.regs.write(reg::ADCPOWER, 0x09)
    }

    /// `es8388_set_adc_dac_volume`: `half_db` at or below zero, in 0.5 dB
    /// steps, clamped at −96 dB, for the ADC, the DAC or both.
    pub fn set_volume(&mut self, module: Module, half_db: i16) -> Result<()> {
        let v = volume_reg(half_db);
        let r = &mut self.regs;
        if matches!(module, Module::Adc | Module::AdcDac) {
            r.write(reg::ADCCONTROL8, v)?;
            r.write(reg::ADCCONTROL9, v)?;
        }
        if matches!(module, Module::Dac | Module::AdcDac) {
            r.write(reg::DACCONTROL5, v)?;
            r.write(reg::DACCONTROL4, v)?;
        }
        Ok(())
    }

    /// `es8388_i2s_config_clock`, master mode: the BCLK divider code into
    /// `MASTERMODE` and the LRCK ratio code into both `ADCCONTROL5` and
    /// `DACCONTROL2`.
    pub fn set_clock_dividers(&mut self, sclk_div_code: u8, lrck_ratio_code: u8) -> Result<()> {
        let r = &mut self.regs;
        r.write(reg::MASTERMODE, sclk_div_code)?;
        r.write(reg::ADCCONTROL5, lrck_ratio_code)?;
        r.write(reg::DACCONTROL2, lrck_ratio_code)
    }

    /// `es8388_config_dac_output`: which line outputs the DAC drives.
    pub fn set_dac_outputs(&mut self, outputs: DacOutputs) -> Result<()> {
        let v = self.regs.read(reg::DACPOWER)? & 0xC3;
        self.regs.write(reg::DACPOWER, v | outputs.0)
    }

    /// `es8388_config_adc_input`.
    pub fn set_adc_input(&mut self, input: AdcInput) -> Result<()> {
        let v = self.regs.read(reg::ADCCONTROL2)? & 0x0F;
        self.regs.write(reg::ADCCONTROL2, v | input as u8)
    }

    /// `es8388_set_mic_gain`: 0 to 24 dB in 3 dB steps, both PGAs.
    pub fn set_mic_gain(&mut self, gain_db: u8) -> Result<()> {
        if gain_db > 24 || gain_db % 3 != 0 {
            return Err(Error::Unsupported);
        }
        let n = gain_db / 3;
        self.regs.write(reg::ADCCONTROL1, (n << 4) | n)
    }

    /// `es8388_start(ES_MODULE_LINE)`: line-in through the mixers to
    /// line-out, the DAC still summed in.
    pub fn start_line_bypass(&mut self) -> Result<()> {
        let prev = self.regs.read(reg::DACCONTROL21)?;
        let r = &mut self.regs;
        r.write(reg::DACCONTROL16, 0x09)?;
        r.write(reg::DACCONTROL17, 0x50)?;
        r.write(reg::DACCONTROL20, 0x50)?;
        r.write(reg::DACCONTROL21, 0xC0)?;
        self.restart_if_changed(prev)?;
        let r = &mut self.regs;
        r.write(reg::ADCPOWER, 0x00)?;
        r.write(reg::DACPOWER, 0x3C)?;
        self.set_mute(false)
    }

    /// `es8388_stop(ES_MODULE_LINE)`: mixers back to DAC only.
    pub fn stop_line_bypass(&mut self) -> Result<()> {
        let r = &mut self.regs;
        r.write(reg::DACCONTROL21, 0x80)?;
        r.write(reg::DACCONTROL16, 0x00)?;
        r.write(reg::DACCONTROL17, 0x90)?;
        r.write(reg::DACCONTROL20, 0x90)
    }

    /// `es8388_stop` for one module (the trait's `stop` is both).
    pub fn stop_module(&mut self, module: Module) -> Result<()> {
        if matches!(module, Module::Dac | Module::AdcDac) {
            self.regs.write(reg::DACPOWER, 0x00)?;
            self.set_mute(true)?;
        }
        if matches!(module, Module::Adc | Module::AdcDac) {
            self.regs.write(reg::ADCPOWER, 0xFF)?;
        }
        if module == Module::AdcDac {
            self.regs.write(reg::DACCONTROL21, 0x9C)?;
        }
        Ok(())
    }

    /// The vendor's "start state machine" pulse on `CHIPPOWER`, issued only
    /// when `DACCONTROL21` actually changed.
    fn restart_if_changed(&mut self, prev: u8) -> Result<()> {
        let now = self.regs.read(reg::DACCONTROL21)?;
        if now != prev {
            self.regs.write(reg::CHIPPOWER, 0xF0)?;
            self.regs.write(reg::CHIPPOWER, 0x00)?;
        }
        Ok(())
    }
}

impl<I: I2c> CodecChip for Es8388<I> {
    /// The MCLK : LRCK ratio into both converters' clock registers
    /// (single speed); a ratio the chip has no code for is `Unsupported`,
    /// and nothing is written.
    fn set_sample_rate(&mut self, sample_rate_hz: u32, mclk_hz: u32) -> Result<()> {
        if sample_rate_hz == 0 || mclk_hz % sample_rate_hz != 0 {
            return Err(Error::Unsupported);
        }
        let code = lrck_ratio_code(mclk_hz / sample_rate_hz).ok_or(Error::Unsupported)?;
        self.regs.write(reg::ADCCONTROL5, code)?;
        self.regs.write(reg::DACCONTROL2, code)
    }

    /// `es8388_config_fmt` and `es8388_set_bits_per_sample` on both
    /// converters: `ADCCONTROL4` bits 1:0 / 4:2, `DACCONTROL1` bits 2:1 / 5:3.
    fn set_format(&mut self, fmt: I2sFormat, bits: Bits) -> Result<()> {
        let f: u8 = match fmt {
            I2sFormat::Standard => 0,
            I2sFormat::LeftJustified => 1,
            I2sFormat::Dsp => 3,
        };
        let b: u8 = match bits {
            Bits::B16 => 0x03,
            Bits::B24 => 0x00,
            Bits::B32 => 0x04,
        };
        let r = &mut self.regs;
        let adc = r.read(reg::ADCCONTROL4)? & 0xFC;
        r.write(reg::ADCCONTROL4, adc | f)?;
        let dac = r.read(reg::DACCONTROL1)? & 0xF9;
        r.write(reg::DACCONTROL1, dac | (f << 1))?;
        let adc = r.read(reg::ADCCONTROL4)? & 0xE3;
        r.write(reg::ADCCONTROL4, adc | (b << 2))?;
        let dac = r.read(reg::DACCONTROL1)? & 0xC7;
        r.write(reg::DACCONTROL1, dac | (b << 3))
    }

    /// `es8388_start`: enable the DAC clocks, pulse the state machine if
    /// that changed anything, power the converters asked for, unmute.
    fn start(&mut self, module: Module) -> Result<()> {
        let prev = self.regs.read(reg::DACCONTROL21)?;
        self.regs.write(reg::DACCONTROL21, 0x80)?;
        self.restart_if_changed(prev)?;
        if matches!(module, Module::Adc | Module::AdcDac) {
            self.regs.write(reg::ADCPOWER, 0x00)?;
        }
        if matches!(module, Module::Dac | Module::AdcDac) {
            self.regs.write(reg::DACPOWER, 0x3C)?;
            self.set_mute(false)?;
        }
        Ok(())
    }

    /// `es8388_stop(ES_MODULE_ADC_DAC)`.
    fn stop(&mut self) -> Result<()> {
        self.stop_module(Module::AdcDac)
    }

    /// `es8388_set_voice_mute`: `DACCONTROL3` bit 2.
    fn set_mute(&mut self, mute: bool) -> Result<()> {
        let v = self.regs.read(reg::DACCONTROL3)? & 0xFB;
        self.regs.write(reg::DACCONTROL3, v | (u8::from(mute) << 2))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::chip::fake::FakeCodec;

    #[test]
    fn init_reproduces_the_vendor_sequence() {
        let mut chip = Es8388::new(FakeCodec::default(), ADDR);
        chip.init(&Config::slave()).unwrap();
        let bus = chip.release();
        assert_eq!(bus.addr_seen, Some(0x10));
        let expected: &[(u8, u8)] = &[
            (0x19, 0x04),
            (0x01, 0x50),
            (0x02, 0x00),
            (0x35, 0xA0),
            (0x37, 0xD0),
            (0x39, 0xD0),
            (0x08, 0x00), // slave
            (0x04, 0xC0),
            (0x00, 0x12),
            (0x17, 0x18),
            (0x18, 0x02),
            (0x26, 0x00),
            (0x27, 0x90),
            (0x2A, 0x90),
            (0x2B, 0x80),
            (0x2D, 0x00),
            (0x2E, 0x1E),
            (0x2F, 0x1E),
            (0x30, 0x00),
            (0x31, 0x00),
            (0x04, 0x3C), // every output
            (0x03, 0xFF),
            (0x09, 0xBB),
            (0x0A, 0x00), // line input 1
            (0x0B, 0x02),
            (0x0C, 0x0C),
            (0x0D, 0x02),
            (0x10, 0x00), // ADC volume 0 dB, left
            (0x11, 0x00), // right
            (0x03, 0x09),
        ];
        assert_eq!(bus.writes, expected);
    }

    #[test]
    fn master_line2_and_a_microphone_set_their_registers() {
        let mut chip = Es8388::new(FakeCodec::default(), ADDR);
        chip.init(&Config {
            mode: Mode::Master,
            dac_output: DacOutputs::LINE2,
            adc_input: AdcInput::Mic1,
        })
        .unwrap();
        let bus = chip.release();
        assert_eq!(bus.reg(0x08), 0x01, "master");
        assert_eq!(bus.reg(0x04), 0x28, "LOUT2 | ROUT2");
        assert_eq!(bus.reg(0x0A), 0x05, "mic 1");
    }

    #[test]
    fn start_stop_pulse_the_state_machine_only_when_the_clock_register_changes() {
        let mut chip = Es8388::new(FakeCodec::default(), ADDR);
        chip.init(&Config::slave()).unwrap();
        chip.start(Module::AdcDac).unwrap();
        {
            let bus = chip.regs().bus();
            assert_eq!(
                bus.history(0x02),
                [0x00],
                "no pulse: DACCONTROL21 was already 0x80"
            );
            assert_eq!(bus.reg(0x03), 0x00, "ADC powered");
            assert_eq!(bus.reg(0x04), 0x3C, "DAC and outputs powered");
            assert_eq!(bus.reg(0x19) & 0x04, 0, "unmuted");
        }
        chip.stop().unwrap();
        {
            let bus = chip.regs().bus();
            assert_eq!(bus.reg(0x04), 0x00);
            assert_eq!(bus.reg(0x19) & 0x04, 0x04, "muted");
            assert_eq!(bus.reg(0x03), 0xFF);
            assert_eq!(bus.reg(0x2B), 0x9C, "MCLK off");
        }
        chip.start(Module::Dac).unwrap();
        let bus = chip.release();
        assert_eq!(
            bus.history(0x02),
            [0x00, 0xF0, 0x00],
            "the pulse after 0x9C -> 0x80"
        );
        assert_eq!(
            bus.reg(0x03),
            0xFF,
            "the ADC stays down when only the DAC starts"
        );
    }

    #[test]
    fn format_width_rate_volume_gain_and_bypass() {
        let mut chip = Es8388::new(FakeCodec::default(), ADDR);
        chip.init(&Config::slave()).unwrap();
        chip.set_format(I2sFormat::Standard, Bits::B16).unwrap();
        assert_eq!(chip.regs().bus().reg(0x0C), 0x0C);
        assert_eq!(chip.regs().bus().reg(0x17), 0x18);
        chip.set_format(I2sFormat::Dsp, Bits::B32).unwrap();
        assert_eq!(chip.regs().bus().reg(0x0C), 0x13);
        assert_eq!(chip.regs().bus().reg(0x17), 0x26);
        chip.set_sample_rate(48_000, 48_000 * 384).unwrap();
        assert_eq!(chip.regs().bus().reg(0x0D), 3);
        assert_eq!(chip.regs().bus().reg(0x18), 3);
        let writes_before = chip.regs().bus().writes.len();
        assert_eq!(
            chip.set_sample_rate(44_100, 4_096_000),
            Err(Error::Unsupported)
        );
        assert_eq!(chip.set_sample_rate(0, 4_096_000), Err(Error::Unsupported));
        assert_eq!(
            chip.regs().bus().writes.len(),
            writes_before,
            "refused before any write"
        );
        chip.set_volume(Module::Dac, -100).unwrap();
        assert_eq!(
            chip.regs().bus().reg(0x1A),
            0x64,
            "-50 dB, the vendor comment's point"
        );
        assert_eq!(chip.regs().bus().reg(0x1B), 0x64);
        chip.set_mic_gain(24).unwrap();
        assert_eq!(chip.regs().bus().reg(0x09), 0x88);
        chip.set_mic_gain(9).unwrap();
        assert_eq!(chip.regs().bus().reg(0x09), 0x33);
        assert_eq!(chip.set_mic_gain(10), Err(Error::Unsupported));
        chip.set_dac_outputs(DacOutputs::LINE1).unwrap();
        assert_eq!(chip.regs().bus().reg(0x04) & 0x3C, 0x14);
        chip.set_adc_input(AdcInput::Line2).unwrap();
        assert_eq!(chip.regs().bus().reg(0x0A), 0x50);
        chip.start_line_bypass().unwrap();
        {
            let bus = chip.regs().bus();
            assert_eq!(bus.reg(0x26), 0x09);
            assert_eq!(bus.reg(0x27), 0x50);
            assert_eq!(bus.reg(0x2A), 0x50);
            assert_eq!(bus.reg(0x2B), 0xC0);
            assert_eq!(bus.history(0x02), [0x00, 0xF0, 0x00], "0x80 -> 0xC0 pulses");
        }
        chip.stop_line_bypass().unwrap();
        let bus = chip.release();
        assert_eq!(
            (bus.reg(0x2B), bus.reg(0x26), bus.reg(0x27), bus.reg(0x2A)),
            (0x80, 0x00, 0x90, 0x90)
        );
    }

    #[test]
    fn volume_and_ratio_tables() {
        assert_eq!(volume_reg(0), 0x00);
        assert_eq!(volume_reg(-1), 0x01);
        assert_eq!(volume_reg(-192), 0xC0);
        assert_eq!(volume_reg(-300), 0xC0);
        assert_eq!(volume_reg(20), 0x00);
        assert_eq!(lrck_ratio_code(256), Some(2));
        assert_eq!(lrck_ratio_code(1500), Some(27));
        assert_eq!(lrck_ratio_code(300), None);
    }
}
