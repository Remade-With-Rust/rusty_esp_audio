//! Track A: `PdmIn`, a PDM microphone behind `AudioSource` via esp-idf-hal's
//! I2S PDM receive mode (ESP32 and ESP32-S3 have the PDM front end).
//!
//! The hardware decimates PDM to 16-bit PCM at the configured rate. This
//! module owns nothing but the driver handle: DMA buffers belong to ESP-IDF,
//! the block buffer belongs to the caller. Timestamps are the monotonic time
//! at which the read for a block began, minus nothing — the DMA latency
//! (one DMA buffer, a few ms) is documented rather than guessed.

use std::time::Instant;

use esp_idf_hal::delay::TickType;
use esp_idf_hal::gpio::{InputPin, OutputPin};
use esp_idf_hal::i2s::config::{
    Config, DataBitWidth, PdmRxClkConfig, PdmRxConfig, PdmRxGpioConfig, PdmRxSlotConfig, SlotMode,
};
use esp_idf_hal::i2s::{I2s, I2sDriver, I2sRx};
use rusty_esp_audio_core::AudioSource;
use rusty_esp_audio_core::esp_core::error::{Error, Result};
use rusty_esp_audio_core::esp_core::pcm::{PcmBlock, PcmFormat, SampleFormat};
use rusty_esp_audio_core::esp_core::time::Micros;

/// A running PDM receive channel producing mono i16.
pub struct PdmIn<'d> {
    driver: I2sDriver<'d, I2sRx>,
    format: PcmFormat,
    epoch: Instant,
    timeout_ms: u32,
    /// Blocks delivered.
    pub blocks: u64,
    /// Driver reads that returned fewer bytes than asked (the block was
    /// completed with further reads).
    pub short_reads: u64,
}

impl core::fmt::Debug for PdmIn<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("PdmIn")
            .field("format", &self.format)
            .field("blocks", &self.blocks)
            .field("short_reads", &self.short_reads)
            .finish_non_exhaustive()
    }
}

impl<'d> PdmIn<'d> {
    /// Open `i2s` in PDM RX mode on `clk`/`din` at `sample_rate_hz`, mono
    /// 16-bit, and start it. The XIAO ESP32-S3 Sense microphone is on
    /// CLK GPIO42, DATA GPIO41.
    pub fn new<I2S: I2s + 'd>(
        i2s: I2S,
        clk: impl OutputPin + 'd,
        din: impl InputPin + 'd,
        sample_rate_hz: u32,
    ) -> Result<Self> {
        let format = PcmFormat::new(sample_rate_hz, 1, SampleFormat::I16)?;
        let cfg = PdmRxConfig::new(
            Config::default(),
            PdmRxClkConfig::from_sample_rate_hz(sample_rate_hz),
            PdmRxSlotConfig::from_bits_per_sample_and_slot_mode(
                DataBitWidth::Bits16,
                SlotMode::Mono,
            ),
            PdmRxGpioConfig::new(false),
        );
        let mut driver = I2sDriver::new_pdm_rx(i2s, &cfg, clk, din).map_err(|_| Error::Hardware)?;
        driver.rx_enable().map_err(|_| Error::Hardware)?;
        Ok(PdmIn {
            driver,
            format,
            epoch: Instant::now(),
            timeout_ms: 1000,
            blocks: 0,
            short_reads: 0,
        })
    }

    /// How long one block read may wait for DMA before `Err(Timeout)`.
    pub fn set_timeout_ms(&mut self, ms: u32) {
        self.timeout_ms = ms;
    }
}

impl AudioSource for PdmIn<'_> {
    fn format(&self) -> PcmFormat {
        self.format
    }

    fn read<'b>(&mut self, out: &'b mut [u8]) -> Result<PcmBlock<'b>> {
        let fb = self.format.frame_bytes();
        if out.is_empty() || out.len() % fb != 0 {
            return Err(Error::InvalidGeometry);
        }
        let started = Micros(self.epoch.elapsed().as_micros() as u64);
        let ticks = TickType::new_millis(u64::from(self.timeout_ms)).ticks();
        let mut filled = 0usize;
        while filled < out.len() {
            let n = self
                .driver
                .read(&mut out[filled..], ticks)
                .map_err(|_| Error::Hardware)?;
            if n == 0 {
                return Err(Error::Timeout);
            }
            if filled + n < out.len() {
                self.short_reads += 1;
            }
            filled += n;
        }
        self.blocks += 1;
        PcmBlock::new(self.format, started, out)
    }
}
