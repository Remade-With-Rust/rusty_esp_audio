#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]
//! `rusty_esp_audio-esp` — chip backends and the Track A transport for
//! `rusty_esp_audio`.
//!
//! This is the **wrap** crate of the package: where the silicon must be
//! touched, it calls the esp-rs HAL (Track B, `esp-hal`) or ESP-IDF (Track A,
//! `esp-idf`) and exposes the core crate's traits over it. Nothing product- or
//! codec-specific lives here; that is the core's job.
//!
//! With `std` alone it also carries the transport a Track A device and its
//! hub share: [`net`] (raw PCM over UDP, the datagrams `ffplay` and `rff`
//! read directly) and [`wavfile`] (WAV files over `std::fs`). That code is
//! the same on the laptop, the Pi and the chip, which is why its oracle tests
//! run on the host.
//!
//! `unsafe` is denied crate-wide; a backend that must use it at a DMA or FFI
//! boundary opts in per block with `#[allow(unsafe_code)]` and a `// SAFETY:`
//! comment stating the invariant.

#[cfg(feature = "alloc")]
extern crate alloc;

#[cfg(all(feature = "esp-hal", feature = "esp-idf"))]
compile_error!("enable exactly one track: `esp-hal` (no_std) or `esp-idf` (std)");

pub use rusty_esp_audio_core as core;

#[cfg(feature = "std")]
pub mod net;
#[cfg(feature = "std")]
pub mod wavfile;

/// Which track this build of the backend crate was compiled for.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Track {
    /// No chip backend compiled in: host build, traits and transport only.
    Host,
    /// Track B — bare metal, esp-hal + Embassy.
    EspHal,
    /// Track A — std on ESP-IDF.
    EspIdf,
}

/// The track this crate was built with.
pub const TRACK: Track = if cfg!(feature = "esp-hal") {
    Track::EspHal
} else if cfg!(feature = "esp-idf") {
    Track::EspIdf
} else {
    Track::Host
};

#[cfg(feature = "esp-hal")]
pub mod hal {
    //! Track B backends. Drivers land here with their esp-hal pin.
}

#[cfg(feature = "esp-idf")]
pub mod idf {
    //! Track A backends over esp-idf-hal 0.46.
    pub mod pdm;
    pub use pdm::PdmIn;
}
