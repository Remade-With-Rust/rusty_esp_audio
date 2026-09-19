//! A TOTAL proof, not a sample: `round_sat16`'s fused form agrees with the
//! `libm::roundf`-then-cast form it replaced on every one of the 2^32 `f32`
//! bit patterns, NaNs and infinities and subnormals included.
//!
//! `#[ignore]`d because it is a few minutes in release and pointless in debug;
//! run it whenever either form is touched:
//!
//!   cargo test -p rusty_esp_audio-core --release --test round_sat16_exhaustive -- --ignored --nocapture
//!
//! It exists because the fused form is an ALGEBRAIC claim about how `roundf`
//! is implemented, and a corpus can only ever fail to refute one of those.

/// The form that shipped before the fusion, kept verbatim as the oracle.
fn round_sat16_ref(v: f32) -> i16 {
    let r = libm::roundf(v);
    if r >= 32767.0 {
        i16::MAX
    } else if r <= -32768.0 {
        i16::MIN
    } else {
        r as i16
    }
}

/// The form that ships now. Kept here rather than imported because the real
/// one is `pub(crate)`; if this copy and that one ever drift, the corpus gate
/// in `element_oracle.rs` is what notices.
fn round_sat16_new(v: f32) -> i16 {
    let t = v + libm::copysignf(0.5 - 0.25 * f32::EPSILON, v);
    if t >= 32767.0 {
        i16::MAX
    } else if t <= -32768.0 {
        i16::MIN
    } else {
        t as i16
    }
}

#[test]
#[ignore = "sweeps all 2^32 f32 bit patterns; run with --release --ignored"]
fn round_sat16_matches_libm_exhaustively() {
    let mut checked: u64 = 0;
    let mut bits: u32 = 0;
    loop {
        let v = f32::from_bits(bits);
        let (a, b) = (round_sat16_ref(v), round_sat16_new(v));
        assert_eq!(a, b, "bits={bits:#010x} v={v:?}: ref={a} new={b}");
        checked += 1;
        if bits == u32::MAX {
            break;
        }
        bits += 1;
    }
    assert_eq!(checked, 1u64 << 32);
    println!("round_sat16: {checked} bit patterns, all identical");
}

/// The same claim on the values that actually reach it, so a normal `cargo
/// test` run still carries a real check of the boundaries.
#[test]
fn round_sat16_matches_at_the_boundaries() {
    let mut cases: Vec<f32> = vec![
        0.0,
        -0.0,
        0.5,
        -0.5,
        1.5,
        -1.5,
        f32::NAN,
        f32::INFINITY,
        f32::NEG_INFINITY,
        f32::MIN_POSITIVE,
        -f32::MIN_POSITIVE,
        f32::from_bits(0x3EFF_FFFF), // the largest f32 below 0.5
        -f32::from_bits(0x3EFF_FFFF),
    ];
    for edge in [32766.0f32, 32767.0, 32768.0, -32767.0, -32768.0, -32769.0] {
        for d in [-1.0f32, -0.5, -0.0001, 0.0, 0.0001, 0.5, 1.0] {
            cases.push(edge + d);
        }
    }
    for i in -70_000i32..70_000 {
        cases.push(i as f32 * 0.5);
    }
    for v in cases {
        assert_eq!(
            round_sat16_ref(v),
            round_sat16_new(v),
            "v={v:?} bits={:#010x}",
            v.to_bits()
        );
    }
}
