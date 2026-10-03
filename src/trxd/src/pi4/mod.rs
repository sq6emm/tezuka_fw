// Copied from sdroxide (crates/sdroxide-digi/src/pi4, rev 77c8ee3d, GPL-3.0-or-later,
// same author) so trxd does not pull in all of sdroxide-digi. Keep in sync by
// re-copying; local changes: decode.rs FREQ_RADIUS_HZ 300 (was 150).
//! PI4 — the "Next Generation Beacon" digital mode.
//!
//! 4-FSK, 146 symbols at 166.667 ms each (24.333 s), rate-1/2 K=32
//! convolutionally coded and interleaved, carrying up to eight characters —
//! ordinarily a beacon's callsign. Explicitly built as a JT4 derivative for
//! the IARU Region 1 VHF Committee's one-minute mixed-mode beacon sequence
//! (PI4, then CW identification, then an unmodulated carrier), and it shows:
//! the FEC is the exact Layland-Lushbaugh rate-1/2 K=32 code WSPR and JT9
//! both use, just with a shorter message.
//!
//! ## What is here and what is not
//!
//! [`spec`] is the protocol itself — the sync vector, the message alphabet
//! and its base-38 packing, and the FEC wired up over `mfsk-core`'s generic
//! conv-code primitives rather than a re-implementation of them (see that
//! module's doc comment for why, and for the worked-example regression test
//! that keeps it honest against the spec). [`demod`] is the audio-domain
//! tone measurement — an FFT-resolution coarse scan plus an exact-frequency
//! Goertzel refinement. [`decode`] is the search that ties them together:
//! start time × beacon variant × tone-0 frequency, scored cheaply by sync
//! agreement before anything pays for a real Fano attempt, and gated by
//! [`decode::fit_of`] against the FEC's lack of a CRC — see that module's doc
//! comment.
//!
//! Upstream this is decode-only; trxd's beacon transmitter also uses
//! [`spec::encode_reference`] to turn a callsign into the 146 tone indices.

pub mod decode;
pub mod demod;
pub mod spec;

pub use decode::decode_window;
pub use spec::Variant;
