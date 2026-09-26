//! TCI transmit pacing: how much TX audio to ask a keying client for.
//!
//! Ported from sdroxide-radio's `TciTxPace` (same author; see its doc there
//! for the full reasoning, issue #202). The demand is the transmit clock, not
//! the queue: the running total asked for is held at "everything the
//! transmitter has consumed, plus a lead", so a client that falls behind is
//! asked for more rather than less, and one that honours chronos can never
//! overrun the FIFO.

/// Frames kept requested ahead of the transmit clock (100 ms at 48 kHz).
pub const LEAD: usize = 4_800;
/// Consecutive blocks of silence from the client before what it still owes is
/// written off and asked for again.
const ASK_TIMEOUT_BLOCKS: u32 = 50;

#[derive(Debug, Clone, Copy, Default)]
pub struct TxPace {
    asked: usize,
    supplied: usize,
    played: usize,
    quiet: u32,
}

impl TxPace {
    /// One block of `block` frames: `queued` in hand, `got` arrived since the
    /// last call, `playing` whether this block went on the air. Returns the
    /// frame count to chrono for, if any.
    pub fn request(&mut self, block: usize, queued: usize, got: usize, playing: bool) -> Option<u32> {
        self.supplied += got;
        if playing {
            self.played += block;
        }
        self.quiet = if got > 0 { 0 } else { self.quiet + 1 };
        if self.quiet >= ASK_TIMEOUT_BLOCKS {
            self.quiet = 0;
            self.asked = self.supplied;
        }
        if queued >= LEAD * 2 {
            return None;
        }
        let deficit = (self.played + LEAD).saturating_sub(self.asked);
        if deficit < block {
            return None;
        }
        self.asked += deficit;
        Some(deficit as u32)
    }

    pub fn rekey(&mut self) {
        *self = TxPace::default();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn asks_for_the_lead_then_tracks_real_time() {
        let mut p = TxPace::default();
        assert_eq!(p.request(480, 0, 0, false), Some(LEAD as u32));
        assert_eq!(p.request(480, 0, 0, false), None);
        // The client answers in full and each block plays: ask one block per block.
        assert_eq!(p.request(480, LEAD, LEAD, true), Some(480));
        assert_eq!(p.request(480, LEAD, 480, true), Some(480));
    }

    #[test]
    fn a_slow_client_is_asked_for_more_not_less() {
        let mut p = TxPace::default();
        p.request(480, 0, 0, false);
        let mut asked = 0;
        for _ in 0..10 {
            // Client delivers nothing; the transmitter still consumes.
            asked += p.request(480, 0, 0, true).unwrap_or(0);
        }
        assert_eq!(asked as usize, 10 * 480);
    }
}
