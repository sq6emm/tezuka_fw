//! Cut continuous, timestamped audio into UTC-aligned T/R periods.
//!
//! A [`SlotRecorder`] for period `P` hands out, once per period, the audio
//! from `lead` seconds before the slot boundary up to `ready_at` seconds after
//! it, plus where the boundary falls inside that buffer. Decoders time-search
//! around the boundary, so a slot that started a little late or a clock that
//! is a little off still decodes; a gap in the audio (dropped blocks) is
//! filled with silence rather than shifting everything after it.

use std::collections::VecDeque;

pub struct Slot {
    /// UTC (Unix seconds) of the slot boundary.
    pub utc: i64,
    pub audio: Vec<f32>,
    /// Index of the boundary inside `audio`.
    pub boundary: usize,
}

pub struct SlotRecorder {
    rate: f64,
    period: i64,
    lead: f64,
    ready_at: f64,
    /// UTC of `buf[0]`.
    buf_t0: f64,
    buf: VecDeque<f32>,
    /// Next slot boundary to emit.
    next: Option<i64>,
}

impl SlotRecorder {
    /// `ready_at`: seconds after the boundary at which the slot is complete
    /// enough to decode (Q65-60: 59).
    pub fn new(rate: f64, period_s: i64, lead_s: f64, ready_at_s: f64) -> Self {
        SlotRecorder {
            rate,
            period: period_s,
            lead: lead_s,
            ready_at: ready_at_s,
            buf_t0: 0.0,
            buf: VecDeque::new(),
            next: None,
        }
    }

    pub fn period(&self) -> i64 {
        self.period
    }

    /// Drop everything buffered; the next push starts a fresh recording
    /// (call when the audio stops for a while, e.g. entering DATV).
    pub fn reset(&mut self) {
        self.buf.clear();
        self.next = None;
    }

    /// Append `audio` whose first sample is at UTC `t0`; return any slots
    /// that became complete.
    pub fn push(&mut self, t0: f64, audio: &[f32]) -> Vec<Slot> {
        if self.buf.is_empty() {
            self.buf_t0 = t0;
            // Audio resumes after the next slot's lead-in began (a gap, or
            // everything was drained): that slot cannot be whole any more.
            if self.next.is_some_and(|n| t0 > n as f64 - self.lead) {
                self.next = None;
            }
        } else {
            let expected = self.buf_t0 + self.buf.len() as f64 / self.rate;
            let gap = ((t0 - expected) * self.rate).round() as i64;
            // A gap longer than a whole slot (receiver busy elsewhere, clock
            // stepped forward) is not filled: that would allocate minutes or
            // hours of zeros and emit a run of empty slots. Start over.
            let max_fill = ((self.period as f64 + self.lead) * self.rate) as i64;
            if gap > max_fill {
                self.reset();
                self.buf_t0 = t0;
            } else if gap > 0 {
                self.buf.extend(std::iter::repeat_n(0.0, gap as usize));
            } else if gap < -(self.rate as i64) {
                // Time went backwards by more than a second: start over.
                self.buf.clear();
                self.buf_t0 = t0;
                self.next = None;
            }
        }
        self.buf.extend(audio.iter().copied());
        let now = self.buf_t0 + self.buf.len() as f64 / self.rate;

        let next = *self.next.get_or_insert_with(|| {
            // First boundary we can fully cover, lead-in included.
            ((self.buf_t0 + self.lead) / self.period as f64).ceil() as i64 * self.period
        });

        let mut out = Vec::new();
        let mut slot = next;
        while now >= slot as f64 + self.ready_at {
            let start_t = slot as f64 - self.lead;
            let start = ((start_t - self.buf_t0) * self.rate).round().max(0.0) as usize;
            let end = (((slot as f64 + self.ready_at) - self.buf_t0) * self.rate).round() as usize;
            let end = end.min(self.buf.len());
            if start < end {
                let audio: Vec<f32> = self.buf.range(start..end).copied().collect();
                let boundary = ((slot as f64 - self.buf_t0) * self.rate).round() as usize - start;
                out.push(Slot { utc: slot, audio, boundary });
            }
            slot += self.period;
        }
        self.next = Some(slot);

        // Keep only what the next slot's lead-in can still need.
        let keep_from = slot as f64 - self.lead - 1.0;
        let drop = ((keep_from - self.buf_t0) * self.rate).floor();
        if drop > 0.0 {
            let n = (drop as usize).min(self.buf.len());
            self.buf.drain(..n);
            self.buf_t0 += n as f64 / self.rate;
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn emits_aligned_slots_with_lead_in() {
        let rate = 100.0;
        let mut r = SlotRecorder::new(rate, 15, 1.0, 14.0);
        // Audio sample value = its own UTC time, so alignment is checkable.
        let mut t = 1000.0; // not a multiple of 15
        let mut slots = Vec::new();
        for _ in 0..500 {
            let chunk: Vec<f32> = (0..10).map(|i| (t + i as f64 / rate) as f32).collect();
            slots.extend(r.push(t, &chunk));
            t += 0.1;
        }
        // Boundaries 1005 (needs lead from 1004), 1020, 1035 are complete by 1050.
        let utcs: Vec<i64> = slots.iter().map(|s| s.utc).collect();
        assert_eq!(utcs, vec![1005, 1020, 1035]);
        for s in &slots {
            assert_eq!(s.boundary, 100);
            assert!((s.audio[s.boundary] - s.utc as f32).abs() < 0.02);
            assert_eq!(s.audio.len(), 1500);
        }
    }

    #[test]
    fn gaps_are_filled_not_collapsed() {
        let rate = 100.0;
        let mut r = SlotRecorder::new(rate, 10, 0.0, 10.0);
        let mut slots = Vec::new();
        slots.extend(r.push(100.0, &vec![1.0; 300])); // 100..103
        slots.extend(r.push(105.0, &vec![2.0; 600])); // 105..111, 2 s missing
        assert_eq!(slots.len(), 1);
        let a = &slots[0].audio;
        assert_eq!(a.len(), 1000);
        assert_eq!(a[250], 1.0);
        assert_eq!(a[400], 0.0);
        assert_eq!(a[600], 2.0);
    }

    #[test]
    fn long_gap_restarts_instead_of_filling() {
        let rate = 100.0;
        let mut r = SlotRecorder::new(rate, 10, 1.0, 10.0);
        assert!(r.push(100.0, &vec![1.0; 300]).is_empty());
        // An hour later: no zero fill, no burst of empty slots.
        let slots = r.push(3700.0, &vec![2.0; 300]);
        assert!(slots.is_empty());
        assert!(r.buf.len() <= 300);
        let mut slots = Vec::new();
        let mut t = 3703.0;
        for _ in 0..20 {
            slots.extend(r.push(t, &vec![3.0; 100]));
            t += 1.0;
        }
        assert_eq!(slots.iter().map(|s| s.utc).collect::<Vec<_>>(), vec![3710]);
        assert!(slots[0].audio.iter().all(|&x| x != 0.0));
    }

    #[test]
    fn reset_forgets_the_buffer() {
        let mut r = SlotRecorder::new(100.0, 10, 0.0, 10.0);
        r.push(100.0, &vec![1.0; 300]);
        r.reset();
        assert!(r.buf.is_empty() && r.next.is_none());
    }
}
