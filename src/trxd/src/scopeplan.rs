//! Where the web scope's picture comes from, decided in one place from what
//! it has to show (the view: centre and span) and where the radio is (LO,
//! rates) - never from the mode. The modes only say where the LO may go
//! (trx.rs `retune`); this module says what the view needs from it.
//!
//! Sources, narrowest first:
//! * the channel (48 kS/s around the receive frequency, the narrow views);
//! * the stream (the FPGA's x8 decimated 384 kS/s around the LO);
//! * Maia's spectrometer (the full ADC rate around the LO, 4096 bins), with
//!   the AD936x's analog RX filter opened over the view (its default, about
//!   1 MHz, hid everything further than 500 kHz from the LO in the wide
//!   views);
//! * a sweep: the LO stepped across a view wider than one look, Maia's rows
//!   stitched into one line (no audio, no transmitting meanwhile).
//!
//! Each source is taken only where it covers the whole view; a view the
//! LO's placement leaves uncovered moves the LO (`lo_window`) or falls to the
//! next source - a gap in the picture is never the answer.

/// Up to this span the channel (48 kS/s) can serve the scope.
pub const NARROW_SPAN_MAX: f64 = 20_000.0;
/// Up to this the stream's FFT serves it (finer than Maia's 750 Hz bins).
pub const STREAM_SPAN_MAX: f64 = 300_000.0;
/// The widest view on offer: the FM broadcast band around its middle,
/// +/-10 MHz (one look on the wide LibreSDR image at 24.576 MS/s, a sweep
/// where the converter runs at 3.072 MS/s).
pub const SWEEP_SPAN_MAX: f64 = 20_000_000.0;
/// Room left at the edges of one Maia look (the AD936x's own decimation
/// filters roll off towards the Nyquist edge).
const MAIA_USE: f64 = 0.45;

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Source {
    Channel,
    Stream,
    /// One Maia look; `rf_bw`: the analog RX filter that covers the view.
    Maia { rf_bw: f64 },
    Sweep,
}

impl Source {
    pub fn name(self) -> &'static str {
        match self {
            Source::Channel => "channel",
            Source::Stream => "stream",
            Source::Maia { .. } => "maia",
            Source::Sweep => "sweep",
        }
    }
}

/// The radio as the scope sees it.
#[derive(Debug, Clone, Copy)]
pub struct Radio {
    /// The LO (on the air, transverters included).
    pub lo: f64,
    /// The stream around it (384 kS/s).
    pub stream_rate: f64,
    /// The channel's centre (the receive frequency) and rate.
    pub chan_center: f64,
    pub chan_rate: f64,
    pub adc_rate: f64,
    /// Maia's spectrometer is in the bitstream.
    pub maia: bool,
    /// A sweep may move the LO now (not transmitting, no DATV receiver).
    pub sweep_ok: bool,
}

impl Radio {
    /// How far from the LO one Maia look reaches.
    pub fn maia_half(&self) -> f64 {
        self.adc_rate * MAIA_USE
    }
}

/// The widest span this radio can show.
pub fn span_max(r: &Radio) -> f64 {
    if r.maia { SWEEP_SPAN_MAX } else { r.stream_rate }
}

/// The source for a view centred on `c`, `span` wide.
pub fn source(c: f64, span: f64, r: &Radio) -> Source {
    let half = span / 2.0;
    if span <= NARROW_SPAN_MAX && (c - r.chan_center).abs() + half <= r.chan_rate * 0.45 {
        return Source::Channel;
    }
    let reach = (c - r.lo).abs() + half;
    if !r.maia || (span <= STREAM_SPAN_MAX && reach <= r.stream_rate / 2.0) {
        return Source::Stream;
    }
    // One look whenever one can show the view, the LO where it is or not
    // (lo_window brings it over; a sweep that held the LO kept on sweeping
    // a view one look could take). A sweep only for wider views.
    if half <= r.maia_half() || !r.sweep_ok {
        return Source::Maia { rf_bw: rf_bw_for(reach.min(r.maia_half()), r) };
    }
    Source::Sweep
}

/// Whether `src` shows all of the view (a sweep always does).
pub fn covers(c: f64, span: f64, r: &Radio, src: Source) -> bool {
    let (half, reach) = (span / 2.0, (c - r.lo).abs() + span / 2.0);
    match src {
        Source::Channel => (c - r.chan_center).abs() + half <= r.chan_rate * 0.45,
        Source::Stream => reach <= r.stream_rate / 2.0,
        Source::Maia { .. } => reach <= r.maia_half(),
        Source::Sweep => true,
    }
}

/// The analog RX filter for a Maia look reaching `reach` from the LO, in
/// 100 kHz steps (each change is a driver call) with a margin.
fn rf_bw_for(reach: f64, r: &Radio) -> f64 {
    let want = (2.0 * reach * 1.05 / 100e3).ceil() * 100e3;
    want.min(2.0 * r.maia_half())
}

/// Where the LO has to be for the view's preferred source: within `max_off`
/// of `center`. None: anywhere (the channel follows the receive frequency;
/// a sweep moves the LO itself).
pub fn lo_window(c: f64, span: f64, r: &Radio) -> Option<(f64, f64)> {
    let half = span / 2.0;
    if span <= NARROW_SPAN_MAX && (c - r.chan_center).abs() + half <= r.chan_rate * 0.45 {
        None
    } else if span <= STREAM_SPAN_MAX || !r.maia {
        Some((c, (r.stream_rate / 2.0 - half).max(0.0)))
    } else if half <= r.maia_half() {
        Some((c, r.maia_half() - half))
    } else {
        None
    }
}

/// The LOs of a sweep over the view, from the low end up, each look using
/// `MAIA_USE` of the ADC rate either side, kept inside `[fmin, fmax]`.
pub fn sweep_los(c: f64, span: f64, adc_rate: f64, fmin: f64, fmax: f64) -> Vec<f64> {
    let step = 2.0 * adc_rate * MAIA_USE;
    let (lo_end, hi_end) = (c - span / 2.0, c + span / 2.0);
    let n = (span / step).ceil().max(1.0) as usize;
    (0..n).map(|k| (lo_end + step * (k as f64 + 0.5)).clamp(fmin, fmax)).filter(|f| *f - step / 2.0 < hi_end).collect()
}

/// One sweep line being put together: a fixed-resolution row over the view
/// (Maia's bin width), filled look by look.
pub struct Stitch {
    pub center: f64,
    pub span: f64,
    bin: f64,
    row: Vec<f32>,
}

impl Stitch {
    pub fn new(center: f64, span: f64, adc_rate: f64, bins: usize) -> Self {
        let bin = adc_rate / bins as f64;
        let n = (span / bin).ceil() as usize + 1;
        Stitch { center, span, bin, row: vec![0.0; n] }
    }

    /// One Maia row (`bins` over the ADC rate around `lo`, DC at the
    /// middle): the part within `MAIA_USE` of the LO goes in; the bins next
    /// to DC (the LO leak) are left to the neighbouring look's or bridged.
    pub fn add(&mut self, lo: f64, maia: &[f32], adc_rate: f64) {
        let n = maia.len();
        let bin = adc_rate / n as f64;
        let start = self.center - self.span / 2.0;
        let use_hz = adc_rate * MAIA_USE;
        for (k, &p) in maia.iter().enumerate() {
            let off = (k as f64 - n as f64 / 2.0) * bin;
            if off.abs() > use_hz || off.abs() < 2.5 * bin {
                continue;
            }
            let at = ((lo + off - start) / self.bin).round();
            if at >= 0.0 && (at as usize) < self.row.len() {
                self.row[at as usize] = p;
            }
        }
    }

    /// The row: empty bins (the DC bins, an edge) take their left neighbour.
    pub fn finish(mut self) -> Vec<f32> {
        for k in 1..self.row.len() {
            if self.row[k] == 0.0 {
                self.row[k] = self.row[k - 1];
            }
        }
        self.row
    }

    /// The row's own rate (Hz covered), for `scope::render`.
    pub fn rate(&self) -> f64 {
        self.bin * self.row.len() as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn radio(lo: f64, vfo: f64) -> Radio {
        Radio { lo, stream_rate: 384_000.0, chan_center: vfo, chan_rate: 48_000.0, adc_rate: 3_072_000.0, maia: true, sweep_ok: true }
    }

    /// The view the source draws from covers the whole view.
    fn covered(c: f64, span: f64, r: &Radio) -> bool {
        let reach = (c - r.lo).abs() + span / 2.0;
        match source(c, span, r) {
            Source::Channel => (c - r.chan_center).abs() + span / 2.0 <= r.chan_rate / 2.0,
            Source::Stream => reach <= r.stream_rate / 2.0,
            Source::Maia { rf_bw } => reach <= r.maia_half() && rf_bw / 2.0 >= reach,
            Source::Sweep => true,
        }
    }

    #[test]
    fn narrow_views_use_the_channel_wider_the_stream() {
        let r = radio(144_275_000.0, 144_300_000.0);
        assert_eq!(source(144_300_000.0, 10_000.0, &r), Source::Channel);
        assert_eq!(source(144_300_000.0, 100_000.0, &r), Source::Stream);
        assert_eq!(source(144_300_000.0, 250_000.0, &r), Source::Stream);
    }

    #[test]
    fn a_view_the_stream_misses_takes_maia() {
        // (the grey band at +/-125k: LO parked 101 kHz below for a +/-50k view)
        let r = radio(144_199_200.0, 144_300_000.0);
        assert!(matches!(source(144_300_000.0, 250_000.0, &r), Source::Maia { .. }));
        // DATV S2 at 500 kS/s: the LO 347 kHz below the signal
        let r = radio(143_826_500.0, 144_174_000.0);
        assert!(matches!(source(144_174_000.0, 250_000.0, &r), Source::Maia { .. }));
    }

    #[test]
    fn wide_views_open_the_analog_filter_over_them() {
        // FM broadcast at +/-1.25 MHz, the LO 25 kHz below the station
        let r = radio(99_975_000.0, 100_000_000.0);
        match source(100_000_000.0, 2_500_000.0, &r) {
            Source::Maia { rf_bw } => assert!(rf_bw >= 2.0 * 1_275_000.0 && rf_bw <= 2.0 * r.maia_half(), "{rf_bw}"),
            s => panic!("{s:?}"),
        }
    }

    #[test]
    fn wider_than_one_look_sweeps_or_does_its_best() {
        let r = radio(99_975_000.0, 100_000_000.0);
        assert_eq!(source(97_750_000.0, 20_500_000.0, &r), Source::Sweep);
        let busy = Radio { sweep_ok: false, ..r };
        assert!(matches!(source(97_750_000.0, 20_500_000.0, &busy), Source::Maia { .. }));
    }

    #[test]
    fn every_view_is_covered_where_the_lo_window_puts_the_lo() {
        let vfo = 144_300_000.0;
        for span in [5e3, 10e3, 25e3, 50e3, 100e3, 250e3, 500e3, 1e6, 2.5e6] {
            for dc in [-40e3, 0.0, 30e3] {
                let c = vfo + dc;
                let r = radio(vfo - 25e3, vfo);
                if let Some((w, max_off)) = lo_window(c, span, &r) {
                    for lo in [w - max_off, w, w + max_off] {
                        let r = radio(lo, vfo);
                        assert!(covered(c, span, &r), "span {span} view {c} lo {lo}: {:?}", source(c, span, &r));
                    }
                }
            }
        }
    }

    #[test]
    fn a_view_one_look_can_take_never_sweeps() {
        // (clicking a station in a swept view: +/-1.25 MHz around it, the LO
        // still 12 MHz away where the sweep left it)
        let r = radio(99_975_000.0, 87_534_000.0);
        assert!(matches!(source(87_534_000.0, 2_500_000.0, &r), Source::Maia { .. }));
        assert!(lo_window(87_534_000.0, 2_500_000.0, &r).is_some());
    }

    #[test]
    fn without_maia_the_stream_is_all_there_is() {
        let r = Radio { maia: false, ..radio(100e6, 100e6) };
        assert_eq!(source(100e6, 2.5e6, &r), Source::Stream);
        assert_eq!(span_max(&r), 384_000.0);
    }

    #[test]
    fn a_sweep_covers_the_fm_band() {
        let (c, span) = (97_750_000.0, 20_500_000.0);
        let los = sweep_los(c, span, 3_072_000.0, 47e6, 6e9);
        let step = 2.0 * 3_072_000.0 * MAIA_USE;
        assert_eq!(los.len(), (span / step).ceil() as usize);
        assert!(los[0] - step / 2.0 <= c - span / 2.0 + 1.0);
        assert!(los.last().unwrap() + step / 2.0 >= c + span / 2.0 - 1.0);
        for w in los.windows(2) {
            assert!((w[1] - w[0] - step).abs() < 1.0);
        }
    }

    #[test]
    fn stitched_rows_put_each_look_where_it_belongs() {
        let adc = 3_072_000.0;
        let (c, span) = (100e6, 5e6);
        let mut st = Stitch::new(c, span, adc, 4096);
        for lo in sweep_los(c, span, adc, 47e6, 6e9) {
            // a flat floor with a tone 300 kHz above each LO
            let mut row = vec![1e-9f32; 4096];
            row[2048 + (300e3 / (adc / 4096.0)) as usize] = 1.0;
            st.add(lo, &row, adc);
        }
        let rate = st.rate();
        let row = st.finish();
        assert!(row.iter().all(|&p| p > 0.0), "no gaps");
        let tones: Vec<f64> = row.iter().enumerate().filter(|(_, p)| **p > 0.5).map(|(k, _)| c - span / 2.0 + k as f64 * rate / row.len() as f64).collect();
        assert_eq!(tones.len(), 2);
        for (t, lo) in tones.iter().zip(sweep_los(c, span, adc, 47e6, 6e9)) {
            assert!((t - (lo + 300e3)).abs() < 1_000.0, "{t} vs {lo}");
        }
    }
}
