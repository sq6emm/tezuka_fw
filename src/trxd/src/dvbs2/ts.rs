//! MPEG-TS for DATV at a few tens of kbit/s: one program, H.264 video and
//! Opus audio from the browser (WebCodecs), multiplexed at the constant rate
//! the DVB-S2 modulator pulls packets at.
//!
//! The modulator asks for one packet at a time ([`Mux::next`]), so every
//! packet leaves at a known moment of the constant-rate stream: PCRs are
//! stamped then, exactly. Every video packet reserves an adaptation field
//! for a PCR, filled when one is due as it leaves; when a PCR is due and the
//! next packet is not video (or nothing is queued), a PCR-only packet goes
//! first. Idle slots otherwise carry null packets.
//!
//! At these rates the 188-byte packet is the unit of cost: audio is packed
//! ~200 ms to a PES (one 20 ms Opus frame per PES would take 50 packets a
//! second). Repetition as ISO/IEC 13818-1 and TR 101 290 ask: PCR at least
//! within 100 ms (13818-1's limit; 90 ms below 200 kbit/s, where TR 101
//! 290's recommended 40 ms would cost a third of the packets) and every
//! 40 ms from 200 kbit/s up, PAT and PMT every 0.5 s, SDT within 2 s. The
//! lean profile below [`LEAN_BELOW_BPS`] (where even 3 packets take over
//! 100 ms) is a deliberate exception: tables every second, a PCR every
//! 0.5 s, PTS up to 1.4 s after the data arrives (13818-1's T-STD limit,
//! 1 s, is kept from there up).

use std::collections::VecDeque;

use super::{TS_LEN, null_packet};

const PID_PMT: u16 = 0x1000;
const PID_NIT: u16 = 0x0010;
const PID_SDT: u16 = 0x0011;
const PID_EIT: u16 = 0x0012;
const PID_TDT: u16 = 0x0014;
/// DVB SI repetition (EN 300 468 / TS 101 211: NIT within 10 s, EIT
/// present/following within 2 s, TDT within 30 s), seconds.
const NIT_EVERY_S: f64 = 9.5;
// (1.7: PCRs and PAT/PMT go first, at the lowest rates a slot each)
const EIT_EVERY_S: f64 = 1.7;
const TDT_EVERY_S: f64 = 25.0;
const NETWORK_NAME: &[u8] = b"SQTRX DATV";
/// Index of each table's version in [`Mux::versions`].
const T_PAT: usize = 0;
const T_PMT: usize = 1;
const T_SDT: usize = 2;
const T_NIT: usize = 3;
const T_EIT: usize = 4;
/// Every PES must be in by its PTS (T-STD); this much to spare.
const PTS_MARGIN_S: f64 = 0.05;
/// A keyframe at least this often (the browser is asked for one).
const KEY_EVERY_S: f64 = 2.0;
pub const PID_VIDEO: u16 = 0x0100;
pub const PID_AUDIO: u16 = 0x0101;
/// 27 MHz system clock.
const HZ27: f64 = 27_000_000.0;
/// PTS = arrival on the mux clock + this (plus the audio PES length), so the
/// receiver's buffer never runs dry.
const DELAY_S: f64 = 0.6;
/// ... but never more than this: data spends at most 1 s in the T-STD
/// buffers (ISO/IEC 13818-1 2.4.2.6).
const DELAY_MAX_S: f64 = 1.0;
/// Below this TS rate 3 packets (the least a PCR interval may be, so data
/// moves between PCRs) last over 100 ms: the lean profile.
pub const LEAN_BELOW_BPS: f64 = 30.0 * TS_LEN as f64 * 8.0;
/// Two sections of a table on one PID at least this far apart (TS 101 211
/// 4.4: 25 ms between sections of the same table_id_extension).
const SECTION_GAP_S: f64 = 0.025;
/// Service Description Table: the latest the SDT may follow the previous one
/// (EN 300 468: at most 2 s).
const SDT_MAX_S: f64 = 1.9;
/// original_network_id: from the private range (0xFF01-0xFFFF, ETSI TS 101
/// 162); 0x0001 is a registered satellite network.
const ONID: u16 = 0xFF01;
/// service_type: H.264 SD digital television (EN 300 468 table 87).
const SERVICE_TYPE: u8 = 0x16;
/// Payload bytes of a video packet (the adaptation field for a PCR takes 8).
const VIDEO_PAYLOAD: f64 = 176.0;
/// What the browser sends, and how it is packed, for a TS rate. Below
/// [`LEAN_BELOW_BPS`] the 188-byte packet overhead dominates: small
/// pictures, fewer of them, lean audio packed 800 ms to a PES, tables every
/// second.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Profile {
    pub width: u32,
    pub height: u32,
    pub fps: f64,
    pub audio_bps: f64,
    /// 20 ms Opus frames per audio PES.
    pub audio_per_pes: usize,
    /// PAT + PMT this often, seconds (the SDT within [`SDT_MAX_S`]).
    pub psi_every_s: f64,
    /// A PCR at least this often, seconds.
    pub pcr_every_s: f64,
    /// PTS at most this long after the data arrives, seconds: 1 s
    /// (13818-1's T-STD limit) except in the lean profile.
    pub tstd_max_s: f64,
}

impl Profile {
    pub fn for_rate(ts_rate: f64) -> Profile {
        if ts_rate < LEAN_BELOW_BPS {
            // A deliberate exception: at 15-30 packets a second a PCR every
            // 100 ms (13818-1's limit) would leave the picture almost nothing.
            // Tables every second, a PCR every 0.5 s; only MiniTiouner-class
            // receivers work at these rates anyway. Audio 800 ms to a PES,
            // and so PTS up to 1.4 s after the data (13818-1's T-STD allows
            // 1 s): with 1 s the audio PES (4 packets at 15-30 a second)
            // and large pictures miss their PTS.
            Profile { width: 160, height: 120, fps: 2.0, audio_bps: 6_000.0, audio_per_pes: 40, psi_every_s: 1.0, pcr_every_s: 0.5, tstd_max_s: 1.4 }
        } else if ts_rate < 80_000.0 {
            // (audio 8 kbit/s, 300 ms to a PES: 12 kbit/s in 200 ms PES took
            // a third of the packets at 46 kbit/s and left no picture)
            Profile { width: 320, height: 240, fps: 5.0, audio_bps: 8_000.0, audio_per_pes: 15, psi_every_s: 0.5, pcr_every_s: 0.09, tstd_max_s: DELAY_MAX_S }
        } else if ts_rate < 200_000.0 {
            Profile { width: 320, height: 240, fps: 10.0, audio_bps: 16_000.0, audio_per_pes: 10, psi_every_s: 0.5, pcr_every_s: 0.09, tstd_max_s: DELAY_MAX_S }
        } else {
            // 192 kS/s and up at 2/3-3/4 (FPGA front end): 240-360 kbit/s.
            Profile { width: 640, height: 480, fps: 10.0, audio_bps: 24_000.0, audio_per_pes: 10, psi_every_s: 0.5, pcr_every_s: 0.04, tstd_max_s: DELAY_MAX_S }
        }
    }

    /// PSI bursts per SDT (the SDT rides along every this many bursts).
    fn sdt_rounds(&self) -> u64 {
        ((SDT_MAX_S / self.psi_every_s).floor() as u64).max(1)
    }

    /// Packets a second for tables and for PCR-only packets.
    fn overhead_pps(&self, pps: f64) -> (f64, f64) {
        let psi = (2.0 + 1.0 / self.sdt_rounds() as f64) / self.psi_every_s + 2.0 / EIT_EVERY_S + 1.0 / NIT_EVERY_S + 1.0 / TDT_EVERY_S;
        // PCR-only packets while audio or tables hold the queue: about half
        // of the PCRs at the interval the mux keeps (whole slots, at least
        // 3), three quarters below 40 packets a second with PCRs that often
        // (audio and tables are then most of the slots; measured 0.7 at 46
        // kbit/s).
        let pcr_rate = pps / (self.pcr_every_s * pps).floor().max(3.0);
        (psi, pcr_rate * if pps < 40.0 && self.pcr_every_s < 0.2 { 0.75 } else { 0.5 })
    }

    /// Audio packets a second: an audio PES has 3 control bytes per frame
    /// and 14 header bytes.
    fn audio_pps(&self) -> f64 {
        let pes_per_s = 50.0 / self.audio_per_pes as f64;
        let audio_bytes = self.audio_bps / 8.0 / pes_per_s + (3 * self.audio_per_pes + 14) as f64;
        pes_per_s * (audio_bytes / 184.0).ceil()
    }

    /// Video bit rate left for the browser (after PSI, audio and per-frame
    /// packetization), with a margin.
    pub fn video_budget(&self, ts_rate: f64) -> f64 {
        let pps = ts_rate / (TS_LEN as f64 * 8.0);
        let (psi, pcr_only) = self.overhead_pps(pps);
        let audio = self.audio_pps();
        // Video packets carry 176 bytes (8 reserved for a PCR); a frame
        // also costs its PES header, AUD and half a packet of padding.
        let video_bytes_s = ((pps - psi - audio - pcr_only) * VIDEO_PAYLOAD - self.fps * (14.0 + 6.0 + VIDEO_PAYLOAD / 2.0)).max(0.0);
        video_bytes_s * 8.0 * 0.85
    }
}
/// Video backlog, seconds of the stream: above it non-key frames are dropped
/// (and a keyframe asked for); above the hard limit everything queued goes.
const BACKLOG_S: f64 = 1.0;
const BACKLOG_HARD_S: f64 = 3.0;

/// MPEG-2 CRC-32 (PSI sections): polynomial 0x04C11DB7, MSB first, no reflection.
fn crc32(data: &[u8]) -> u32 {
    let mut c = 0xFFFF_FFFFu32;
    for &b in data {
        c ^= (b as u32) << 24;
        for _ in 0..8 {
            c = if c & 0x8000_0000 != 0 { (c << 1) ^ 0x04C1_1DB7 } else { c << 1 };
        }
    }
    c
}

/// A queued packet: the 184 bytes after the header are final; the header's
/// continuity counter (and a reserved PCR) are filled in when it leaves.
struct Queued {
    pid: u16,
    pkt: [u8; TS_LEN],
    /// Adaptation field reserved for a PCR at bytes 6..12 (every video
    /// packet): the PCR flag and value are set when one is due as it leaves.
    pcr: bool,
}

/// What the browser hands over.
pub enum Media {
    /// H.264 Annex B access unit; `ts_us` is the capture time (browser clock).
    Video { ts_us: i64, key: bool, data: Vec<u8> },
    /// One Opus packet (20 ms).
    Audio { ts_us: i64, data: Vec<u8> },
}

impl Media {
    /// A browser's binary WebSocket message: `[4][flags: bit 0 = key][i64 LE
    /// capture time, us][H.264 Annex B]` or `[5][i64 LE us][Opus packet]`.
    pub fn from_ws(b: &[u8]) -> Option<Media> {
        match *b.first()? {
            4 if b.len() > 10 => Some(Media::Video {
                key: b[1] & 1 == 1,
                ts_us: i64::from_le_bytes(b[2..10].try_into().ok()?),
                data: b[10..].to_vec(),
            }),
            5 if b.len() > 9 => Some(Media::Audio { ts_us: i64::from_le_bytes(b[1..9].try_into().ok()?), data: b[9..].to_vec() }),
            _ => None,
        }
    }
    fn ts_us(&self) -> i64 {
        match self {
            Media::Video { ts_us, .. } | Media::Audio { ts_us, .. } => *ts_us,
        }
    }
}

/// What the NIT says about the carrier (its delivery system descriptor).
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Delivery {
    /// DVB-S2: satellite_delivery_system_descriptor (0x43).
    S2 { freq_hz: f64, symbol_rate: f64, rolloff: f32, modcod: u8 },
    /// DVB-T2: T2_delivery_system_descriptor (extension 0x04).
    T2 { freq_hz: f64, bw_hz: f64, plp_id: u8, t2_system_id: u16 },
}

/// `v` as `digits` BCD digits (rounded, saturated).
fn bcd(v: f64, digits: u32) -> u32 {
    let mut x = (v.round().max(0.0) as u64).min(10u64.pow(digits) - 1);
    let mut out = 0u32;
    for i in 0..digits {
        out |= ((x % 10) as u32) << (4 * i);
        x /= 10;
    }
    out
}

impl Delivery {
    pub fn descriptor(&self) -> Vec<u8> {
        match *self {
            Delivery::S2 { freq_hz, symbol_rate, rolloff, modcod } => {
                // frequency: 8 BCD digits, GHz with the point after the third
                // (10 kHz units); orbital position 0; polarization linear
                // horizontal; symbol rate: 7 BCD digits, Msym/s with the point
                // after the third (100 sym/s units), then FEC_inner.
                let ro = if rolloff <= 0.21 { 2 } else if rolloff <= 0.26 { 1 } else { 0 };
                let (mtype, fec) = match modcod {
                    1..=11 => (1u8, [0u8, 0, 0, 1, 7, 2, 3, 8, 4, 6, 9][modcod as usize - 1]),
                    12..=17 => (2, [7u8, 2, 3, 4, 6, 9][modcod as usize - 12]),
                    _ => (0, 0),
                };
                let mut d = vec![0x43, 11];
                d.extend_from_slice(&bcd(freq_hz / 1e4, 8).to_be_bytes());
                d.extend_from_slice(&[0x00, 0x00]);
                // west_east 0, polarization 00, roll_off, modulation_system 1 (S2), modulation_type
                d.push((ro << 3) | 0x04 | mtype);
                let sr = bcd(symbol_rate / 100.0, 7);
                d.extend_from_slice(&((sr << 4) | fec as u32).to_be_bytes());
                d
            }
            Delivery::T2 { freq_hz, bw_hz, plp_id, t2_system_id } => {
                let mut d = vec![0x7F, 0, 0x04, plp_id];
                d.extend_from_slice(&t2_system_id.to_be_bytes());
                // Only 1.7 MHz of trxd's channels has a bandwidth code
                // (0101 = 1.712 MHz); the others take the short form.
                if (bw_hz - 1.7e6).abs() < 20e3 {
                    // SISO, bandwidth 0101, GI 1/8 (010), 2K (000), no other
                    // frequency, no TFS; one cell: id 0, centre in 10 Hz units.
                    d.push((0x05 << 2) | 0x03);
                    d.push(0x02 << 5);
                    d.extend_from_slice(&[0x00, 0x00]);
                    d.extend_from_slice(&(((freq_hz / 10.0).round().clamp(0.0, u32::MAX as f64)) as u32).to_be_bytes());
                    d.push(0);
                }
                d[1] = (d.len() - 2) as u8;
                d
            }
        }
    }
}

pub struct Mux {
    pub profile: Profile,
    /// Packets per second the modulator pulls.
    pps: f64,
    /// Packets pulled so far: the mux clock.
    sent: u64,
    cc: [u8; 8],
    /// Video packets waiting (and, in `aqueue`, audio: it goes first, its
    /// PES are few and small and must be in by their PTS).
    queue: VecDeque<Queued>,
    aqueue: VecDeque<Queued>,
    next_psi: u64,
    /// Position in the PAT, PMT (, SDT) burst, and bursts sent.
    psi_step: u8,
    psi_round: u64,
    last_pcr: Option<u64>,
    /// NIT, EIT present/following (two sections) and TDT: next packet
    /// index, and the EIT section being sent.
    next_nit: u64,
    next_eit: u64,
    next_tdt: u64,
    eit_step: u8,
    /// The EIT's second section not before this packet.
    next_eit_section: u64,
    /// The NIT's delivery system descriptor, when the modulator says.
    delivery: Option<Delivery>,
    /// Wall clock (Unix seconds) at packet 0: the TDT and EIT times.
    start_unix: f64,
    /// PAT, PMT, SDT, NIT, EIT: version_number and a hash of the content it
    /// was given for (13818-1 / EN 300 468: the version changes with it).
    versions: [(u8, Option<u64>); 5],
    /// Last PTS given, video and audio: never back.
    last_pts: [Option<u64>; 2],
    /// The latest SPS and PPS (Annex B, start codes included): put before
    /// a keyframe that comes without them.
    sps_pps: Vec<u8>,
    /// Packet index of the last keyframe queued.
    last_key: Option<u64>,
    /// Media time `ts_us` that maps to 90 kHz clock `pts90`.
    anchor: Option<(i64, u64)>,
    audio: Vec<(i64, Vec<u8>)>,
    video_dropping: bool,
    /// The browser should send a keyframe next.
    pub want_key: bool,
    service: String,
    pub dropped_frames: u64,
}

impl Mux {
    /// `ts_rate`: the modulator's TS rate, bit/s. `service`: shown by DVB
    /// receivers (the callsign).
    pub fn new(ts_rate: f64, service: &str) -> Self {
        Mux {
            profile: Profile::for_rate(ts_rate),
            pps: ts_rate / (TS_LEN as f64 * 8.0),
            sent: 0,
            cc: [0; 8],
            queue: VecDeque::new(),
            aqueue: VecDeque::new(),
            next_psi: 0,
            psi_step: 0,
            psi_round: 0,
            last_pcr: None,
            next_nit: 0,
            next_eit: 0,
            next_tdt: 0,
            eit_step: 0,
            next_eit_section: 0,
            delivery: None,
            versions: [(0, None); 5],
            last_pts: [None; 2],
            sps_pps: Vec::new(),
            last_key: None,
            start_unix: std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).map_or(0.0, |d| d.as_secs_f64()),
            anchor: None,
            audio: Vec::new(),
            video_dropping: false,
            want_key: false,
            service: service.chars().filter(|c| c.is_ascii_graphic() || *c == ' ').take(32).collect(),
            dropped_frames: 0,
        }
    }

    /// The NIT names the delivery system (the modulator's settings).
    pub fn with_delivery(mut self, d: Delivery) -> Self {
        self.delivery = Some(d);
        self
    }

    /// The mux clock, 27 MHz ticks, at packet `n`.
    fn clock27(&self, n: u64) -> u64 {
        (n as f64 / self.pps * HZ27) as u64
    }

    /// Queued media, seconds of stream.
    pub fn backlog_s(&self) -> f64 {
        self.queue.len() as f64 / self.pps
    }

    fn pts90(&mut self, ts_us: i64) -> u64 {
        let now90 = self.clock27(self.sent) / 300;
        let delay90 = ((DELAY_S + self.profile.audio_per_pes as f64 * 0.02).min(self.profile.tstd_max_s) * 90_000.0) as u64;
        if let Some((t0, p0)) = self.anchor {
            let pts = p0 as i64 + (ts_us - t0) * 9 / 100;
            // Browser and board clocks drift apart slowly; re-anchor when late or far ahead.
            if pts > (now90 + delay90 / 4) as i64 && pts < (now90 + 3 * delay90) as i64 {
                return pts as u64;
            }
        }
        self.anchor = Some((ts_us, now90 + delay90));
        now90 + delay90
    }

    /// A stream's PTS, never before the last one given (a re-anchored
    /// clock would otherwise step it back): 1/90000 s later at least.
    fn monotonic(&mut self, k: usize, pts: u64) -> u64 {
        let p = self.last_pts[k].map_or(pts, |l| pts.max(l + 1));
        self.last_pts[k] = Some(p);
        p
    }

    /// When everything queued now has gone out, on the 90 kHz clock (the
    /// PCR-only, table and audio packets between counted, with a tenth to
    /// spare).
    fn done_at_90(&self) -> u64 {
        // (audio goes first: what is queued, and what comes meanwhile)
        let (psi, pcr_only) = self.profile.overhead_pps(self.pps);
        let share = (1.0 - (psi + pcr_only + self.profile.audio_pps()) / self.pps).max(0.2);
        let n = self.sent as f64 + self.aqueue.len() as f64 + self.queue.len() as f64 * 1.1 / share;
        (n / self.pps * 90_000.0) as u64
    }

    /// Would the queue's last PES come in after `pts` (with the margin)?
    fn late(&self, pts: u64) -> bool {
        self.done_at_90() + (PTS_MARGIN_S * 90_000.0) as u64 > pts
    }

    /// Should the browser send a keyframe? After a dropped frame, or when
    /// none came for KEY_EVERY_S (decoders start and recover at
    /// keyframes). Asking resets both, so it is asked once.
    pub fn take_key_request(&mut self) -> bool {
        let overdue = self.last_key.is_some_and(|k| (self.sent - k) as f64 > KEY_EVERY_S * self.pps);
        if overdue {
            self.last_key = Some(self.sent);
        }
        std::mem::take(&mut self.want_key) || overdue
    }

    pub fn push(&mut self, m: Media) {
        match m {
            Media::Video { ts_us, key, data } => {
                if self.backlog_s() > BACKLOG_HARD_S {
                    self.queue.retain(|q| q.pid != PID_VIDEO);
                    self.video_dropping = true;
                }
                if key {
                    self.video_dropping = false;
                } else if self.video_dropping || self.backlog_s() > BACKLOG_S {
                    self.video_dropping = true;
                    self.want_key = true;
                    self.dropped_frames += 1;
                    return;
                }
                let pts = self.pts90(ts_us);
                let pts = self.monotonic(0, pts);
                let mut es = Vec::with_capacity(data.len() + 6);
                // Access unit delimiter first, unless the encoder put one there
                // (after a 3- or 4-byte start code).
                if !(data.starts_with(&[0, 0, 0, 1]) && data.get(4).is_some_and(|b| b & 0x1F == 9) || data.starts_with(&[0, 0, 1]) && data.get(3).is_some_and(|b| b & 0x1F == 9)) {
                    es.extend_from_slice(&[0, 0, 0, 1, 0x09, 0xF0]);
                }
                // A keyframe starts decoding: it needs the SPS and PPS with it.
                let sets = parameter_sets(&data);
                if !sets.is_empty() {
                    self.sps_pps = sets;
                } else if key && !self.sps_pps.is_empty() {
                    es.extend_from_slice(&self.sps_pps.clone());
                }
                es.extend_from_slice(&data);
                let before = self.queue.len();
                self.queue_pes(PID_VIDEO, 0xE0, pts, &es, true);
                // T-STD: the whole PES in before its PTS. Late: a non-key
                // frame goes (a keyframe asked for); a keyframe makes room by
                // dropping the video queued before it, and if even so late
                // takes a later PTS (the timeline moves on, never back).
                if self.late(pts) {
                    if !key {
                        self.queue.truncate(before);
                        self.video_dropping = true;
                        self.want_key = true;
                        self.dropped_frames += 1;
                        return;
                    }
                    let mine: Vec<Queued> = self.queue.drain(before..).collect();
                    let dropped = self.queue.iter().filter(|q| q.pid == PID_VIDEO).count();
                    self.queue.retain(|q| q.pid != PID_VIDEO);
                    self.queue.extend(mine);
                    if dropped > 0 {
                        self.dropped_frames += 1;
                    }
                    if self.late(pts) {
                        let need = self.done_at_90() + (PTS_MARGIN_S * 90_000.0) as u64;
                        // re-stamp: a new anchor so this and what follows are in time
                        let shift = need - pts;
                        if let Some((t0, p0)) = self.anchor {
                            self.anchor = Some((t0, p0 + shift));
                        }
                        let first = self.queue.iter().position(|q| q.pid == PID_VIDEO).unwrap_or(0);
                        let new_pts = pts + shift;
                        let pkt = restamp(&self.queue[first].pkt, new_pts);
                        self.queue[first].pkt = pkt;
                        self.last_pts[0] = Some(new_pts);
                    }
                }
                if key {
                    self.last_key = Some(self.sent);
                }
            }
            Media::Audio { ts_us, data } => {
                self.audio.push((ts_us, data));
                if self.audio.len() >= self.profile.audio_per_pes {
                    self.flush_audio();
                }
            }
        }
    }

    fn flush_audio(&mut self) {
        let frames = std::mem::take(&mut self.audio);
        let Some(&(ts, _)) = frames.first() else { return };
        let pts = self.pts90(ts);
        let pts = self.monotonic(1, pts);
        let mut es = Vec::new();
        for (_, f) in &frames {
            // Opus control header (the Opus-in-MPEG-TS mapping, as ffmpeg writes it).
            es.extend_from_slice(&[0x7F, 0xE0]);
            let mut n = f.len();
            while n >= 255 {
                es.push(0xFF);
                n -= 255;
            }
            es.push(n as u8);
            es.extend_from_slice(f);
        }
        self.queue_pes(PID_AUDIO, 0xBD, pts, &es, false);
    }

    /// Split one PES into queued packets; the last is padded with adaptation
    /// field stuffing. `pcr`: reserve a PCR slot in every packet.
    fn queue_pes(&mut self, pid: u16, stream_id: u8, pts: u64, es: &[u8], pcr: bool) {
        let mut pes = vec![0, 0, 1, stream_id];
        let len = if stream_id == 0xE0 { 0 } else { (es.len() + 8).min(0xFFFF) };
        pes.extend_from_slice(&(len as u16).to_be_bytes());
        pes.extend_from_slice(&[0x84, 0x80, 5]); // data aligned; PTS only
        pes.extend_from_slice(&pts_bytes(pts));
        pes.extend_from_slice(es);
        let mut first = true;
        let mut rest = &pes[..];
        while !rest.is_empty() {
            let af_pcr = pcr;
            let room = 184 - if af_pcr { 8 } else { 0 };
            let n = rest.len().min(room);
            let mut pkt = [0xFFu8; TS_LEN];
            pkt[0] = 0x47;
            pkt[1] = ((first as u8) << 6) | (pid >> 8) as u8;
            pkt[2] = pid as u8;
            // Adaptation field: [length][flags][PCR, filled when the packet
            // leaves][0xFF stuffing]. Without a PCR and one byte short, it is
            // the length byte alone.
            let stuff = room - n;
            let payload_at = if af_pcr || stuff > 0 {
                let af_len = if af_pcr { 7 + stuff } else { stuff - 1 };
                pkt[3] = 0x30;
                pkt[4] = af_len as u8;
                if af_len > 0 {
                    // (flags 0; the PCR flag is set as the packet leaves, else
                    // the 6 bytes after the flags are stuffing)
                    pkt[5] = 0x00;
                }
                5 + af_len
            } else {
                pkt[3] = 0x10;
                4
            };
            pkt[payload_at..payload_at + n].copy_from_slice(&rest[..n]);
            debug_assert_eq!(payload_at + n, TS_LEN);
            rest = &rest[n..];
            let q = Queued { pid, pkt, pcr: af_pcr };
            if pid == PID_AUDIO {
                self.aqueue.push_back(q);
            } else {
                self.queue.push_back(q);
            }
            first = false;
        }
    }

    fn cc_index(pid: u16) -> usize {
        match pid {
            0 => 0,
            PID_PMT => 1,
            PID_SDT => 2,
            PID_VIDEO => 3,
            PID_NIT => 5,
            PID_EIT => 6,
            PID_TDT => 7,
            _ => 4,
        }
    }

    /// The next packet of the constant-rate stream.
    pub fn next(&mut self) -> [u8; TS_LEN] {
        let n = self.sent;
        self.sent += 1;
        let now27 = self.clock27(n);
        // PCR first: due when the profile's interval (in whole slots, never
        // over it) has passed. Never below 3 slots, so data keeps moving
        // between PCRs at the lowest rates (from LEAN_BELOW_BPS up 3 slots
        // are within 13818-1's 100 ms).
        let every = ((self.profile.pcr_every_s * self.pps).floor() as u64).max(3);
        // (the first tables go before anything: a receiver learns the PIDs)
        let pcr_due = self.psi_round > 0 && self.last_pcr.is_none_or(|l| n - l >= every);
        if pcr_due {
            if self.aqueue.is_empty() && self.queue.front().is_some_and(|q| q.pcr) {
                // a video packet: its reserved adaptation field takes the PCR
                let mut q = self.queue.pop_front().expect("front");
                q.pkt[5] |= 0x10;
                q.pkt[6..12].copy_from_slice(&pcr_bytes(now27));
                self.last_pcr = Some(n);
                return self.stamp(q);
            }
            // Adaptation field only, no payload: the counter does not move.
            let mut pkt = [0xFFu8; TS_LEN];
            pkt[..4].copy_from_slice(&[0x47, (PID_VIDEO >> 8) as u8, PID_VIDEO as u8, 0x20 | self.cc[3].wrapping_sub(1) & 0x0F]);
            pkt[4] = 183;
            pkt[5] = 0x10;
            pkt[6..12].copy_from_slice(&pcr_bytes(now27));
            self.last_pcr = Some(n);
            return pkt;
        }
        // Tables: PAT and PMT every psi_every_s (counted from burst start to
        // burst start, two slots early for PCRs that may cut in), the SDT every
        // sdt_rounds bursts.
        if self.psi_step > 0 || n >= self.next_psi {
            let with_sdt = self.psi_round % self.profile.sdt_rounds() == 0;
            if self.psi_step == 0 {
                self.next_psi = n + ((self.profile.psi_every_s * self.pps).floor() as u64).saturating_sub(2).max(1);
            }
            let pkt = match self.psi_step {
                0 => {
                    let sec = self.versioned(T_PAT, self.pat(), true);
                    self.section(0, &sec)
                }
                1 => {
                    let sec = self.versioned(T_PMT, self.pmt(), true);
                    self.section(PID_PMT, &sec)
                }
                _ => {
                    let sec = self.versioned(T_SDT, self.sdt(), true);
                    self.section(PID_SDT, &sec)
                }
            };
            self.psi_step += 1;
            // (the rest of the burst goes in the next slots)
            if self.psi_step == if with_sdt { 3 } else { 2 } {
                self.psi_step = 0;
                self.psi_round += 1;
            }
            return pkt;
        }
        // DVB SI: the EIT's two sections (at least 25 ms apart), the NIT,
        // the TDT.
        if (self.eit_step == 0 && n >= self.next_eit) || (self.eit_step > 0 && n >= self.next_eit_section) {
            if self.eit_step == 0 {
                self.next_eit = n + (EIT_EVERY_S * self.pps) as u64;
                self.next_eit_section = n + (SECTION_GAP_S * self.pps).ceil() as u64;
            }
            // (both sections share the table's version; the present one sets it)
            let sec = self.eit(self.eit_step, n);
            let sec = self.versioned(T_EIT, sec, self.eit_step == 0);
            self.eit_step = (self.eit_step + 1) % 2;
            return self.section(PID_EIT, &sec);
        }
        if n >= self.next_nit {
            self.next_nit = n + (NIT_EVERY_S * self.pps) as u64;
            let sec = self.versioned(T_NIT, self.nit(), true);
            return self.section(PID_NIT, &sec);
        }
        if n >= self.next_tdt {
            self.next_tdt = n + (TDT_EVERY_S * self.pps) as u64;
            let sec = tdt(self.unix(n));
            return self.section(PID_TDT, &sec);
        }
        if let Some(q) = self.aqueue.pop_front().or_else(|| self.queue.pop_front()) {
            return self.stamp(q);
        }
        null_packet()
    }

    /// A finished section with this table's version_number, bumped when the
    /// content (all but the version and the CRC) differs from last time
    /// (`track`: this section decides; else it takes the current version).
    fn versioned(&mut self, t: usize, mut sec: Vec<u8>, track: bool) -> Vec<u8> {
        use std::hash::{Hash, Hasher};
        let n = sec.len();
        let (ver, last) = &mut self.versions[t];
        if track {
            let mut h = std::collections::hash_map::DefaultHasher::new();
            sec[..5].hash(&mut h);
            sec[6..n - 4].hash(&mut h);
            let hv = h.finish();
            if last.is_some_and(|l| l != hv) {
                *ver = (*ver + 1) & 0x1F;
            }
            *last = Some(hv);
        }
        sec[5] = (sec[5] & 0xC1) | (*ver << 1);
        sec.truncate(n - 4);
        let c = crc32(&sec);
        sec.extend_from_slice(&c.to_be_bytes());
        sec
    }

    /// Wall clock at packet `n`, Unix seconds.
    fn unix(&self, n: u64) -> f64 {
        self.start_unix + n as f64 / self.pps
    }

    /// NIT actual: the network's name, this TS and its service.
    fn nit(&self) -> Vec<u8> {
        let mut net = vec![0x40, NETWORK_NAME.len() as u8];
        net.extend_from_slice(NETWORK_NAME);
        // service_list_descriptor: service 1, its type; the delivery system
        // descriptor (EN 300 468 6.2.13.3 / 6.4.6.3) when known
        let mut ts_desc = vec![0x41, 3, 0x00, 0x01, SERVICE_TYPE];
        if let Some(d) = &self.delivery {
            ts_desc.extend_from_slice(&d.descriptor());
        }
        let mut s = vec![0x40, 0xF0, 0, (ONID >> 8) as u8, ONID as u8, 0xC1, 0, 0];
        s.extend_from_slice(&[0xF0 | (net.len() >> 8) as u8, net.len() as u8]);
        s.extend_from_slice(&net);
        let tsl = 6 + ts_desc.len();
        s.extend_from_slice(&[0xF0 | (tsl >> 8) as u8, tsl as u8, 0x00, 0x01, (ONID >> 8) as u8, ONID as u8]);
        s.extend_from_slice(&[0xF0 | (ts_desc.len() >> 8) as u8, ts_desc.len() as u8]);
        s.extend_from_slice(&ts_desc);
        finish_section(s)
    }

    /// EIT actual present/following, section 0 (present: this hour's
    /// "DATV <call>", running) or 1 (following: none).
    fn eit(&self, section: u8, n: u64) -> Vec<u8> {
        let mut s = vec![0x4E, 0xF0, 0, 0x00, 0x01, 0xC1, section, 1, 0x00, 0x01, (ONID >> 8) as u8, ONID as u8, 1, 0x4E];
        if section == 0 {
            let now = self.unix(n);
            let start = (now / 3600.0).floor() * 3600.0;
            let name = format!("DATV {}", self.service);
            let text = b"Amateur television";
            let mut d = vec![0x4D, (5 + name.len() + text.len()) as u8, b'e', b'n', b'g', name.len() as u8];
            d.extend_from_slice(name.as_bytes());
            d.push(text.len() as u8);
            d.extend_from_slice(text);
            s.extend_from_slice(&[0x00, 0x01]); // event_id
            s.extend_from_slice(&mjd_utc(start));
            s.extend_from_slice(&[0x01, 0x00, 0x00]); // duration 01:00:00 (BCD)
            // running_status 4 (running), free_CA 0, descriptors_loop_length
            s.extend_from_slice(&[0x80 | (d.len() >> 8) as u8, d.len() as u8]);
            s.extend_from_slice(&d);
        }
        finish_section(s)
    }

    /// A queued packet leaves: its continuity counter.
    fn stamp(&mut self, mut q: Queued) -> [u8; TS_LEN] {
        let k = Self::cc_index(q.pid);
        q.pkt[3] = (q.pkt[3] & 0xF0) | self.cc[k];
        self.cc[k] = (self.cc[k] + 1) & 0x0F;
        q.pkt
    }

    /// A PSI section in one packet (all of ours fit).
    fn section(&mut self, pid: u16, sec: &[u8]) -> [u8; TS_LEN] {
        let k = Self::cc_index(pid);
        let mut pkt = [0xFFu8; TS_LEN];
        pkt[..5].copy_from_slice(&[0x47, 0x40 | (pid >> 8) as u8, pid as u8, 0x10 | self.cc[k], 0]);
        self.cc[k] = (self.cc[k] + 1) & 0x0F;
        pkt[5..5 + sec.len()].copy_from_slice(sec);
        pkt
    }

    fn pat(&self) -> Vec<u8> {
        let mut s = vec![0x00, 0xB0, 0, 0x00, 0x01, 0xC1, 0, 0];
        // program 0: the network PID (NIT)
        s.extend_from_slice(&[0x00, 0x00, 0xE0 | (PID_NIT >> 8) as u8, PID_NIT as u8]);
        s.extend_from_slice(&[0x00, 0x01, 0xE0 | (PID_PMT >> 8) as u8, PID_PMT as u8]);
        finish_section(s)
    }

    fn pmt(&self) -> Vec<u8> {
        let mut s = vec![0x02, 0xB0, 0, 0x00, 0x01, 0xC1, 0, 0];
        s.extend_from_slice(&[0xE0 | (PID_VIDEO >> 8) as u8, PID_VIDEO as u8, 0xF0, 0]); // PCR PID, no program info
        s.extend_from_slice(&[0x1B, 0xE0 | (PID_VIDEO >> 8) as u8, PID_VIDEO as u8, 0xF0, 0]);
        // Opus: private data, registration "Opus", DVB extension: 1 channel.
        let desc = [0x05, 4, b'O', b'p', b'u', b's', 0x7F, 2, 0x80, 1];
        s.extend_from_slice(&[0x06, 0xE0 | (PID_AUDIO >> 8) as u8, PID_AUDIO as u8, 0xF0, desc.len() as u8]);
        s.extend_from_slice(&desc);
        finish_section(s)
    }

    fn sdt(&self) -> Vec<u8> {
        let provider = b"SQTRX";
        let name = self.service.as_bytes();
        let mut d = vec![0x48, (3 + provider.len() + name.len()) as u8, SERVICE_TYPE, provider.len() as u8];
        d.extend_from_slice(provider);
        d.push(name.len() as u8);
        d.extend_from_slice(name);
        let mut s = vec![0x42, 0xF0, 0, 0x00, 0x01, 0xC1, 0, 0, (ONID >> 8) as u8, ONID as u8, 0xFF];
        // EIT present/following there (flag), running, not scrambled
        s.extend_from_slice(&[0x00, 0x01, 0xFD, 0x80 | (d.len() >> 8) as u8, d.len() as u8]);
        s.extend_from_slice(&d);
        finish_section(s)
    }
}

/// Pulls H.264 access units and Opus packets out of a received TS (the PMT
/// says where), as browser messages: `[6][flags: bit 0 = key][i64 LE PTS,
/// us][Annex B]` and `[7][i64 LE PTS, us][Opus packet]`. A PES hit by a
/// continuity error is dropped whole.
/// DVB service information a receiver shows: from the SDT, NIT, EIT
/// present/following and TDT (single-packet sections, as ours are).
#[derive(Debug, Clone, Default)]
pub struct Si {
    pub service: Option<String>,
    pub provider: Option<String>,
    pub service_type: Option<u8>,
    pub network: Option<String>,
    /// The present event: name, text, start (Unix s), duration (s).
    pub event: Option<(String, String, f64, u32)>,
    /// The last TDT: the transmitter's UTC (Unix s) and when it came.
    pub tdt: Option<(f64, std::time::Instant)>,
}

impl Si {
    /// For the browser; the TDT as the transmitter's clock now, extrapolated.
    pub fn json(&self) -> serde_json::Value {
        serde_json::json!({
            "service": self.service, "provider": self.provider, "service_type": self.service_type,
            "network": self.network,
            "event": self.event.as_ref().map(|(n, t, st, d)| serde_json::json!({"name": n, "text": t, "start": st, "duration": d})),
            "tx_utc": self.tdt.map(|(t, at)| t + at.elapsed().as_secs_f64()),
        })
    }
}

/// A DVB SI text: an optional character-table byte first (dropped), then
/// Latin text (control codes dropped).
fn si_text(b: &[u8]) -> String {
    let b = if b.first().is_some_and(|&c| c < 0x20) { &b[1..] } else { b };
    b.iter().filter(|&&c| c >= 0x20 && c != 0x7F).map(|&c| c as char).collect()
}

/// MJD + BCD UTC (5 bytes) to Unix seconds.
fn si_time(b: &[u8]) -> f64 {
    let mjd = u16::from_be_bytes([b[0], b[1]]) as f64;
    let bcd = |v: u8| ((v >> 4) * 10 + (v & 0x0F)) as f64;
    (mjd - 40_587.0) * 86_400.0 + bcd(b[2]) * 3600.0 + bcd(b[3]) * 60.0 + bcd(b[4])
}

#[derive(Default)]
pub struct Demux {
    pmt: Option<u16>,
    video: Option<u16>,
    audio: Option<u16>,
    pes: std::collections::HashMap<u16, (Vec<u8>, bool)>,
    cc: std::collections::HashMap<u16, u8>,
    /// PSI/SI sections being put together (they may span packets).
    secs: std::collections::HashMap<u16, Vec<u8>>,
    /// PTS unwrapping (33 bits wrap every 26.5 h): the last PTS and the
    /// wraps so far.
    pts_last: Option<u64>,
    pts_wraps: u64,
    pub pes_dropped: u64,
    pub si: Si,
}

impl Demux {
    pub fn push(&mut self, p: &[u8; TS_LEN], out: &mut Vec<Vec<u8>>) {
        if p[0] != 0x47 || p[1] & 0x80 != 0 {
            return;
        }
        let pid = ((p[1] as u16 & 0x1F) << 8) | p[2] as u16;
        let pusi = p[1] & 0x40 != 0;
        let afc = (p[3] >> 4) & 3;
        if afc & 1 == 0 {
            return; // no payload
        }
        let start = if afc & 2 != 0 { 5 + p[4] as usize } else { 4 };
        if start >= TS_LEN {
            return;
        }
        let payload = &p[start..];
        let psi = pid == 0 || Some(pid) == self.pmt || matches!(pid, PID_NIT | PID_SDT | PID_EIT | PID_TDT);
        if !psi && Some(pid) != self.video && Some(pid) != self.audio {
            return;
        }
        let cc = p[3] & 0x0F;
        let prev = self.cc.insert(pid, cc);
        // 13818-1 2.4.3.3: one duplicate packet (same counter) may follow.
        if prev == Some(cc) {
            return;
        }
        let lost = prev.is_some_and(|prev| cc != (prev + 1) & 0x0F);
        if psi {
            self.psi(pid, pusi, lost, payload);
            return;
        }
        let entry = self.pes.entry(pid).or_insert_with(|| (Vec::new(), false));
        if pusi {
            let (buf, ok) = std::mem::replace(entry, (payload.to_vec(), true));
            if ok && !lost {
                self.pes_out(pid, &buf, out);
            } else if !buf.is_empty() {
                self.pes_dropped += 1;
            }
        } else if lost {
            entry.1 = false;
        } else if entry.1 {
            entry.0.extend_from_slice(payload);
        }
        // Audio PES say how long they are: out as soon as complete.
        if let Some((buf, true)) = self.pes.get(&pid) {
            let len = buf.get(4..6).map_or(0, |b| u16::from_be_bytes([b[0], b[1]]) as usize);
            if len > 0 && buf.len() >= 6 + len {
                let buf = std::mem::take(&mut self.pes.get_mut(&pid).unwrap().0);
                self.pes.get_mut(&pid).unwrap().1 = false;
                self.pes_out(pid, &buf[..6 + len], out);
            }
        }
    }

    /// PSI/SI payload: sections may start anywhere (pointer_field), span
    /// packets and follow each other in one.
    fn psi(&mut self, pid: u16, pusi: bool, lost: bool, payload: &[u8]) {
        let mut buf = self.secs.remove(&pid).unwrap_or_default();
        if lost {
            buf.clear();
        }
        let mut rest = payload;
        if pusi {
            let ptr = payload[0] as usize;
            let Some(tail) = payload.get(1..1 + ptr) else { return };
            // The end of the section begun before (if it was being collected).
            if !buf.is_empty() {
                buf.extend_from_slice(tail);
                self.sections(pid, &mut buf);
            }
            buf.clear();
            rest = &payload[1 + ptr..];
            buf.extend_from_slice(rest);
        } else if !buf.is_empty() {
            buf.extend_from_slice(rest);
        } else {
            return;
        }
        self.sections(pid, &mut buf);
        if !buf.is_empty() {
            self.secs.insert(pid, buf);
        }
    }

    /// Every complete section at the front of `buf` out; stuffing (0xFF)
    /// ends the packet's sections.
    fn sections(&mut self, pid: u16, buf: &mut Vec<u8>) {
        loop {
            if buf.first().is_none_or(|&t| t == 0xFF) {
                buf.clear();
                return;
            }
            if buf.len() < 3 {
                return;
            }
            let len = (((buf[1] as usize) & 0x0F) << 8 | buf[2] as usize) + 3;
            if buf.len() < len {
                if len > 4096 + 3 {
                    buf.clear();
                }
                return;
            }
            let sec: Vec<u8> = buf.drain(..len).collect();
            self.section(pid, &sec);
        }
    }

    fn section(&mut self, pid: u16, s: &[u8]) {
        // TDT: a short section, no CRC
        if pid == PID_TDT && s.len() >= 8 && s[0] == 0x70 {
            self.si.tdt = Some((si_time(&s[3..8]), std::time::Instant::now()));
            return;
        }
        if s.len() < 12 {
            return;
        }
        let len = (((s[1] as usize) & 0x0F) << 8 | s[2] as usize) + 3;
        if len > s.len() || crc32(&s[..len]) != 0 {
            return;
        }
        let body = &s[8..len - 4];
        if pid == 0 && s[0] == 0x00 {
            // The first program's PMT; a new PID (another transmitter, a
            // remux) starts over.
            if let Some(e) = body.chunks_exact(4).find(|e| u16::from_be_bytes([e[0], e[1]]) != 0) {
                let pmt = u16::from_be_bytes([e[2], e[3]]) & 0x1FFF;
                if self.pmt != Some(pmt) {
                    if let Some(old) = self.pmt {
                        self.secs.remove(&old);
                    }
                    self.pmt = Some(pmt);
                    self.set_streams(None, None);
                }
            }
        } else if Some(pid) == self.pmt && s[0] == 0x02 && body.len() >= 4 {
            // The streams as this PMT has them (they may change).
            let info = (((body[2] as usize) & 0x0F) << 8) | body[3] as usize;
            let mut i = 4 + info;
            let (mut video, mut audio) = (None, None);
            while i + 5 <= body.len() {
                let (st, epid) = (body[i], u16::from_be_bytes([body[i + 1], body[i + 2]]) & 0x1FFF);
                let dl = (((body[i + 3] as usize) & 0x0F) << 8) | body[i + 4] as usize;
                let desc = body.get(i + 5..i + 5 + dl).unwrap_or(&[]);
                match st {
                    0x1B if video.is_none() => video = Some(epid),
                    0x06 if audio.is_none() && desc.windows(4).any(|w| w == b"Opus") => audio = Some(epid),
                    _ => {}
                }
                i += 5 + dl;
            }
            self.set_streams(video, audio);
        } else if pid == PID_SDT && s[0] == 0x42 {
            // services from byte 11: id, flags, then descriptors
            let end = len - 4;
            let mut i = 11;
            while i + 5 <= end {
                let dl = (((s[i + 3] as usize) & 0x0F) << 8) | s[i + 4] as usize;
                let mut j = i + 5;
                while j + 2 <= (i + 5 + dl).min(end) {
                    let (tag, l) = (s[j], s[j + 1] as usize);
                    let d = s.get(j + 2..j + 2 + l).unwrap_or(&[]);
                    if tag == 0x48 && d.len() >= 2 {
                        let pl = d[1] as usize;
                        let prov = d.get(2..2 + pl).unwrap_or(&[]);
                        let nl = d.get(2 + pl).copied().unwrap_or(0) as usize;
                        let name = d.get(3 + pl..3 + pl + nl).unwrap_or(&[]);
                        self.si.service_type = Some(d[0]);
                        self.si.provider = Some(si_text(prov));
                        self.si.service = Some(si_text(name));
                    }
                    j += 2 + l;
                }
                i += 5 + dl;
            }
        } else if pid == PID_NIT && s[0] == 0x40 {
            let nl = (((s[8] as usize) & 0x0F) << 8) | s[9] as usize;
            let mut j = 10;
            while j + 2 <= (10 + nl).min(len - 4) {
                let (tag, l) = (s[j], s[j + 1] as usize);
                if tag == 0x40 {
                    self.si.network = Some(si_text(s.get(j + 2..j + 2 + l).unwrap_or(&[])));
                }
                j += 2 + l;
            }
        } else if pid == PID_EIT && s[0] == 0x4E && s[6] == 0 {
            // present event (section 0): the first event's short descriptor
            let end = len - 4;
            if 14 + 12 <= end {
                let e = &s[14..end];
                let start = si_time(&e[2..7]);
                let bcd = |v: u8| ((v >> 4) * 10 + (v & 0x0F)) as u32;
                let dur = bcd(e[7]) * 3600 + bcd(e[8]) * 60 + bcd(e[9]);
                let dl = (((e[10] as usize) & 0x0F) << 8) | e[11] as usize;
                let (mut name, mut text) = (String::new(), String::new());
                let mut j = 12;
                while j + 2 <= (12 + dl).min(e.len()) {
                    let (tag, l) = (e[j], e[j + 1] as usize);
                    let d = e.get(j + 2..j + 2 + l).unwrap_or(&[]);
                    if tag == 0x4D && d.len() >= 4 {
                        let nl = d[3] as usize;
                        name = si_text(d.get(4..4 + nl).unwrap_or(&[]));
                        let tl = d.get(4 + nl).copied().unwrap_or(0) as usize;
                        text = si_text(d.get(5 + nl..5 + nl + tl).unwrap_or(&[]));
                    }
                    j += 2 + l;
                }
                self.si.event = Some((name, text, start, dur));
            }
        }
    }

    /// Video and audio PIDs; a change drops what was collected for the old.
    fn set_streams(&mut self, video: Option<u16>, audio: Option<u16>) {
        if (video, audio) != (self.video, self.audio) {
            for p in [self.video, self.audio].into_iter().flatten() {
                self.pes.remove(&p);
                self.cc.remove(&p);
            }
            self.video = video;
            self.audio = audio;
        }
    }

    /// A 33-bit PTS made continuous (the 90 kHz counter wraps every 26.5 h).
    fn unwrap_pts(&mut self, pts: u64) -> u64 {
        const WRAP: u64 = 1 << 33;
        if let Some(last) = self.pts_last {
            if last > pts && last - pts > WRAP / 2 {
                self.pts_wraps += 1;
            } else if pts > last && pts - last > WRAP / 2 && self.pts_wraps > 0 {
                // (a late packet from before the wrap)
                return pts + (self.pts_wraps - 1) * WRAP;
            }
        }
        self.pts_last = Some(pts);
        pts + self.pts_wraps * WRAP
    }

    fn pes_out(&mut self, pid: u16, pes: &[u8], out: &mut Vec<Vec<u8>>) {
        if pes.len() < 9 || pes[..3] != [0, 0, 1] {
            return;
        }
        let hl = pes[8] as usize;
        let Some(data) = pes.get(9 + hl..) else { return };
        let pts90 = if pes[7] & 0x80 != 0 && hl >= 5 {
            let b = &pes[9..14];
            let pts = ((b[0] as u64 >> 1) & 7) << 30 | (b[1] as u64) << 22 | (b[2] as u64 >> 1) << 15 | (b[3] as u64) << 7 | b[4] as u64 >> 1;
            self.unwrap_pts(pts)
        } else {
            0
        };
        let us = (pts90 * 1000 / 90) as i64;
        if Some(pid) == self.video {
            let key = data.windows(4).any(|w| w[..3] == [0, 0, 1] && w[3] & 0x1F == 5);
            let mut m = vec![6, key as u8];
            m.extend_from_slice(&us.to_le_bytes());
            m.extend_from_slice(data);
            out.push(m);
        } else {
            // Opus access units, each behind a control header.
            let (mut i, mut k) = (0usize, 0i64);
            while i + 3 <= data.len() && (u16::from_be_bytes([data[i], data[i + 1]]) >> 5) == 0x3FF {
                let flags = data[i + 1];
                i += 2;
                let mut n = 0usize;
                while i < data.len() {
                    let b = data[i];
                    i += 1;
                    n += b as usize;
                    if b != 0xFF {
                        break;
                    }
                }
                if flags & 0x10 != 0 {
                    i += 2;
                }
                if flags & 0x08 != 0 {
                    i += 2;
                }
                if flags & 0x04 != 0 {
                    i += 1 + data.get(i).copied().unwrap_or(0) as usize;
                }
                let Some(pkt) = data.get(i..i + n) else { break };
                let mut m = vec![7];
                m.extend_from_slice(&(us + k * 20_000).to_le_bytes());
                m.extend_from_slice(pkt);
                out.push(m);
                i += n;
                k += 1;
            }
        }
    }
}

/// `trxd --datv-mux MEDIA OUT.ts SYMBOL_RATE [RATE] [pilots]`: play a file of
/// browser messages (`[u32 LE length][message]`...) into the mux in real time
/// on the mux's own clock and write the constant-rate TS, 2 s past the end.
pub fn mux_cli(input: &str, output: &str, rest: &[String]) -> Result<(), String> {
    let sr: f64 = rest.first().ok_or("symbol rate")?.parse().map_err(|_| "symbol rate: a number")?;
    let rate = rest.get(1).map_or(Some(super::Rate::R1_2), |s| super::Rate::parse(s)).ok_or("rate: 1/4, 1/3, 1/2, 2/3 or 3/4")?;
    let p = super::Params { rate, pilots: rest.iter().any(|s| s == "pilots"), rolloff: 0.35 };
    let raw = std::fs::read(input).map_err(|e| format!("{input}: {e}"))?;
    let mut msgs = Vec::new();
    let mut i = 0;
    while i + 4 <= raw.len() {
        let n = u32::from_le_bytes(raw[i..i + 4].try_into().unwrap()) as usize;
        let m = Media::from_ws(raw.get(i + 4..i + 4 + n).ok_or("truncated record")?).ok_or("bad record")?;
        msgs.push(m);
        i += 4 + n;
    }
    msgs.sort_by_key(|m| m.ts_us());
    let t0 = msgs.first().map_or(0, |m| m.ts_us());
    let end = msgs.last().map_or(0, |m| m.ts_us()) - t0 + 2_000_000;
    let ts_rate = p.ts_rate(sr);
    let mut mux = Mux::new(ts_rate, "SQ6EMM");
    let mut out = Vec::new();
    let mut it = msgs.into_iter().peekable();
    let mut n = 0u64;
    loop {
        let now_us = (n as f64 * TS_LEN as f64 * 8.0 / ts_rate * 1e6) as i64;
        if now_us > end {
            break;
        }
        while it.peek().is_some_and(|m| m.ts_us() - t0 <= now_us) {
            mux.push(it.next().unwrap());
        }
        mux.want_key = false;
        out.extend_from_slice(&mux.next());
        n += 1;
    }
    std::fs::write(output, &out).map_err(|e| format!("{output}: {e}"))?;
    eprintln!(
        "TS {:.0} bit/s ({} {}), {} packets, video frames dropped {}, {:?}, video budget {:.0} bit/s",
        ts_rate,
        sr,
        rate.label(),
        n,
        mux.dropped_frames,
        mux.profile,
        mux.profile.video_budget(ts_rate)
    );
    Ok(())
}

/// Fill in section_length and append the CRC.
/// A PES's first packet with a new PTS (the header after the adaptation
/// field: 00 00 01 id len(2) flags(2) hdr_len, then the PTS).
fn restamp(p: &[u8; TS_LEN], pts: u64) -> [u8; TS_LEN] {
    let mut q = *p;
    let at = if q[3] & 0x20 != 0 { 5 + q[4] as usize } else { 4 };
    if at + 14 <= TS_LEN && q[at..at + 3] == [0, 0, 1] {
        q[at + 9..at + 14].copy_from_slice(&pts_bytes(pts));
    }
    q
}

/// The SPS and PPS NAL units of an Annex B access unit (start codes
/// included), in order; empty when it has none.
fn parameter_sets(au: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut starts = Vec::new();
    let mut i = 0;
    while i + 3 <= au.len() {
        if au[i] == 0 && au[i + 1] == 0 && au[i + 2] == 1 {
            let s0 = if i > 0 && au[i - 1] == 0 { i - 1 } else { i };
            starts.push((s0, i + 3));
            i += 3;
        } else {
            i += 1;
        }
    }
    for (k, &(s0, h)) in starts.iter().enumerate() {
        let end = starts.get(k + 1).map_or(au.len(), |&(n0, _)| n0);
        if h < au.len() && matches!(au[h] & 0x1F, 7 | 8) {
            out.extend_from_slice(&au[s0..end]);
        }
    }
    out
}

/// UTC as DVB SI writes it: 16-bit Modified Julian Date and hh:mm:ss in BCD.
fn mjd_utc(unix: f64) -> [u8; 5] {
    let t = unix.max(0.0) as u64;
    let mjd = 40_587 + t / 86_400;
    let sec = t % 86_400;
    let bcd = |v: u64| (((v / 10) << 4) | (v % 10)) as u8;
    [(mjd >> 8) as u8, mjd as u8, bcd(sec / 3600), bcd(sec / 60 % 60), bcd(sec % 60)]
}

/// Time and Date Table: UTC; a short section without CRC.
fn tdt(unix: f64) -> Vec<u8> {
    let mut s = vec![0x70, 0x70, 0x05];
    s.extend_from_slice(&mjd_utc(unix));
    s
}

fn finish_section(mut s: Vec<u8>) -> Vec<u8> {
    let len = s.len() - 3 + 4;
    s[1] = (s[1] & 0xF0) | ((len >> 8) as u8 & 0x0F);
    s[2] = len as u8;
    let c = crc32(&s);
    s.extend_from_slice(&c.to_be_bytes());
    s
}

fn pts_bytes(pts: u64) -> [u8; 5] {
    let p = pts & ((1 << 33) - 1);
    [
        0x21 | ((p >> 29) & 0x0E) as u8,
        (p >> 22) as u8,
        0x01 | ((p >> 14) & 0xFE) as u8,
        (p >> 7) as u8,
        0x01 | ((p << 1) & 0xFE) as u8,
    ]
}

fn pcr_bytes(t27: u64) -> [u8; 6] {
    let base = (t27 / 300) & ((1 << 33) - 1);
    let ext = t27 % 300;
    [
        (base >> 25) as u8,
        (base >> 17) as u8,
        (base >> 9) as u8,
        (base >> 1) as u8,
        (((base & 1) << 7) as u8) | 0x7E | ((ext >> 8) & 1) as u8,
        ext as u8,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_is_the_mpeg2_one() {
        // A PAT with the CRC appended checks to zero.
        let pat = finish_section(vec![0x00, 0xB0, 0, 0x00, 0x01, 0xC1, 0, 0, 0x00, 0x01, 0xF0, 0x00]);
        assert_eq!(crc32(&pat), 0);
        assert_eq!(crc32(b"123456789"), 0x0376_E6E7);
    }

    /// A stream at each profile's rate, loaded with video at its budget and
    /// audio: the repetition intervals TR 101 290 checks (PCR, PAT, PMT,
    /// SDT), the SDT's service type and network id. TS_DUMP=<dir>: also
    /// writes each stream there (for TSDuck's tsanalyze).
    #[test]
    fn repetition_meets_tr101290() {
        for rate in [30_000.0, 40_000.0, 45_000.0, 46_000.0, 50_000.0, 54_325.0, 120_000.0, 300_000.0, 360_000.0] {
            let mut m = Mux::new(rate, "SQ6EMM");
            let p = m.profile;
            let pps = rate / (TS_LEN as f64 * 8.0);
            let frame_bytes = (p.video_budget(rate) / 8.0 / p.fps) as usize;
            let (mut next_v, mut next_a, mut vi) = (0.0f64, 0.0f64, 0u64);
            // frame sizes vary around the budget (50-150 %), as an encoder's
            // do: the budget counts half a packet of padding a frame
            let mut lcg = 12345u32;
            let mut out = Vec::new();
            let n_pkts = (60.0 * pps) as usize;
            for n in 0..n_pkts {
                let t = n as f64 / pps;
                while next_v <= t {
                    let key = vi % 50 == 0;
                    lcg = lcg.wrapping_mul(1_103_515_245).wrapping_add(12_345);
                    let size = (frame_bytes as f64 * (0.5 + (lcg >> 8) as f64 / (1u32 << 24) as f64)) as usize;
                    let data = [vec![0, 0, 0, 1, if key { 0x65 } else { 0x41 }], vec![(vi % 251) as u8; size.max(20)]].concat();
                    m.push(Media::Video { ts_us: (next_v * 1e6) as i64, key, data });
                    vi += 1;
                    next_v += 1.0 / p.fps;
                }
                while next_a <= t {
                    m.push(Media::Audio { ts_us: (next_a * 1e6) as i64, data: vec![0xAB; (p.audio_bps / 8.0 * 0.02) as usize] });
                    next_a += 0.02;
                }
                out.push(m.next());
            }
            let gap = |f: &dyn Fn(&[u8; TS_LEN]) -> bool| -> f64 {
                let idx: Vec<usize> = out.iter().enumerate().filter(|(_, p)| f(p)).map(|(i, _)| i).collect();
                assert!(idx.len() > 2, "rate {rate}: too few");
                idx.windows(2).map(|w| (w[1] - w[0]) as f64 / pps).fold(0.0, f64::max)
            };
            let pid = |p: &[u8; TS_LEN]| ((p[1] as u16 & 0x1F) << 8) | p[2] as u16;
            let start = |p: &[u8; TS_LEN]| p[1] & 0x40 != 0;
            let pcr = gap(&|p| p[3] & 0x20 != 0 && p[4] > 0 && p[5] & 0x10 != 0);
            let pat = gap(&|p| pid(p) == 0 && start(p));
            let pmt = gap(&|p| pid(p) == PID_PMT && start(p));
            let sdt = gap(&|p| pid(p) == PID_SDT && start(p));
            let nit = gap(&|p| pid(p) == PID_NIT && start(p));
            let eit = gap(&|p| pid(p) == PID_EIT && start(p) && p[5 + 6] == 0);
            let tdt = gap(&|p| pid(p) == PID_TDT && start(p));
            assert!(nit <= 10.0 && eit <= 2.0 && tdt <= 30.0, "rate {rate}: NIT {nit} EIT {eit} TDT {tdt}");
            // the SI sections are whole and their CRCs check (the TDT has none)
            for p in out.iter().filter(|p| start(p) && matches!(pid(p), 0 | PID_PMT | PID_SDT | PID_NIT | PID_EIT)) {
                let sec = &p[5..];
                let len = ((sec[1] as usize & 0x0F) << 8 | sec[2] as usize) + 3;
                assert_eq!(crc32(&sec[..len]), 0, "rate {rate}: pid {:#x}", pid(p));
            }
            let dropped = m.dropped_frames;
            {
                let secs = out.len() as f64 / pps;
                let cnt = |f: &dyn Fn(&[u8; TS_LEN]) -> bool| out.iter().filter(|p| f(p)).count() as f64 / secs;
                let pidp = |p: &[u8; TS_LEN]| ((p[1] as u16 & 0x1F) << 8) | p[2] as u16;
                eprintln!("  per s: video {:.1} (budget {:.1}), pcr-only {:.1}, audio {:.1}, tables {:.1}, null {:.1} of {:.1}",
                    cnt(&|p| pidp(p) == PID_VIDEO && p[3] & 0x10 != 0), p.video_budget(rate) / 8.0 / VIDEO_PAYLOAD,
                    cnt(&|p| pidp(p) == PID_VIDEO && p[3] & 0x30 == 0x20), cnt(&|p| pidp(p) == PID_AUDIO),
                    cnt(&|p| pidp(p) == 0 || pidp(p) == PID_PMT || pidp(p) == PID_SDT), cnt(&|p| pidp(p) == 0x1FFF), pps);
            }
            eprintln!("{rate:>8.0} bit/s: PCR {:.0} ms, PAT {:.2} s, PMT {:.2} s, SDT {:.2} s, NIT {nit:.1} s, EIT {eit:.2} s, TDT {tdt:.0} s, frames dropped {dropped} of {vi}", pcr * 1e3, pat, pmt, sdt);
            // PCR: the profile's interval, at least 3 slots; within 13818-1's
            // 100 ms from 40 kbit/s up, TR 101 290's 40 ms from 200 kbit/s.
            let slot = 1.0 / pps;
            let pcr_max = p.pcr_every_s.max(3.0 * slot);
            let psi_max = p.psi_every_s;
            assert!(pcr <= pcr_max + 1e-9, "rate {rate}: PCR gap {pcr}");
            if rate >= LEAN_BELOW_BPS {
                assert!(pcr <= 0.1, "rate {rate}: PCR gap {pcr} over 13818-1's 100 ms");
            }
            // EIT: the two sections at least 25 ms apart (TS 101 211 4.4)
            let eit_idx: Vec<usize> = out.iter().enumerate().filter(|(_, p)| pid(p) == PID_EIT && start(p)).map(|(i, _)| i).collect();
            for w in eit_idx.windows(2) {
                assert!((w[1] - w[0]) as f64 / pps >= SECTION_GAP_S - 1e-9, "rate {rate}: EIT sections {} s apart", (w[1] - w[0]) as f64 / pps);
            }
            if rate >= 200_000.0 {
                assert!(pcr <= 0.04 + 1e-9, "rate {rate}: PCR gap {pcr} over TR 101 290's 40 ms");
            }
            assert!(pat <= psi_max + 1e-9 && pmt <= psi_max + 1e-9, "rate {rate}: PAT {pat} PMT {pmt}");
            assert!(sdt <= 2.0, "rate {rate}: SDT {sdt}");
            assert!(dropped * 20 < vi, "rate {rate}: the video budget overflows ({dropped} of {vi} dropped)");
            // T-STD: each PES in whole before its PTS, never over 1 s
            // there (the first byte's arrival to the PTS), PTS never back
            for want in [PID_VIDEO, PID_AUDIO] {
                let starts: Vec<usize> = out.iter().enumerate().filter(|(_, p)| pid(p) == want && start(p)).map(|(i, _)| i).collect();
                let mut last = 0u64;
                for (k, &i) in starts.iter().enumerate() {
                    let p = &out[i];
                    let at = if p[3] & 0x20 != 0 { 5 + p[4] as usize } else { 4 };
                    let b = &p[at + 9..at + 14];
                    let pts = ((b[0] as u64 >> 1) & 7) << 30 | (b[1] as u64) << 22 | ((b[2] as u64) >> 1) << 15 | (b[3] as u64) << 7 | (b[4] as u64) >> 1;
                    assert!(pts > last || k == 0, "rate {rate}: pid {want:#x} PTS went back");
                    last = pts;
                    // the PES's last packet: before the next start on its PID
                    let Some(&next) = starts.get(k + 1) else { break };
                    let end = (i..next).rev().find(|&j| pid(&out[j]) == want && out[j][3] & 0x10 != 0).unwrap();
                    let in_by = (end + 1) as f64 / pps;
                    assert!(in_by <= pts as f64 / 90_000.0, "rate {rate}: pid {want:#x} PES {k} in at {in_by:.3} s, PTS {:.3} s", pts as f64 / 90_000.0);
                    let first_in = i as f64 / pps;
                    assert!(pts as f64 / 90_000.0 - first_in <= m.profile.tstd_max_s + 1e-6, "rate {rate}: pid {want:#x} PES {k} {:.3} s in the T-STD", pts as f64 / 90_000.0 - first_in);
                }
            }
            // service_type and original_network_id in the SDT
            let sdt_pkt = out.iter().find(|p| pid(p) == PID_SDT && start(p)).unwrap();
            let sec = &sdt_pkt[5..];
            assert_eq!(u16::from_be_bytes([sec[8], sec[9]]), ONID);
            assert_eq!(sec[16], 0x48);
            assert_eq!(sec[18], SERVICE_TYPE);
            if let Some(dir) = std::env::var_os("TS_DUMP") {
                let f = std::path::Path::new(&dir).join(format!("mux_{}.ts", rate as u64));
                std::fs::write(f, out.concat()).unwrap();
            }
        }
    }

    /// The receiver reads back the service information the mux sends.
    #[test]
    fn demux_reads_the_service_information() {
        let mut m = Mux::new(300_000.0, "SQ6EMM");
        m.start_unix = 1_790_000_000.0 + 1234.0;
        let mut dmx = Demux::default();
        let mut msgs = Vec::new();
        for _ in 0..(30.0 * m.pps) as usize {
            dmx.push(&m.next(), &mut msgs);
        }
        let si = &dmx.si;
        assert_eq!(si.service.as_deref(), Some("SQ6EMM"));
        assert_eq!(si.provider.as_deref(), Some("SQTRX"));
        assert_eq!(si.service_type, Some(SERVICE_TYPE));
        assert_eq!(si.network.as_deref(), Some("SQTRX DATV"));
        let (name, text, start, dur) = si.event.clone().unwrap();
        assert_eq!((name.as_str(), text.as_str(), dur), ("DATV SQ6EMM", "Amateur television", 3600));
        assert_eq!(start, ((m.start_unix / 3600.0).floor() * 3600.0));
        let (tdt, _) = si.tdt.unwrap();
        assert!((tdt - (m.start_unix + 25.0)).abs() <= 1.0, "{tdt}");
    }

    /// A table's version_number follows its content: the EIT's present
    /// event moves at the hour, PAT/PMT/SDT/NIT stay.
    #[test]
    fn versions_follow_the_content() {
        let mut m = Mux::new(300_000.0, "SQ6EMM");
        m.start_unix = 1_790_002_800.0 - 5.0; // 5 s before a full hour
        let pid = |p: &[u8; TS_LEN]| ((p[1] as u16 & 0x1F) << 8) | p[2] as u16;
        let mut seen: std::collections::HashMap<u16, std::collections::BTreeSet<u8>> = Default::default();
        for _ in 0..(12.0 * m.pps) as usize {
            let p = m.next();
            if p[1] & 0x40 != 0 && matches!(pid(&p), 0 | PID_PMT | PID_SDT | PID_NIT | PID_EIT) {
                seen.entry(pid(&p)).or_default().insert((p[5 + 5] >> 1) & 0x1F);
            }
        }
        assert_eq!(seen[&PID_EIT].len(), 2, "EIT versions {:?}", seen[&PID_EIT]);
        for k in [0, PID_PMT, PID_SDT, PID_NIT] {
            assert_eq!(seen[&k].len(), 1, "pid {k:#x}: {:?}", seen[&k]);
        }
    }

    /// A keyframe without SPS/PPS gets the last ones; a keyframe is asked
    /// for after KEY_EVERY_S without one.
    #[test]
    fn keyframes_carry_parameter_sets_and_come_often() {
        let mut m = Mux::new(300_000.0, "SQ6EMM");
        let sps_pps = [0u8, 0, 0, 1, 0x67, 1, 2, 3, 0, 0, 0, 1, 0x68, 4, 5];
        let key1 = [&sps_pps[..], &[0, 0, 0, 1, 0x65, 9, 9]].concat();
        m.push(Media::Video { ts_us: 0, key: true, data: key1 });
        m.push(Media::Video { ts_us: 100_000, key: true, data: vec![0, 0, 0, 1, 0x65, 7, 7] });
        let mut dmx = Demux::default();
        let mut msgs = Vec::new();
        let half = (0.5 * m.pps) as usize;
        for _ in 0..half {
            dmx.push(&m.next(), &mut msgs);
        }
        assert!(!m.take_key_request(), "asked for a keyframe with one just sent");
        m.push(Media::Video { ts_us: 200_000, key: false, data: vec![0, 0, 0, 1, 0x41, 1] });
        for _ in 0..half {
            dmx.push(&m.next(), &mut msgs);
        }
        let video: Vec<&Vec<u8>> = msgs.iter().filter(|m| m[0] == 6).collect();
        assert!(video.len() >= 2);
        assert!(video[1].windows(sps_pps.len()).any(|w| w == sps_pps), "second keyframe without SPS/PPS");
        for _ in 0..(KEY_EVERY_S * m.pps) as usize {
            m.next();
        }
        assert!(m.take_key_request(), "no keyframe asked for");
        assert!(!m.take_key_request(), "asked twice");
    }

    #[test]
    fn packets_are_well_formed_and_counters_run() {
        let mut m = Mux::new(54_325.0, "SQ6EMM");
        let mut t = 0i64;
        let mut out = Vec::new();
        for i in 0..400 {
            if i % 7 == 0 {
                m.push(Media::Video { ts_us: t, key: i % 70 == 0, data: vec![0, 0, 0, 1, 0x65, i as u8].repeat(60) });
            }
            if i % 3 == 0 {
                m.push(Media::Audio { ts_us: t, data: vec![0xAB; 30] });
                t += 20_000;
            }
            out.push(m.next());
        }
        let mut last_cc = std::collections::HashMap::new();
        for p in &out {
            assert_eq!(p[0], 0x47);
            let pid = ((p[1] as u16 & 0x1F) << 8) | p[2] as u16;
            let afc = (p[3] >> 4) & 3;
            if afc & 2 != 0 {
                assert!(p[4] as usize <= 183);
            }
            if pid != 0x1FFF && afc & 1 != 0 {
                let cc = p[3] & 0x0F;
                if let Some(&prev) = last_cc.get(&pid) {
                    assert_eq!(cc, (prev + 1) & 0x0F, "pid {pid:#x}");
                }
                last_cc.insert(pid, cc);
            }
        }
        assert!(last_cc.contains_key(&PID_VIDEO) && last_cc.contains_key(&PID_AUDIO) && last_cc.contains_key(&0));
    }

    #[test]
    fn demux_gives_back_what_the_mux_took() {
        let mut m = Mux::new(54_325.0, "SQ6EMM");
        let frames: Vec<Vec<u8>> = (0..6u8).map(|i| [vec![0, 0, 0, 1, 0x09, 0xF0, 0, 0, 0, 1, if i == 0 { 0x65 } else { 0x41 }], vec![i; 300 + i as usize * 150]].concat()).collect();
        // (two whole audio PES of the 15 frames this profile packs)
        let opus: Vec<Vec<u8>> = (0..30u8).map(|i| vec![i; 30 + i as usize * 7]).collect();
        let (mut vi, mut ai) = (0, 0);
        let mut dmx = Demux::default();
        let mut msgs = Vec::new();
        for n in 0..3000 {
            if n % 60 == 0 && vi < frames.len() {
                // The mux adds its own AUD only when the frame has none: ours do.
                // (on the stream's clock: the frames are 1.7 s apart)
                m.push(Media::Video { ts_us: (n as f64 * 1504.0 / 54_325.0 * 1e6) as i64, key: vi == 0, data: frames[vi].clone() });
                vi += 1;
            }
            if n % 6 == 0 && ai < opus.len() {
                let t = (n as f64 * 1504.0 / 54_325.0 * 1e6) as i64;
                m.push(Media::Audio { ts_us: t, data: opus[ai].clone() });
                ai += 1;
            }
            dmx.push(&m.next(), &mut msgs);
        }
        let video: Vec<&Vec<u8>> = msgs.iter().filter(|m| m[0] == 6).collect();
        let audio: Vec<&Vec<u8>> = msgs.iter().filter(|m| m[0] == 7).collect();
        // The last video PES is only closed by the next one.
        assert_eq!(video.len(), frames.len() - 1);
        for (got, want) in video.iter().zip(&frames) {
            assert_eq!(&got[10..], &want[..]);
        }
        assert_eq!(video[0][1], 1, "first frame is a keyframe");
        assert_eq!(audio.len(), opus.len());
        for (got, want) in audio.iter().zip(&opus) {
            assert_eq!(&got[9..], &want[..]);
        }
        // 20 ms apart on the PTS clock.
        let ts = |m: &Vec<u8>| i64::from_le_bytes(m[1..9].try_into().unwrap());
        assert_eq!(ts(audio[1]) - ts(audio[0]), 20_000);
    }

    #[test]
    fn budget_leaves_room_for_audio_and_tables() {
        let v64 = Profile::for_rate(54_325.0).video_budget(54_325.0);
        let v128 = Profile::for_rate(108_650.0).video_budget(108_650.0);
        // QPSK 1/4 at 64 kS/s: the lean profile still leaves a usable picture.
        let v14 = Profile::for_rate(22_878.0).video_budget(22_878.0);
        // (tables every 0.5 s and a PCR slot in every video packet, as TR
        // 101 290 asks, take about 0.5 kbit/s more than before)
        assert!(v64 > 12_000.0 && v64 < 40_000.0, "{v64}");
        assert!(v128 > 40_000.0 && v128 < 90_000.0, "{v128}");
        // (tables every second instead of every 2 s, about 1.9 kbit/s, and
        // the NIT, EIT and TDT, about 1.9 kbit/s more)
        assert!(v14 > 4_000.0, "{v14}");
    }

    /// The delivery system descriptors (EN 300 468 6.2.13.3, 6.4.6.3).
    #[test]
    fn nit_names_the_delivery_system() {
        let d = Delivery::S2 { freq_hz: 1_255_000_000.0, symbol_rate: 333_000.0, rolloff: 0.35, modcod: 14 }.descriptor();
        // 001.25500 GHz, orbit 0, S2 8PSK roll-off 0.35, 000.3330 Msym/s, FEC 3/4
        assert_eq!(d, [0x43, 11, 0x00, 0x12, 0x55, 0x00, 0x00, 0x00, 0x06, 0x00, 0x03, 0x33, 0x03]);
        let d = Delivery::S2 { freq_hz: 10_491_500_000.0, symbol_rate: 1_500_000.0, rolloff: 0.2, modcod: 4 }.descriptor();
        assert_eq!(d, [0x43, 11, 0x01, 0x04, 0x91, 0x50, 0x00, 0x00, 0x15, 0x00, 0x15, 0x00, 0x01]);
        let d = Delivery::T2 { freq_hz: 437_000_000.0, bw_hz: 1.7e6, plp_id: 0, t2_system_id: 0x8001 }.descriptor();
        assert_eq!(d, [0x7F, 13, 0x04, 0, 0x80, 0x01, 0x17, 0x40, 0, 0, 0x02, 0x9A, 0xCF, 0x20, 0]);
        // 2 MHz: no bandwidth code, the short form
        let d = Delivery::T2 { freq_hz: 437_000_000.0, bw_hz: 2.0e6, plp_id: 0, t2_system_id: 0x8001 }.descriptor();
        assert_eq!(d, [0x7F, 4, 0x04, 0, 0x80, 0x01]);
        // in the NIT's transport stream loop, after the service list
        let mut m = Mux::new(300_000.0, "SQ6EMM").with_delivery(Delivery::S2 { freq_hz: 1_255_000_000.0, symbol_rate: 333_000.0, rolloff: 0.35, modcod: 14 });
        let pkt = (0..2000).map(|_| m.next()).find(|p| ((p[1] as u16 & 0x1F) << 8 | p[2] as u16) == PID_NIT).unwrap();
        assert!(pkt.windows(3).any(|w| w == [0x43, 11, 0x00]));
        let sec = &pkt[5..];
        let len = ((sec[1] as usize & 0x0F) << 8 | sec[2] as usize) + 3;
        assert_eq!(crc32(&sec[..len]), 0);
    }

    /// An access unit delimiter after a 3-byte start code is the encoder's:
    /// no second one.
    #[test]
    fn one_aud_per_access_unit() {
        for (data, auds) in [(vec![0, 0, 1, 0x09, 0xF0, 0, 0, 1, 0x65, 1, 2], 1), (vec![0, 0, 0, 1, 0x09, 0xF0, 0, 0, 1, 0x65, 1], 1), (vec![0, 0, 0, 1, 0x65, 1, 2, 3], 1)] {
            let mut m = Mux::new(300_000.0, "X");
            m.push(Media::Video { ts_us: 0, key: true, data });
            let es: Vec<u8> = (0..400).map(|_| m.next()).filter(|p| ((p[1] as u16 & 0x1F) << 8 | p[2] as u16) == PID_VIDEO && p[3] & 0x10 != 0).flat_map(|p| {
                let at = if p[3] & 0x20 != 0 { 5 + p[4] as usize } else { 4 };
                p[at..].to_vec()
            }).collect();
            let n = es.windows(4).filter(|w| w[..3] == [0, 0, 1] && w[3] & 0x1F == 9).count();
            assert_eq!(n, auds, "{es:02x?}");
        }
    }

    /// Sections over several packets (and two in one), a PMT that moves the
    /// streams, a duplicate packet, a PTS that wraps.
    #[test]
    fn demux_survives_foreign_streams() {
        let mut dmx = Demux::default();
        let mut msgs = Vec::new();
        let mut cc = std::collections::HashMap::<u16, u8>::new();
        let mut pk = |pid: u16, pusi: bool, payload: &[u8]| -> [u8; TS_LEN] {
            let c = cc.entry(pid).or_insert(0);
            let mut p = [0xFFu8; TS_LEN];
            p[..4].copy_from_slice(&[0x47, ((pusi as u8) << 6) | (pid >> 8) as u8, pid as u8, 0x10 | *c]);
            *c = (*c + 1) & 15;
            p[4..4 + payload.len()].copy_from_slice(payload);
            p
        };
        let pat = finish_section(vec![0x00, 0xB0, 0, 0x00, 0x01, 0xC1, 0, 0, 0x00, 0x01, 0xE0 | 0x10, 0x00]);
        // a PMT over 300 bytes (a long descriptor loop), two packets
        let mut pmt = vec![0x02, 0xB0, 0, 0x00, 0x01, 0xC1, 0, 0, 0xE2, 0x00, 0xF0, 0];
        pmt.extend_from_slice(&[0x1B, 0xE2, 0x00, 0xF0, 0]);
        let mut desc = vec![0x05, 4, b'O', b'p', b'u', b's'];
        for _ in 0..40 {
            desc.extend_from_slice(&[0x0A, 4, b'e', b'n', b'g', 0]);
        }
        pmt.extend_from_slice(&[0x06, 0xE2, 0x01, 0xF0 | (desc.len() >> 8) as u8, desc.len() as u8]);
        pmt.extend_from_slice(&desc);
        let pmt = finish_section(pmt);
        assert!(pmt.len() > 184);
        let mut first = vec![0u8];
        first.extend_from_slice(&pat);
        dmx.push(&pk(0, true, &first), &mut msgs);
        let mut a = vec![0u8];
        a.extend_from_slice(&pmt[..183]);
        dmx.push(&pk(0x1000, true, &a), &mut msgs);
        let b = pk(0x1000, false, &pmt[183..]);
        dmx.push(&b, &mut msgs);
        // (a duplicate of the last packet: ignored)
        dmx.push(&b, &mut msgs);
        assert_eq!((dmx.pmt, dmx.video, dmx.audio), (Some(0x1000), Some(0x200), Some(0x201)));
        // a PES near the 33-bit wrap, then one just after it
        let pes = |pts: u64, i: u8| -> Vec<u8> {
            let mut v = vec![0, 0, 1, 0xE0, 0, 0, 0x84, 0x80, 5];
            v.extend_from_slice(&pts_bytes(pts));
            v.extend_from_slice(&[0, 0, 0, 1, 0x09, 0xF0, 0, 0, 1, 0x41, i]);
            v
        };
        for (i, pts) in [(1u64 << 33) - 9000, 9000, 18000].into_iter().enumerate() {
            dmx.push(&pk(0x200, true, &pes(pts, i as u8)), &mut msgs);
        }
        let ts: Vec<i64> = msgs.iter().filter(|m| m[0] == 6).map(|m| i64::from_le_bytes(m[2..10].try_into().unwrap())).collect();
        assert_eq!(ts.len(), 2);
        assert_eq!(ts[1] - ts[0], 200_000, "{ts:?}");
        // the PAT moves the PMT: streams forgotten until it comes
        let pat2 = finish_section(vec![0x00, 0xB0, 0, 0x00, 0x01, 0xC3, 0, 0, 0x00, 0x01, 0xE0 | 0x03, 0x00]);
        let mut v = vec![0u8];
        v.extend_from_slice(&pat2);
        dmx.push(&pk(0, true, &v), &mut msgs);
        assert_eq!((dmx.pmt, dmx.video), (Some(0x300), None));
    }
}
