//! DATV audio as AAC-LC (ADTS in the transport stream, stream type 0x0F),
//! as DVB receivers and TVs expect. The browser sends Opus (WebCodecs has no
//! AAC encoder on Linux or in Firefox, and Opus keeps the uplink small);
//! [`Transcoder`] decodes it and encodes AAC-LC with a minimal static
//! libavcodec (package/ffmpeg-aac, aacx.c). Built without it (no
//! `FFMPEG_AAC_DIR`, e.g. a plain `cargo test`), [`Transcoder::new`] says so
//! and the stream goes out without sound.

/// Bytes of an ADTS header without CRC.
pub const ADTS_HEADER: usize = 7;
/// Samples per AAC-LC frame.
pub const FRAME: usize = 1024;

const RATES: [u32; 13] = [96000, 88200, 64000, 48000, 44100, 32000, 24000, 22050, 16000, 12000, 11025, 8000, 7350];

/// The ADTS header (MPEG-4, AAC-LC, mono, no CRC) of a raw frame of `len`
/// bytes at `rate` Hz.
pub fn adts(len: usize, rate: u32) -> [u8; ADTS_HEADER] {
    let sf = RATES.iter().position(|&r| r == rate).unwrap_or(8) as u8;
    let total = len + ADTS_HEADER;
    let (profile, ch) = (1u8, 1u8); // AAC LC = object type 2, written as 1
    [
        0xFF,
        0xF1, // sync, MPEG-4, layer 0, no CRC
        (profile << 6) | (sf << 2) | (ch >> 2),
        ((ch & 3) << 6) | ((total >> 11) & 3) as u8,
        ((total >> 3) & 0xFF) as u8,
        (((total & 7) << 5) as u8) | 0x1F,
        0xFC, // buffer fullness 0x7FF (VBR), one raw block
    ]
}

/// An ADTS frame at the start of `b`: its whole length (header included) and
/// sample rate.
pub fn adts_parse(b: &[u8]) -> Option<(usize, u32)> {
    if b.len() < ADTS_HEADER || b[0] != 0xFF || b[1] & 0xF6 != 0xF0 {
        return None;
    }
    let rate = *RATES.get(((b[2] >> 2) & 0xF) as usize)?;
    let len = ((b[3] as usize & 3) << 11) | (b[4] as usize) << 3 | (b[5] as usize) >> 5;
    (len >= ADTS_HEADER).then_some((len, rate))
}

#[cfg(has_aac)]
mod ffi {
    use std::os::raw::{c_float, c_int, c_void};
    unsafe extern "C" {
        pub fn aacx_open(rate: c_int, bitrate: c_int) -> *mut c_void;
        pub fn aacx_close(x: *mut c_void);
        pub fn aacx_frame_size(x: *const c_void) -> c_int;
        pub fn aacx_opus(x: *mut c_void, data: *const u8, len: c_int, out: *mut c_float, max: c_int) -> c_int;
        pub fn aacx_encode(x: *mut c_void, pcm: *const c_float, pts: i64, out: *mut u8, max: c_int, out_pts: *mut i64) -> c_int;
    }
}

/// The browser's Opus packets in, AAC-LC frames (with ADTS headers) out, on
/// the browser's clock.
pub struct Transcoder {
    #[cfg(has_aac)]
    x: *mut std::os::raw::c_void,
    rate: u32,
    decim: sdroxide_dsp::RealFirDecim,
    /// 48 kHz samples of the last packet, then at `rate`, not yet encoded.
    pcm48: Vec<f32>,
    pcm: Vec<f32>,
    /// Samples at `rate` taken in so far, and when sample 0 was (us).
    taken: i64,
    base_us: Option<i64>,
}

// The libavcodec contexts belong to this one value; it moves between threads
// only whole.
#[cfg(has_aac)]
unsafe impl Send for Transcoder {}

/// A jump in the browser's timestamps bigger than this restarts the timeline.
const JUMP_US: i64 = 200_000;

impl Transcoder {
    /// `rate`: 8000, 12000, 16000 or 24000 (divides 48 kHz); `bitrate`: bit/s.
    pub fn new(rate: u32, bitrate: u32) -> Result<Transcoder, String> {
        if 48_000 % rate != 0 {
            return Err(format!("AAC at {rate} Hz: not a divisor of 48 kHz"));
        }
        let m = (48_000 / rate) as usize;
        // (a longer filter the further down: 63 taps at 24 kHz to 191 at 8)
        let decim = sdroxide_dsp::RealFirDecim::new(63 + 32 * (m - 2), rate as f64 * 0.45, 48_000.0, m);
        #[cfg(has_aac)]
        {
            // SAFETY: plain values in; the context is checked and owned here.
            let x = unsafe { ffi::aacx_open(rate as i32, bitrate as i32) };
            if x.is_null() {
                return Err("libavcodec: no Opus decoder or AAC encoder".into());
            }
            // SAFETY: x is live.
            let fs = unsafe { ffi::aacx_frame_size(x) };
            if fs as usize != FRAME {
                // SAFETY: x is live and not used again.
                unsafe { ffi::aacx_close(x) };
                return Err(format!("AAC frame of {fs} samples"));
            }
            Ok(Transcoder { x, rate, decim, pcm48: Vec::new(), pcm: Vec::new(), taken: 0, base_us: None })
        }
        #[cfg(not(has_aac))]
        {
            let _ = (bitrate, decim);
            Err("built without the AAC encoder (package/ffmpeg-aac, FFMPEG_AAC_DIR)".into())
        }
    }

    pub fn rate(&self) -> u32 {
        self.rate
    }

    /// One Opus packet (capture time `ts_us`): the AAC frames it completes,
    /// each `(start time us, ADTS frame)`.
    pub fn push(&mut self, ts_us: i64, opus: &[u8]) -> Vec<(i64, Vec<u8>)> {
        let mut out = Vec::new();
        #[cfg(has_aac)]
        {
            self.pcm48.resize(5760, 0.0); // 120 ms, Opus's longest frame
            // SAFETY: x is live; the buffer is 5760 floats long.
            let n = unsafe { ffi::aacx_opus(self.x, opus.as_ptr(), opus.len() as i32, self.pcm48.as_mut_ptr(), 5760) };
            if n <= 0 {
                return out;
            }
            // The timeline: where these samples fall, re-anchored on a jump
            // (a new stream, a gap in the browser's audio).
            let rate = self.rate as i64;
            let at = |base: i64, taken: i64| base + taken * 1_000_000 / rate;
            match self.base_us {
                Some(b) if (at(b, self.taken + self.pcm.len() as i64) - ts_us).abs() <= JUMP_US => {}
                _ => self.base_us = Some(ts_us - (self.taken + self.pcm.len() as i64) * 1_000_000 / rate),
            }
            self.decim.process(&self.pcm48[..n as usize], &mut self.pcm);
            let base = self.base_us.unwrap_or(ts_us);
            let mut raw = vec![0u8; 2048];
            while self.pcm.len() >= FRAME {
                let mut pts = 0i64;
                // SAFETY: x is live; FRAME floats in, 2048 bytes room out.
                let len = unsafe { ffi::aacx_encode(self.x, self.pcm.as_ptr(), self.taken, raw.as_mut_ptr(), raw.len() as i32, &mut pts) };
                self.pcm.drain(..FRAME);
                self.taken += FRAME as i64;
                if len > 0 {
                    let mut f = adts(len as usize, self.rate).to_vec();
                    f.extend_from_slice(&raw[..len as usize]);
                    out.push((at(base, pts), f));
                }
            }
        }
        #[cfg(not(has_aac))]
        let _ = (ts_us, opus);
        out
    }
}

impl Drop for Transcoder {
    fn drop(&mut self) {
        #[cfg(has_aac)]
        // SAFETY: x is live and dropped once.
        unsafe {
            ffi::aacx_close(self.x)
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn adts_round_trip() {
        for (len, rate) in [(30, 16000), (200, 24000), (2000, 48000)] {
            let h = adts(len, rate);
            assert_eq!(adts_parse(&h), Some((len + ADTS_HEADER, rate)));
        }
        assert_eq!(adts_parse(&[0xFF, 0xF1, 0, 0, 0, 0]), None);
    }

    /// 20 ms Opus packets of silence (a CELT frame as libopus writes in DTX:
    /// there is no Opus encoder in package/ffmpeg-aac to make real ones)
    /// through the whole chain: AAC frames 64 ms apart on the browser's clock.
    #[cfg(has_aac)]
    #[test]
    fn opus_silence_becomes_aac_frames_on_the_browser_clock() {
        let mut t = Transcoder::new(16_000, 16_000).unwrap();
        let mut frames = Vec::new();
        for i in 0..50i64 {
            frames.extend(t.push(1_000_000 + i * 20_000, &[0xF8, 0xFF, 0xFE]));
        }
        // 1 s of 48 kHz -> 16000 samples -> 15 frames (one held by the encoder).
        assert!((13..=15).contains(&frames.len()), "{} frames", frames.len());
        for w in frames.windows(2) {
            assert_eq!(w[1].0 - w[0].0, 64_000);
        }
        for (_, f) in &frames {
            assert_eq!(adts_parse(f).map(|x| x.0), Some(f.len()));
        }
        // The first frame starts where the browser's first sample was, less
        // the encoder's priming (the PTS the encoder hands back).
        assert!((frames[0].0 - 1_000_000).abs() <= 64_000, "{}", frames[0].0);
    }
}
