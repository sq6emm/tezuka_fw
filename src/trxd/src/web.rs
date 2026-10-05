//! The web UI: HTTPS + one WebSocket per browser, served by trxd itself.
//!
//! * `GET /` — the single-page UI (`web/index.html`, compiled in).
//! * `POST /login` — password in, `Set-Cookie: trxd=<session>` out.
//! * `GET /ws` — WebSocket, session cookie required:
//!   - text frames, both ways: JSON (state and decodes out, commands in);
//!   - binary frames out: `[1][f64 center][f64 span][u16 n][n x u8]` a
//!     spectrum row (u8 = (dBFS + 160) * 1.5, 0 = no data),
//!     `[2][u8...]` 12 kHz mu-law RX audio (`[9][u8...]` 24 kHz in WFM);
//!   - binary frames in: `[3][u8...]` 12 kHz mu-law microphone audio;
//!     `[4][flags][i64 us][H.264]` and `[5][i64 us][Opus]` DATV video and
//!     audio (see [`crate::dvbs2::ts::Media::from_ws`]), from the client
//!     that holds the transmitter only;
//!   - binary frames out for DATV reception: `[6]` video, `[7]` audio (see
//!     [`crate::dvbs2::ts::Demux`]), `[8][i8 re, i8 im]...` the constellation
//!     of the last frame (ideal QPSK points at +-40).
//! * Port 80 only redirects to HTTPS: browsers give the microphone to secure
//!   pages alone.
//!
//! Operator-only: every logged-in browser may do everything. Sessions live in
//! memory and /run/trxd-sessions (tmpfs): a restart of trxd keeps them (an
//! FPGA bitstream switch restarts it), a reboot logs everyone out.
//!
//! Threads: an acceptor per port, one thread per connection. The engine talks
//! to all of them through [`WebHandle`] and never blocks on a slow browser —
//! each connection has a short queue that drops when full.

use std::collections::{HashMap, VecDeque};
use std::io::{BufRead, BufReader, Read, Write};
use std::net::{IpAddr, TcpListener, TcpStream};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use crossbeam_channel::{Receiver, Sender, TrySendError, bounded, unbounded};
use serde::Deserialize;
use tracing::{debug, info, warn};
use tungstenite::protocol::Role;
use tungstenite::{Message, WebSocket};

static INDEX_HTML: &[u8] = include_bytes!("../web/index.html");
static RADE_WORKER: &[u8] = include_bytes!("../web/rade-worker.js");

const SESSION_TTL: Duration = Duration::from_secs(7 * 24 * 3600);
const SESSIONS_FILE: &str = "/run/trxd-sessions";

/// The sessions to /run: a token and its age (s) a line.
fn save_sessions(s: &HashMap<String, Instant>) {
    let text: String = s.iter().map(|(tok, t)| format!("{tok} {}\n", t.elapsed().as_secs())).collect();
    write_private(Path::new(SESSIONS_FILE), text.as_bytes());
}

/// Write a file only its owner may read (a key, session tokens).
fn write_private(p: &Path, data: &[u8]) -> bool {
    use std::os::unix::fs::{OpenOptionsExt, PermissionsExt};
    let f = std::fs::OpenOptions::new().write(true).create(true).truncate(true).mode(0o600).open(p);
    let ok = f.and_then(|mut f| f.write_all(data)).is_ok();
    let _ = std::fs::set_permissions(p, std::fs::Permissions::from_mode(0o600));
    ok
}

fn load_sessions() -> HashMap<String, Instant> {
    let now = Instant::now();
    std::fs::read_to_string(SESSIONS_FILE)
        .unwrap_or_default()
        .lines()
        .filter_map(|l| {
            let (tok, age) = l.split_once(' ')?;
            let age = Duration::from_secs(age.trim().parse().ok()?);
            (age < SESSION_TTL).then_some(())?;
            Some((tok.to_string(), now.checked_sub(age)?))
        })
        .collect()
}
/// Microphone audio kept at most (12 kHz samples): 0.5 s.
const MIC_CAP: usize = 6_000;

#[derive(Debug, Clone, Deserialize)]
#[serde(default, deny_unknown_fields)]
pub struct WebConfig {
    pub enabled: bool,
    pub bind: String,
    pub https_port: u16,
    /// Plain-HTTP port that redirects to HTTPS (0 = off).
    pub http_port: u16,
    /// Login password. Empty: one is generated on first start and kept in
    /// `<state_dir>/password` (read it over SSH).
    pub password: String,
    /// Certificate, key and generated password live here.
    pub state_dir: String,
    /// RADE module for the page (src/trxd/src/rade.rs): a `RAD1` blob or
    /// `rade.wasm.gz`. Empty: the flash partition labelled `model`.
    pub rade_wasm: String,
}

impl Default for WebConfig {
    fn default() -> Self {
        WebConfig {
            enabled: true,
            bind: "0.0.0.0".into(),
            https_port: 443,
            http_port: 80,
            password: String::new(),
            state_dir: "/mnt/jffs2/trxd-web".into(),
            rade_wasm: String::new(),
        }
    }
}

enum Out {
    Text(String),
    Bin(Vec<u8>),
}

struct Client {
    id: u64,
    tx: Sender<Out>,
    /// DATV video/audio: its own, deeper queue. A DVB-S2 frame at the low
    /// rates decodes a second of stream at once (40 audio packets, video,
    /// SI), more than the 64 of `tx` that spectrum rows and state share;
    /// those overflowed and dropped most of the sound.
    media: Sender<Vec<u8>>,
}

/// DATV media messages queued per browser.
const MEDIA_QUEUE: usize = 1024;
/// Media messages dropped because a browser stopped reading (logged).
static MEDIA_DROPPED: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);

struct Shared {
    clients: Mutex<Vec<Client>>,
    sessions: Mutex<HashMap<String, Instant>>,
    password: String,
    mic: Mutex<VecDeque<f32>>,
    /// Client currently transmitting from its microphone (or camera).
    mic_owner: Mutex<Option<u64>>,
    /// DATV video/audio messages from that client, as received.
    media: Mutex<VecDeque<Vec<u8>>>,
    /// The RADE module, once read from flash (None: there is none).
    rade: std::sync::OnceLock<Option<crate::rade::Wasm>>,
}

/// A command from a browser, tagged with the connection it came from.
pub struct WebCmd {
    pub client: u64,
    pub msg: serde_json::Value,
}

/// The engine's side of the web server.
pub struct WebHandle {
    shared: Arc<Shared>,
    cmd_rx: Receiver<WebCmd>,
    joined_rx: Receiver<u64>,
}

impl WebHandle {
    pub fn clients(&self) -> usize {
        self.shared.clients.lock().unwrap().len()
    }

    fn broadcast(&self, make: impl Fn() -> Out) {
        let mut clients = self.shared.clients.lock().unwrap();
        clients.retain(|c| match c.tx.try_send(make()) {
            Ok(()) | Err(TrySendError::Full(_)) => true,
            Err(TrySendError::Disconnected(_)) => false,
        });
    }

    pub fn send_json(&self, v: &serde_json::Value) {
        let s = v.to_string();
        self.broadcast(|| Out::Text(s.clone()));
    }

    /// Send to one client only (e.g. the full state on connect).
    pub fn send_json_to(&self, client: u64, v: &serde_json::Value) {
        let clients = self.shared.clients.lock().unwrap();
        if let Some(c) = clients.iter().find(|c| c.id == client) {
            let _ = c.tx.try_send(Out::Text(v.to_string()));
        }
    }

    pub fn send_spectrum(&self, center_hz: f64, span_hz: f64, db: &[u8]) {
        let mut b = Vec::with_capacity(19 + db.len());
        b.push(1u8);
        b.extend_from_slice(&center_hz.to_le_bytes());
        b.extend_from_slice(&span_hz.to_le_bytes());
        b.extend_from_slice(&(db.len() as u16).to_le_bytes());
        b.extend_from_slice(db);
        self.broadcast(|| Out::Bin(b.clone()));
    }

    /// Any binary message to every browser (DATV video/audio), on the media
    /// queue: never silently dropped while the browser keeps reading.
    pub fn send_bin(&self, b: Vec<u8>) {
        let mut clients = self.shared.clients.lock().unwrap();
        clients.retain(|c| match c.media.try_send(b.clone()) {
            Ok(()) => true,
            Err(TrySendError::Full(_)) => {
                let n = MEDIA_DROPPED.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                if n % 100 == 0 {
                    warn!(client = c.id, dropped = n + 1, "web: DATV media dropped (the browser is not reading)");
                }
                true
            }
            Err(TrySendError::Disconnected(_)) => false,
        });
    }

    /// 12 kHz audio, -1..1.
    pub fn send_audio(&self, audio: &[f32]) {
        if audio.is_empty() {
            return;
        }
        let mut b = Vec::with_capacity(1 + audio.len());
        b.push(2u8);
        b.extend(audio.iter().map(|&s| mulaw_encode(s)));
        self.broadcast(|| Out::Bin(b.clone()));
    }

    /// 24 kHz mu-law receive audio (frame type 9): wide FM, whose audio
    /// reaches 15 kHz and would sound like a telephone at 12 kHz.
    pub fn send_audio24(&self, audio: &[f32]) {
        if audio.is_empty() {
            return;
        }
        let mut b = Vec::with_capacity(1 + audio.len());
        b.push(9u8);
        b.extend(audio.iter().map(|&s| mulaw_encode(s)));
        self.broadcast(|| Out::Bin(b.clone()));
    }

    pub fn poll_cmds(&self) -> Vec<WebCmd> {
        self.cmd_rx.try_iter().collect()
    }

    /// Connections opened since the last call (to send them the full state).
    pub fn poll_joined(&self) -> Vec<u64> {
        self.joined_rx.try_iter().collect()
    }

    /// Take up to `n` samples of 12 kHz microphone audio.
    pub fn take_mic(&self, n: usize, out: &mut Vec<f32>) -> usize {
        let mut m = self.shared.mic.lock().unwrap();
        let k = n.min(m.len());
        out.extend(m.drain(..k));
        k
    }

    pub fn mic_queued(&self) -> usize {
        self.shared.mic.lock().unwrap().len()
    }

    pub fn set_mic_owner(&self, client: Option<u64>) {
        *self.shared.mic_owner.lock().unwrap() = client;
        if client.is_none() {
            self.shared.mic.lock().unwrap().clear();
            self.shared.media.lock().unwrap().clear();
        }
    }

    /// DATV messages received since the last call.
    pub fn take_media(&self) -> Vec<Vec<u8>> {
        self.shared.media.lock().unwrap().drain(..).collect()
    }
}

// ---------------------------------------------------------------- mu-law

/// ITU-T G.711 mu-law, the whole codec in two small functions: 8 bits a
/// sample, ~38 dB of dynamic range, 96 kbit/s at 12 kHz. Plenty for a 3 kHz
/// SSB channel and trivial for the browser to decode.
pub fn mulaw_encode(x: f32) -> u8 {
    const BIAS: i32 = 0x84;
    let mut s = (x.clamp(-1.0, 1.0) * 32_635.0) as i32;
    let sign = if s < 0 {
        s = -s;
        0x80
    } else {
        0
    };
    s += BIAS;
    let exp = (7 - (s.leading_zeros() as i32 - 17)).clamp(0, 7);
    let mant = (s >> (exp + 3)) & 0x0f;
    !(sign | (exp << 4) as i32 | mant) as u8
}

pub fn mulaw_decode(u: u8) -> f32 {
    let u = !u;
    let sign = u & 0x80;
    let exp = ((u >> 4) & 0x07) as i32;
    let mant = (u & 0x0f) as i32;
    let mag = (((mant << 3) + 0x84) << exp) - 0x84;
    let v = if sign != 0 { -mag } else { mag };
    v as f32 / 32_768.0
}

// ---------------------------------------------------------------- TLS

fn load_or_make_tls(dir: &PathBuf) -> Result<Arc<rustls::ServerConfig>, String> {
    let cert_p = dir.join("cert.pem");
    let key_p = dir.join("key.pem");
    if !cert_p.exists() || !key_p.exists() {
        std::fs::create_dir_all(dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        let host = crate::config::Config::hostname();
        let names = vec![host.clone(), format!("{host}.local"), "localhost".into()];
        let ck = rcgen::generate_simple_self_signed(names).map_err(|e| format!("certificate: {e}"))?;
        std::fs::write(&cert_p, ck.cert.pem()).map_err(|e| e.to_string())?;
        if !write_private(&key_p, ck.key_pair.serialize_pem().as_bytes()) {
            return Err(format!("{}: cannot write", key_p.display()));
        }
        info!(dir = %dir.display(), "generated a self-signed web certificate");
    }
    let cert_pem = std::fs::read(&cert_p).map_err(|e| e.to_string())?;
    let key_pem = std::fs::read(&key_p).map_err(|e| e.to_string())?;
    let certs = rustls::pki_types::CertificateDer::pem_slice_iter(&cert_pem)
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| format!("cert.pem: {e:?}"))?;
    let key = rustls::pki_types::PrivateKeyDer::from_pem_slice(&key_pem).map_err(|e| format!("key.pem: {e:?}"))?;
    let provider = Arc::new(rustls::crypto::ring::default_provider());
    let cfg = rustls::ServerConfig::builder_with_provider(provider)
        .with_safe_default_protocol_versions()
        .map_err(|e| e.to_string())?
        .with_no_client_auth()
        .with_single_cert(certs, key)
        .map_err(|e| format!("TLS: {e}"))?;
    Ok(Arc::new(cfg))
}

use rustls::pki_types::pem::PemObject;

fn load_or_make_password(cfg: &WebConfig, dir: &PathBuf) -> String {
    if !cfg.password.is_empty() {
        return cfg.password.clone();
    }
    let p = dir.join("password");
    if let Ok(s) = std::fs::read_to_string(&p) {
        if !s.trim().is_empty() {
            return s.trim().to_string();
        }
    }
    const ALPHABET: &[u8] = b"abcdefghjkmnpqrstuvwxyz23456789";
    let mut raw = [0u8; 12];
    let _ = getrandom::fill(&mut raw);
    let pw: String = raw.iter().map(|b| ALPHABET[*b as usize % ALPHABET.len()] as char).collect();
    let _ = std::fs::create_dir_all(dir);
    let _ = std::fs::write(&p, format!("{pw}\n"));
    warn!(file = %p.display(), "web password generated; read it there, or set web.password");
    pw
}

fn new_token() -> String {
    let mut raw = [0u8; 16];
    let _ = getrandom::fill(&mut raw);
    raw.iter().map(|b| format!("{b:02x}")).collect()
}

/// Constant-time comparison, so the password cannot be guessed by timing.
fn same(a: &str, b: &str) -> bool {
    let (a, b) = (a.as_bytes(), b.as_bytes());
    let mut d = (a.len() ^ b.len()) as u8;
    for i in 0..a.len().max(b.len()) {
        d |= a.get(i).copied().unwrap_or(0) ^ b.get(i).copied().unwrap_or(0);
    }
    d == 0
}

// ---------------------------------------------------------------- HTTP

struct Request {
    method: String,
    path: String,
    headers: HashMap<String, String>,
    body: Vec<u8>,
}

/// Longest request line or header line taken.
const MAX_LINE: u64 = 8192;
/// The whole request (TLS handshake included) within this, however slowly
/// it trickles in.
const REQUEST_DEADLINE: Duration = Duration::from_secs(20);
/// Connections at once, and from one address.
const MAX_CONNS: usize = 32;
const MAX_CONNS_PER_IP: usize = 8;
/// Failed logins from one address in a minute before it has to wait.
const MAX_LOGIN_FAILS: u32 = 5;
/// A WebSocket that has answered nothing (pongs included) this long is dead.
const WS_DEAD: Duration = Duration::from_secs(15);

fn read_line_capped<S: Read>(r: &mut BufReader<S>, line: &mut String) -> Option<usize> {
    let n = r.by_ref().take(MAX_LINE).read_line(line).ok()?;
    if n as u64 >= MAX_LINE && !line.ends_with('\n') {
        return None;
    }
    Some(n)
}

/// A connection's socket: until `until`, reads give up at that instant
/// whatever trickles in (the request phase); `None` for the WebSocket.
struct Sock {
    tcp: TcpStream,
    until: Option<Instant>,
}

impl Read for Sock {
    fn read(&mut self, b: &mut [u8]) -> std::io::Result<usize> {
        if let Some(u) = self.until {
            let left = u.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Err(std::io::ErrorKind::TimedOut.into());
            }
            let _ = self.tcp.set_read_timeout(Some(left));
        }
        self.tcp.read(b)
    }
}

impl Write for Sock {
    fn write(&mut self, b: &[u8]) -> std::io::Result<usize> {
        self.tcp.write(b)
    }
    fn flush(&mut self) -> std::io::Result<()> {
        self.tcp.flush()
    }
}

/// Connections open, per address; a [`Slot`] holds one until dropped.
static CONNS: Mutex<Option<HashMap<IpAddr, usize>>> = Mutex::new(None);

struct Slot(IpAddr);

impl Slot {
    fn take(ip: IpAddr) -> Option<Slot> {
        let mut g = CONNS.lock().ok()?;
        let m = g.get_or_insert_with(HashMap::new);
        let total: usize = m.values().sum();
        let mine = m.get(&ip).copied().unwrap_or(0);
        if total >= MAX_CONNS || mine >= MAX_CONNS_PER_IP {
            return None;
        }
        m.insert(ip, mine + 1);
        Some(Slot(ip))
    }
}

impl Drop for Slot {
    fn drop(&mut self) {
        if let Ok(mut g) = CONNS.lock() {
            if let Some(m) = g.as_mut() {
                if let Some(n) = m.get_mut(&self.0) {
                    *n -= 1;
                    if *n == 0 {
                        m.remove(&self.0);
                    }
                }
            }
        }
    }
}

/// Failed logins per address: (count, since).
static LOGIN_FAILS: Mutex<Option<HashMap<IpAddr, (u32, Instant)>>> = Mutex::new(None);

/// May `ip` try a password now? Every address, every connection counted.
fn login_allowed(ip: IpAddr) -> bool {
    let Ok(mut g) = LOGIN_FAILS.lock() else { return false };
    let m = g.get_or_insert_with(HashMap::new);
    m.retain(|_, (_, t)| t.elapsed() < Duration::from_secs(60));
    m.get(&ip).is_none_or(|(n, _)| *n < MAX_LOGIN_FAILS)
}

fn login_failed(ip: IpAddr) {
    if let Ok(mut g) = LOGIN_FAILS.lock() {
        let e = g.get_or_insert_with(HashMap::new).entry(ip).or_insert((0, Instant::now()));
        e.0 += 1;
    }
}

fn read_request<S: Read>(r: &mut BufReader<S>) -> Option<Request> {
    let mut line = String::new();
    read_line_capped(r, &mut line)?;
    let mut parts = line.split_whitespace();
    let method = parts.next()?.to_string();
    let path = parts.next()?.to_string();
    let mut headers = HashMap::new();
    loop {
        let mut h = String::new();
        if read_line_capped(r, &mut h)? == 0 {
            return None;
        }
        let h = h.trim_end();
        if h.is_empty() {
            break;
        }
        if let Some((k, v)) = h.split_once(':') {
            headers.insert(k.trim().to_ascii_lowercase(), v.trim().to_string());
        }
        if headers.len() > 64 {
            return None;
        }
    }
    let len: usize = headers.get("content-length").and_then(|v| v.parse().ok()).unwrap_or(0);
    if len > 4096 {
        return None;
    }
    let mut body = vec![0u8; len];
    r.read_exact(&mut body).ok()?;
    Some(Request { method, path, headers, body })
}

fn cookie(req: &Request, name: &str) -> Option<String> {
    req.headers.get("cookie")?.split(';').find_map(|kv| {
        let (k, v) = kv.trim().split_once('=')?;
        (k == name).then(|| v.to_string())
    })
}

fn respond(w: &mut impl Write, status: &str, headers: &[(&str, String)], body: &[u8]) {
    let mut h = format!("HTTP/1.1 {status}\r\nContent-Length: {}\r\nConnection: close\r\n", body.len());
    for (k, v) in headers {
        h.push_str(&format!("{k}: {v}\r\n"));
    }
    h.push_str("Strict-Transport-Security: max-age=31536000\r\nX-Frame-Options: DENY\r\n\r\n");
    let _ = w.write_all(h.as_bytes());
    let _ = w.write_all(body);
    let _ = w.flush();
}

fn url_decode(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b'+' => out.push(b' '),
            b'%' if i + 2 < b.len() => {
                if let Ok(v) = u8::from_str_radix(std::str::from_utf8(&b[i + 1..i + 3]).unwrap_or("zz"), 16) {
                    out.push(v);
                    i += 2;
                }
            }
            c => out.push(c),
        }
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

fn session_ok(shared: &Shared, req: &Request) -> bool {
    let Some(tok) = cookie(req, "trxd") else { return false };
    let mut s = shared.sessions.lock().unwrap();
    s.retain(|_, t| t.elapsed() < SESSION_TTL);
    s.contains_key(&tok)
}

type Tls = rustls::StreamOwned<rustls::ServerConnection, Sock>;

fn handle_https(
    tcp: TcpStream,
    tls: Arc<rustls::ServerConfig>,
    shared: Arc<Shared>,
    cmd_tx: Sender<WebCmd>,
    joined_tx: Sender<u64>,
    id: u64,
) {
    let Ok(peer) = tcp.peer_addr() else { return };
    let Some(_slot) = Slot::take(peer.ip()) else {
        debug!(%peer, "web: too many connections");
        return;
    };
    let _ = tcp.set_nodelay(true);
    let Ok(conn) = rustls::ServerConnection::new(tls) else { return };
    let sock = Sock { tcp, until: Some(Instant::now() + REQUEST_DEADLINE) };
    let mut reader = BufReader::new(rustls::StreamOwned::new(conn, sock));
    let Some(req) = read_request(&mut reader) else { return };
    debug!(method = %req.method, path = %req.path, "web request");
    let authed = session_ok(&shared, &req);
    let path = req.path.split('?').next().unwrap_or("/").to_string();
    match (req.method.as_str(), path.as_str()) {
        ("GET", "/" | "/index.html") => respond(
            reader.get_mut(),
            "200 OK",
            &[
                ("Content-Type", "text/html; charset=utf-8".into()),
                ("Cache-Control", "no-cache".into()),
                ("Content-Security-Policy", "default-src 'self' 'unsafe-inline' 'wasm-unsafe-eval' blob:; connect-src 'self' wss:".into()),
            ],
            INDEX_HTML,
        ),
        ("GET", "/favicon.ico") => respond(reader.get_mut(), "204 No Content", &[], b""),
        ("GET", "/rade-info" | "/rade-worker.js" | "/rade.wasm") if !authed => {
            respond(reader.get_mut(), "403 Forbidden", &[], b"log in first")
        }
        ("GET", "/rade-info") => {
            let info = match shared.rade.get() {
                Some(Some(w)) => serde_json::json!({ "tag": w.tag, "bytes": w.gz.len() }),
                Some(None) => serde_json::json!({ "tag": null }),
                None => serde_json::json!({ "tag": null, "loading": true }),
            };
            respond(
                reader.get_mut(),
                "200 OK",
                &[("Content-Type", "application/json".into()), ("Cache-Control", "no-store".into())],
                info.to_string().as_bytes(),
            )
        }
        ("GET", "/rade-worker.js") => respond(
            reader.get_mut(),
            "200 OK",
            &[("Content-Type", "text/javascript".into()), ("Cache-Control", "no-cache".into())],
            RADE_WORKER,
        ),
        ("GET", "/rade.wasm") => match shared.rade.get() {
            Some(Some(w)) => respond(
                reader.get_mut(),
                "200 OK",
                &[
                    ("Content-Type", "application/wasm".into()),
                    ("Content-Encoding", "gzip".into()),
                    // The page asks for /rade.wasm?v=<tag>.
                    ("Cache-Control", "private, max-age=31536000, immutable".into()),
                ],
                &w.gz,
            ),
            _ => respond(reader.get_mut(), "404 Not Found", &[], b"no RADE module on this board"),
        },
        ("GET", "/session") => respond(
            reader.get_mut(),
            "200 OK",
            &[("Content-Type", "application/json".into()), ("Cache-Control", "no-store".into())],
            if authed { b"{\"ok\":true}" } else { b"{\"ok\":false}" },
        ),
        ("POST", "/login") if !login_allowed(peer.ip()) => {
            warn!(%peer, "web login: too many failures, refused for a minute");
            respond(reader.get_mut(), "429 Too Many Requests", &[("Retry-After", "60".into())], b"too many attempts, wait a minute");
        }
        ("POST", "/login") => {
            let body = String::from_utf8_lossy(&req.body).to_string();
            let pw = body
                .split('&')
                .find_map(|kv| kv.strip_prefix("password=").map(url_decode))
                .unwrap_or_default();
            if same(&pw, &shared.password) {
                let tok = new_token();
                {
                    let mut s = shared.sessions.lock().unwrap();
                    s.insert(tok.clone(), Instant::now());
                    save_sessions(&s);
                }
                info!("web login");
                respond(
                    reader.get_mut(),
                    "303 See Other",
                    &[
                        ("Location", "/".into()),
                        ("Set-Cookie", format!("trxd={tok}; Path=/; Max-Age=604800; HttpOnly; Secure; SameSite=Strict")),
                    ],
                    b"",
                );
            } else {
                warn!(%peer, "web login refused");
                login_failed(peer.ip());
                std::thread::sleep(Duration::from_secs(1));
                respond(reader.get_mut(), "303 See Other", &[("Location", "/?bad=1".into())], b"");
            }
        }
        ("GET", "/logout") => {
            if let Some(tok) = cookie(&req, "trxd") {
                let mut s = shared.sessions.lock().unwrap();
                s.remove(&tok);
                save_sessions(&s);
            }
            respond(
                reader.get_mut(),
                "303 See Other",
                &[("Location", "/".into()), ("Set-Cookie", "trxd=; Path=/; Max-Age=0; HttpOnly; Secure".into())],
                b"",
            );
        }
        ("GET", "/ws") => {
            let key = req.headers.get("sec-websocket-key").cloned();
            let upgrade = req.headers.get("upgrade").is_some_and(|u| u.eq_ignore_ascii_case("websocket"));
            match (authed, upgrade, key) {
                (true, true, Some(key)) => {
                    let accept = tungstenite::handshake::derive_accept_key(key.as_bytes());
                    let resp = format!(
                        "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: {accept}\r\n\r\n"
                    );
                    let mut stream = reader.into_inner();
                    if stream.write_all(resp.as_bytes()).and_then(|_| stream.flush()).is_err() {
                        return;
                    }
                    run_ws(stream, shared, cmd_tx, joined_tx, id);
                }
                (false, ..) => respond(reader.get_mut(), "401 Unauthorized", &[], b"login required"),
                _ => respond(reader.get_mut(), "400 Bad Request", &[], b""),
            }
        }
        _ => respond(reader.get_mut(), "404 Not Found", &[], b"not found"),
    }
}

fn run_ws(stream: Tls, shared: Arc<Shared>, cmd_tx: Sender<WebCmd>, joined_tx: Sender<u64>, id: u64) {
    // Short read timeout: this thread alternates between reading the browser
    // and writing whatever the engine queued for it.
    let mut stream = stream;
    stream.sock.until = None;
    let _ = stream.sock.tcp.set_read_timeout(Some(Duration::from_millis(15)));
    let mut ws = WebSocket::from_raw_socket(stream, Role::Server, None);
    let (tx, rx) = bounded::<Out>(64);
    let (media_tx, media_rx) = bounded::<Vec<u8>>(MEDIA_QUEUE);
    shared.clients.lock().unwrap().push(Client { id, tx, media: media_tx });
    let _ = joined_tx.send(id);
    info!(client = id, "web client connected");
    let mut last_ping = Instant::now();
    let mut last_heard = Instant::now();
    'outer: loop {
        loop {
            let got = ws.read();
            if got.is_ok() {
                last_heard = Instant::now();
            }
            match got {
                Ok(Message::Text(t)) => match serde_json::from_str::<serde_json::Value>(t.as_str()) {
                    Ok(v) => {
                        let _ = cmd_tx.send(WebCmd { client: id, msg: v });
                    }
                    Err(e) => debug!("web: bad JSON: {e}"),
                },
                Ok(Message::Binary(b)) => {
                    let owner = *shared.mic_owner.lock().unwrap() == Some(id);
                    match b.first() {
                        Some(&3) if owner => {
                            let mut m = shared.mic.lock().unwrap();
                            m.extend(b[1..].iter().map(|&u| mulaw_decode(u)));
                            let excess = m.len().saturating_sub(MIC_CAP);
                            m.drain(..excess);
                        }
                        Some(&4 | &5) if owner => {
                            let mut m = shared.media.lock().unwrap();
                            // The engine drains this every block; a cap only for a stuck engine.
                            if m.len() < 500 {
                                m.push_back(b.to_vec());
                            }
                        }
                        _ => {}
                    }
                }
                Ok(Message::Close(_)) => break 'outer,
                Ok(_) => {}
                Err(tungstenite::Error::Io(e))
                    if matches!(e.kind(), std::io::ErrorKind::WouldBlock | std::io::ErrorKind::TimedOut) =>
                {
                    break;
                }
                Err(_) => break 'outer,
            }
        }
        for b in media_rx.try_iter().take(256) {
            if ws.write(Message::binary(b)).is_err() {
                break 'outer;
            }
        }
        for out in rx.try_iter().take(64) {
            let msg = match out {
                Out::Text(s) => Message::text(s),
                Out::Bin(b) => Message::binary(b),
            };
            if ws.write(msg).is_err() {
                break 'outer;
            }
        }
        // Pinged every 5 s; a browser answers at once, so silence means a
        // dead link (and a key held down through it is released).
        if last_ping.elapsed() > Duration::from_secs(5) {
            last_ping = Instant::now();
            let _ = ws.write(Message::Ping(Vec::new().into()));
        }
        if last_heard.elapsed() > WS_DEAD {
            warn!(client = id, "web client silent for {} s: dropped", WS_DEAD.as_secs());
            break;
        }
        match ws.flush() {
            Ok(()) => {}
            Err(tungstenite::Error::Io(e)) if e.kind() == std::io::ErrorKind::WouldBlock => {}
            Err(_) => break,
        }
    }
    shared.clients.lock().unwrap().retain(|c| c.id != id);
    {
        let mut owner = shared.mic_owner.lock().unwrap();
        if *owner == Some(id) {
            *owner = None;
            shared.mic.lock().unwrap().clear();
        }
    }
    let _ = cmd_tx.send(WebCmd { client: id, msg: serde_json::json!({"cmd": "disconnected"}) });
    info!(client = id, "web client gone");
}

fn redirect_http(listener: TcpListener, https_port: u16) {
    for tcp in listener.incoming().flatten() {
        let Some(slot) = tcp.peer_addr().ok().and_then(|p| Slot::take(p.ip())) else { continue };
        std::thread::spawn(move || {
            let _slot = slot;
            let mut r = BufReader::new(Sock { tcp, until: Some(Instant::now() + Duration::from_secs(5)) });
            let Some(req) = read_request(&mut r) else { return };
            let host = req.headers.get("host").map(|h| h.split(':').next().unwrap_or("").to_string()).unwrap_or_default();
            let port = if https_port == 443 { String::new() } else { format!(":{https_port}") };
            respond(r.get_mut(), "301 Moved Permanently", &[("Location", format!("https://{host}{port}/"))], b"");
        });
    }
}

/// Start the web server. `None` if disabled or it cannot bind / get a cert.
pub fn start(cfg: &WebConfig) -> Option<WebHandle> {
    if !cfg.enabled {
        return None;
    }
    let dir = PathBuf::from(&cfg.state_dir);
    let tls = match load_or_make_tls(&dir) {
        Ok(t) => t,
        Err(e) => {
            warn!("web UI disabled: {e}");
            return None;
        }
    };
    let password = load_or_make_password(cfg, &dir);
    let listener = match TcpListener::bind((cfg.bind.as_str(), cfg.https_port)) {
        Ok(l) => l,
        Err(e) => {
            warn!("web UI disabled: bind {}:{}: {e}", cfg.bind, cfg.https_port);
            return None;
        }
    };
    if cfg.http_port != 0 {
        match TcpListener::bind((cfg.bind.as_str(), cfg.http_port)) {
            Ok(l) => {
                let p = cfg.https_port;
                std::thread::Builder::new().name("web-http".into()).spawn(move || redirect_http(l, p)).ok();
            }
            Err(e) => warn!("web: HTTP redirect port {}: {e}", cfg.http_port),
        }
    }
    let shared = Arc::new(Shared {
        clients: Mutex::new(Vec::new()),
        sessions: Mutex::new(load_sessions()),
        password,
        mic: Mutex::new(VecDeque::new()),
        mic_owner: Mutex::new(None),
        media: Mutex::new(VecDeque::new()),
        rade: std::sync::OnceLock::new(),
    });
    {
        // 3.3 MB from QSPI: off the start-up path.
        let (sh, path) = (shared.clone(), cfg.rade_wasm.clone());
        std::thread::Builder::new()
            .name("rade-load".into())
            .spawn(move || {
                let _ = sh.rade.set(crate::rade::load(&path));
            })
            .ok();
    }
    let (cmd_tx, cmd_rx) = unbounded();
    let (joined_tx, joined_rx) = unbounded();
    let sh = shared.clone();
    std::thread::Builder::new()
        .name("web-https".into())
        .spawn(move || {
            let mut next_id = 1u64;
            for tcp in listener.incoming().flatten() {
                let (tls, sh, cmd_tx, joined_tx) = (tls.clone(), sh.clone(), cmd_tx.clone(), joined_tx.clone());
                let id = next_id;
                next_id += 1;
                let _ = std::thread::Builder::new()
                    .name(format!("web-{id}"))
                    .spawn(move || handle_https(tcp, tls, sh, cmd_tx, joined_tx, id));
            }
        })
        .ok()?;
    info!(port = cfg.https_port, "web UI (HTTPS)");
    Some(WebHandle { shared, cmd_rx, joined_rx })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn request_lines_are_capped() {
        let ok = b"GET / HTTP/1.1\r\nHost: x\r\n\r\n".to_vec();
        assert!(read_request(&mut BufReader::new(&ok[..])).is_some());
        let mut long = b"GET /".to_vec();
        long.extend(std::iter::repeat(b'a').take(MAX_LINE as usize + 10));
        long.extend(b" HTTP/1.1\r\n\r\n");
        assert!(read_request(&mut BufReader::new(&long[..])).is_none());
        let mut hdr = b"GET / HTTP/1.1\r\nX: ".to_vec();
        hdr.extend(std::iter::repeat(b'a').take(MAX_LINE as usize * 2));
        assert!(read_request(&mut BufReader::new(&hdr[..])).is_none());
    }

    #[test]
    fn connections_per_address_are_capped() {
        let ip: IpAddr = "192.0.2.77".parse().unwrap();
        let held: Vec<Slot> = (0..MAX_CONNS_PER_IP).map(|_| Slot::take(ip).expect("under the cap")).collect();
        assert!(Slot::take(ip).is_none());
        drop(held);
        assert!(Slot::take(ip).is_some());
    }

    #[test]
    fn logins_are_rate_limited_per_address() {
        let ip: IpAddr = "192.0.2.78".parse().unwrap();
        for _ in 0..MAX_LOGIN_FAILS {
            assert!(login_allowed(ip));
            login_failed(ip);
        }
        assert!(!login_allowed(ip));
        assert!(login_allowed("192.0.2.79".parse().unwrap()));
    }

    #[test]
    fn mulaw_round_trip_is_close() {
        for x in [-1.0f32, -0.5, -0.01, 0.0, 0.003, 0.1, 0.7, 0.99] {
            let y = mulaw_decode(mulaw_encode(x));
            let tol = 0.04 * x.abs().max(0.02); // G.711 steps are ~3 % of the level
            assert!((x - y).abs() <= tol.max(0.001), "{x} -> {y}");
        }
    }

    #[test]
    fn password_compare_and_url_decoding() {
        assert!(same("abc", "abc"));
        assert!(!same("abc", "abd"));
        assert!(!same("abc", "abcd"));
        assert_eq!(url_decode("a%20b+c%26"), "a b c&");
    }
}
