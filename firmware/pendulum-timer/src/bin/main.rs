//! Pendulum timer: ESP32 + IR beam on GPIO22, live stats page over Wi-Fi.
//!
//! - First boot (no Wi-Fi saved): open hotspot "Pendulum-Setup". Join it, open
//!   http://192.168.4.1, pick your network, type the password. It reboots onto it.
//! - After that: http://pendulum.local (or the IP printed on the serial log).
//!   /data JSON, /reset re-zero, /forget erase Wi-Fi.
//! - Flag in or out of the beam at boot doesn't matter: which sensor level means
//!   "blocked" is learned from the first few swings.

#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]

extern crate alloc;

use alloc::{string::String, vec::Vec};
use core::cell::RefCell;
use core::fmt::Write as _;
use core::net::{Ipv4Addr, SocketAddrV4};
use core::sync::atomic::{AtomicU8, AtomicU32, Ordering};

use critical_section::Mutex;
use embassy_executor::Spawner;
use embassy_net::{
    IpEndpoint, IpListenEndpoint, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4,
    tcp::TcpSocket,
    udp::{PacketMetadata, UdpSocket},
};
use embassy_time::{Duration, Timer, with_timeout};
use embedded_io_async::Write as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    gpio::{Event, Input, InputConfig, Io, Pull},
    handler, ram,
    rng::Rng,
    time::Instant,
    timer::timg::TimerGroup,
};
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config, ControllerConfig, Interface, PowerSaveMode, WifiController,
    ap::AccessPointConfig, scan::ScanConfig, sta::StationConfig,
};
use esp_storage::FlashStorage;
use log::{info, warn};
use pendulum_timer::{creds, mdns, timing::{GLITCH_US, Polarity, Timing}};

esp_bootloader_esp_idf::esp_app_desc!();

const HOSTNAME: &str = "pendulum";
const AP_SSID: &str = "Pendulum-Setup";
const AP_IP: Ipv4Addr = Ipv4Addr::new(192, 168, 4, 1);
const PAGE: &str = include_str!("../page.html");

macro_rules! mk_static {
    ($t:ty, $val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

fn now_us() -> u64 {
    Instant::now().duration_since_epoch().as_micros()
}

// ---------------------------------------------------------------- beam sensor
// The interrupt only timestamps raw edges into a ring buffer; `beam_task` works out
// which level means "blocked" from the swing itself, then does the maths.

struct Beam {
    input: Option<Input<'static>>,
    last_t: u64,
    last_level: bool,
    buf: [(u64, bool); 64],
    head: usize,
    tail: usize,
}

static BEAM: Mutex<RefCell<Beam>> = Mutex::new(RefCell::new(Beam {
    input: None,
    last_t: 0,
    last_level: false,
    buf: [(0, false); 64],
    head: 0,
    tail: 0,
}));
static TIMING: Mutex<RefCell<Timing>> = Mutex::new(RefCell::new(Timing::new()));
/// 0 = not known yet, 1 = blocked reads LOW, 2 = blocked reads HIGH
static BLOCKED_LEVEL: AtomicU8 = AtomicU8::new(0);

#[handler]
#[ram]
fn beam_isr() {
    let t = now_us();
    critical_section::with(|cs| {
        let mut b = BEAM.borrow_ref_mut(cs);
        let b = &mut *b;
        let Some(input) = b.input.as_mut() else { return };
        if !input.is_interrupt_set() {
            return;
        }
        input.clear_interrupt();
        let level = input.is_high();
        if level == b.last_level || t.wrapping_sub(b.last_t) < GLITCH_US {
            return; // bounce
        }
        b.last_level = level;
        b.last_t = t;
        b.buf[b.head % 64] = (t, level);
        b.head = b.head.wrapping_add(1);
    });
}

fn beam_blocked_now() -> bool {
    let bh = match BLOCKED_LEVEL.load(Ordering::Relaxed) {
        0 => return false,
        l => l == 2,
    };
    critical_section::with(|cs| {
        BEAM.borrow_ref_mut(cs).input.as_mut().map(|i| i.is_high() == bh).unwrap_or(false)
    })
}

#[embassy_executor::task]
async fn beam_task() {
    let mut pol = Polarity::new();
    info!("Waiting for a few swings to learn which sensor level means 'blocked'...");
    loop {
        loop {
            let edge = critical_section::with(|cs| {
                let mut b = BEAM.borrow_ref_mut(cs);
                if b.tail == b.head {
                    return None;
                }
                let e = b.buf[b.tail % 64];
                b.tail = b.tail.wrapping_add(1);
                Some(e)
            });
            let Some((t, high)) = edge else { break };
            let known = BLOCKED_LEVEL.load(Ordering::Relaxed) != 0;
            let Some(bh) = pol.edge(t, high) else { continue };
            if !known {
                BLOCKED_LEVEL.store(if bh { 2 } else { 1 }, Ordering::Relaxed);
                info!("Beam blocked = {}", if bh { "HIGH" } else { "LOW" });
                info!("beat  half_ms  period_ms  blocked_ms  rate_s/day  err_s  amp_deg");
            }
            let pass = critical_section::with(|cs| TIMING.borrow_ref_mut(cs).edge(t, high == bh));
            if let Some(p) = pass {
                info!(
                    "{}  {:.2}  {:.2}  {:.1}  {:+.0}  {:+.2}  {:.2}",
                    p.beats, p.half_ms, p.period_ms, p.blocked_ms, p.rate_s_per_day, p.err_s, p.amp_deg
                );
            }
        }
        Timer::after(Duration::from_millis(5)).await;
    }
}

// ---------------------------------------------------------------------- main

static SCAN: Mutex<RefCell<Vec<(String, i8)>>> = Mutex::new(RefCell::new(Vec::new()));
static LAST_REQ_S: AtomicU32 = AtomicU32::new(0);

#[allow(clippy::large_stack_frames)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    esp_alloc::heap_allocator!(size: 32 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // Beam sensor on GPIO22 with pull-up. Flag in or out of the beam at boot is fine.
    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(beam_isr);
    let mut input = Input::new(peripherals.GPIO22, InputConfig::default().with_pull(Pull::Up));
    let idle_high = input.is_high();
    critical_section::with(|cs| {
        input.listen(Event::AnyEdge);
        let mut b = BEAM.borrow_ref_mut(cs);
        b.last_level = idle_high;
        b.input = Some(input);
    });
    info!("Pendulum timer ready. GPIO22 reads {} at boot", if idle_high { "HIGH" } else { "LOW" });
    spawner.spawn(beam_task().unwrap());

    creds::init(FlashStorage::new(peripherals.FLASH));
    let saved = creds::load();

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let sta_conf = match &saved {
        Some(c) => station_config(&c.ssid, &c.pass),
        None => StationConfig::default(),
    };
    let mut controller = WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(Config::Station(sta_conf)),
    )
    .expect("Wi-Fi init failed");
    // No power saving: keeps the page snappy and mDNS reliable.
    let _ = controller.set_power_saving(PowerSaveMode::None);

    // ---- normal mode: join the saved network
    if let Some(c) = &saved {
        info!("Connecting to \"{}\"...", c.ssid.as_str());
        let (stack, runner) = embassy_net::new(
            Interface::station(),
            embassy_net::Config::dhcpv4(Default::default()),
            mk_static!(StackResources<8>, StackResources::<8>::new()),
            seed,
        );
        spawner.spawn(net_task(runner).unwrap());

        let joined = matches!(
            with_timeout(Duration::from_secs(20), controller.connect_async()).await,
            Ok(Ok(_))
        );
        if joined && with_timeout(Duration::from_secs(20), stack.wait_config_up()).await.is_ok() {
            let ip = stack.config_v4().map(|c| c.address.address()).unwrap_or(Ipv4Addr::UNSPECIFIED);
            info!("Stats page: http://{HOSTNAME}.local  or  http://{ip}");
            spawner.spawn(keep_connected(controller).unwrap());
            spawner.spawn(mdns_task(stack).unwrap());
            for _ in 0..3 {
                spawner.spawn(http_task(stack, false).unwrap());
            }
            loop {
                Timer::after(Duration::from_secs(3600)).await;
            }
        }
        warn!("Couldn't join \"{}\" - starting setup hotspot", c.ssid.as_str());
        let _ = controller.disconnect_async().await;
    }

    // ---- setup mode: scan, then become an open hotspot with a config page
    match controller.scan_async(&ScanConfig::default().with_max(25)).await {
        Ok(list) => critical_section::with(|cs| {
            let mut s = SCAN.borrow_ref_mut(cs);
            for ap in list {
                let name = String::from(ap.ssid.as_str());
                if !name.is_empty() && !s.iter().any(|(n, _)| *n == name) {
                    s.push((name, ap.signal_strength));
                }
            }
            s.sort_by(|a, b| b.1.cmp(&a.1));
        }),
        Err(e) => warn!("scan failed: {e:?}"),
    }
    controller
        .set_config(&Config::AccessPoint(
            AccessPointConfig::default().with_ssid(AP_SSID.try_into().unwrap()),
        ))
        .expect("AP config failed");

    let (stack, runner) = embassy_net::new(
        Interface::access_point(),
        embassy_net::Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(AP_IP, 24),
            gateway: Some(AP_IP),
            dns_servers: Default::default(),
        }),
        mk_static!(StackResources<8>, StackResources::<8>::new()),
        seed,
    );
    spawner.spawn(net_task(runner).unwrap());
    spawner.spawn(dhcp_task(stack).unwrap());
    for _ in 0..3 {
        spawner.spawn(http_task(stack, true).unwrap());
    }
    info!("Setup mode: join hotspot \"{AP_SSID}\" and open http://{AP_IP}");

    let _keep = controller; // the AP lives as long as the controller does
    loop {
        Timer::after(Duration::from_secs(30)).await;
        // If a network is saved but was just unreachable (router reboot, power cut),
        // retry it after a while unless someone is using the setup page.
        let up_s = Instant::now().duration_since_epoch().as_secs() as u32;
        let idle_s = up_s - LAST_REQ_S.load(Ordering::Relaxed);
        if saved.is_some() && up_s > 300 && idle_s > 120 {
            info!("Retrying saved network...");
            esp_hal::system::software_reset();
        }
    }
}

fn station_config(ssid: &str, pass: &str) -> StationConfig {
    let mut c = StationConfig::default();
    if let Ok(s) = ssid.try_into() {
        c = c.with_ssid(s);
    }
    if pass.is_empty() {
        c.with_authentication(AuthenticationMethodConfig::Open)
    } else if let Ok(p) = pass.try_into() {
        c.with_authentication(AuthenticationMethodConfig::Wpa2Personal(p))
    } else {
        c
    }
}

#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}

#[embassy_executor::task]
async fn keep_connected(mut controller: WifiController<'static>) {
    loop {
        if controller.is_connected() {
            let _ = controller.wait_for_disconnect_async().await;
            warn!("Wi-Fi dropped, reconnecting");
        }
        Timer::after(Duration::from_secs(2)).await;
        if let Err(e) = controller.connect_async().await {
            warn!("reconnect failed: {e:?}");
            Timer::after(Duration::from_secs(5)).await;
        }
    }
}

// ---------------------------------------------------------------------- mDNS

#[embassy_executor::task]
async fn mdns_task(stack: Stack<'static>) {
    let group = Ipv4Addr::new(224, 0, 0, 251);
    if let Err(e) = stack.join_multicast_group(group) {
        warn!("mDNS join failed: {e:?}");
    }
    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx = [0u8; 1024];
    let mut tx = [0u8; 512];
    let mut sock = UdpSocket::new(stack, &mut rx_meta, &mut rx, &mut tx_meta, &mut tx);
    if sock.bind(5353).is_err() {
        warn!("mDNS bind failed");
        return;
    }
    let mut buf = [0u8; 512];
    let mut out = [0u8; 128];
    loop {
        let Ok((n, meta)) = sock.recv_from(&mut buf).await else { continue };
        let Some(ip) = stack.config_v4().map(|c| c.address.address().octets()) else { continue };
        let legacy = meta.endpoint.port != 5353;
        if let Some(len) = mdns::answer(&buf[..n], HOSTNAME, ip, legacy, &mut out) {
            let dest = if legacy { meta.endpoint } else { IpEndpoint::new(group.into(), 5353) };
            let _ = sock.send_to(&out[..len], dest).await;
        }
    }
}

// --------------------------------------------------------------- DHCP (setup)

#[embassy_executor::task]
async fn dhcp_task(stack: Stack<'static>) {
    use edge_dhcp::{io::{self, DEFAULT_SERVER_PORT}, server::{Server, ServerOptions}};
    use edge_nal::UdpBind;
    use edge_nal_embassy::{Udp, UdpBuffers};

    let mut buf = [0u8; 1500];
    let mut gw_buf = [Ipv4Addr::UNSPECIFIED];
    let buffers = UdpBuffers::<3, 1024, 1024, 10>::new();
    let udp = Udp::new(stack, &buffers);
    let mut sock = udp
        .bind(core::net::SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::UNSPECIFIED, DEFAULT_SERVER_PORT)))
        .await
        .unwrap();
    loop {
        let _ = io::server::run(
            &mut Server::<_, 64>::new_with_et(AP_IP),
            &ServerOptions::new(AP_IP, Some(&mut gw_buf)),
            &mut sock,
            &mut buf,
        )
        .await
        .inspect_err(|e| warn!("DHCP server error: {e:?}"));
        Timer::after(Duration::from_millis(500)).await;
    }
}

// ---------------------------------------------------------------------- HTTP

#[embassy_executor::task(pool_size = 3)]
async fn http_task(stack: Stack<'static>, setup: bool) {
    let mut rx = [0u8; 1536];
    let mut tx = [0u8; 2048];
    let mut req = [0u8; 1024];
    let mut body = String::new();
    loop {
        let mut sock = TcpSocket::new(stack, &mut rx, &mut tx);
        sock.set_timeout(Some(Duration::from_secs(10)));
        if sock.accept(IpListenEndpoint { addr: None, port: 80 }).await.is_err() {
            continue;
        }
        LAST_REQ_S.store(Instant::now().duration_since_epoch().as_secs() as u32, Ordering::Relaxed);
        let after = handle(&mut sock, setup, &mut req, &mut body).await;
        let _ = sock.flush().await;
        sock.close();
        Timer::after(Duration::from_millis(50)).await;
        sock.abort();
        match after {
            After::Nothing => {}
            After::Reboot => {
                Timer::after(Duration::from_millis(1500)).await;
                esp_hal::system::software_reset();
            }
        }
    }
}

enum After {
    Nothing,
    Reboot,
}

fn find(h: &[u8], needle: &[u8]) -> Option<usize> {
    h.windows(needle.len()).position(|w| w == needle)
}

async fn handle(sock: &mut TcpSocket<'_>, setup: bool, req: &mut [u8], body: &mut String) -> After {
    // read headers (and a small form body for POST /save)
    let mut n = 0;
    let hdr_end = loop {
        match sock.read(&mut req[n..]).await {
            Ok(0) | Err(_) => return After::Nothing,
            Ok(k) => n += k,
        }
        if let Some(i) = find(&req[..n], b"\r\n\r\n") {
            break i + 4;
        }
        if n == req.len() {
            return After::Nothing;
        }
    };
    let head = core::str::from_utf8(&req[..hdr_end]).unwrap_or("");
    let mut parts = head.split_ascii_whitespace();
    let method = String::from(parts.next().unwrap_or(""));
    let path = String::from(parts.next().unwrap_or("/").split('?').next().unwrap_or("/"));
    let (method, path) = (method.as_str(), path.as_str());
    let clen: usize = head
        .lines()
        .find_map(|l| {
            let (k, v) = l.split_once(':')?;
            k.trim().eq_ignore_ascii_case("content-length").then(|| v.trim().parse().ok())?
        })
        .unwrap_or(0);
    while method == "POST" && n < hdr_end + clen && n < req.len() {
        match sock.read(&mut req[n..]).await {
            Ok(0) | Err(_) => break,
            Ok(k) => n += k,
        }
    }

    match (method, path) {
        ("GET", "/") if setup => {
            setup_page(body);
            send(sock, "200 OK", "text/html", body.as_bytes()).await;
        }
        ("GET", "/") | ("GET", "/stats") => send(sock, "200 OK", "text/html", PAGE.as_bytes()).await,
        ("GET", "/data") => {
            let now = now_us();
            let beam = beam_blocked_now();
            let known = BLOCKED_LEVEL.load(Ordering::Relaxed) != 0;
            critical_section::with(|cs| TIMING.borrow_ref(cs).json(now, beam, known, body));
            send(sock, "200 OK", "application/json", body.as_bytes()).await;
        }
        ("GET", "/reset") => {
            critical_section::with(|cs| TIMING.borrow_ref_mut(cs).reset());
            info!("Re-zeroed");
            send(sock, "200 OK", "text/plain", b"ok").await;
        }
        ("POST", "/save") => {
            let form = core::str::from_utf8(&req[hdr_end..n.min(hdr_end + clen)]).unwrap_or("");
            let mut ssid = form_value(form, "m");
            if ssid.trim().is_empty() {
                ssid = form_value(form, "s");
            }
            let pass = form_value(form, "p");
            if creds::save(ssid.trim(), &pass) {
                info!("Saved Wi-Fi \"{}\", rebooting", ssid.trim());
                msg_page(body, "Saved. Rebooting and joining the network&hellip; rejoin your home Wi-Fi and open <b>http://pendulum.local</b>.");
                send(sock, "200 OK", "text/html", body.as_bytes()).await;
                return After::Reboot;
            }
            msg_page(body, "Couldn't save that (network name 1-32 characters, password up to 64). <a href='/'>Back</a>");
            send(sock, "400 Bad Request", "text/html", body.as_bytes()).await;
        }
        ("GET", "/forget") => {
            creds::forget();
            msg_page(body, "Wi-Fi forgotten. Rebooting into setup mode (hotspot &quot;Pendulum-Setup&quot;).");
            send(sock, "200 OK", "text/html", body.as_bytes()).await;
            return After::Reboot;
        }
        _ if setup => {
            // phones probe random URLs on a new hotspot: send them to the setup page
            let _ = sock
                .write_all(b"HTTP/1.1 302 Found\r\nLocation: http://192.168.4.1/\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")
                .await;
        }
        _ => send(sock, "404 Not Found", "text/plain", b"not found").await,
    }
    After::Nothing
}

async fn send(sock: &mut TcpSocket<'_>, status: &str, ctype: &str, body: &[u8]) {
    let mut h: heapless_header::Buf = heapless_header::Buf::new();
    let _ = write!(
        h,
        "HTTP/1.1 {status}\r\nContent-Type: {ctype}; charset=utf-8\r\nContent-Length: {}\r\n\
         Cache-Control: no-store\r\nConnection: close\r\n\r\n",
        body.len()
    );
    if sock.write_all(h.as_bytes()).await.is_ok() {
        let _ = sock.write_all(body).await;
    }
}

/// Small fixed buffer for response headers (avoids a heap allocation per request).
mod heapless_header {
    pub struct Buf {
        b: [u8; 256],
        n: usize,
    }
    impl Buf {
        pub const fn new() -> Self {
            Self { b: [0; 256], n: 0 }
        }
        pub fn as_bytes(&self) -> &[u8] {
            &self.b[..self.n]
        }
    }
    impl core::fmt::Write for Buf {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let end = self.n + s.len();
            if end > self.b.len() {
                return Err(core::fmt::Error);
            }
            self.b[self.n..end].copy_from_slice(s.as_bytes());
            self.n = end;
            Ok(())
        }
    }
}

fn form_value(form: &str, key: &str) -> String {
    let raw = form
        .split('&')
        .find_map(|kv| kv.split_once('=').filter(|(k, _)| *k == key).map(|(_, v)| v))
        .unwrap_or("");
    let b = raw.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        match b[i] {
            b'+' => out.push(b' '),
            b'%' if i + 2 < b.len() => {
                let hex = core::str::from_utf8(&b[i + 1..i + 3]).ok().and_then(|h| u8::from_str_radix(h, 16).ok());
                match hex {
                    Some(v) => {
                        out.push(v);
                        i += 2;
                    }
                    None => out.push(b'%'),
                }
            }
            c => out.push(c),
        }
        i += 1;
    }
    String::from_utf8(out).unwrap_or_default()
}

fn esc(out: &mut String, s: &str) {
    for c in s.chars() {
        match c {
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&#39;"),
            '&' => out.push_str("&amp;"),
            c => out.push(c),
        }
    }
}

const SETUP_HEAD: &str = "<!doctype html><html><head><meta charset='utf-8'>\
<meta name='viewport' content='width=device-width,initial-scale=1'><title>Pendulum Setup</title>\
<style>body{font:16px system-ui;max-width:420px;margin:24px auto;padding:0 16px}\
input,select,button{width:100%;box-sizing:border-box;font-size:16px;padding:8px;margin:6px 0 14px}\
.s{color:#666;font-size:14px}</style></head><body><h2>Pendulum Timer &ndash; Wi-Fi setup</h2>";

fn setup_page(out: &mut String) {
    out.clear();
    out.push_str(SETUP_HEAD);
    out.push_str("<form method='POST' action='/save'><label>Network</label><select name='s'>");
    critical_section::with(|cs| {
        for (name, rssi) in SCAN.borrow_ref(cs).iter() {
            out.push_str("<option value=\"");
            esc(out, name);
            out.push_str("\">");
            esc(out, name);
            let _ = write!(out, " ({rssi} dBm)</option>");
        }
    });
    out.push_str(
        "</select><label>&hellip;or type a network name</label><input name='m' autocapitalize='none'>\
         <label>Password</label><input name='p' type='password'><button>Save and connect</button></form>\
         <p class='s'>After saving, rejoin your home Wi-Fi and open <b>http://pendulum.local</b>. \
         The pendulum stats are also here: <a href='/stats'>/stats</a></p></body></html>",
    );
}

fn msg_page(out: &mut String, msg: &str) {
    out.clear();
    out.push_str(SETUP_HEAD);
    out.push_str("<p>");
    out.push_str(msg);
    out.push_str("</p></body></html>");
}
