//! Pendulum timer: ESP32 + two IR beams (sensor 1 on GPIO22, sensor 2 on
//! GPIO23), live stats page over Wi-Fi, and a run log you can download as CSV.
//!
//! - First boot (no Wi-Fi saved): open hotspot "Pendulum-Setup". Join it, open
//!   http://192.168.4.1, pick your network, type the password. It reboots onto it.
//! - After that: http://pendulum.local (or the IP printed on the serial log).
//!   /data JSON, /reset re-zero, /log.csv run log, /log/clear start a new log,
//!   /forget erase Wi-Fi.
//! - The two beams sit near the two ends of the swing. Their passes are combined
//!   into turn times (see `swing.rs`): period, half swings, amplitude each side,
//!   beam positions, and per-tooth timing. The run log holds every raw pass.
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
use pendulum_timer::{
    creds, mdns,
    runlog::{CSV_HEADER, CsvState, Full, LOG_N, RunLog},
    swing::Swing,
    timing::{GLITCH_US, Polarity},
};

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

/// Sensor index 0 = sensor 1 (GPIO22), 1 = sensor 2 (GPIO23).
const PINS: [u8; 2] = [22, 23];

struct Beam {
    input: [Option<Input<'static>>; 2],
    last_t: [u64; 2],
    last_level: [bool; 2],
    /// (time, sensor index, level after the edge)
    buf: [(u64, u8, bool); 128],
    head: usize,
    tail: usize,
}

static BEAM: Mutex<RefCell<Beam>> = Mutex::new(RefCell::new(Beam {
    input: [None, None],
    last_t: [0; 2],
    last_level: [false; 2],
    buf: [(0, 0, false); 128],
    head: 0,
    tail: 0,
}));
static SWING: Mutex<RefCell<Swing>> = Mutex::new(RefCell::new(Swing::new()));
/// Passes seen per sensor since boot.
static PASSES: [AtomicU32; 2] = [AtomicU32::new(0), AtomicU32::new(0)];
static LOG: Mutex<RefCell<RunLog>> = Mutex::new(RefCell::new(RunLog::new()));
/// Per sensor: 0 = not known yet, 1 = blocked reads LOW, 2 = blocked reads HIGH
static BLOCKED_LEVEL: [AtomicU8; 2] = [AtomicU8::new(0), AtomicU8::new(0)];

#[handler]
#[ram]
fn beam_isr() {
    let t = now_us();
    critical_section::with(|cs| {
        let mut b = BEAM.borrow_ref_mut(cs);
        let b = &mut *b;
        for s in 0..2 {
            let Some(input) = b.input[s].as_mut() else { continue };
            if !input.is_interrupt_set() {
                continue;
            }
            input.clear_interrupt();
            let level = input.is_high();
            if level == b.last_level[s] || t.wrapping_sub(b.last_t[s]) < GLITCH_US {
                continue; // bounce
            }
            b.last_level[s] = level;
            b.last_t[s] = t;
            b.buf[b.head % 128] = (t, s as u8, level);
            b.head = b.head.wrapping_add(1);
        }
    });
}

fn beam_blocked_now(s: usize) -> bool {
    let bh = match BLOCKED_LEVEL[s].load(Ordering::Relaxed) {
        0 => return false,
        l => l == 2,
    };
    critical_section::with(|cs| {
        BEAM.borrow_ref_mut(cs).input[s].as_mut().map(|i| i.is_high() == bh).unwrap_or(false)
    })
}

#[embassy_executor::task]
async fn beam_task() {
    let mut pol = [Polarity::new(), Polarity::new()];
    let mut block_t: [Option<u64>; 2] = [None, None];
    let mut log_full_told = false;
    info!("Waiting for a few swings to learn which sensor level means 'blocked'...");
    loop {
        loop {
            let edge = critical_section::with(|cs| {
                let mut b = BEAM.borrow_ref_mut(cs);
                if b.tail == b.head {
                    return None;
                }
                if b.head.wrapping_sub(b.tail) > 128 {
                    b.tail = b.head.wrapping_sub(128); // overrun: drop the oldest
                }
                let e = b.buf[b.tail % 128];
                b.tail = b.tail.wrapping_add(1);
                Some(e)
            });
            let Some((t, s, high)) = edge else { break };
            let s = s as usize;
            let known = BLOCKED_LEVEL[s].load(Ordering::Relaxed) != 0;
            let Some(bh) = pol[s].edge(t, high) else { continue };
            if !known {
                BLOCKED_LEVEL[s].store(if bh { 2 } else { 1 }, Ordering::Relaxed);
                info!("Sensor {} (GPIO{}): beam blocked = {}", s + 1, PINS[s], if bh { "HIGH" } else { "LOW" });
                info!("turn: side  t_s  amp_mm  beam_offset_mm");
            }
            if high == bh {
                block_t[s] = Some(t);
                continue;
            }
            let Some(b) = block_t[s].take() else { continue };
            let (width, mid) = (t - b, b + (t - b) / 2);
            PASSES[s].fetch_add(1, Ordering::Relaxed);
            let logged = critical_section::with(|cs| LOG.borrow_ref_mut(cs).push(s as u8 + 1, mid, width));
            if !logged && !log_full_told {
                warn!("Run log full - download /log.csv, then /log/clear to start a new one");
            }
            log_full_told = !logged;
            let turn = critical_section::with(|cs| SWING.borrow_ref_mut(cs).pass(s, mid, width));
            log_turn(turn);
        }
        let turn = critical_section::with(|cs| SWING.borrow_ref_mut(cs).poll(now_us()));
        log_turn(turn);
        Timer::after(Duration::from_millis(5)).await;
    }
}

fn log_turn(turn: Option<pendulum_timer::swing::Turn>) {
    if let Some(tn) = turn {
        match tn.amp_off {
            Some((a, d)) => info!("turn S{}  {:.6}  {a:.2}  {d:.2}", tn.side + 1, tn.t_us as f64 / 1e6),
            None => info!("turn S{}  {:.6}  (flag didn't clear the beam)", tn.side + 1, tn.t_us as f64 / 1e6),
        }
    }
}

// ---------------------------------------------------------------------- main

static SCAN: Mutex<RefCell<Vec<(String, i8)>>> = Mutex::new(RefCell::new(Vec::new()));
static LAST_REQ_S: AtomicU32 = AtomicU32::new(0);
/// Bob calibration, f32 bits: ms of period per mm (+ = bob down). NaN = none.
static CAL_K: AtomicU32 = AtomicU32::new(0x7fc0_0000);
/// Period (ms, f32 bits) noted when calibration started. 0 = not in progress.
static CAL_P1: AtomicU32 = AtomicU32::new(0);

#[allow(clippy::large_stack_frames)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let peripherals = esp_hal::init(esp_hal::Config::default().with_cpu_clock(CpuClock::max()));

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 98768);
    esp_alloc::heap_allocator!(size: 32 * 1024);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    // Beam sensors on GPIO22 and GPIO23 with pull-ups. Flag in or out of a beam
    // at boot is fine. A sensor that isn't plugged in just never reports.
    let mut io = Io::new(peripherals.IO_MUX);
    io.set_interrupt_handler(beam_isr);
    let inputs = [
        Input::new(peripherals.GPIO22, InputConfig::default().with_pull(Pull::Up)),
        Input::new(peripherals.GPIO23, InputConfig::default().with_pull(Pull::Up)),
    ];
    for (s, mut input) in inputs.into_iter().enumerate() {
        let idle_high = input.is_high();
        critical_section::with(|cs| {
            input.listen(Event::AnyEdge);
            let mut b = BEAM.borrow_ref_mut(cs);
            b.last_level[s] = idle_high;
            b.input[s] = Some(input);
        });
        info!("Sensor {} on GPIO{} reads {} at boot", s + 1, PINS[s], if idle_high { "HIGH" } else { "LOW" });
    }
    info!("Pendulum timer ready. Run log holds {LOG_N} passes.");
    spawner.spawn(beam_task().unwrap());

    creds::init(FlashStorage::new(peripherals.FLASH));
    let saved = creds::load();
    if let Some(k) = creds::load_cal() {
        CAL_K.store(k.to_bits(), Ordering::Relaxed);
        info!("Bob calibration: {k:.4} ms per mm");
    }

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

        let mut joined = false;
        for attempt in 1..=3 {
            match with_timeout(Duration::from_secs(15), controller.connect_async()).await {
                Ok(Ok(_)) => {
                    joined = true;
                    break;
                }
                Ok(Err(e)) => warn!("Join attempt {attempt}/3 failed: {e:?}"),
                Err(_) => warn!("Join attempt {attempt}/3 timed out"),
            }
            let _ = controller.disconnect_async().await;
            Timer::after(Duration::from_secs(2)).await;
        }
        if !joined {
            report_scan(&mut controller, c.ssid.as_str()).await;
        }
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

/// After a failed join: say whether the network is visible and how it's set up.
async fn report_scan(controller: &mut WifiController<'static>, want: &str) {
    match controller.scan_async(&ScanConfig::default().with_max(30)).await {
        Ok(list) => {
            let mut found = false;
            for ap in list.iter().filter(|ap| ap.ssid.as_str() == want) {
                found = true;
                warn!(
                    "Saw \"{want}\": channel {}, signal {} dBm, security {:?}",
                    ap.channel, ap.signal_strength, ap.auth_method
                );
            }
            if !found {
                warn!("\"{want}\" is not visible from here (ESP32 is 2.4 GHz only; check the name's spelling and case). Networks seen:");
                for ap in list.iter() {
                    warn!("  \"{}\" ch {} {} dBm {:?}", ap.ssid.as_str(), ap.channel, ap.signal_strength, ap.auth_method);
                }
            }
        }
        Err(e) => warn!("Scan failed: {e:?}"),
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
        c.with_authentication(AuthenticationMethodConfig::WpaWpa2Personal(p))
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
    let target = parts.next().unwrap_or("/");
    let (path, query) = target.split_once('?').unwrap_or((target, ""));
    let (path, query) = (String::from(path), String::from(query));
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
        ("GET", "/data") => send_data(sock, body).await,
        ("GET", "/cal/start") => {
            let p = critical_section::with(|cs| SWING.borrow_ref(cs).avg_period_ms()) as f32;
            if p > 0.0 {
                CAL_P1.store(p.to_bits(), Ordering::Relaxed);
                info!("Calibration started at {p:.2} ms");
                send(sock, "200 OK", "text/plain", b"ok").await;
            } else {
                send(sock, "400 Bad Request", "text/plain", b"No period measured yet - let it swing first.").await;
            }
        }
        ("GET", "/cal/finish") => {
            let p1 = f32::from_bits(CAL_P1.load(Ordering::Relaxed));
            let p2 = critical_section::with(|cs| SWING.borrow_ref(cs).avg_period_ms()) as f32;
            let mm: f32 = form_value(&query, "mm").trim().parse().unwrap_or(0.0);
            let msg: &[u8] = if p1 <= 0.0 {
                b"Press Calibrate first, before moving the bob."
            } else if mm == 0.0 || !mm.is_finite() {
                b"Enter how many mm you moved the bob (+ down, - up)."
            } else if (p2 - p1).abs() < 0.3 {
                b"The period hasn't changed measurably yet. Wait a minute after moving the bob, or move it further."
            } else {
                let k = (p2 - p1) / mm;
                CAL_K.store(k.to_bits(), Ordering::Relaxed);
                CAL_P1.store(0, Ordering::Relaxed);
                creds::save_cal(Some(k));
                info!("Calibrated: {p1:.2} -> {p2:.2} ms over {mm} mm = {k:.4} ms/mm");
                b"ok"
            };
            let st = if msg == b"ok" { "200 OK" } else { "400 Bad Request" };
            send(sock, st, "text/plain", msg).await;
        }
        ("GET", "/cal/cancel") => {
            CAL_P1.store(0, Ordering::Relaxed);
            send(sock, "200 OK", "text/plain", b"ok").await;
        }
        ("GET", "/cal/clear") => {
            CAL_K.store(f32::NAN.to_bits(), Ordering::Relaxed);
            CAL_P1.store(0, Ordering::Relaxed);
            creds::save_cal(None);
            send(sock, "200 OK", "text/plain", b"ok").await;
        }
        ("GET", "/reset") => {
            critical_section::with(|cs| SWING.borrow_ref_mut(cs).reset());
            info!("Re-zeroed");
            send(sock, "200 OK", "text/plain", b"ok").await;
        }
        ("GET", "/log.csv") => send_log(sock, body).await,
        ("GET", "/log/clear") => {
            critical_section::with(|cs| LOG.borrow_ref_mut(cs).clear());
            info!("Run log cleared");
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

/// Dashboard JSON, streamed a part at a time (no big buffer, no long lock).
async fn send_data(sock: &mut TcpSocket<'_>, body: &mut String) {
    let _ = sock
        .write_all(
            b"HTTP/1.1 200 OK\r\nContent-Type: application/json; charset=utf-8\r\n\
              Cache-Control: no-store\r\nConnection: close\r\n\r\n",
        )
        .await;
    let now = now_us();
    let mut part = 0;
    loop {
        let more = critical_section::with(|cs| SWING.borrow_ref(cs).json_part(part, now, body));
        if !more || sock.write_all(body.as_bytes()).await.is_err() {
            break;
        }
        part += 1;
    }
    // sensors, run log, calibration, and close the object
    body.clear();
    body.push_str(",\"sens\":[");
    for o in 0..2 {
        let pol = BLOCKED_LEVEL[o].load(Ordering::Relaxed) != 0;
        let _ = write!(
            body,
            "{}{{\"pin\":{},\"pol\":{pol},\"n\":{},\"beam\":{}}}",
            if o > 0 { "," } else { "" },
            PINS[o],
            PASSES[o].load(Ordering::Relaxed),
            beam_blocked_now(o)
        );
    }
    let (ln, lps, span, full) = critical_section::with(|cs| {
        let l = LOG.borrow_ref(cs);
        (l.len(), l.per_sensor(), l.span_s(), l.full())
    });
    let full = match full {
        Full::No => "null",
        Full::Buffer => "\"buffer\"",
        Full::Time => "\"time\"",
    };
    let _ = write!(
        body,
        "],\"log\":{{\"n\":{ln},\"cap\":{LOG_N},\"s1\":{},\"s2\":{},\"span\":{span:.1},\"full\":{full}}}",
        lps[0], lps[1]
    );
    // calibration: "k" ms per mm (null = not calibrated), "cal1" period noted at start
    let k = f32::from_bits(CAL_K.load(Ordering::Relaxed));
    let c1 = f32::from_bits(CAL_P1.load(Ordering::Relaxed));
    if k.is_finite() { let _ = write!(body, ",\"k\":{k:.4}"); } else { body.push_str(",\"k\":null"); }
    if c1 > 0.0 { let _ = write!(body, ",\"cal1\":{c1:.3}}}"); } else { body.push_str(",\"cal1\":null}"); }
    let _ = sock.write_all(body.as_bytes()).await;
}

/// Stream the run log as CSV, a chunk at a time so the log is never locked for long
/// and no big buffer is needed. No Content-Length: the end of the file is the
/// connection closing.
async fn send_log(sock: &mut TcpSocket<'_>, body: &mut String) {
    let _ = sock
        .write_all(
            b"HTTP/1.1 200 OK\r\nContent-Type: text/csv; charset=utf-8\r\n\
              Content-Disposition: attachment; filename=\"pendulum-run.csv\"\r\n\
              Cache-Control: no-store\r\nConnection: close\r\n\r\n",
        )
        .await;
    if sock.write_all(CSV_HEADER.as_bytes()).await.is_err() {
        return;
    }
    let mut st = CsvState::default();
    let mut i = 0;
    loop {
        body.clear();
        let (next, total) = critical_section::with(|cs| {
            let l = LOG.borrow_ref(cs);
            (l.csv_rows(i, i + 40, &mut st, body), l.len())
        });
        if body.is_empty() || sock.write_all(body.as_bytes()).await.is_err() {
            break;
        }
        i = next;
        if i >= total {
            break;
        }
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
