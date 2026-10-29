//! Host-side tests for the firmware's hardware-free modules.
extern crate alloc;

#[path = "../../pendulum-timer/src/timing.rs"]
pub mod timing;
#[path = "../../pendulum-timer/src/mdns.rs"]
pub mod mdns;
#[path = "../../pendulum-timer/src/runlog.rs"]
pub mod runlog;
#[path = "../../pendulum-timer/src/swing.rs"]
pub mod swing;
#[path = "../../pendulum-timer/src/wifitxt.rs"]
pub mod wifitxt;

#[cfg(test)]
mod tests {
    use super::timing::*;
    use alloc::string::String;

    /// Simulate a pendulum x(t) = A sin(2 pi t / T) past a beam at `offset_mm`
    /// with a flag `FLAG_WIDTH_MM` wide, sampled every `dt_us`. Returns edges.
    fn simulate(period_s: f64, amp_mm: f64, offset_mm: f64, secs: f64) -> Vec<(u64, bool)> {
        let dt_us = 10u64;
        let mut edges = vec![];
        let mut blocked = false;
        let mut t = 1_000_000u64; // start 1 s after boot
        let end = t + (secs * 1e6) as u64;
        while t < end {
            let ts = (t - 1_000_000) as f64 / 1e6;
            let x = amp_mm * (2.0 * std::f64::consts::PI * ts / period_s + 0.3).sin();
            let b = (x - offset_mm).abs() < FLAG_WIDTH_MM / 2.0;
            if b != blocked {
                edges.push((t, b));
                blocked = b;
            }
            t += dt_us;
        }
        edges
    }

    fn run(period_s: f64, amp_mm: f64, offset_mm: f64, secs: f64) -> Timing {
        let mut tm = Timing::new();
        for (t, b) in simulate(period_s, amp_mm, offset_mm, secs) {
            tm.edge(t, b);
        }
        tm
    }

    fn field(json: &str, key: &str) -> f64 {
        let k = format!("\"{key}\":");
        let i = json.find(&k).unwrap() + k.len();
        let rest = &json[i..];
        let end = rest.find(|c| c == ',' || c == '}').unwrap();
        rest[..end].parse().unwrap()
    }

    #[test]
    fn slow_clock_matches_screenshot() {
        // Luke's numbers: beats alternate ~500 / ~590 ms -> period ~1090 ms, ~3.5 deg.
        let tm = run(1.090, 15.0, 2.0, 120.0);
        let mut s = String::new();
        let now = 1_000_000 + 120_000_000;
        tm.json(now, false, true, &mut s);
        let per = field(&s, "per");
        let rate = field(&s, "rate");
        let err = field(&s, "err");
        let real = field(&s, "real");
        let be = field(&s, "be");
        let amp = field(&s, "amp");
        println!("{s:.300}");
        let lrate = field(&s, "lrate");
        assert!((lrate - rate).abs() < 30.0, "lrate {lrate}");
        println!("per {per} rate {rate} err {err} real {real} be {be} amp {amp}");
        assert!((per - 1090.0).abs() < 0.2, "period {per}");
        // rate = (1/1.090 - 1) * 86400 = -7134 s/day
        assert!((rate + 7134.0).abs() < 5.0, "rate {rate}");
        // after ~120 s, clock is behind by ~120 * 0.09/1.09 ~ 9.9 s
        let expect = -real * (0.090 / 1.090);
        assert!((err - expect).abs() < 0.6, "err {err} expect {expect}");
        assert!(be > 5.0, "offset beam should show beat error, got {be}");
        let want = (15.0f64 / FLAG_RADIUS_MM).asin().to_degrees();
        assert!((amp - want).abs() / want < 0.1, "amp {amp} want {want}");
    }

    #[test]
    fn polarity_learned_from_swing_either_way() {
        // edges as raw levels; blocked = HIGH for one sensor, LOW for another,
        // and the flag sitting in the beam at power-up
        for blocked_is_high in [true, false] {
            let mut pol = Polarity::new();
            let mut tm = Timing::new();
            let mut fed = 0;
            for (t, blocked) in simulate(1.0, 15.0, 0.0, 30.0) {
                let high = blocked == blocked_is_high;
                if let Some(bh) = pol.edge(t, high) {
                    tm.edge(t, high == bh);
                    fed += 1;
                }
            }
            assert_eq!(pol.blocked_high, Some(blocked_is_high));
            assert!(fed > 40);
            let mut s = String::new();
            tm.json(1_000_000 + 30_000_000, false, true, &mut s);
            assert!((field(&s, "per") - 1000.0).abs() < 0.5, "{s:.200}");
        }
    }

    #[test]
    fn perfect_clock_is_on_time() {
        let tm = run(1.000, 15.0, 0.0, 300.0);
        let mut s = String::new();
        tm.json(1_000_000 + 300_000_000, false, true, &mut s);
        let err = field(&s, "err");
        let rate = field(&s, "rate");
        let be = field(&s, "be");
        assert!(err.abs() < 0.002, "err {err}");
        assert!(rate.abs() < 2.0, "rate {rate}");
        assert!(be < 0.1, "be {be}");
        assert_eq!(field(&s, "gaps"), 0.0);
    }

    #[test]
    fn stop_then_falls_behind_and_reset() {
        let mut tm = run(1.000, 15.0, 0.0, 60.0);
        let last = 1_000_000 + 60_000_000;
        assert!(!tm.stopped(last));
        let later = last + 30_000_000;
        assert!(tm.stopped(later));
        let e = tm.live_err(later);
        assert!(e < -29.0 && e > -31.5, "stopped err {e}");
        tm.reset();
        let mut s = String::new();
        tm.json(later, false, true, &mut s);
        assert_eq!(field(&s, "n"), 0.0);
        assert_eq!(field(&s, "ago"), -1.0);
    }

    /// Fast edge generator for long runs: exact crossing times of a sine swing,
    /// no sampling. Two passes per period, each a (blocked, clear) pair.
    fn edges_fast(period_s: f64, amp_mm: f64, offset_mm: f64, periods: u64) -> impl Iterator<Item = (u64, bool)> {
        let w = 2.0 * std::f64::consts::PI / period_s;
        let a1 = ((offset_mm - FLAG_WIDTH_MM / 2.0) / amp_mm).asin();
        let a2 = ((offset_mm + FLAG_WIDTH_MM / 2.0) / amp_mm).asin();
        (0..periods).flat_map(move |k| {
            let base = 1.0 + k as f64 * period_s;
            let us = |t: f64| (t * 1e6).round() as u64;
            // rising side: x goes a1 -> a2; falling side: a2 -> a1
            [
                (us(base + a1 / w), true),
                (us(base + a2 / w), false),
                (us(base + (std::f64::consts::PI - a2) / w), true),
                (us(base + (std::f64::consts::PI - a1) / w), false),
            ]
        })
    }

    #[test]
    fn offset_beam_has_no_sawtooth() {
        // Perfect 1.000 s pendulum, beam 4.27 mm off centre: ~80 ms "beat error",
        // but clock vs real must stay at zero on every update.
        let mut tm = Timing::new();
        let mut worst: f64 = 0.0;
        for (t, b) in edges_fast(1.0, 17.4, 4.27, 200) {
            if let Some(p) = tm.edge(t, b) {
                worst = worst.max(p.err_s.abs());
            }
        }
        let mut s = String::new();
        tm.json(1_000_000 + 200_000_000, false, true, &mut s);
        assert!(field(&s, "be") > 70.0, "be {}", field(&s, "be"));
        assert!(worst < 2e-6, "worst err {worst}");
        assert!(field(&s, "lrate").abs() < 0.01, "lrate {}", field(&s, "lrate"));
    }

    #[test]
    fn microsecond_resolution() {
        // 1 µs slow per period -> after 1000 periods, 1.000 ms behind.
        let mut tm = Timing::new();
        for (t, b) in edges_fast(1.000_001, 17.4, 0.0, 1001) {
            tm.edge(t, b);
        }
        let mut s = String::new();
        tm.json(1_000_000 + 1_001_000_000, false, true, &mut s);
        let err = field(&s, "err");
        assert!((err + 0.001).abs() < 2e-6, "err {err}");
        assert!((field(&s, "per") - 1000.001).abs() < 0.0006, "per {}", field(&s, "per"));
    }

    #[test]
    fn week_run_keeps_precision_and_fits() {
        // Screenshot rate (1002.52 ms) for 7 days: expect -217.18 s/day * 7.
        let p = 1.00252;
        let n = (7.0 * 86400.0 / p) as u64;
        let mut tm = Timing::new();
        for (t, b) in edges_fast(p, 17.4, 4.27, n) {
            tm.edge(t, b);
        }
        let end = 1_000_000 + ((n as f64 - 1.0) * p * 1e6) as u64 + 300_000;
        let mut s = String::new();
        tm.json(end, false, true, &mut s);
        let real = field(&s, "real");
        let expect = -(p - 1.0) / p * real; // err = (1/p - 1) * real
        let err = field(&s, "err");
        assert!((err - expect).abs() < 0.002, "err {err} expect {expect}");
        assert!((field(&s, "lrate") + 217.18).abs() < 0.01, "lrate {}", field(&s, "lrate"));
        assert!(s.len() < 16_000, "json {} bytes", s.len());
        println!("week: err {err} lrate {} json {} bytes", field(&s, "lrate"), s.len());
    }

    #[test]
    fn long_run_history_decimates() {
        let tm = run(1.000, 15.0, 0.0, 1200.0); // 2400 beats -> decimated to <=300 points
        let mut s = String::new();
        tm.json(1_000_000 + 1_200_000_000, false, true, &mut s);
        let ht = &s[s.find("\"ht\":[").unwrap() + 6..];
        let n = ht[..ht.find(']').unwrap()].split(',').count();
        assert!(n <= HIST_N && n >= HIST_N / 2, "{n} points");
        assert!(s.len() < 12_000, "json {} bytes", s.len());
    }
}

#[cfg(test)]
mod swing_tests {
    use super::swing::*;
    use super::timing::FLAG_WIDTH_MM;
    use alloc::string::String;
    use std::f64::consts::PI;

    struct Sim {
        amp: f64,
        beams: [f64; 2],
        period_s: f64,
        /// half swing moving toward -x is this many ms longer than the other
        asym_ms: f64,
        bad_tooth: Option<(u64, f64)>,
    }

    /// Returns passes (sensor, mid_us, width_us) and true turn times.
    fn simulate(sim: &Sim, secs: f64) -> (Vec<(usize, u64, u64)>, Vec<u64>) {
        let dt = 5u64;
        let mut theta = 0.3f64;
        let mut t = 1_000_000u64;
        let end = t + (secs * 1e6) as u64;
        let mut blocked = [false; 2];
        let mut start = [0u64; 2];
        let mut out = vec![];
        let mut turns = vec![];
        let mut last_sign = 0.0f64;
        while t < end {
            let cycle = ((theta + PI / 2.0) / (2.0 * PI)).floor() as u64;
            let mut per = sim.period_s;
            if let Some((k, d)) = sim.bad_tooth {
                if cycle % 30 == k {
                    per -= d / 1000.0;
                }
            }
            // escapement asymmetry: going toward -x, the swing is a little slower
            // through the middle (|x| < 1/3 A, never at the turns, like an impulse)
            let moving_neg = theta.cos() < 0.0;
            let mid = theta.sin().abs() < 1.0 / 3.0;
            // the middle band is 2 asin(1/3)/pi = 21.6% of a half swing
            let k = if moving_neg && mid { 1.0 + sim.asym_ms / 1000.0 / (per / 2.0 * 0.21635) } else { 1.0 };
            theta += 2.0 * PI * (dt as f64 / 1e6) / per / k;
            let x = sim.amp * theta.sin();
            let v = theta.cos();
            if last_sign != 0.0 && v.signum() != last_sign {
                turns.push(t);
            }
            last_sign = v.signum();
            for s in 0..2 {
                let b = (x - sim.beams[s]).abs() < FLAG_WIDTH_MM / 2.0;
                if b && !blocked[s] {
                    start[s] = t;
                }
                if !b && blocked[s] {
                    out.push((s, (start[s] + t) / 2, t - start[s]));
                }
                blocked[s] = b;
            }
            t += dt;
        }
        (out, turns)
    }

    fn run(sim: &Sim, secs: f64) -> (Swing, u32) {
        let (passes, _) = simulate(sim, secs);
        let mut sw = Swing::new();
        let mut turns = 0;
        for (s, mid, w) in passes {
            sw.poll(mid);
            if sw.pass(s, mid, w).is_some() {
                turns += 1;
            }
        }
        (sw, turns)
    }

    fn json(sw: &Swing, now: u64) -> String {
        let mut all = String::new();
        let mut part = String::new();
        let mut i = 0;
        while sw.json_part(i, now, &mut part) {
            all.push_str(&part);
            i += 1;
        }
        all.push('}');
        all
    }

    fn arr(json: &str, key: &str) -> Vec<f64> {
        let k = format!("\"{key}\":[");
        let i = json.find(&k).unwrap() + k.len();
        json[i..i + json[i..].find(']').unwrap()]
            .split(',')
            .filter(|s| !s.is_empty())
            .map(|s| s.parse().unwrap_or(f64::NAN))
            .collect()
    }

    /// `DUMP=out.json cargo test --release dump_dashboard_json -- --ignored`
    /// writes realistic /data JSON for trying the page without hardware.
    #[test]
    #[ignore]
    fn dump_dashboard_json() {
        let sim = Sim { amp: 24.5, beams: [-14.6, 15.8], period_s: 1.0012, asym_ms: 3.5, bad_tooth: Some((11, 8.0)) };
        let (sw, _) = run(&sim, 420.0);
        let mut s = json(&sw, 1_000_000 + 420_000_000);
        s.pop();
        s.push_str(",\"sens\":[{\"pin\":22,\"pol\":true,\"n\":840,\"beam\":false},{\"pin\":23,\"pol\":true,\"n\":840,\"beam\":false}],\
            \"log\":{\"n\":1680,\"cap\":6144,\"s1\":840,\"s2\":840,\"span\":419.6,\"full\":null},\"k\":null,\"cal1\":null}");
        std::fs::write(std::env::var("DUMP").unwrap_or("dash.json".into()), s).unwrap();
    }

    #[test]
    fn amp_and_offset_exact() {
        // A = 25, d = 15, T = 1 s: out-and-back 292.08 ms, blocked 51.14 ms
        let w = 2.0 * PI;
        let tau1 = ((15.0 - FLAG_WIDTH_MM / 2.0) / 25.0f64).acos() / w;
        let tau2 = ((15.0 + FLAG_WIDTH_MM / 2.0) / 25.0f64).acos() / w;
        let (a, d) = amp_and_offset(tau1 + tau2, tau1 - tau2, 1.0).unwrap();
        assert!((a - 25.0).abs() < 1e-9 && (d - 15.0).abs() < 1e-9, "{a} {d}");
    }

    #[test]
    fn two_beams_find_period_halves_amplitude_beams_and_bad_tooth() {
        let sim = Sim { amp: 25.0, beams: [-14.0, 16.0], period_s: 1.0015, asym_ms: 3.5, bad_tooth: Some((7, 8.0)) };
        let (sw, turns) = run(&sim, 400.0);
        let s = json(&sw, 1_000_000 + 400_000_000);
        println!("{}", &s[..s.find("\"hp\"").unwrap()]);
        assert!(turns > 780, "{turns} turns");
        let amp = arr(&s, "amp");
        let off = arr(&s, "off");
        // within ~2%: the sim's asymmetry stretches the measured period but not
        // the swing near the turns, which the amplitude maths assumes
        assert!((amp[0] - 25.0).abs() < 0.6 && (amp[1] - 25.0).abs() < 0.6, "amp {amp:?}");
        assert!((off[0] - 14.0).abs() < 0.6 && (off[1] - 16.0).abs() < 0.6, "off {off:?}");
        // halves: sensor 1 is on the -x side, so the half ending there is the long one
        let h = arr(&s, "h");
        assert!((h[0] - h[1] - 3.5).abs() < 0.05, "halves {h:?}");
        // per tooth: the short cycle (8 ms) lands on one or two neighbouring
        // teeth (the sim's cycle starts at the other end of the swing from the
        // measured period, so it's split 4 + 4); every other tooth is 1005 ms
        let avg = arr(&s, "avg");
        let dev: Vec<f64> = avg.iter().map(|v| v - 1005.0).collect();
        let bad: Vec<usize> = (0..30).filter(|&i| dev[i].abs() > 0.05).collect();
        println!("tooth deviations {:?}", dev.iter().map(|d| (d * 1000.0).round() / 1000.0).collect::<Vec<_>>());
        assert!(!bad.is_empty() && bad.len() <= 2, "{bad:?}");
        if bad.len() == 2 {
            assert!(bad[1] - bad[0] == 1 || (bad[0] == 0 && bad[1] == 29));
        }
        let total: f64 = bad.iter().map(|&i| dev[i]).sum();
        assert!((total + 8.0).abs() < 0.05, "{total}");
        let _ = &sim;
        // clock vs real is consistent with the mean period
        let per = (30.0 * 1005.0 - 8.0) / 30.0;
        let n = sw.periods() as f64;
        let expect = n - n * per / 1000.0;
        let err: f64 = s[s.find("\"err\":").unwrap() + 6..].split(',').next().unwrap().parse().unwrap();
        assert!((err - expect).abs() < 0.02, "err {err} expect {expect}");
    }

    #[test]
    fn small_swing_that_doesnt_clear_a_beam_still_times() {
        // beam 2 at +16 mm, swing 18 mm: the flag never clears it (needs 19.2)
        let sim = Sim { amp: 18.0, beams: [-14.0, 16.0], period_s: 1.0, asym_ms: 0.0, bad_tooth: None };
        let (sw, _) = run(&sim, 120.0);
        let s = json(&sw, 1_000_000 + 120_000_000);
        let per: f64 = s[s.find("\"per\":").unwrap() + 6..].split(',').next().unwrap().parse().unwrap();
        assert!((per - 1000.0).abs() < 0.05, "{per}");
        let h = arr(&s, "h");
        assert!((h[0] - 500.0).abs() < 1.0 && (h[1] - 500.0).abs() < 1.0, "{h:?}");
        assert!(sw.periods() > 110);
        let amp = arr(&s, "amp");
        assert!((amp[0] - 18.0).abs() < 0.1 && amp[1] == 0.0, "{amp:?}");
    }

    #[test]
    fn one_missed_turn_counts_a_gap_not_a_double_period() {
        let sim = Sim { amp: 25.0, beams: [-15.0, 15.0], period_s: 1.0, asym_ms: 0.0, bad_tooth: None };
        let (passes, _) = simulate(&sim, 60.0);
        let mut sw = Swing::new();
        for (i, (s, mid, w)) in passes.into_iter().enumerate() {
            if (40..42).contains(&i) {
                continue; // lose one out-and-back pair
            }
            sw.poll(mid);
            sw.pass(s, mid, w);
        }
        let s = json(&sw, 1_000_000 + 60_000_000);
        assert!(s.contains("\"gaps\":1"), "{}", &s[..200]);
        let err: f64 = s[s.find("\"err\":").unwrap() + 6..].split(',').next().unwrap().parse().unwrap();
        assert!(err.abs() < 0.01, "{err}");
    }
}

#[cfg(test)]
mod runlog_tests {
    use super::runlog::*;
    use super::timing::*;
    use alloc::string::String;

    /// Two beams 30 mm apart (±15 mm from centre), flag 6.35 mm, swing ±25 mm,
    /// 1.000 s period, 10 minutes. Mirrors the planned end-of-stroke layout.
    fn two_sensor_run(secs: f64) -> (RunLog, [u32; 2]) {
        let (period, amp, d) = (1.0f64, 25.0f64, 15.0f64);
        let mut tm = [Timing::new(), Timing::new()];
        let mut log = RunLog::new();
        let mut blocked = [false; 2];
        let mut passes = [0u32; 2];
        let mut t = 1_000_000u64;
        let end = t + (secs * 1e6) as u64;
        while t < end {
            let ts = (t - 1_000_000) as f64 / 1e6;
            let x = amp * (2.0 * std::f64::consts::PI * ts / period + 0.3).sin();
            for (s, off) in [(0usize, -d), (1, d)] {
                let b = (x - off).abs() < FLAG_WIDTH_MM / 2.0;
                if b != blocked[s] {
                    blocked[s] = b;
                    if let Some(p) = tm[s].edge(t, b) {
                        passes[s] += 1;
                        log.push(s as u8 + 1, p.mid_us, p.width_us);
                    }
                }
            }
            t += 5;
        }
        (log, passes)
    }

    #[test]
    fn ten_minute_two_sensor_run_fits_and_csv_is_right() {
        let (log, passes) = two_sensor_run(600.0);
        assert_eq!(log.full(), Full::No);
        assert_eq!(log.len() as u32, passes[0] + passes[1]);
        assert!(log.len() > 2350 && log.len() < LOG_N / 2, "{} passes", log.len());
        let mut out = String::from(CSV_HEADER);
        let mut st = CsvState::default();
        let mut i = 0;
        while i < log.len() {
            i = log.csv_rows(i, i + 40, &mut st, &mut out);
        }
        let rows: Vec<&str> = out.lines().collect();
        assert_eq!(rows.len(), log.len() + 1);
        // every full period on either sensor is 1000 ms to within the 5 µs sim step
        let mut n = 0;
        for r in &rows[1..] {
            let c: Vec<&str> = r.split(',').collect();
            assert_eq!(c.len(), 6, "{r}");
            if !c[5].is_empty() {
                let p: f64 = c[5].parse().unwrap();
                assert!((p - 1000.0).abs() < 0.02, "{r}");
                n += 1;
            }
        }
        assert!(n > 2300);
        println!("{}\n{}\n{}\n{}", rows[0], rows[1], rows[2], rows[5]);
    }

    #[test]
    fn stops_when_full_and_keeps_the_start() {
        let mut log = RunLog::new();
        for k in 0..LOG_N as u64 + 10 {
            log.push(1, 5_000_000 + k * 500_000, 40_000);
        }
        assert_eq!(log.len(), LOG_N);
        assert_eq!(log.full(), Full::Buffer);
        let mut out = String::new();
        log.csv_rows(0, 1, &mut CsvState::default(), &mut out);
        assert_eq!(out, "1,1,0.000000,40.000,,\r\n");
        log.clear();
        assert!(log.is_empty() && log.full() == Full::No);
    }

    #[test]
    fn stops_at_the_u32_clock_limit() {
        let mut log = RunLog::new();
        assert!(log.push(2, 1_000, 1));
        assert!(log.push(2, 1_000 + u32::MAX as u64, 1));
        assert!(!log.push(2, 2_000 + u32::MAX as u64, 1));
        assert_eq!(log.full(), Full::Time);
        assert_eq!(log.per_sensor(), [0, 2]);
    }
}

#[cfg(test)]
mod wifitxt_tests {
    use super::wifitxt::parse;

    /// Pad like erased flash after esptool writes the file.
    fn flash(s: &[u8]) -> Vec<u8> {
        let mut v = s.to_vec();
        v.resize(512, 0xFF);
        v
    }

    #[test]
    fn basic() {
        assert_eq!(parse(&flash(b"ssid=Home\npass=secret\n")), Some(("Home", "secret")));
    }

    #[test]
    fn notepad_style() {
        let f = flash(b"\xEF\xBB\xBF# my wifi\r\n\r\n SSID = My Net \r\nPassword=a=b=c\r\n");
        assert_eq!(parse(&f), Some(("My Net", "a=b=c")));
    }

    #[test]
    fn open_network_and_no_trailing_newline() {
        assert_eq!(parse(&flash(b"ssid=Cafe")), Some(("Cafe", "")));
    }

    #[test]
    fn rejects_junk() {
        assert_eq!(parse(&[0xFF; 512]), None); // erased
        assert_eq!(parse(&[0x00; 512]), None); // forgotten
        assert_eq!(parse(&flash(b"pass=nossid\n")), None);
        assert_eq!(parse(&flash(b"ssid=\n")), None);
        assert_eq!(parse(&flash(b"ssid=123456789012345678901234567890123\n")), None); // 33 chars
        assert_eq!(parse(&flash(b"PWF1\x04\x00garbage")), None); // binary record, not text
    }

    #[test]
    fn stops_at_zeroed_bytes() {
        let mut f = flash(b"ssid=Home\npass=secret\n");
        f[..106].fill(0); // what /forget does
        assert_eq!(parse(&f), None);
    }
}
