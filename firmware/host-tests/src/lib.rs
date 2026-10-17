//! Host-side tests for the firmware's hardware-free modules.
extern crate alloc;

#[path = "../../pendulum-timer/src/timing.rs"]
pub mod timing;
#[path = "../../pendulum-timer/src/mdns.rs"]
pub mod mdns;

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
        tm.json(now, false, &mut s);
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
    fn perfect_clock_is_on_time() {
        let tm = run(1.000, 15.0, 0.0, 300.0);
        let mut s = String::new();
        tm.json(1_000_000 + 300_000_000, false, &mut s);
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
        tm.json(later, false, &mut s);
        assert_eq!(field(&s, "n"), 0.0);
        assert_eq!(field(&s, "ago"), -1.0);
    }

    #[test]
    fn long_run_history_decimates() {
        let tm = run(1.000, 15.0, 0.0, 1200.0); // 2400 beats -> decimated to <=300 points
        let mut s = String::new();
        tm.json(1_000_000 + 1_200_000_000, false, &mut s);
        let ht = &s[s.find("\"ht\":[").unwrap() + 6..];
        let n = ht[..ht.find(']').unwrap()].split(',').count();
        assert!(n <= HIST_N && n >= HIST_N / 2, "{n} points");
        assert!(s.len() < 12_000, "json {} bytes", s.len());
    }
}
