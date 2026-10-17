//! Prints /data JSON for a simulated pendulum: cargo run --example dump -- <period_s> <amp_mm> <offset_mm> <secs>
use host_tests::timing::Timing;
fn main() {
    let a: Vec<f64> = std::env::args().skip(1).map(|x| x.parse().unwrap()).collect();
    let (p, amp, off, secs) = (a[0], a[1], a[2], a[3]);
    let mut tm = Timing::new();
    let mut blocked = false;
    let mut t = 1_000_000u64;
    let end = t + (secs * 1e6) as u64;
    while t < end {
        let ts = (t - 1_000_000) as f64 / 1e6;
        let am = amp * (1.0 - 0.1 * (ts / secs)); // slowly decaying swing
        let x = am * (2.0 * std::f64::consts::PI * ts / p + 0.3).sin();
        let b = (x - off).abs() < 6.35 / 2.0;
        if b != blocked { tm.edge(t, b); blocked = b; }
        t += 10;
    }
    let mut s = String::new();
    tm.json(end + 200_000, false, true, &mut s);
    println!("{s}");
}
