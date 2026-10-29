//! Pendulum timing maths. Pure logic, no hardware: fed (timestamp, beam level)
//! edges, it works out beats, period, rate, clock-vs-real-time error, beat error
//! and swing amplitude. Host-testable (see `host-tests/`).
//!
//! Geometry: one IR beam at roughly the bottom of the swing, interrupted by a
//! paper flag. Every pass through the beam is one beat; two beats = one second.
//!
//! Timestamps are the *middle* of each blocked pulse, so the timing doesn't move
//! when swing amplitude (and therefore pulse width) changes.

use alloc::string::String;
use core::fmt::Write;

/// One beat (one pass through the beam). Two beats per second.
pub const BEAT_S: f64 = 0.5;
pub const PERIOD_S: f64 = 2.0 * BEAT_S;
/// Paper flag width, ~1/4 inch. Only used for the amplitude estimate.
pub const FLAG_WIDTH_MM: f64 = 6.35;
/// Pivot to flag distance (measured on the built pendulum). Only used to turn
/// amplitude in mm into degrees.
pub const FLAG_RADIUS_MM: f64 = 293.0;
/// Edges closer together than this are treated as bounce and ignored (done in the ISR).
pub const GLITCH_US: u64 = 2_000;
/// Periods in the short-term rolling average.
pub const AVG_N: usize = 30;
/// Points kept per chart.
pub const HIST_N: usize = 300;
/// No pass for this long = pendulum stopped.
pub const STOP_US: u64 = 3_000_000;

/// Works out which sensor level means "blocked" by watching the pendulum swing,
/// so it doesn't matter whether the flag is in the beam or clear at power-up.
/// The flag is narrow compared with the swing, so the blocked level is the one
/// that's held for the *shorter* time. Needs 3 pulses of each level.
#[derive(Default)]
pub struct Polarity {
    last: Option<(u64, bool)>,
    sum: [u64; 2],
    n: [u32; 2],
    /// `Some(true)` = blocked reads HIGH, `Some(false)` = blocked reads LOW.
    pub blocked_high: Option<bool>,
}

impl Polarity {
    pub const fn new() -> Self {
        Self { last: None, sum: [0; 2], n: [0; 2], blocked_high: None }
    }

    /// Feed a raw edge (`high` = level after the edge). Returns the decision once made.
    pub fn edge(&mut self, t_us: u64, high: bool) -> Option<bool> {
        if self.blocked_high.is_some() {
            return self.blocked_high;
        }
        if let Some((lt, lh)) = self.last {
            let d = t_us.saturating_sub(lt);
            let i = lh as usize;
            if d < 5_000_000 {
                // ignore long holds (pendulum at rest / being started by hand)
                self.sum[i] += d;
                self.n[i] += 1;
            }
        }
        self.last = Some((t_us, high));
        if self.n[0] >= 3 && self.n[1] >= 3 {
            let avg_low = self.sum[0] / self.n[0] as u64;
            let avg_high = self.sum[1] / self.n[1] as u64;
            self.blocked_high = Some(avg_high < avg_low);
        }
        self.blocked_high
    }
}

/// What happened on one pass, for logging.
#[derive(Clone, Copy, Debug, Default)]
pub struct Pass {
    pub beats: u32,
    pub half_ms: f64,
    pub period_ms: f64,
    pub blocked_ms: f64,
    pub rate_s_per_day: f64,
    pub err_s: f64,
    pub amp_deg: f64,
    /// Middle of the blocked pulse, µs since boot.
    pub mid_us: u64,
    /// How long the beam was blocked, µs.
    pub width_us: u64,
}

pub struct Timing {
    block_t: Option<u64>,
    mid1: Option<u64>,
    mid2: Option<u64>,
    t0: Option<u64>,
    /// Mid-pass time of the last even-numbered beat (same side of the swing as t0).
    t_even: u64,
    last_pass_us: u64,
    beats: u32,
    gaps: u32,
    last_half_ms: f64,
    prev_half_ms: f64,
    last_period_ms: f64,
    last_blocked_ms: f64,
    avg_period_ms: f64,
    rate30: f64,
    beat_err_ms: f64,
    amp_mm: f64,
    amp_deg: f64,
    err_s: f64,
    ring: [f64; AVG_N],
    ring_n: usize,
    ring_i: usize,
    // last HIST_N periods
    hp: [f32; HIST_N],
    ha: [f32; HIST_N],
    h_n: usize,
    h_head: usize,
    // whole-run error, decimated (halves its sample rate each time it fills)
    he: [f64; HIST_N], // f64: f32 would lose µs once the error passes ~8 s
    ht: [f32; HIST_N],
    e_n: usize,
    e_step: u32,
    e_count: u32,
}

impl Default for Timing {
    fn default() -> Self {
        Self::new()
    }
}

impl Timing {
    pub const fn new() -> Self {
        Self {
            block_t: None,
            mid1: None,
            mid2: None,
            t0: None,
            t_even: 0,
            last_pass_us: 0,
            beats: 0,
            gaps: 0,
            last_half_ms: 0.0,
            prev_half_ms: 0.0,
            last_period_ms: 0.0,
            last_blocked_ms: 0.0,
            avg_period_ms: 0.0,
            rate30: 0.0,
            beat_err_ms: 0.0,
            amp_mm: 0.0,
            amp_deg: 0.0,
            err_s: 0.0,
            ring: [0.0; AVG_N],
            ring_n: 0,
            ring_i: 0,
            hp: [0.0; HIST_N],
            ha: [0.0; HIST_N],
            h_n: 0,
            h_head: 0,
            he: [0.0; HIST_N],
            ht: [0.0; HIST_N],
            e_n: 0,
            e_step: 1,
            e_count: 0,
        }
    }

    /// Re-zero: the next pass becomes the reference where clock == real time.
    pub fn reset(&mut self) {
        let block_t = self.block_t;
        *self = Self::new();
        self.block_t = block_t; // don't lose a pulse that's in progress
    }

    /// Feed one debounced edge. Returns `Some` when a pass completes.
    pub fn edge(&mut self, t_us: u64, blocked: bool) -> Option<Pass> {
        if blocked {
            self.block_t = Some(t_us);
            None
        } else {
            let b = self.block_t.take()?;
            let width = t_us.saturating_sub(b);
            let mid = b + width / 2;
            let mut p = self.pass(mid, width as f64 / 1000.0);
            p.mid_us = mid;
            p.width_us = width;
            Some(p)
        }
    }

    fn push_err(&mut self, minutes: f64, err: f64) {
        self.e_count += 1;
        if self.e_count < self.e_step {
            return;
        }
        self.e_count = 0;
        if self.e_n == HIST_N {
            for i in 0..HIST_N / 2 {
                self.he[i] = self.he[2 * i + 1];
                self.ht[i] = self.ht[2 * i + 1];
            }
            self.e_n = HIST_N / 2;
            self.e_step *= 2;
        }
        self.he[self.e_n] = err;
        self.ht[self.e_n] = minutes as f32;
        self.e_n += 1;
    }

    fn pass(&mut self, mid: u64, blocked_ms: f64) -> Pass {
        self.last_blocked_ms = blocked_ms;
        self.last_pass_us = mid;

        let Some(t0) = self.t0 else {
            // first pass after boot / re-zero is the reference: clock == real here
            self.t0 = Some(mid);
            self.t_even = mid;
            self.mid1 = Some(mid);
            self.push_err(0.0, 0.0);
            return Pass { blocked_ms, ..Default::default() };
        };

        if let Some(m1) = self.mid1 {
            let half = (mid - m1) as f64 / 1000.0;
            if half > BEAT_S * 1500.0 {
                self.gaps += 1;
            }
            self.prev_half_ms = self.last_half_ms;
            self.last_half_ms = half;
        }
        self.beats += 1;
        // Clock vs real only on even beats: whole periods, so both ends of the
        // comparison are on the same side of the swing. On odd beats the uneven
        // tick/tock split (beat error or an off-centre beam) would add a sawtooth.
        if self.beats % 2 == 0 {
            self.t_even = mid;
            let real_s = (mid - t0) as f64 / 1e6;
            self.err_s = (self.beats / 2) as f64 * PERIOD_S - real_s;
            self.push_err(real_s / 60.0, self.err_s);
        }

        if let Some(m2) = self.mid2 {
            // Full period = tick + tock. Immune to the beam not being at dead centre.
            self.last_period_ms = (mid - m2) as f64 / 1000.0;
            self.ring[self.ring_i] = self.last_period_ms;
            self.ring_i = (self.ring_i + 1) % AVG_N;
            if self.ring_n < AVG_N {
                self.ring_n += 1;
            }
            self.avg_period_ms = self.ring[..self.ring_n].iter().sum::<f64>() / self.ring_n as f64;
            // seconds gained per day: the clock shows PERIOD_S for every real avg period
            self.rate30 = (PERIOD_S * 1000.0 / self.avg_period_ms - 1.0) * 86400.0;
            self.beat_err_ms = if self.prev_half_ms > 0.0 {
                libm::fabs(self.last_half_ms - self.prev_half_ms) / 2.0
            } else {
                0.0
            };

            // x(t) = A sin(wt) takes t_b = (2/w) asin(w_flag / 2A) to cross the flag,
            // so A = w_flag / (2 sin(w t_b / 2)) = w_flag / (2 sin(pi t_b / T)).
            let s = libm::sin(core::f64::consts::PI * blocked_ms / self.last_period_ms);
            self.amp_mm = if s > 0.0 { FLAG_WIDTH_MM / (2.0 * s) } else { 0.0 };
            self.amp_deg = if self.amp_mm < FLAG_RADIUS_MM {
                libm::asin(self.amp_mm / FLAG_RADIUS_MM).to_degrees()
            } else {
                0.0
            };

            self.hp[self.h_head] = self.last_period_ms as f32;
            self.ha[self.h_head] = self.amp_deg as f32;
            self.h_head = (self.h_head + 1) % HIST_N;
            if self.h_n < HIST_N {
                self.h_n += 1;
            }
        }
        self.mid2 = self.mid1;
        self.mid1 = Some(mid);

        Pass {
            beats: self.beats,
            half_ms: self.last_half_ms,
            mid_us: 0,
            width_us: 0,
            period_ms: self.last_period_ms,
            blocked_ms,
            rate_s_per_day: self.rate30,
            err_s: self.err_s,
            amp_deg: self.amp_deg,
        }
    }

    /// Average of the last few periods in ms (0 until the first period).
    pub fn avg_period_ms(&self) -> f64 {
        self.avg_period_ms
    }

    /// Passes seen since boot / re-zero.
    pub fn beats(&self) -> u32 {
        self.beats
    }

    pub fn stopped(&self, now_us: u64) -> bool {
        self.t0.is_some() && now_us.saturating_sub(self.last_pass_us) > STOP_US
    }

    /// Clock minus real time, in seconds. Positive = clock ahead (fast).
    /// While running it's the value at the last beat (no 0.5 s sawtooth); once the
    /// pendulum stops the clock isn't advancing, so it keeps falling behind.
    pub fn live_err(&self, now_us: u64) -> f64 {
        match self.t0 {
            Some(t0) if self.stopped(now_us) => {
                (self.beats / 2) as f64 * PERIOD_S - (now_us - t0) as f64 / 1e6
            }
            _ => self.err_s,
        }
    }

    pub fn json(&self, now_us: u64, beam_blocked: bool, polarity_known: bool, out: &mut String) {
        let stopped = self.stopped(now_us);
        let real_s = self.t0.map(|t0| now_us.saturating_sub(t0) as f64 / 1e6).unwrap_or(0.0);
        let err = self.live_err(now_us);
        let run_s = match self.t0 {
            _ if stopped => real_s,
            Some(t0) if self.t_even > t0 => (self.t_even - t0) as f64 / 1e6,
            _ => 0.0,
        };
        let lrate = if run_s > 0.0 { err / run_s * 86400.0 } else { 0.0 };
        let ago: i64 = if self.t0.is_some() {
            (now_us.saturating_sub(self.last_pass_us) / 1000) as i64
        } else {
            -1
        };

        out.clear();
        let _ = write!(
            out,
            "{{\"n\":{},\"np\":{},\"err\":{:.6},\"real\":{:.6},\"rate\":{:.4},\"lrate\":{:.4},\
             \"per\":{:.4},\"be\":{:.3},\"amp\":{:.3},\"ampmm\":{:.2},\"blk\":{:.2},\"gaps\":{},\
             \"stopped\":{},\"pol\":{},\"beam\":{},\"ago\":{},\"up\":{}",
            self.beats,
            self.h_n,
            err,
            real_s,
            self.rate30,
            lrate,
            self.avg_period_ms,
            self.beat_err_ms,
            self.amp_deg,
            self.amp_mm,
            self.last_blocked_ms,
            self.gaps,
            stopped,
            polarity_known,
            beam_blocked,
            ago,
            now_us / 1000
        );
        let start = (self.h_head + HIST_N - self.h_n) % HIST_N;
        let list = |out: &mut String, name: &str, n: usize, f: &dyn Fn(usize) -> f64, prec: usize, extra: Option<f64>| {
            let _ = write!(out, ",\"{name}\":[");
            for k in 0..n {
                if k > 0 {
                    out.push(',');
                }
                let _ = write!(out, "{:.*}", prec, f(k));
            }
            if let Some(x) = extra {
                if n > 0 {
                    out.push(',');
                }
                let _ = write!(out, "{:.*}", prec, x);
            }
            out.push(']');
        };
        list(out, "hp", self.h_n, &|k| self.hp[(start + k) % HIST_N] as f64, 3, None);
        list(out, "ha", self.h_n, &|k| self.ha[(start + k) % HIST_N] as f64, 2, None);
        // when stopped, append a live point so the chart shows the clock falling behind
        list(out, "ht", self.e_n, &|k| self.ht[k] as f64, 2, stopped.then_some(real_s / 60.0));
        list(out, "he", self.e_n, &|k| self.he[k], 6, stopped.then_some(err));
        out.push('}');
    }
}
