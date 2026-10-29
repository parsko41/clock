//! Combined two-sensor timing. Pure logic, host-testable (see `host-tests/`).
//!
//! Layout: one IR beam near each end of the stroke (sensor 1 on one side,
//! sensor 2 on the other). Each time the pendulum swings out past a beam and
//! comes back, the flag passes through it twice. The midpoint of that pair is
//! the moment the pendulum turned round - a clean timestamp that doesn't
//! depend on where the beam is.
//!
//! From the turns:
//! - period    = turn to turn on the same side (sensor 1's side is the reference)
//! - half swings = turn on one side to turn on the other. Their difference is
//!   real escapement asymmetry (beat error), not sensor placement.
//! - amplitude, each side, and how far each beam is from the centre of the
//!   swing, from the out-and-back time and how long the flag blocks the beam.
//! - per-tooth timing: periods folded by escape-wheel tooth (30 teeth, one
//!   tooth per period), so a bad tooth shows up as one odd bar.
//!
//! If the swing gets too small for the flag to clear a beam, that sensor sees
//! one long pulse per swing instead of two; its middle is still the turn, so
//! timing carries on (amplitude for that side is unknown until it clears again).

use alloc::string::String;
use core::f64::consts::PI;
use core::fmt::Write;

use crate::timing::{AVG_N, FLAG_RADIUS_MM, FLAG_WIDTH_MM, HIST_N, PERIOD_S, STOP_US};

/// Escape wheel teeth = periods per escape-wheel revolution.
pub const TEETH: usize = 30;

/// One turn of the pendulum at one end of its swing.
#[derive(Clone, Copy, Debug, Default)]
pub struct Turn {
    /// 0 = sensor 1's side, 1 = sensor 2's side.
    pub side: usize,
    pub t_us: u64,
    /// Amplitude at the flag (mm) and the beam's distance from centre (mm), when
    /// the flag cleared the beam.
    pub amp_off: Option<(f64, f64)>,
}

/// Turn the out-and-back pair at one beam into amplitude and beam position.
///
/// The flag (width f) is in the beam while its centre is between d - f/2 and
/// d + f/2. With x = A cos(wt) measured from the turn, the flag enters (going
/// back) at tau2 = acos((d + f/2)/A)/w and leaves at tau1 = acos((d - f/2)/A)/w.
/// The out pass is the mirror image, so mid-to-mid = tau1 + tau2 and the
/// blocked time = tau1 - tau2. Solving: A = f / (cos w.tau2 - cos w.tau1),
/// d = A (cos w.tau1 + cos w.tau2) / 2. Exact, no small-flag approximation.
pub fn amp_and_offset(t_out_s: f64, blocked_s: f64, period_s: f64) -> Option<(f64, f64)> {
    let w = 2.0 * PI / period_s;
    let tau1 = (t_out_s + blocked_s) / 2.0;
    let tau2 = (t_out_s - blocked_s) / 2.0;
    if tau2 <= 0.0 || w * tau1 >= PI / 2.0 {
        return None;
    }
    let (c1, c2) = (libm::cos(w * tau1), libm::cos(w * tau2));
    if c2 - c1 <= 1e-9 {
        return None;
    }
    let a = FLAG_WIDTH_MM / (c2 - c1);
    Some((a, a * (c1 + c2) / 2.0))
}

/// Ring of the last `N` values with a running mean.
#[derive(Clone, Copy)]
struct Avg<const N: usize> {
    v: [f64; N],
    n: usize,
    i: usize,
}

impl<const N: usize> Avg<N> {
    const fn new() -> Self {
        Self { v: [0.0; N], n: 0, i: 0 }
    }
    fn push(&mut self, x: f64) {
        self.v[self.i] = x;
        self.i = (self.i + 1) % N;
        if self.n < N {
            self.n += 1;
        }
    }
    fn mean(&self) -> f64 {
        if self.n == 0 { 0.0 } else { self.v[..self.n].iter().sum::<f64>() / self.n as f64 }
    }
}

/// Last `HIST_N` values of a series, oldest first when read.
#[derive(Clone, Copy)]
struct Hist {
    v: [f32; HIST_N],
    n: usize,
    head: usize,
}

impl Hist {
    const fn new() -> Self {
        Self { v: [0.0; HIST_N], n: 0, head: 0 }
    }
    fn push(&mut self, x: f64) {
        self.v[self.head] = x as f32;
        self.head = (self.head + 1) % HIST_N;
        if self.n < HIST_N {
            self.n += 1;
        }
    }
    fn get(&self, k: usize) -> f64 {
        self.v[(self.head + HIST_N - self.n + k) % HIST_N] as f64
    }
}

pub struct Swing {
    /// Per sensor: the first pass of a possible out-and-back pair (mid, width µs).
    pending: [Option<(u64, u64)>; 2],
    /// Per sensor: the first turn after boot is thrown away, in case it came
    /// from the back half of a pair whose out half came before we were watching.
    primed: [bool; 2],
    last_turn: [Option<u64>; 2],
    t0: Option<u64>,
    /// Side whose turns count periods (the first side seen, normally sensor 1).
    ref_side: usize,
    last_ref: u64,
    last_any: u64,
    periods: u32,
    gaps: u32,
    err_s: f64,
    last_per_ms: f64,
    per: Avg<AVG_N>,
    /// Half swing ending on side 0 / side 1.
    half: [Avg<AVG_N>; 2],
    last_half: [f64; 2],
    amp: [f64; 2],
    off: [f64; 2],
    hp: Hist,
    ha: [Hist; 2],
    hh: [Hist; 2],
    // whole-run error, decimated (halves its sample rate each time it fills)
    he: [f64; HIST_N],
    ht: [f32; HIST_N],
    e_n: usize,
    e_step: u32,
    e_count: u32,
    // per escape-wheel tooth
    t_sum: [f64; TEETH],
    t_n: [u32; TEETH],
    t_last: [f32; TEETH],
    t_amp: [[f64; TEETH]; 2],
    t_amp_n: [[u32; TEETH]; 2],
    /// half swing ending on side 0 / 1, per tooth
    t_half: [[f64; TEETH]; 2],
    t_half_n: [[u32; TEETH]; 2],
}

impl Default for Swing {
    fn default() -> Self {
        Self::new()
    }
}

impl Swing {
    pub const fn new() -> Self {
        Self {
            pending: [None; 2],
            primed: [false; 2],
            last_turn: [None; 2],
            t0: None,
            ref_side: 0,
            last_ref: 0,
            last_any: 0,
            periods: 0,
            gaps: 0,
            err_s: 0.0,
            last_per_ms: 0.0,
            per: Avg::new(),
            half: [Avg::new(), Avg::new()],
            last_half: [0.0; 2],
            amp: [0.0; 2],
            off: [0.0; 2],
            hp: Hist::new(),
            ha: [Hist::new(), Hist::new()],
            hh: [Hist::new(), Hist::new()],
            he: [0.0; HIST_N],
            ht: [0.0; HIST_N],
            e_n: 0,
            e_step: 1,
            e_count: 0,
            t_sum: [0.0; TEETH],
            t_n: [0; TEETH],
            t_last: [f32::NAN; TEETH],
            t_amp: [[0.0; TEETH]; 2],
            t_amp_n: [[0; TEETH]; 2],
            t_half: [[0.0; TEETH]; 2],
            t_half_n: [[0; TEETH]; 2],
        }
    }

    /// Re-zero: the next turn becomes the reference where clock == real time.
    /// Per-tooth stats restart too (tooth numbers count from the zero).
    pub fn reset(&mut self) {
        let (pending, primed) = (self.pending, self.primed);
        *self = Self::new();
        self.pending = pending;
        self.primed = primed;
    }

    /// Average period (ms), or the nominal 1000 ms before there is one.
    pub fn period_est_ms(&self) -> f64 {
        let p = self.per.mean();
        if p > 0.0 { p } else { PERIOD_S * 1000.0 }
    }

    /// Average period in ms (0 until measured).
    pub fn avg_period_ms(&self) -> f64 {
        self.per.mean()
    }

    pub fn periods(&self) -> u32 {
        self.periods
    }

    /// Feed one pass through a beam (`s` = 0 for sensor 1, 1 for sensor 2).
    pub fn pass(&mut self, s: usize, mid_us: u64, width_us: u64) -> Option<Turn> {
        let half_t = (self.period_est_ms() * 500.0) as u64; // half a period, µs
        if let Some((pm, pw)) = self.pending[s] {
            let gap = mid_us.saturating_sub(pm);
            if gap < half_t {
                self.pending[s] = None;
                let t = pm + gap / 2;
                let w = (pw + width_us) as f64 / 2.0 / 1e6;
                let ao = amp_and_offset(gap as f64 / 1e6, w, self.period_est_ms() / 1000.0);
                return self.emit(Turn { side: s, t_us: t, amp_off: ao });
            }
        }
        // no partner within half a period: the old one was a single pulse
        let out = match self.pending[s].take() {
            Some((pm, _)) => self.emit(Turn { side: s, t_us: pm, amp_off: None }),
            None => None,
        };
        self.pending[s] = Some((mid_us, width_us));
        out
    }

    /// Call often. Emits a turn for a single pulse whose partner never came
    /// (the flag didn't clear the beam: the pulse middle is the turn).
    pub fn poll(&mut self, now_us: u64) -> Option<Turn> {
        let mut out = None;
        for s in 0..2 {
            if let Some(t) = self.flush(s, now_us) {
                out = Some(t);
            }
        }
        out
    }

    fn flush(&mut self, s: usize, now_us: u64) -> Option<Turn> {
        let (pm, _) = self.pending[s]?;
        let half_t = (self.period_est_ms() * 500.0) as u64;
        if now_us.saturating_sub(pm) < half_t + 20_000 {
            return None;
        }
        self.pending[s] = None;
        self.emit(Turn { side: s, t_us: pm, amp_off: None })
    }

    fn emit(&mut self, tn: Turn) -> Option<Turn> {
        if !self.primed[tn.side] {
            self.primed[tn.side] = true;
            return None;
        }
        Some(self.turn(tn))
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

    fn turn(&mut self, tn: Turn) -> Turn {
        let s = tn.side;
        let t = tn.t_us;
        self.last_any = t;
        if let Some((a, d)) = tn.amp_off {
            self.amp[s] = a;
            self.off[s] = d;
        }
        let per_est = self.period_est_ms();
        if let Some(lo) = self.last_turn[1 - s] {
            let h = (t.saturating_sub(lo)) as f64 / 1000.0;
            if h > per_est * 0.25 && h < per_est * 0.75 {
                self.last_half[s] = h;
                self.half[s].push(h);
                if self.t0.is_some() {
                    let tooth = self.periods as usize % TEETH;
                    self.t_half[s][tooth] += h;
                    self.t_half_n[s][tooth] += 1;
                }
            }
        }
        self.last_turn[s] = Some(t);

        let Some(t0) = self.t0 else {
            self.t0 = Some(t);
            self.ref_side = s;
            self.last_ref = t;
            self.push_err(0.0, 0.0);
            return tn;
        };
        if s != self.ref_side {
            return tn;
        }

        if t <= self.last_ref {
            return tn; // out of order (shouldn't happen)
        }
        let dt_ms = (t - self.last_ref) as f64 / 1000.0;
        let n = libm::round(dt_ms / per_est).max(1.0) as u32;
        self.last_ref = t;
        self.periods += n;
        if n > 1 {
            self.gaps += n - 1;
        } else {
            let p = dt_ms;
            self.last_per_ms = p;
            self.per.push(p);
            self.hp.push(p);
            for k in 0..2 {
                self.ha[k].push(self.amp[k]);
                self.hh[k].push(self.last_half[k]);
            }
            // the period that just ended belongs to tooth (periods - 1)
            let tooth = (self.periods as usize + TEETH - 1) % TEETH;
            self.t_sum[tooth] += p;
            self.t_n[tooth] += 1;
            self.t_last[tooth] = p as f32;
            for k in 0..2 {
                if self.amp[k] > 0.0 {
                    self.t_amp[k][tooth] += self.amp[k];
                    self.t_amp_n[k][tooth] += 1;
                }
            }
        }
        let real_s = (t - t0) as f64 / 1e6;
        self.err_s = self.periods as f64 * PERIOD_S - real_s;
        self.push_err(real_s / 60.0, self.err_s);
        tn
    }

    pub fn stopped(&self, now_us: u64) -> bool {
        self.t0.is_some() && now_us.saturating_sub(self.last_any) > STOP_US
    }

    /// Clock minus real time (s). Once stopped it keeps falling behind.
    pub fn live_err(&self, now_us: u64) -> f64 {
        match self.t0 {
            Some(t0) if self.stopped(now_us) => self.periods as f64 * PERIOD_S - (now_us - t0) as f64 / 1e6,
            _ => self.err_s,
        }
    }

    /// JSON for the dashboard, written in parts so the whole thing never has to
    /// sit in RAM at once. Call with part = 0, 1, 2... until it returns false.
    /// The object is left open: the caller adds its own fields and the `}`.
    pub fn json_part(&self, part: usize, now_us: u64, out: &mut String) -> bool {
        out.clear();
        let stopped = self.stopped(now_us);
        let real_s = self.t0.map(|t0| now_us.saturating_sub(t0) as f64 / 1e6).unwrap_or(0.0);
        let list = |out: &mut String, name: &str, h: &Hist, prec: usize| {
            let _ = write!(out, ",\"{name}\":[");
            for k in 0..h.n {
                if k > 0 {
                    out.push(',');
                }
                let _ = write!(out, "{:.*}", prec, h.get(k));
            }
            out.push(']');
        };
        match part {
            0 => {
                let err = self.live_err(now_us);
                let run_s = match self.t0 {
                    _ if stopped => real_s,
                    Some(t0) if self.last_ref > t0 => (self.last_ref - t0) as f64 / 1e6,
                    _ => 0.0,
                };
                let lrate = if run_s > 0.0 { err / run_s * 86400.0 } else { 0.0 };
                let avg = self.per.mean();
                let rate = if avg > 0.0 { (PERIOD_S * 1000.0 / avg - 1.0) * 86400.0 } else { 0.0 };
                let ago: i64 = if self.t0.is_some() { (now_us.saturating_sub(self.last_any) / 1000) as i64 } else { -1 };
                let deg = |a: f64| if a > 0.0 && a < FLAG_RADIUS_MM { libm::asin(a / FLAG_RADIUS_MM).to_degrees() } else { 0.0 };
                let _ = write!(
                    out,
                    "{{\"n\":{},\"np\":{},\"err\":{:.6},\"real\":{:.6},\"rate\":{:.4},\"lrate\":{:.4},\
                     \"per\":{:.4},\"lper\":{:.3},\"gaps\":{},\"stopped\":{},\"ago\":{},\"up\":{},\"ref\":{},\
                     \"h\":[{:.3},{:.3}],\"lh\":[{:.3},{:.3}],\"amp\":[{:.2},{:.2}],\"deg\":[{:.3},{:.3}],\
                     \"off\":[{:.2},{:.2}]",
                    self.periods,
                    self.per.n,
                    err,
                    real_s,
                    rate,
                    lrate,
                    avg,
                    self.last_per_ms,
                    self.gaps,
                    stopped,
                    ago,
                    now_us / 1000,
                    self.ref_side + 1,
                    self.half[0].mean(),
                    self.half[1].mean(),
                    self.last_half[0],
                    self.last_half[1],
                    self.amp[0],
                    self.amp[1],
                    deg(self.amp[0]),
                    deg(self.amp[1]),
                    self.off[0],
                    self.off[1],
                );
            }
            1 => list(out, "hp", &self.hp, 3),
            2 => list(out, "ha1", &self.ha[0], 2),
            3 => list(out, "ha2", &self.ha[1], 2),
            4 => list(out, "hh1", &self.hh[0], 2),
            5 => list(out, "hh2", &self.hh[1], 2),
            6 => {
                // when stopped, append a live point so the chart shows the clock falling behind
                out.push_str(",\"ht\":[");
                for k in 0..self.e_n {
                    let _ = write!(out, "{}{:.2}", if k > 0 { "," } else { "" }, self.ht[k]);
                }
                if stopped {
                    let _ = write!(out, "{}{:.2}", if self.e_n > 0 { "," } else { "" }, real_s / 60.0);
                }
                out.push_str("],\"he\":[");
                for k in 0..self.e_n {
                    let _ = write!(out, "{}{:.6}", if k > 0 { "," } else { "" }, self.he[k]);
                }
                if stopped {
                    let _ = write!(out, "{}{:.6}", if self.e_n > 0 { "," } else { "" }, self.live_err(now_us));
                }
                out.push(']');
            }
            7 => {
                // per tooth: mean period, last-rev period, mean amplitude each side
                let _ = write!(out, ",\"tooth\":{{\"cur\":{},\"n\":[", self.periods as usize % TEETH);
                for k in 0..TEETH {
                    let _ = write!(out, "{}{}", if k > 0 { "," } else { "" }, self.t_n[k]);
                }
                out.push_str("],\"avg\":[");
                for k in 0..TEETH {
                    let sep = if k > 0 { "," } else { "" };
                    if self.t_n[k] > 0 {
                        let _ = write!(out, "{sep}{:.3}", self.t_sum[k] / self.t_n[k] as f64);
                    } else {
                        let _ = write!(out, "{sep}null");
                    }
                }
                out.push_str("],\"last\":[");
                for k in 0..TEETH {
                    let sep = if k > 0 { "," } else { "" };
                    if self.t_last[k].is_finite() {
                        let _ = write!(out, "{sep}{:.3}", self.t_last[k]);
                    } else {
                        let _ = write!(out, "{sep}null");
                    }
                }
                for (name, sums, ns) in [("a", &self.t_amp, &self.t_amp_n), ("h", &self.t_half, &self.t_half_n)] {
                    for side in 0..2 {
                        let _ = write!(out, "],\"{name}{}\":[", side + 1);
                        for k in 0..TEETH {
                            let sep = if k > 0 { "," } else { "" };
                            let n = ns[side][k];
                            if n > 0 {
                                let _ = write!(out, "{sep}{:.3}", sums[side][k] / n as f64);
                            } else {
                                let _ = write!(out, "{sep}null");
                            }
                        }
                    }
                }
                out.push_str("]}");
            }
            _ => return false,
        }
        true
    }
}
