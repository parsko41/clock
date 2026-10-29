//! Run log: every beam pass from both sensors, kept in RAM so a whole run
//! (wind it, let it run down) can be downloaded as a CSV afterwards.
//!
//! One record per pass: the middle of the blocked pulse and how long the beam
//! was blocked. That's lossless - the beam was blocked from mid - w/2 to
//! mid + w/2 - so anything can be worked out later from the CSV.
//!
//! 8 bytes a record. Times are µs from the first logged pass, as u32, so a log
//! covers at most ~71 minutes; it also stops when the buffer fills. Either way
//! the *start* of the run is kept, which is what a run-down test needs.

use alloc::string::String;
use core::fmt::Write;

/// Records kept (48 KB of RAM - about what's free). 2 passes per second per
/// sensor: 6144 = ~25 min with both sensors running, ~51 min with one.
pub const LOG_N: usize = 6144;
/// Sensors (1 and 2).
pub const SENSORS: usize = 2;

#[derive(Clone, Copy, Default)]
struct Rec {
    /// Pulse middle, µs since the first logged pass.
    t: u32,
    /// Bit 31 = sensor 2, bits 0..30 = blocked time in µs.
    w: u32,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Full {
    No,
    /// Buffer filled up.
    Buffer,
    /// Run longer than the u32 µs clock (~71 min).
    Time,
}

pub struct RunLog {
    start_us: Option<u64>,
    recs: [Rec; LOG_N],
    n: usize,
    full: Full,
    /// Passes per sensor in the log.
    per: [u32; SENSORS],
}

impl Default for RunLog {
    fn default() -> Self {
        Self::new()
    }
}

/// Per-sensor state carried between CSV chunks.
#[derive(Default, Clone, Copy)]
pub struct CsvState {
    count: [u32; SENSORS],
    prev: [Option<u32>; SENSORS],
    prev2: [Option<u32>; SENSORS],
}

pub const CSV_HEADER: &str = "sensor,pass,t_s,blocked_ms,since_prev_ms,period_ms\r\n";

impl RunLog {
    pub const fn new() -> Self {
        Self { start_us: None, recs: [Rec { t: 0, w: 0 }; LOG_N], n: 0, full: Full::No, per: [0; SENSORS] }
    }

    pub fn clear(&mut self) {
        self.start_us = None;
        self.n = 0;
        self.full = Full::No;
        self.per = [0; SENSORS];
    }

    /// Log one pass. `sensor` is 1 or 2. Returns false once the log is full.
    pub fn push(&mut self, sensor: u8, mid_us: u64, width_us: u64) -> bool {
        if self.full != Full::No {
            return false;
        }
        if self.n == LOG_N {
            self.full = Full::Buffer;
            return false;
        }
        let start = *self.start_us.get_or_insert(mid_us);
        let dt = mid_us.saturating_sub(start);
        if dt > u32::MAX as u64 {
            self.full = Full::Time;
            return false;
        }
        let w = (width_us.min(0x7fff_ffff) as u32) | if sensor == 2 { 1 << 31 } else { 0 };
        self.recs[self.n] = Rec { t: dt as u32, w };
        self.n += 1;
        self.per[(sensor == 2) as usize] += 1;
        true
    }

    pub fn len(&self) -> usize {
        self.n
    }

    pub fn is_empty(&self) -> bool {
        self.n == 0
    }

    pub fn full(&self) -> Full {
        self.full
    }

    /// Passes logged for sensor 1 / 2.
    pub fn per_sensor(&self) -> [u32; SENSORS] {
        self.per
    }

    /// Seconds from the first to the last logged pass.
    pub fn span_s(&self) -> f64 {
        if self.n == 0 { 0.0 } else { self.recs[self.n - 1].t as f64 / 1e6 }
    }

    /// Append CSV rows for records `from..to` (clamped). Returns the index reached.
    /// Columns: sensor, pass number for that sensor, mid-pulse time (s, µs
    /// precision), blocked time (ms), time since that sensor's previous pass
    /// (ms - tick or tock), and since the one before that (ms - a full period).
    pub fn csv_rows(&self, from: usize, to: usize, st: &mut CsvState, out: &mut String) -> usize {
        let to = to.min(self.n);
        for r in &self.recs[from.min(to)..to] {
            let s = (r.w >> 31) as usize;
            let w = r.w & 0x7fff_ffff;
            st.count[s] += 1;
            let _ = write!(out, "{},{},{}.{:06},{}.{:03},", s + 1, st.count[s], r.t / 1_000_000, r.t % 1_000_000, w / 1000, w % 1000);
            let ms = |a: u32, b: u32| {
                let d = a.wrapping_sub(b);
                (d / 1000, d % 1000)
            };
            if let Some(p) = st.prev[s] {
                let (a, b) = ms(r.t, p);
                let _ = write!(out, "{a}.{b:03}");
            }
            out.push(',');
            if let Some(p) = st.prev2[s] {
                let (a, b) = ms(r.t, p);
                let _ = write!(out, "{a}.{b:03}");
            }
            out.push_str("\r\n");
            st.prev2[s] = st.prev[s];
            st.prev[s] = Some(r.t);
        }
        to
    }
}
