//! Jet thrust that follows the spool. The game's combustor turns throttle into thrust in the same tick
//! (`air_out = air_in + throttle·rps·1600`), so a throttle step gives a thrust spike long before the shaft
//! spools up, and a throttle blip to 0 drops thrust to nothing for that tick. Here a nozzle's thrust is
//! smoothed (gas and combustion don't react within a tick; ticks the game skips are filled in), and never exceeds what the engine makes while
//! *holding* the current shaft RPS, learned from its own steady running. The engine's internal state is
//! left as the game runs it.

/// Shaft RPS per learned bin.
const BIN: f64 = 5.0;
/// Per-tick smoothing of thrust (≈0.5 s time constant at 60 Hz), both directions.
const SMOOTH: f64 = 1.0 / 30.0;
/// RPS counts as held when its 0.25 s and 1 s averages agree within this share for `STEADY_TICKS`.
const STEADY_SHARE: f64 = 0.03;
const STEADY_TICKS: u32 = 30;

#[derive(Clone, Debug, Default)]
pub struct Spool {
    /// Per RPS bin: (rps, average smoothed thrust while holding it); thrust 0 = not learned.
    env: Vec<(f64, f64)>,
    fast: f64,
    slow: f64,
    steady: u32,
    smooth: f64,
    rps: f64,
}

impl Spool {
    /// The game's thrust magnitude `f` at shaft `rps` this tick (the nozzle queued thrust).
    pub fn sample(&mut self, rps: f64, f: f64) {
        if !(rps.is_finite() && f.is_finite()) || rps < 0.0 || f < 0.0 {
            return;
        }
        self.rps = rps;
        self.smooth += (f - self.smooth) * SMOOTH;
        if self.slow == 0.0 {
            (self.fast, self.slow) = (rps, rps);
        }
        self.fast += (rps - self.fast) * (1.0 / 15.0);
        self.slow += (rps - self.slow) * (1.0 / 60.0);
        self.steady = if (self.fast - self.slow).abs() <= STEADY_SHARE * self.slow.max(1.0) { self.steady + 1 } else { 0 };
        if self.steady >= STEADY_TICKS {
            let b = (self.slow / BIN) as usize;
            if self.env.len() <= b {
                self.env.resize(b + 1, (0.0, 0.0));
            }
            let e = &mut self.env[b];
            *e = if e.1 == 0.0 { (self.slow, self.smooth) } else { (e.0 + (self.slow - e.0) * 0.05, e.1 + (self.smooth - e.1) * 0.05) };
        }
    }

    /// A tick where the game queued no thrust (combustion off this tick): the smoothed thrust decays.
    pub fn idle(&mut self) {
        self.smooth *= 1.0 - SMOOTH;
    }

    /// Thrust to apply now: smoothed, capped by the steady thrust at the current spool.
    pub fn output(&self) -> f64 {
        match self.cap(self.rps) {
            Some(c) => self.smooth.min(c),
            None => self.smooth,
        }
    }

    /// Steady thrust at `rps` from the learned points: linear between them, ∝ rps² below the first.
    /// None above the highest learned point: never hold an engine below what hasn't been seen yet.
    fn cap(&self, rps: f64) -> Option<f64> {
        let pts: Vec<(f64, f64)> = self.env.iter().copied().filter(|p| p.1 > 0.0).collect();
        let (first, last) = (*pts.first()?, *pts.last()?);
        if rps > last.0 {
            return None;
        }
        if rps <= first.0 {
            return Some(first.1 * (rps / first.0).powi(2));
        }
        let w = pts.windows(2).find(|w| rps <= w[1].0)?;
        let t = (rps - w[0].0) / (w[1].0 - w[0].0);
        Some(w[0].1 + (w[1].1 - w[0].1) * t)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn spool_limits_spikes_and_rides_through_throttle_blips() {
        let mut s = Spool::default();
        // Holding 50 and 150 RPS teaches the steady thrust there.
        for _ in 0..240 {
            s.sample(50.0, 4_000.0);
        }
        for _ in 0..600 {
            s.sample(150.0, 20_000.0);
        }
        assert!((s.output() - 20_000.0).abs() < 50.0, "steady thrust is untouched");
        // A PID cutting throttle for 2 ticks (no thrust queued) barely dents it.
        s.idle();
        s.idle();
        assert!(s.output() > 18_000.0, "blip: {}", s.output());
        // Spooling through 100 RPS at full throttle: the spike is cut to the steady curve (~12 kN).
        let mut spool = Spool { env: s.env.clone(), ..Default::default() };
        for i in 0..120 {
            spool.sample(60.0 + i as f64 * 0.33, 60_000.0);
        }
        assert!(spool.output() < 14_000.0, "spool-up capped: {}", spool.output());
        // Above anything learned the engine is never held back (only smoothed).
        let mut fresh = Spool::default();
        for _ in 0..300 {
            fresh.sample(180.0, 30_000.0);
        }
        assert!(fresh.output() > 29_000.0, "unlearned: {}", fresh.output());
    }
}
