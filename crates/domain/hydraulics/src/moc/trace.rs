//! Recorded head and flow histories at chosen points ("probes").
//!
//! A probe is the simulation's equivalent of a pressure sensor: the engine
//! compares these histories with what real sensors record. Each solver
//! decides what its probes are (a node index, a point on a pipe, a network
//! node) and pushes one sample per probe per time step.

use jk_kernel::{Flow, Head, Time};

/// Head and flow histories, one per probe, in the order the probes were
/// requested.
///
/// Sample k is at time k * dt; sample 0 is the initial (steady) state.
#[derive(Debug, Clone, PartialEq)]
pub struct Trace {
    time_step: Time,
    heads: Vec<Vec<Head>>,
    flows: Vec<Vec<Flow>>,
}

impl Trace {
    /// An empty trace for `probes` probes, with room for `capacity` samples.
    pub(crate) fn new(time_step: Time, probes: usize, capacity: usize) -> Self {
        Self {
            time_step,
            heads: vec![Vec::with_capacity(capacity); probes],
            flows: vec![Vec::with_capacity(capacity); probes],
        }
    }

    /// Appends one sample to probe `i`.
    pub(crate) fn push(&mut self, i: usize, head: f64, flow: f64) {
        self.heads[i].push(Head::new(head));
        self.flows[i].push(Flow::new(flow));
    }

    /// The time step dt.
    #[must_use]
    pub fn time_step(&self) -> Time {
        self.time_step
    }

    /// Number of probes.
    #[must_use]
    pub fn probe_count(&self) -> usize {
        self.heads.len()
    }

    /// Number of samples per probe (steps + 1).
    #[must_use]
    pub fn len(&self) -> usize {
        self.heads.first().map_or(0, Vec::len)
    }

    /// True if nothing was recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Time of sample `k`.
    #[must_use]
    #[allow(clippy::cast_precision_loss)] // k < MAX_STEPS, exact in f64
    pub fn time(&self, k: usize) -> Time {
        self.time_step * k as f64
    }

    /// Index of the first sample at or after time `t` (clamped to the end).
    #[must_use]
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // clamped below
    pub fn sample_at(&self, t: Time) -> usize {
        let k = (t / self.time_step).ceil().max(0.0) as usize;
        k.min(self.len().saturating_sub(1))
    }

    /// Head history of the `i`-th probe.
    ///
    /// # Panics
    /// If `i` is not less than the number of probes.
    #[must_use]
    pub fn head(&self, i: usize) -> &[Head] {
        &self.heads[i]
    }

    /// Flow history of the `i`-th probe. What "flow" means depends on the
    /// probe: the pipe flow for a point on a pipe, the outflow for a
    /// network node.
    ///
    /// # Panics
    /// If `i` is not less than the number of probes.
    #[must_use]
    pub fn flow(&self, i: usize) -> &[Flow] {
        &self.flows[i]
    }

    /// Highest head seen at the `i`-th probe.
    ///
    /// # Panics
    /// If `i` is not less than the number of probes.
    #[must_use]
    pub fn max_head(&self, i: usize) -> Head {
        self.heads[i]
            .iter()
            .copied()
            .fold(Head::new(f64::NEG_INFINITY), Head::max)
    }

    /// Lowest head seen at the `i`-th probe.
    ///
    /// # Panics
    /// If `i` is not less than the number of probes.
    #[must_use]
    pub fn min_head(&self, i: usize) -> Head {
        self.heads[i]
            .iter()
            .copied()
            .fold(Head::new(f64::INFINITY), Head::min)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keeps_samples_per_probe_in_order() {
        let mut t = Trace::new(Time::new(0.5), 2, 2);
        t.push(0, 12.0, 0.3);
        t.push(1, 10.0, 0.1);
        t.push(0, 15.0, 0.6);
        t.push(1, 13.0, 0.4);
        assert_eq!(t.len(), 2);
        assert_eq!(t.probe_count(), 2);
        assert_eq!(t.head(0), &[Head::new(12.0), Head::new(15.0)]);
        assert_eq!(t.flow(1), &[Flow::new(0.1), Flow::new(0.4)]);
        assert_eq!(t.time(1), Time::new(0.5));
        assert_eq!(t.max_head(1), Head::new(13.0));
        assert_eq!(t.min_head(0), Head::new(12.0));
        assert_eq!(t.sample_at(Time::new(0.4)), 1);
        assert_eq!(t.sample_at(Time::new(99.0)), 1);
    }
}
