//! Recorded head and flow histories at chosen nodes ("probes").
//!
//! A probe is the simulation's equivalent of a pressure sensor: the engine
//! compares these histories with what real sensors record.

use jk_kernel::{Flow, Head, Time};

/// Head and flow histories at the requested probe nodes.
///
/// Sample k is at time k * dt; sample 0 is the initial (steady) state.
#[derive(Debug, Clone, PartialEq)]
pub struct Trace {
    time_step: Time,
    probes: Vec<usize>,
    heads: Vec<Vec<Head>>,
    flows: Vec<Vec<Flow>>,
}

impl Trace {
    /// An empty trace for `probes`, with room for `capacity` samples each.
    pub(crate) fn new(time_step: Time, probes: &[usize], capacity: usize) -> Self {
        Self {
            time_step,
            probes: probes.to_vec(),
            heads: vec![Vec::with_capacity(capacity); probes.len()],
            flows: vec![Vec::with_capacity(capacity); probes.len()],
        }
    }

    /// Appends one sample per probe from the full node arrays.
    pub(crate) fn record(&mut self, heads: &[f64], flows: &[f64]) {
        for (j, &node) in self.probes.iter().enumerate() {
            self.heads[j].push(Head::new(heads[node]));
            self.flows[j].push(Flow::new(flows[node]));
        }
    }

    /// The time step dt.
    #[must_use]
    pub fn time_step(&self) -> Time {
        self.time_step
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

    /// Node indices that were recorded, in request order.
    #[must_use]
    pub fn probes(&self) -> &[usize] {
        &self.probes
    }

    /// Head history of the `i`-th requested probe.
    ///
    /// # Panics
    /// If `i` is not less than the number of probes.
    #[must_use]
    pub fn head(&self, i: usize) -> &[Head] {
        &self.heads[i]
    }

    /// Flow history of the `i`-th requested probe.
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
    fn records_only_the_probes_in_request_order() {
        let mut t = Trace::new(Time::new(0.5), &[2, 0], 2);
        t.record(&[10.0, 11.0, 12.0], &[0.1, 0.2, 0.3]);
        t.record(&[13.0, 14.0, 15.0], &[0.4, 0.5, 0.6]);
        assert_eq!(t.len(), 2);
        assert_eq!(t.probes(), &[2, 0]);
        assert_eq!(t.head(0), &[Head::new(12.0), Head::new(15.0)]);
        assert_eq!(t.flow(1), &[Flow::new(0.1), Flow::new(0.4)]);
        assert_eq!(t.time(1), Time::new(0.5));
        assert_eq!(t.max_head(1), Head::new(13.0));
        assert_eq!(t.min_head(0), Head::new(12.0));
    }
}
