//! Fastest wave travel times on the pipe graph (Dijkstra, our own).

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use jk_hydraulics::{Network, Node};

use super::Devices;

/// Fastest travel time from one source to every node, and each node's
/// route (the pipes the wave travels along, from the node back to the
/// source): [`TravelGraph::routes_from`].
pub(crate) type Routes = (Vec<f64>, Vec<Vec<usize>>);

/// The network as a wave-travel graph.
#[derive(Debug, Clone, PartialEq)]
pub struct TravelGraph {
    /// For each node: (neighbour, pipe index) for every pipe touching it.
    neighbours: Vec<Vec<(usize, usize)>>,
    /// For each node: may a wave pass through it? (false for reservoirs
    /// and shut devices)
    passes: Vec<bool>,
}

/// Heap entry ordered so the SMALLEST time comes out first.
#[derive(Debug, Clone, Copy, PartialEq)]
struct Entry(f64, usize);

impl Eq for Entry {}

impl Ord for Entry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .0
            .total_cmp(&self.0)
            .then_with(|| other.1.cmp(&self.1))
    }
}

impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl TravelGraph {
    /// Builds the graph of `network`, its devices as its steady state
    /// leaves them ([`Devices::of`]).
    #[must_use]
    pub fn new(network: &Network) -> Self {
        Self::with_devices(network, &Devices::of(network))
    }

    /// Builds the graph of `network` with `devices` already known.
    #[must_use]
    pub fn with_devices(network: &Network, devices: &Devices) -> Self {
        let n = network.nodes().len();
        let mut neighbours = vec![Vec::new(); n];
        for (i, p) in network.pipes().iter().enumerate() {
            neighbours[p.from.index()].push((p.to.index(), i));
            neighbours[p.to.index()].push((p.from.index(), i));
        }
        let passes = network
            .nodes()
            .iter()
            .enumerate()
            .map(|(i, node)| !matches!(node, Node::Reservoir { .. }) && !devices.is_shut(i))
            .collect();
        Self { neighbours, passes }
    }

    /// Number of nodes.
    #[must_use]
    pub fn len(&self) -> usize {
        self.neighbours.len()
    }

    /// True if the graph has no nodes.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.neighbours.is_empty()
    }

    /// Fastest travel time from `source` to every node, with pipe `i` taking
    /// `pipe_times[i]` seconds end to end. Unreachable nodes get infinity.
    ///
    /// # Panics
    /// If `source` is not a node or `pipe_times` is too short.
    #[must_use]
    pub fn times_from(&self, source: usize, pipe_times: &[f64]) -> Vec<f64> {
        self.routes_from(source, pipe_times).0
    }

    /// As [`Self::times_from`], and also each node's ROUTE: the pipes the
    /// fastest wave travels along from `source` to it. Knowing the route is
    /// what lets a pipe's uncertain wave speed be carried into the arrival
    /// times of every sensor whose wave passes through it.
    ///
    /// # Panics
    /// If `source` is not a node or `pipe_times` is too short.
    #[must_use]
    pub fn routes_from(&self, source: usize, pipe_times: &[f64]) -> (Vec<f64>, Vec<Vec<usize>>) {
        let mut previous: Vec<Option<(usize, usize)>> = vec![None; self.len()];
        let mut best = vec![f64::INFINITY; self.len()];
        let mut heap = BinaryHeap::new();
        best[source] = 0.0;
        heap.push(Entry(0.0, source));
        while let Some(Entry(t, node)) = heap.pop() {
            if t > best[node] {
                continue; // a stale entry
            }
            // A wave may leave the source, but not pass through a reservoir
            // or a shut device.
            if node != source && !self.passes[node] {
                continue;
            }
            for &(next, pipe) in &self.neighbours[node] {
                let arrival = t + pipe_times[pipe];
                if arrival < best[next] {
                    best[next] = arrival;
                    previous[next] = Some((pipe, node));
                    heap.push(Entry(arrival, next));
                }
            }
        }
        let routes = (0..self.len())
            .map(|mut node| {
                let mut pipes = Vec::new();
                while let Some((pipe, from)) = previous[node] {
                    pipes.push(pipe);
                    node = from;
                }
                pipes
            })
            .collect();
        (best, routes)
    }

    /// Fastest travel time from `source` to a point `fraction` of the way
    /// along a pipe from node `from` to node `to`, taking `pipe_time` end to
    /// end, given `times` = [`Self::times_from`]`(source)`. The wave reaches
    /// the point through one of the pipe's ends - never through a reservoir
    /// end, unless the source is that reservoir.
    #[must_use]
    #[inline]
    pub fn point_time(
        &self,
        source: usize,
        times: &[f64],
        (from, to): (usize, usize),
        fraction: f64,
        pipe_time: f64,
    ) -> f64 {
        self.point_route(source, times, (from, to), fraction, pipe_time)
            .0
    }

    /// When a wave from `source` can enter the pipe from `from` to `to`
    /// through each end (infinity through a reservoir end, unless the
    /// source is that reservoir), given `times` = [`Self::times_from`]
    /// `(source)`. A point `fraction` along the pipe is then reached at
    /// min(from + fraction x `pipe_time`, to + (1 - fraction) x `pipe_time`).
    #[must_use]
    #[inline]
    pub fn end_times(
        &self,
        source: usize,
        times: &[f64],
        (from, to): (usize, usize),
    ) -> (f64, f64) {
        let via = |end: usize| {
            if end == source || self.passes[end] {
                times[end]
            } else {
                f64::INFINITY
            }
        };
        (via(from), via(to))
    }

    /// As [`Self::point_time`], also returning which end of the pipe the
    /// wave comes in through and the fraction of the pipe it then travels
    /// (None if the point cannot be reached).
    #[must_use]
    #[inline]
    pub fn point_route(
        &self,
        source: usize,
        times: &[f64],
        (from, to): (usize, usize),
        fraction: f64,
        pipe_time: f64,
    ) -> (f64, Option<(usize, f64)>) {
        let (at_from, at_to) = self.end_times(source, times, (from, to));
        let (t_from, t_to) = (
            at_from + fraction * pipe_time,
            at_to + (1.0 - fraction) * pipe_time,
        );
        if !t_from.is_finite() && !t_to.is_finite() {
            (f64::INFINITY, None)
        } else if t_from <= t_to {
            (t_from, Some((from, fraction)))
        } else {
            (t_to, Some((to, 1.0 - fraction)))
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use jk_hydraulics::Pipe;
    use jk_kernel::{Flow, Head, Length, STANDARD_GRAVITY, WaveSpeed};

    fn pipe(l: f64) -> Pipe {
        Pipe::new(
            Length::new(l),
            Length::new(0.15),
            WaveSpeed::new(1000.0),
            0.02,
        )
        .unwrap()
    }

    #[test]
    fn fastest_path_and_no_passing_through_reservoirs() {
        // A --100-- B --100-- R(reservoir) --100-- C, and A --500-- C.
        // From A: C is reached in 0.5 s directly; the 0.3 s route through
        // the reservoir is not physical.
        let mut net = Network::new(STANDARD_GRAVITY);
        let j = || Node::Junction { demand: Flow::ZERO };
        let (a, b) = (net.add_node(j()), net.add_node(j()));
        let r = net.add_node(Node::Reservoir {
            head: Head::new(50.0),
        });
        let c = net.add_node(j());
        for (u, v, l) in [(a, b, 100.0), (b, r, 100.0), (r, c, 100.0), (a, c, 500.0)] {
            net.add_pipe(u, v, pipe(l)).unwrap();
        }
        let g = TravelGraph::new(&net);
        let times: Vec<f64> = net
            .pipes()
            .iter()
            .map(|p| p.pipe.travel_time().value())
            .collect();
        let t = g.times_from(a.index(), &times);
        assert!((t[b.index()] - 0.1).abs() < 1e-12);
        assert!(
            (t[r.index()] - 0.2).abs() < 1e-12,
            "the reservoir itself is reached"
        );
        assert!(
            (t[c.index()] - 0.5).abs() < 1e-12,
            "not 0.3 through the reservoir"
        );
    }
}
