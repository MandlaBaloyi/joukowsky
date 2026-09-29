//! The trial bursts every placement is scored on - shared by all the
//! placements one advice compares.
//!
//! Scoring a placement means locating a burst at every trial point (every
//! `source_step` along every pipe) with the loggers that would feel it, and
//! measuring its 90% region. The advisor scores thousands of placements
//! that differ by one logger. Two facts make that cheap without changing
//! any answer:
//! - a trial's region depends only on WHICH loggers feel it (their arrival
//!   times follow from where the trial is), so each (trial, loggers) pair is
//!   located once and remembered;
//! - adding or swapping one logger changes which loggers feel only the
//!   trials that logger feels - a 1 l/s burst is felt a few junctions away,
//!   not across town - so only those trials are looked at again.
//!
//! The graph, the candidate grid and each logger's routes are built once and
//! shared by every localiser. Before each round of comparisons the advisor
//! lists every (trial, loggers) pair it will need and locates the new ones
//! on all processor cores ([`Bench::prefetch`]); each is independent, so the
//! answers do not depend on how the work is split.

use std::collections::{BTreeSet, HashMap};
use std::sync::Arc;

use jk_hydraulics::{Network, NodeId, PipeId};
use jk_kernel::{Length, Time};

use super::coverage::{Coverage, PipeCoverage, PlacementOptions};
use super::detectability::Fronts;
use crate::graph::{Routes, TravelGraph};
use crate::locate::{Arrival, Localiser, nominal_times};
use crate::posterior::grid::Grid;

/// Loggers (node indices, ascending) and the trials they all feel.
type Group = (Vec<usize>, Vec<usize>);

/// A located (trial, loggers) pair and its 90% region [m].
type Found = ((usize, Vec<usize>), f64);

/// Most threads used to locate trial bursts.
const MAX_THREADS: usize = 4;

/// At most this many localisers are kept; beyond it the cache starts
/// afresh (regions already found are kept - they are all that is reused
/// often).
const MAX_LOCALISERS: usize = 512;

/// The two figures placements are compared on.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) struct Totals {
    /// Mean 90% region, weighted [m].
    pub expected_region: f64,
    /// Weighted share of trials pinpointed.
    pub pinpointed: f64,
}

impl Totals {
    /// As [`Coverage::beats`].
    pub(super) fn beats(&self, other: &Self) -> bool {
        let (a, b) = (self.expected_region, other.expected_region);
        if (a - b).abs() > 1e-9 * a.max(b) {
            a < b
        } else {
            self.pinpointed > other.pinpointed
        }
    }
}

/// One pipe's trials.
#[derive(Debug, Clone, Copy)]
struct PipeTrials {
    id: PipeId,
    length: Length,
    first: usize,
    count: usize,
    /// Weight of each of its trials: pipe weight x segment length.
    weight: f64,
}

/// Which loggers feel each trial, and each trial's 90% region, for one
/// placement.
#[derive(Debug, Clone)]
pub(super) struct State {
    /// The placement's loggers (node indices), in the caller's order.
    pub sensors: Vec<usize>,
    /// Per trial: the loggers that feel it (node indices, ascending).
    felt_by: Vec<Vec<usize>>,
    /// Per trial: the size of its 90% region [m].
    regions: Vec<f64>,
}

/// The shared trials, and everything learnt about them so far.
pub(super) struct Bench<'a> {
    network: &'a Network,
    options: PlacementOptions,
    graph: Arc<TravelGraph>,
    grid: Arc<Grid>,
    fronts: Option<Fronts>,
    ends: Vec<(usize, usize)>,
    nominal: Vec<f64>,
    total_length: f64,
    /// (pipe, fraction) of every trial, pipe by pipe.
    trials: Vec<(usize, f64)>,
    pipes: Vec<PipeTrials>,
    /// Per logger node: its routes.
    routes: HashMap<usize, Arc<Routes>>,
    /// Per logger node: the trials it feels, ascending, with arrival times.
    hearing: HashMap<usize, Vec<(usize, f64)>>,
    localisers: HashMap<Vec<usize>, Option<Localiser<'a>>>,
    regions: HashMap<(usize, Vec<usize>), f64>,
}

impl<'a> Bench<'a> {
    /// Lays out the trials on `network`. `weights` (one per pipe) default to
    /// 1. Options must have been validated.
    pub(super) fn new(
        network: &'a Network,
        weights: Option<&[f64]>,
        options: &PlacementOptions,
    ) -> Self {
        let pipes = network.pipes();
        let step = options.source_step.value();
        let mut trials = Vec::new();
        let mut per_pipe = Vec::with_capacity(pipes.len());
        for (p, link) in pipes.iter().enumerate() {
            let Some(id) = network.pipe_id(p) else {
                continue;
            };
            let length = link.pipe.length().value();
            #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
            // ceil of a positive ratio
            let n = ((length / step).ceil() as usize).max(1);
            let segment = length / to_f64(n);
            per_pipe.push(PipeTrials {
                id,
                length: link.pipe.length(),
                first: trials.len(),
                count: n,
                weight: weights.map_or(1.0, |w| w[p]) * segment,
            });
            trials.extend((0..n).map(|i| (p, (to_f64(i) + 0.5) / to_f64(n))));
        }
        Self {
            network,
            options: *options,
            graph: Arc::new(TravelGraph::new(network)),
            grid: Arc::new(Grid::new(network, options.localise.spacing.value())),
            fronts: options.burst.map(|_| Fronts::new(network, &[])),
            ends: pipes
                .iter()
                .map(|p| (p.from.index(), p.to.index()))
                .collect(),
            nominal: nominal_times(network),
            total_length: pipes.iter().map(|p| p.pipe.length().value()).sum(),
            trials,
            pipes: per_pipe,
            routes: HashMap::new(),
            hearing: HashMap::new(),
            localisers: HashMap::new(),
            regions: HashMap::new(),
        }
    }

    fn routes_of(&mut self, sensor: usize) -> Arc<Routes> {
        let (graph, nominal) = (&self.graph, &self.nominal);
        Arc::clone(
            self.routes
                .entry(sensor)
                .or_insert_with(|| Arc::new(graph.routes_from(sensor, nominal))),
        )
    }

    /// The trials logger `sensor` feels (ascending), with arrival times.
    fn hearing_of(&mut self, sensor: usize) -> &[(usize, f64)] {
        if !self.hearing.contains_key(&sensor) {
            let routes = self.routes_of(sensor);
            let threshold = self.options.detect_units * self.options.noise.value();
            let heard: Vec<(usize, f64)> = self
                .trials
                .iter()
                .enumerate()
                .filter_map(|(t, &(pipe, fraction))| {
                    let felt = match (&self.fronts, self.options.burst) {
                        (Some(f), Some(q)) => {
                            f.at(sensor, &routes, pipe, fraction) * q.value() >= threshold
                        }
                        _ => true,
                    };
                    let time = self.graph.point_time(
                        sensor,
                        &routes.0,
                        self.ends[pipe],
                        fraction,
                        self.nominal[pipe],
                    );
                    (felt && time.is_finite()).then_some((t, time))
                })
                .collect();
            self.hearing.insert(sensor, heard);
        }
        &self.hearing[&sensor]
    }

    /// The arrivals of trial `t` at loggers `felt` (their hearing must be
    /// known).
    fn arrivals(&self, t: usize, felt: &[usize]) -> Vec<Arrival> {
        felt.iter()
            .map(|&s| {
                let heard = &self.hearing[&s];
                let k = heard
                    .binary_search_by_key(&t, |&(trial, _)| trial)
                    .expect("a logger in `felt` feels the trial");
                Arrival {
                    node: self.network.node_id(s).expect("a node of the network"),
                    time: Time::new(heard[k].1),
                    uncertainty: self.options.timing,
                }
            })
            .collect()
    }

    /// A localiser for loggers `felt` (their routes must be known).
    fn build_localiser(&self, felt: &[usize]) -> Option<Localiser<'a>> {
        let nodes: Vec<NodeId> = felt
            .iter()
            .map(|&s| self.network.node_id(s).expect("a node of the network"))
            .collect();
        let routes: Vec<Arc<Routes>> = felt.iter().map(|&s| Arc::clone(&self.routes[&s])).collect();
        Localiser::with_shared(
            self.network,
            &nodes,
            &self.options.localise,
            Arc::clone(&self.graph),
            Arc::clone(&self.grid),
            routes,
        )
        .ok()
    }

    /// Size of the 90% region of trial `t` felt by `felt`, located with
    /// `localiser` (None: it could not be built) [m].
    fn locate(&self, localiser: Option<&Localiser<'a>>, t: usize, felt: &[usize]) -> f64 {
        localiser
            .and_then(|l| l.credible_length(&self.arrivals(t, felt)).ok())
            .map_or(self.total_length, Length::value)
    }

    /// Makes sure the hearing and routes of loggers `felt` are known.
    fn prepare(&mut self, felt: &[usize]) {
        for &s in felt {
            self.hearing_of(s);
            self.routes_of(s);
        }
    }

    /// Size of the 90% region of trial `t` felt by `felt` (ascending node
    /// indices) [m]: the whole network when fewer than two feel it.
    fn region(&mut self, t: usize, felt: &[usize]) -> f64 {
        if felt.len() < 2 {
            return self.total_length;
        }
        let key = (t, felt.to_vec());
        if let Some(&r) = self.regions.get(&key) {
            return r;
        }
        self.prepare(felt);
        if !self.localisers.contains_key(felt) {
            if self.localisers.len() >= MAX_LOCALISERS {
                self.localisers.clear();
            }
            let l = self.build_localiser(felt);
            self.localisers.insert(felt.to_vec(), l);
        }
        let r = self.locate(self.localisers[felt].as_ref(), t, felt);
        self.regions.insert(key, r);
        r
    }

    /// Locates every (trial, loggers) pair in `wanted` that is not known
    /// yet, on all processor cores, and remembers the regions.
    fn prefetch(&mut self, wanted: impl IntoIterator<Item = (usize, Vec<usize>)>) {
        let mut groups: HashMap<Vec<usize>, Vec<usize>> = HashMap::new();
        for (t, felt) in wanted {
            if felt.len() >= 2 && !self.regions.contains_key(&(t, felt.clone())) {
                groups.entry(felt).or_default().push(t);
            }
        }
        if groups.is_empty() {
            return;
        }
        let loggers: BTreeSet<usize> = groups.keys().flatten().copied().collect();
        for s in loggers {
            self.prepare(&[s]);
        }
        let mut groups: Vec<(Vec<usize>, Vec<usize>)> = groups.into_iter().collect();
        for (_, trials) in &mut groups {
            trials.sort_unstable();
            trials.dedup();
        }
        // Largest groups first, dealt to the least loaded thread.
        groups.sort_by(|a, b| b.1.len().cmp(&a.1.len()).then_with(|| a.0.cmp(&b.0)));
        let threads = std::thread::available_parallelism()
            .map_or(1, std::num::NonZero::get)
            .clamp(1, MAX_THREADS);
        let mut buckets: Vec<(usize, Vec<Group>)> = (0..threads).map(|_| (0, Vec::new())).collect();
        for g in groups {
            let lightest = buckets
                .iter_mut()
                .min_by_key(|(load, _)| *load)
                .expect("at least one thread");
            lightest.0 += g.1.len();
            lightest.1.push(g);
        }
        let this = &*self;
        let found: Vec<Vec<Found>> = std::thread::scope(|scope| {
            let handles: Vec<_> = buckets
                .into_iter()
                .map(|(_, bucket)| {
                    scope.spawn(move || {
                        let mut out = Vec::new();
                        for (felt, trials) in bucket {
                            let built;
                            let localiser = if let Some(l) = this.localisers.get(&felt) {
                                l.as_ref()
                            } else {
                                built = this.build_localiser(&felt);
                                built.as_ref()
                            };
                            for t in trials {
                                let r = this.locate(localiser, t, &felt);
                                out.push(((t, felt.clone()), r));
                            }
                        }
                        out
                    })
                })
                .collect();
            handles
                .into_iter()
                .map(|h| h.join().expect("a placement thread panicked"))
                .collect()
        });
        for (key, r) in found.into_iter().flatten() {
            self.regions.insert(key, r);
        }
    }

    /// Which loggers feel each trial, for loggers at `nodes`.
    fn felt_by(&mut self, nodes: &[usize]) -> Vec<Vec<usize>> {
        let mut felt_by = vec![Vec::new(); self.trials.len()];
        for &s in nodes {
            let heard: Vec<usize> = self.hearing_of(s).iter().map(|&(t, _)| t).collect();
            for t in heard {
                felt_by[t].push(s);
            }
        }
        for felt in &mut felt_by {
            felt.sort_unstable();
        }
        felt_by
    }

    /// Whether loggers at nodes `a` and `b` feel any trial in common.
    pub(super) fn overlap(&mut self, a: usize, b: usize) -> bool {
        self.hearing_of(a);
        self.hearing_of(b);
        let (x, y) = (&self.hearing[&a], &self.hearing[&b]);
        let (mut i, mut j) = (0, 0);
        while i < x.len() && j < y.len() {
            match x[i].0.cmp(&y[j].0) {
                std::cmp::Ordering::Less => i += 1,
                std::cmp::Ordering::Greater => j += 1,
                std::cmp::Ordering::Equal => return true,
            }
        }
        false
    }

    /// Locates, on all cores, everything scoring each of `placements` needs.
    pub(super) fn prefetch_placements(&mut self, placements: &[Vec<NodeId>]) {
        let mut wanted = Vec::new();
        for set in placements {
            let nodes: Vec<usize> = set.iter().map(|s| s.index()).collect();
            wanted.extend(self.felt_by(&nodes).into_iter().enumerate());
        }
        self.prefetch(wanted);
    }

    /// Locates, on all cores, everything [`Self::regions_with`] needs for
    /// each of `moves` (out, add) from `state`.
    pub(super) fn prefetch_moves(&mut self, state: &State, moves: &[(Option<usize>, usize)]) {
        let mut wanted = Vec::new();
        for &(out, add) in moves {
            wanted.extend(self.changed(state, out, add));
        }
        self.prefetch(wanted);
    }

    /// The trials whose loggers change if `state`'s logger number `out`
    /// (None: none) is replaced by - or joined by - the logger at node
    /// `add`, with their new loggers.
    fn changed(
        &mut self,
        state: &State,
        out: Option<usize>,
        add: usize,
    ) -> Vec<(usize, Vec<usize>)> {
        let removed = out.map(|i| state.sensors[i]);
        let mut touched: Vec<usize> = self.hearing_of(add).iter().map(|&(t, _)| t).collect();
        if let Some(r) = removed {
            touched.extend(self.hearing_of(r).iter().map(|&(t, _)| t));
            touched.sort_unstable();
            touched.dedup();
        }
        let heard = &self.hearing[&add];
        touched
            .into_iter()
            .map(|t| {
                let hears = heard.binary_search_by_key(&t, |&(trial, _)| trial).is_ok();
                let mut felt: Vec<usize> = state.felt_by[t]
                    .iter()
                    .copied()
                    .filter(|&s| Some(s) != removed)
                    .collect();
                if hears {
                    let at = felt.partition_point(|&s| s < add);
                    felt.insert(at, add);
                }
                (t, felt)
            })
            .collect()
    }

    /// Which loggers feel each trial, and the regions, for `sensors`.
    pub(super) fn state(&mut self, sensors: &[NodeId]) -> State {
        let nodes: Vec<usize> = sensors.iter().map(|s| s.index()).collect();
        let felt_by = self.felt_by(&nodes);
        let regions = felt_by
            .iter()
            .enumerate()
            .map(|(t, felt)| self.region(t, felt))
            .collect();
        State {
            sensors: nodes,
            felt_by,
            regions,
        }
    }

    /// The regions if `state`'s logger number `out` (None: none) were
    /// replaced by - or joined by - the logger at node `add`.
    pub(super) fn regions_with(
        &mut self,
        state: &State,
        out: Option<usize>,
        add: usize,
    ) -> Vec<f64> {
        let mut regions = state.regions.clone();
        for (t, felt) in self.changed(state, out, add) {
            regions[t] = self.region(t, &felt);
        }
        regions
    }

    /// The comparison figures for per-trial `regions`, summed exactly as
    /// [`Self::coverage`] sums them.
    pub(super) fn totals(&self, regions: &[f64]) -> Totals {
        let pin = self.options.pinpoint.value();
        let (mut weighted_region, mut weighted_pinpointed, mut total_weight) = (0.0, 0.0, 0.0);
        for pt in &self.pipes {
            let r = &regions[pt.first..pt.first + pt.count];
            let pinpointed = r.iter().filter(|&&x| x <= pin).count();
            weighted_region += pt.weight * r.iter().sum::<f64>();
            weighted_pinpointed += pt.weight * to_f64(pinpointed);
            total_weight += pt.weight * to_f64(pt.count);
        }
        if total_weight > 0.0 {
            Totals {
                expected_region: weighted_region / total_weight,
                pinpointed: weighted_pinpointed / total_weight,
            }
        } else {
            Totals {
                expected_region: 0.0,
                pinpointed: 0.0,
            }
        }
    }

    /// The full scorecard for `sensors` with per-trial `regions`.
    pub(super) fn coverage(&self, sensors: &[NodeId], regions: &[f64]) -> Coverage {
        let totals = self.totals(regions);
        let pin = self.options.pinpoint.value();
        let pipes = self
            .pipes
            .iter()
            .map(|pt| {
                let mut r = regions[pt.first..pt.first + pt.count].to_vec();
                let pinpointed = r.iter().filter(|&&x| x <= pin).count();
                r.sort_by(f64::total_cmp);
                PipeCoverage {
                    pipe: pt.id,
                    length: pt.length,
                    pinpointed: to_f64(pinpointed) / to_f64(pt.count),
                    median_region: Length::new(r[pt.count / 2]),
                }
            })
            .collect();
        Coverage {
            sensors: sensors.to_vec(),
            expected_region: Length::new(totals.expected_region),
            pinpointed: totals.pinpointed,
            pipes,
        }
    }
}

impl State {
    /// The per-trial regions.
    pub(super) fn regions(&self) -> &[f64] {
        &self.regions
    }
}

#[allow(clippy::cast_precision_loss)] // counts are far below 2^52
fn to_f64(i: usize) -> f64 {
    i as f64
}
