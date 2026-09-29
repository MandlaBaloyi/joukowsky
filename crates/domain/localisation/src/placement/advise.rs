//! Choosing the sensors: greedily, then improved by swaps.
//!
//! Greedy selection is the established start (`burst_localisation.txt`
//! section 4; "Just one more sensor is enough", arXiv:2406.19900). It also
//! answers the client's real question - what does each extra logger buy? -
//! because every step is recorded. Swaps then repair what greed got wrong
//! (the best pair need not be part of the best three).
//!
//! Large networks (the scale study, V-S4):
//! - Lazy greedy (CELF: Leskovec et al. 2007, "Cost-effective outbreak
//!   detection in networks", developed on water-network sensor placement
//!   [VERIFY details]). A candidate's GAIN - how much it shrinks the
//!   expected region - shrinks as loggers are added (diminishing returns).
//!   So each step re-checks candidates in order of their last known gain
//!   and stops once none of the rest could beat the best one checked. Not
//!   proven for this objective (a second logger can make a first one more
//!   useful); on the test sites it chooses exactly what full greedy does,
//!   and the swaps repair the rest.
//! - Swaps are tried only with each logger's 12 NEAREST unused candidates
//!   (by wave travel time) that feel some of the same bursts. A good swap
//!   moves a logger a little; trying every candidate in town cost 86% of
//!   the time on a 20 km town (FN-020). On a site with 12 candidates or
//!   fewer this is every candidate, as before.

use jk_hydraulics::{Network, NodeId};

use super::bench::{Bench, State, Totals};
use super::coverage::{Coverage, PlacementOptions, validate};
use crate::error::LocalisationError;
use crate::graph::TravelGraph;

/// Beyond this many candidate pairs the first two sensors are not tried
/// exhaustively: the pair furthest apart by wave travel seeds the search.
const MAX_PAIRS: usize = 300;

/// Most rounds of swaps.
const MAX_SWAP_ROUNDS: usize = 5;

/// Unused candidates each logger may be swapped with: its nearest ones.
const SWAP_NEIGHBOURS: usize = 12;

/// Candidates re-checked together (in parallel) per round of lazy greedy.
const LAZY_BATCH: usize = 8;

/// The advisor's answer.
#[derive(Debug, Clone, PartialEq)]
pub struct Advice {
    /// What each extra logger buys: the greedy placement with 2 (or the
    /// number fixed, if more), 3, ... up to `count` sensors.
    pub steps: Vec<Coverage>,
    /// The recommendation for `count` sensors: the greedy placement
    /// improved by swaps.
    pub best: Coverage,
}

/// Chooses `count` sensor nodes among `candidates` (where a logger can be
/// installed), keeping the `fixed` ones (already installed).
///
/// # Errors
/// Fewer than two sensors asked for, too few candidates, or the scoring
/// errors of [`coverage`].
pub fn advise(
    network: &Network,
    candidates: &[NodeId],
    count: usize,
    fixed: &[NodeId],
    weights: Option<&[f64]>,
    options: &PlacementOptions,
) -> Result<Advice, LocalisationError> {
    advise_on(network, candidates, count, fixed, weights, options, None)
}

/// As [`advise`], scoring only bursts on the pipes marked in `only` (one
/// flag per pipe; None: every pipe) - one zone of a large network.
#[allow(clippy::too_many_lines)] // pairs, lazy greedy, swaps: one search, kept together
pub(super) fn advise_on(
    network: &Network,
    candidates: &[NodeId],
    count: usize,
    fixed: &[NodeId],
    weights: Option<&[f64]>,
    options: &PlacementOptions,
    only: Option<&[bool]>,
) -> Result<Advice, LocalisationError> {
    let mut pool: Vec<NodeId> = Vec::new();
    for &c in candidates {
        if !fixed.contains(&c) && !pool.contains(&c) {
            pool.push(c);
        }
    }
    if count < 2 || count < fixed.len() || fixed.len() + pool.len() < count {
        return Err(LocalisationError::InvalidInput(format!(
            "cannot place {count} sensors: {} fixed and {} other candidates (at least 2 needed)",
            fixed.len(),
            pool.len()
        )));
    }
    let all: Vec<NodeId> = fixed.iter().chain(&pool).copied().collect();
    validate(network, &all, weights, options)?;
    // Every placement below is scored on one shared bench: each trial burst
    // is located once per set of loggers that feel it, and a placement that
    // differs by one logger only looks again at the trials that logger
    // feels. The answers are those of `coverage` for each placement.
    let mut bench = Bench::new(network, weights, options, only);

    let mut chosen: Vec<NodeId> = fixed.to_vec();
    if chosen.len() < 2 {
        let sets: Vec<Vec<NodeId>> = if chosen.is_empty() {
            let pairs = pool.len() * (pool.len() - 1) / 2;
            if pairs <= MAX_PAIRS {
                (0..pool.len())
                    .flat_map(|i| (i + 1..pool.len()).map(move |j| (i, j)))
                    .map(|(i, j)| vec![pool[i], pool[j]])
                    .collect()
            } else {
                vec![farthest_pair(network, &pool)]
            }
        } else {
            pool.iter().map(|&c| vec![chosen[0], c]).collect()
        };
        bench.prefetch_placements(&sets);
        let mut best: Option<(Totals, Vec<NodeId>)> = None;
        for set in sets {
            let state = bench.state(&set);
            let totals = bench.totals(state.regions());
            if best.as_ref().is_none_or(|(b, _)| totals.beats(b)) {
                best = Some((totals, set));
            }
        }
        if let Some((_, set)) = best {
            chosen = set;
        }
    }
    let mut state = bench.state(&chosen);
    let mut steps = vec![bench.coverage(&chosen, state.regions())];
    // Per pool candidate: its last known gain, the step it was computed
    // at, and the totals with it added then.
    let mut known: Vec<Option<(f64, usize, Totals)>> = vec![None; pool.len()];
    while chosen.len() < count {
        let step = chosen.len();
        let now = bench.totals(state.regions()).expected_region;
        let tolerance = 1e-9 * now;
        loop {
            let fresh_best = (0..pool.len())
                .filter_map(|k| known[k].filter(|&(_, at, _)| at == step))
                .map(|(gain, _, _)| gain)
                .fold(f64::NEG_INFINITY, f64::max);
            // Candidates that might still beat the best fresh one: never
            // checked, or checked earlier with a gain at least as large.
            let mut stale: Vec<(f64, usize)> = (0..pool.len())
                .filter(|&k| !chosen.contains(&pool[k]))
                .filter_map(|k| match known[k] {
                    None => Some((f64::INFINITY, k)),
                    Some((gain, at, _)) if at != step && gain >= fresh_best - tolerance => {
                        Some((gain, k))
                    }
                    _ => None,
                })
                .collect();
            if stale.is_empty() {
                break;
            }
            stale.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
            let unknown = stale.iter().take_while(|(g, _)| g.is_infinite()).count();
            stale.truncate(unknown.max(LAZY_BATCH));
            let moves: Vec<(Option<usize>, usize)> = stale
                .iter()
                .map(|&(_, k)| (None, pool[k].index()))
                .collect();
            bench.prefetch_moves(&state, &moves);
            for (_, k) in stale {
                let regions = bench.regions_with(&state, None, pool[k].index());
                let totals = bench.totals(&regions);
                known[k] = Some((now - totals.expected_region, step, totals));
            }
        }
        // The best of those checked this step, in pool order (as full
        // greedy would pick it).
        let mut best: Option<(Totals, NodeId)> = None;
        for (k, &c) in pool.iter().enumerate() {
            if let Some((_, at, totals)) = known[k]
                && at == step
                && !chosen.contains(&c)
                && best.as_ref().is_none_or(|(b, _)| totals.beats(b))
            {
                best = Some((totals, c));
            }
        }
        let Some((_, c)) = best else { break };
        chosen.push(c);
        state = bench.state(&chosen);
        steps.push(bench.coverage(&chosen, state.regions()));
    }

    swap_rounds(&mut bench, &pool, &mut chosen, fixed.len(), &mut state);
    let best = bench.coverage(&chosen, state.regions());
    Ok(Advice { steps, best })
}

/// The two candidates furthest apart by (book) wave travel time.
fn farthest_pair(network: &Network, pool: &[NodeId]) -> Vec<NodeId> {
    let graph = TravelGraph::new(network);
    let nominal: Vec<f64> = network
        .pipes()
        .iter()
        .map(|p| p.pipe.travel_time().value())
        .collect();
    let mut best = (f64::NEG_INFINITY, vec![pool[0], pool[1]]);
    for (i, &a) in pool.iter().enumerate() {
        let times = graph.times_from(a.index(), &nominal);
        for &b in &pool[i + 1..] {
            let t = times[b.index()];
            if t.is_finite() && t > best.0 {
                best = (t, vec![a, b]);
            }
        }
    }
    best.1
}

/// Swaps: replaces a chosen logger (from index `first` on - the ones
/// before are fixed) by one of its nearest unused candidates in `pool`
/// whenever that helps, until nothing does. Updates `chosen` and `state`.
pub(super) fn swap_rounds(
    bench: &mut Bench<'_>,
    pool: &[NodeId],
    chosen: &mut [NodeId],
    first: usize,
    state: &mut State,
) {
    let mut best = bench.totals(state.regions());
    for _ in 0..MAX_SWAP_ROUNDS {
        let mut improved = false;
        for i in first..chosen.len() {
            // Everything this logger's swaps need, located in parallel
            // (again after each accepted swap, which changes the rest).
            let unused: Vec<NodeId> = pool
                .iter()
                .copied()
                .filter(|c| !chosen.contains(c) && bench.overlap(chosen[i].index(), c.index()))
                .collect();
            let nearest = bench.nearest(chosen[i].index(), &unused, SWAP_NEIGHBOURS);
            // In pool order, as full swaps would scan them.
            let near: Vec<NodeId> = unused
                .iter()
                .copied()
                .filter(|c| nearest.contains(c))
                .collect();
            let moves: Vec<(Option<usize>, usize)> =
                near.iter().map(|c| (Some(i), c.index())).collect();
            bench.prefetch_moves(state, &moves);
            for &c in &near {
                if chosen.contains(&c) {
                    continue;
                }
                let regions = bench.regions_with(state, Some(i), c.index());
                let totals = bench.totals(&regions);
                if totals.beats(&best) {
                    chosen[i] = c;
                    *state = bench.state(chosen);
                    best = totals;
                    improved = true;
                }
            }
        }
        if !improved {
            break;
        }
    }
}
