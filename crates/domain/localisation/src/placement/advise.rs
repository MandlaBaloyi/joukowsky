//! Choosing the sensors: greedily, then improved by swaps.
//!
//! Greedy selection is the established start (`burst_localisation.txt`
//! section 4; "Just one more sensor is enough", arXiv:2406.19900). It also
//! answers the client's real question - what does each extra logger buy? -
//! because every step is recorded. Swaps then repair what greed got wrong
//! (the best pair need not be part of the best three).

use jk_hydraulics::{Network, NodeId};

use super::coverage::{Coverage, PlacementOptions, coverage};
use crate::error::LocalisationError;
use crate::graph::TravelGraph;

/// Beyond this many candidate pairs the first two sensors are not tried
/// exhaustively: the pair furthest apart by wave travel seeds the search.
const MAX_PAIRS: usize = 300;

/// Most rounds of swaps.
const MAX_SWAP_ROUNDS: usize = 5;

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
    let score = |set: &[NodeId]| coverage(network, set, weights, options);
    let best_of = |sets: Vec<Vec<NodeId>>| -> Result<Option<Coverage>, LocalisationError> {
        let mut best: Option<Coverage> = None;
        for set in sets {
            let c = score(&set)?;
            if best.as_ref().is_none_or(|b| c.beats(b)) {
                best = Some(c);
            }
        }
        Ok(best)
    };

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
        if let Some(best) = best_of(sets)? {
            chosen = best.sensors;
        }
    }
    let mut steps = vec![score(&chosen)?];
    while chosen.len() < count {
        let sets = pool
            .iter()
            .filter(|c| !chosen.contains(c))
            .map(|&c| {
                let mut set = chosen.clone();
                set.push(c);
                set
            })
            .collect();
        let Some(best) = best_of(sets)? else { break };
        chosen.clone_from(&best.sensors);
        steps.push(best);
    }

    // Swaps: replace one chosen (not fixed) sensor by an unused candidate
    // whenever that helps, until nothing does.
    let mut best = steps[steps.len() - 1].clone();
    for _ in 0..MAX_SWAP_ROUNDS {
        let mut improved = false;
        for i in fixed.len()..chosen.len() {
            for &c in &pool {
                if chosen.contains(&c) {
                    continue;
                }
                let mut trial = chosen.clone();
                trial[i] = c;
                let cov = score(&trial)?;
                if cov.beats(&best) {
                    chosen = trial;
                    best = cov;
                    improved = true;
                }
            }
        }
        if !improved {
            break;
        }
    }
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
