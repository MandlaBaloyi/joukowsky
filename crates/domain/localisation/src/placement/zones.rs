//! Planning a large network zone by zone.
//!
//! Choosing loggers for a whole town at once compares every candidate
//! against every trial burst it feels. A 1 l/s burst is felt a kilometre or
//! two away, so on a compact 50 km town (3 km across) each candidate feels
//! most of the town and the search took over half an hour
//! (`10_findings/detection_range_and_scale.txt`). Real planners work by
//! area, and so does this - the "cellphone towers" of that file:
//!
//! 1. Zones. Centres are spread out by wave travel time (each next centre
//!    is the junction furthest from those chosen), about one per
//!    `zone_length` of pipe; each pipe joins the centre its middle is
//!    fastest to reach.
//! 2. Loggers per zone: at least two each, the rest shared by pipe length
//!    (largest remainders).
//! 3. The largest zone is planned first, then the next, each with every
//!    logger already chosen - in any zone - kept fixed. A logger near a
//!    boundary that feels bursts across it counts for both zones; bursts
//!    are scored in the zone being planned only.
//! 4. Polish on the whole network: every new logger tries its nearest
//!    unused candidates (the advisor's swaps), scored on the WHOLE town -
//!    repairing what zone borders and the share-by-length got wrong (a
//!    logger can move across a border, and so from one zone to another).
//! 5. The final placement is scored on the whole network, as one.
//!
//! The price of zones: a zone planned early cannot count on loggers placed
//! later next door - the polish repairs most of it (measured: FN-020). The
//! gain: each search is local, so the cost grows with the number of zones,
//! not with its square.

use jk_hydraulics::{Network, Node, NodeId, PipeId};
use jk_kernel::Length;

use super::advise::{Advice, advise_on, swap_rounds};
use super::bench::Bench;
use super::coverage::{Coverage, PlacementOptions, validate};
use crate::error::{LocalisationError, positive};
use crate::graph::TravelGraph;
use crate::locate::nominal_times;

/// One zone's plan.
#[derive(Debug, Clone, PartialEq)]
pub struct ZoneAdvice {
    /// The zone's pipes.
    pub pipes: Vec<PipeId>,
    /// Their total length.
    pub length: Length,
    /// Loggers placed in this zone (before the whole-network polish, which
    /// may move some; the final set is [`ZonedAdvice::best`]).
    pub loggers: Vec<NodeId>,
    /// The advisor's steps and result for this zone, scored on the zone's
    /// bursts only, with earlier zones' loggers fixed.
    pub advice: Advice,
}

/// A plan for a large network, zone by zone.
#[derive(Debug, Clone, PartialEq)]
pub struct ZonedAdvice {
    /// The zones, in the order they were planned (largest first).
    pub zones: Vec<ZoneAdvice>,
    /// Every logger (fixed ones first), scored on the whole network.
    pub best: Coverage,
}

/// The network's pipes split into about one zone per `zone_length` of pipe
/// (at most `max_zones`): pipe indices per zone, largest zone first.
///
/// # Errors
/// A non-positive `zone_length`, or a network without pipes.
pub fn zones(
    network: &Network,
    zone_length: Length,
    max_zones: usize,
) -> Result<Vec<Vec<usize>>, LocalisationError> {
    positive("zone_length", zone_length.value())?;
    let pipes = network.pipes();
    if pipes.is_empty() {
        return Err(LocalisationError::InvalidInput(
            "the network has no pipes".into(),
        ));
    }
    let total: f64 = pipes.iter().map(|p| p.pipe.length().value()).sum();
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // positive, small
    let wanted = ((total / zone_length.value()).ceil() as usize).clamp(1, max_zones.max(1));
    let graph = TravelGraph::new(network);
    let nominal = nominal_times(network);
    let junctions: Vec<usize> = network
        .nodes()
        .iter()
        .enumerate()
        .filter(|(_, n)| matches!(n, Node::Junction { .. }))
        .map(|(i, _)| i)
        .collect();
    let Some(&first_junction) = junctions.first() else {
        return Ok(vec![(0..pipes.len()).collect()]);
    };

    // Centres, spread out: start at the junction furthest from the first.
    let furthest = |gap: &[f64]| {
        junctions
            .iter()
            .copied()
            .filter(|&j| gap[j].is_finite())
            .max_by(|&a, &b| gap[a].total_cmp(&gap[b]).then(b.cmp(&a)))
    };
    let start = furthest(&graph.times_from(first_junction, &nominal)).unwrap_or(first_junction);
    let mut times = vec![graph.times_from(start, &nominal)];
    let mut gap = times[0].clone();
    while times.len() < wanted {
        let Some(next) = furthest(&gap) else { break };
        if gap[next] <= 0.0 {
            break;
        }
        let t = graph.times_from(next, &nominal);
        for (g, x) in gap.iter_mut().zip(&t) {
            *g = g.min(*x);
        }
        times.push(t);
    }

    // Each pipe to the centre its middle is fastest to reach.
    let mut out = vec![Vec::new(); times.len()];
    for (p, link) in pipes.iter().enumerate() {
        let (a, b) = (link.from.index(), link.to.index());
        let zone = (0..times.len())
            .min_by(|&x, &y| {
                let reach = |z: usize| times[z][a].min(times[z][b]) + 0.5 * nominal[p];
                reach(x).total_cmp(&reach(y)).then(x.cmp(&y))
            })
            .unwrap_or(0);
        out[zone].push(p);
    }
    out.retain(|z| !z.is_empty());
    let length =
        |z: &Vec<usize>| -> f64 { z.iter().map(|&p| pipes[p].pipe.length().value()).sum() };
    out.sort_by(|x, y| length(y).total_cmp(&length(x)).then(x[0].cmp(&y[0])));
    Ok(out)
}

/// Chooses `count` new loggers among `candidates` zone by zone (about one
/// zone per `zone_length` of pipe), keeping the `fixed` ones.
///
/// # Errors
/// As [`super::advise`], or a non-positive `zone_length`.
///
/// # Panics
/// Only if an internal invariant is broken (a pipe index out of range) -
/// that would be a bug.
#[allow(clippy::too_many_arguments)] // advise's inputs plus the zone size
pub fn advise_zoned(
    network: &Network,
    candidates: &[NodeId],
    count: usize,
    fixed: &[NodeId],
    weights: Option<&[f64]>,
    options: &PlacementOptions,
    zone_length: Length,
) -> Result<ZonedAdvice, LocalisationError> {
    if count < 2 {
        return Err(LocalisationError::InvalidInput(format!(
            "cannot place {count} loggers: at least 2 are needed"
        )));
    }
    let all: Vec<NodeId> = fixed
        .iter()
        .chain(candidates)
        .copied()
        .fold(Vec::new(), |mut v, c| {
            if !v.contains(&c) {
                v.push(c);
            }
            v
        });
    validate(network, &all, weights, options)?;
    let pipes = network.pipes();
    let split = zones(network, zone_length, count / 2)?;

    // Loggers per zone: two each, the rest by length (largest remainders).
    let lengths: Vec<f64> = split
        .iter()
        .map(|z| z.iter().map(|&p| pipes[p].pipe.length().value()).sum())
        .collect();
    let total: f64 = lengths.iter().sum();
    let spare = count - 2 * split.len();
    #[allow(clippy::cast_precision_loss)] // counts are small
    let shares: Vec<f64> = lengths.iter().map(|l| spare as f64 * l / total).collect();
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // floor of positive
    let mut per_zone: Vec<usize> = shares.iter().map(|s| 2 + s.floor() as usize).collect();
    let mut left = count - per_zone.iter().sum::<usize>();
    let mut order: Vec<usize> = (0..split.len()).collect();
    order.sort_by(|&a, &b| {
        (shares[b] - shares[b].floor())
            .total_cmp(&(shares[a] - shares[a].floor()))
            .then(a.cmp(&b))
    });
    for &z in order.iter().cycle() {
        if left == 0 {
            break;
        }
        per_zone[z] += 1;
        left -= 1;
    }

    let mut chosen: Vec<NodeId> = fixed.to_vec();
    let mut planned = Vec::with_capacity(split.len());
    for (z, zone_pipes) in split.iter().enumerate() {
        let mut only = vec![false; pipes.len()];
        let mut touching = vec![false; network.nodes().len()];
        for &p in zone_pipes {
            only[p] = true;
            touching[pipes[p].from.index()] = true;
            touching[pipes[p].to.index()] = true;
        }
        let pool: Vec<NodeId> = candidates
            .iter()
            .copied()
            .filter(|c| touching[c.index()] && !chosen.contains(c))
            .collect();
        let want = per_zone[z].min(pool.len());
        if want == 0 || chosen.len() + want < 2 {
            continue;
        }
        let advice = advise_on(
            network,
            &pool,
            chosen.len() + want,
            &chosen,
            weights,
            options,
            Some(&only),
        )?;
        let added: Vec<NodeId> = advice.best.sensors[chosen.len()..].to_vec();
        chosen.extend(&added);
        planned.push(ZoneAdvice {
            pipes: zone_pipes
                .iter()
                .map(|&p| network.pipe_id(p).expect("a pipe of the network"))
                .collect(),
            length: Length::new(lengths[z]),
            loggers: added,
            advice,
        });
    }
    // 4. Polish on the whole network.
    let mut bench = Bench::new(network, weights, options, None);
    let pool: Vec<NodeId> = all.iter().copied().filter(|c| !fixed.contains(c)).collect();
    let mut state = bench.state(&chosen);
    swap_rounds(&mut bench, &pool, &mut chosen, fixed.len(), &mut state);
    let best = bench.coverage(&chosen, state.regions());
    Ok(ZonedAdvice {
        zones: planned,
        best,
    })
}
