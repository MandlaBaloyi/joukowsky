//! From cell probabilities to what an operator needs: which pipes, and the
//! smallest stretch of pipe that holds most of the probability.

use jk_hydraulics::PipeId;
use jk_kernel::Length;

use crate::posterior::grid::Cell;

/// Probability that the source is somewhere on one pipe.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipeProbability {
    /// The pipe.
    pub pipe: PipeId,
    /// Probability (0..1).
    pub probability: f64,
}

/// A stretch of one pipe inside the credible region.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Segment {
    /// The pipe.
    pub pipe: PipeId,
    /// Start, measured from the pipe's `from` end.
    pub from: Length,
    /// End, measured from the pipe's `from` end.
    pub to: Length,
    /// Probability held by this stretch.
    pub probability: f64,
}

/// Total probability per pipe, most likely first.
#[must_use]
pub fn per_pipe(cells: &[Cell], mass: &[f64], pipe_ids: &[PipeId]) -> Vec<PipeProbability> {
    let mut totals = vec![0.0; pipe_ids.len()];
    for (c, m) in cells.iter().zip(mass) {
        totals[c.pipe] += m;
    }
    let mut out: Vec<PipeProbability> = totals
        .into_iter()
        .enumerate()
        .filter(|(_, p)| *p > 0.0)
        .map(|(i, probability)| PipeProbability {
            pipe: pipe_ids[i],
            probability,
        })
        .collect();
    out.sort_by(|a, b| b.probability.total_cmp(&a.probability));
    out
}

/// The smallest set of cells holding at least `level` of the probability,
/// merged into contiguous segments per pipe, most probable first.
#[must_use]
pub fn region(
    cells: &[Cell],
    mass: &[f64],
    pipe_ids: &[PipeId],
    pipe_lengths: &[f64],
    level: f64,
) -> Vec<Segment> {
    let mut order: Vec<usize> = (0..cells.len()).collect();
    order.sort_by(|&a, &b| mass[b].total_cmp(&mass[a]).then(a.cmp(&b)));
    let mut chosen = vec![false; cells.len()];
    let mut held = 0.0;
    for &i in &order {
        if held >= level {
            break;
        }
        chosen[i] = true;
        held += mass[i];
    }
    // Merge runs of chosen neighbouring cells on the same pipe.
    let mut segments: Vec<Segment> = Vec::new();
    for (i, c) in cells.iter().enumerate() {
        if !chosen[i] {
            continue;
        }
        let half = 0.5 * c.length;
        let centre = c.fraction * pipe_lengths[c.pipe];
        let (from, to) = (centre - half, centre + half);
        let continues = i > 0 && chosen[i - 1] && cells[i - 1].pipe == c.pipe;
        match segments.last_mut() {
            Some(last) if continues => {
                last.to = Length::new(to);
                last.probability += mass[i];
            }
            _ => segments.push(Segment {
                pipe: pipe_ids[c.pipe],
                from: Length::new(from.max(0.0)),
                to: Length::new(to),
                probability: mass[i],
            }),
        }
    }
    segments.sort_by(|a, b| b.probability.total_cmp(&a.probability));
    segments
}
