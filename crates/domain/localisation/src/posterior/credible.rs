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

/// Total probability per pipe, most likely first. `mass` lists (cell index,
/// probability) for the cells that hold any - the others hold none.
#[must_use]
pub fn per_pipe(
    cells: &[Cell],
    mass: &[(usize, f64)],
    pipe_ids: &[PipeId],
) -> Vec<PipeProbability> {
    let mut totals = vec![0.0; pipe_ids.len()];
    for &(i, m) in mass {
        totals[cells[i].pipe] += m;
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
/// merged into contiguous segments per pipe, most probable first. `mass`
/// lists (cell index, probability) for the cells that hold any, in cell
/// order.
#[must_use]
pub fn region(
    cells: &[Cell],
    mass: &[(usize, f64)],
    pipe_ids: &[PipeId],
    pipe_lengths: &[f64],
    level: f64,
) -> Vec<Segment> {
    let mut order: Vec<usize> = (0..mass.len()).collect();
    order.sort_by(|&a, &b| mass[b].1.total_cmp(&mass[a].1).then(a.cmp(&b)));
    let mut chosen = vec![false; mass.len()];
    let mut held = 0.0;
    for &k in &order {
        if held >= level {
            break;
        }
        chosen[k] = true;
        held += mass[k].1;
    }
    // Merge runs of chosen neighbouring cells on the same pipe.
    let mut segments: Vec<Segment> = Vec::new();
    let mut previous: Option<usize> = None;
    for (k, &(i, m)) in mass.iter().enumerate() {
        if !chosen[k] {
            continue;
        }
        let c = cells[i];
        let half = 0.5 * c.length;
        let centre = c.fraction * pipe_lengths[c.pipe];
        let (from, to) = (centre - half, centre + half);
        let continues = i > 0 && previous == Some(i - 1) && cells[i - 1].pipe == c.pipe;
        previous = Some(i);
        match segments.last_mut() {
            Some(last) if continues => {
                last.to = Length::new(to);
                last.probability += m;
            }
            _ => segments.push(Segment {
                pipe: pipe_ids[c.pipe],
                from: Length::new(from.max(0.0)),
                to: Length::new(to),
                probability: m,
            }),
        }
    }
    segments.sort_by(|a, b| b.probability.total_cmp(&a.probability));
    segments
}
