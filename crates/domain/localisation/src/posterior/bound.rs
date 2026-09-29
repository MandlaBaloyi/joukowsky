//! An upper bound on how probable any point of a pipe can be - so the
//! locator can skip pipes that cannot hold the source (branch and bound).
//!
//! The log-probability of a candidate (`score.rs`, plus the log of its
//! length as the prior) is
//!
//! ```text
//!   log P = -chi2/2 - (log det C)/2 - (log 1'C^-1 1)/2 + log(cell length)
//! ```
//!
//! with C = D + S: D the timing variances `sigma_j^2` (diagonal), S the
//! wave-speed part, rho^2 `sum_q` `w_jq` `w_kq` over the pipes q both waves cross
//! (`w_jq`: time the wave to sensor j spends on pipe q).
//!
//! 1. The misfit. chi2 = min over t0 of (r - t0 1)' C^-1 (r - t0 1), and for
//!    any contrast a (a'1 = 0), Cauchy-Schwarz gives chi2 >= (a'r)^2 / a'Ca.
//!    Take a = `e_i` - `e_j`, one pair of sensors: a'r = `d_ij` - `D_ij`, the
//!    observed minus the predicted difference of arrival times, and
//!
//!    ```text
//!      a'Ca = sigma_i^2 + sigma_j^2 + rho^2 sum_q (w_iq - w_jq)^2.
//!    ```
//!
//!    Pipes both waves cross in full cancel (`w_iq` = `w_jq`): only the parts
//!    of the routes that differ count. That sum is fixed for each choice
//!    of the ends through which the two waves enter the pipe, except for
//!    the pipe itself, whose share varies linearly along it.
//! 2. Along the pipe, each sensor's time is min(A + f tau, B + (1 - f)
//!    tau), so between the kinks (where a sensor's entry end switches)
//!    every difference `D_ij` is linear in f: its range on an interval comes
//!    from the interval's ends, and so does the largest pipe share.
//! 3. The other two terms: det(A C A') = det C x 1'C^-1 1 x const for a
//!    fixed contrast basis A (the restricted-likelihood identity), and
//!    C >= D, so they are at most their value with C = D:
//!    -(sum log `sigma_j^2`)/2 - (log sum 1/`sigma_j^2`)/2.
//!
//! The bound therefore never undercounts, and it is tight where it
//! matters: far from the sensors the timing misses by seconds while the
//! routes largely coincide.

use std::sync::Arc;

use crate::graph::{Routes, TravelGraph};

/// When and how a wave from one sensor enters a pipe through one end.
#[derive(Debug, Clone, Copy)]
struct Entry {
    /// Arrival time at that end (infinite if it cannot enter there).
    time: f64,
    /// Sum of `tau_q^2` over the route's pipes, this pipe excluded.
    route_squares: f64,
    /// Whether the route to that end crosses this pipe itself.
    through: bool,
}

/// One pipe's arrival-independent figures.
#[derive(Debug, Clone)]
struct PipeBound {
    tau: f64,
    log_cell_length: f64,
    /// Per sensor, entry through the `from` end and the `to` end.
    entries: Vec<[Entry; 2]>,
    /// Per pair (i > j) and entry ends (ki, kj): sum of `tau_q^2` over the
    /// pipes both routes cross, this pipe excluded. Index
    /// `(pair(i, j) * 2 + ki) * 2 + kj`.
    common: Vec<f64>,
}

/// Working space for [`Bounds::upper`], reused call after call.
#[derive(Debug, Default)]
pub(crate) struct Scratch {
    fractions: Vec<f64>,
    end_of: Vec<usize>,
}

/// Arrival-independent figures for every pipe, for one set of sensors.
#[derive(Debug, Clone)]
pub(crate) struct Bounds {
    pipes: Vec<PipeBound>,
}

/// Index of the pair (i, j), i > j.
fn pair(i: usize, j: usize) -> usize {
    i * (i - 1) / 2 + j
}

impl Bounds {
    /// Prepares the bound for `sensors` (node indices) with their `routes`
    /// ([`TravelGraph::routes_from`] each), pipe `ends`, `nominal` travel
    /// times and candidate `cell_lengths` per pipe.
    pub(crate) fn new(
        graph: &TravelGraph,
        sensors: &[usize],
        routes: &[Arc<Routes>],
        ends: &[(usize, usize)],
        nominal: &[f64],
        cell_lengths: &[f64],
    ) -> Self {
        let n = sensors.len();
        let square = |q: usize| nominal[q] * nominal[q];
        // A stamp per pipe marks the route being compared against.
        let mut stamp = vec![usize::MAX; nominal.len()];
        let mut next_stamp = 0usize;
        let pipes = (0..ends.len())
            .map(|p| {
                let (from, to) = ends[p];
                let entries: Vec<[Entry; 2]> = sensors
                    .iter()
                    .zip(routes)
                    .map(|(&s, r)| {
                        let (times, pipe_routes) = (&r.0, &r.1);
                        let (a, b) = graph.end_times(s, times, (from, to));
                        [(from, a), (to, b)].map(|(end, time)| {
                            let route = &pipe_routes[end];
                            Entry {
                                time,
                                route_squares: route
                                    .iter()
                                    .filter(|&&q| q != p)
                                    .map(|&q| square(q))
                                    .sum(),
                                through: route.contains(&p),
                            }
                        })
                    })
                    .collect();
                let mut common = vec![0.0; n * n.saturating_sub(1) / 2 * 4];
                for i in 1..n {
                    for ki in 0..2 {
                        let end_i = if ki == 0 { from } else { to };
                        next_stamp += 1;
                        for &q in &routes[i].1[end_i] {
                            stamp[q] = next_stamp;
                        }
                        for j in 0..i {
                            for kj in 0..2 {
                                let end_j = if kj == 0 { from } else { to };
                                common[(pair(i, j) * 2 + ki) * 2 + kj] = routes[j].1[end_j]
                                    .iter()
                                    .filter(|&&q| q != p && stamp[q] == next_stamp)
                                    .map(|&q| square(q))
                                    .sum();
                            }
                        }
                    }
                }
                PipeBound {
                    tau: nominal[p],
                    log_cell_length: cell_lengths[p].ln(),
                    entries,
                    common,
                }
            })
            .collect();
        Self { pipes }
    }

    /// Sum of `tau_q^2` over the pipes (other than `pipe`) that the route to
    /// sensor `j` entering `pipe` through end `ej` (0 = from, 1 = to) and
    /// the route to sensor `k` entering through `ek` both cross.
    pub(crate) fn shared(&self, pipe: usize, j: usize, ej: usize, k: usize, ek: usize) -> f64 {
        let pb = &self.pipes[pipe];
        match j.cmp(&k) {
            std::cmp::Ordering::Equal => pb.entries[j][ej].route_squares,
            std::cmp::Ordering::Greater => pb.common[(pair(j, k) * 2 + ej) * 2 + ek],
            std::cmp::Ordering::Less => pb.common[(pair(k, j) * 2 + ek) * 2 + ej],
        }
    }

    /// Whether the route to sensor `j` entering `pipe` through end `e`
    /// crosses `pipe` itself on the way.
    pub(crate) fn through(&self, pipe: usize, j: usize, e: usize) -> bool {
        self.pipes[pipe].entries[j][e].through
    }

    /// An upper bound on the log-probability of every candidate on `pipe`
    /// between fractions `from` and `to` of its length, for `observed`
    /// arrival times with one-sigma `sigma`, wave-speed doubt `spread2`
    /// (rho^2) and `constant` = -(sum log sigma^2)/2 - (log sum
    /// 1/sigma^2)/2. Minus infinity if some sensor's wave cannot reach it.
    #[allow(clippy::too_many_arguments)] // the bound's inputs, all needed
    pub(crate) fn upper(
        &self,
        pipe: usize,
        (from, to): (f64, f64),
        observed: &[f64],
        sigma: &[f64],
        spread2: f64,
        constant: f64,
        scratch: &mut Scratch,
    ) -> f64 {
        let pb = &self.pipes[pipe];
        let tau = pb.tau;
        let n = observed.len();
        let fractions = &mut scratch.fractions;
        fractions.clear();
        fractions.extend([from, to]);
        for e in &pb.entries {
            let (a, b) = (e[0].time, e[1].time);
            if !a.is_finite() && !b.is_finite() {
                return f64::NEG_INFINITY;
            }
            let kink = (b - a + tau) / (2.0 * tau);
            if kink > from && kink < to {
                fractions.push(kink);
            }
        }
        fractions.sort_by(f64::total_cmp);
        // A single point (one cell) is an interval of zero width.
        let point = to <= from;
        let mut best = f64::NEG_INFINITY;
        let end_of = &mut scratch.end_of;
        end_of.clear();
        end_of.resize(n, 0);
        for w in fractions.windows(2) {
            let (f0, f1) = (w[0], w[1]);
            if f1 <= f0 && !point {
                continue;
            }
            let fm = f64::midpoint(f0, f1);
            for (j, e) in pb.entries.iter().enumerate() {
                end_of[j] = usize::from(e[0].time + fm * tau > e[1].time + (1.0 - fm) * tau);
            }
            // Time to sensor j at f, and the share of this pipe in its route.
            let time = |j: usize, f: f64| {
                let e = pb.entries[j][end_of[j]];
                let part = if end_of[j] == 0 { f } else { 1.0 - f };
                e.time + part * tau
            };
            let share = |j: usize, f: f64| {
                let e = pb.entries[j][end_of[j]];
                let part = if end_of[j] == 0 { f } else { 1.0 - f };
                (if e.through { tau } else { 0.0 }) + part * tau
            };
            let mut chi_squared: f64 = 0.0;
            for i in 1..n {
                for j in 0..i {
                    let (d0, d1) = (time(i, f0) - time(j, f0), time(i, f1) - time(j, f1));
                    let (lo, hi) = (d0.min(d1), d0.max(d1));
                    let d = observed[i] - observed[j];
                    let miss = if d < lo {
                        lo - d
                    } else if d > hi {
                        d - hi
                    } else {
                        0.0
                    };
                    if miss <= 0.0 || !miss.is_finite() {
                        continue;
                    }
                    let own = (share(i, f0) - share(j, f0))
                        .powi(2)
                        .max((share(i, f1) - share(j, f1)).powi(2));
                    let (ei, ej) = (pb.entries[i][end_of[i]], pb.entries[j][end_of[j]]);
                    let apart = ei.route_squares + ej.route_squares
                        - 2.0 * pb.common[(pair(i, j) * 2 + end_of[i]) * 2 + end_of[j]];
                    let variance = sigma[i] * sigma[i]
                        + sigma[j] * sigma[j]
                        + spread2 * (apart.max(0.0) + own);
                    chi_squared = chi_squared.max(miss * miss / variance);
                }
            }
            let reachable = (0..n).all(|j| time(j, fm).is_finite());
            if reachable {
                best = best.max(-0.5 * chi_squared + constant + pb.log_cell_length);
            }
        }
        best
    }
}

#[cfg(test)]
mod tests {
    use jk_kernel::linalg::{cholesky, log_det_factored, solve_factored};
    use jk_kernel::random::Rng;

    /// Step 3 of the module docs: with C = D + S (S positive semi-definite),
    /// -(log det C)/2 - (log 1'C^-1 1)/2 never exceeds its value at C = D.
    #[test]
    fn the_determinant_terms_are_largest_without_wave_speed_doubt() {
        let mut rng = Rng::new(17);
        let terms = |c: &mut [f64], n: usize| {
            cholesky(c, n).unwrap();
            let mut x = vec![1.0; n];
            solve_factored(c, &mut x, n);
            -0.5 * log_det_factored(c, n) - 0.5 * x.iter().sum::<f64>().ln()
        };
        for _ in 0..200 {
            let n = 2 + (rng.next_u64() % 5) as usize;
            let sigma: Vec<f64> = (0..n).map(|_| 1e-4 + 2e-3 * rng.uniform()).collect();
            let mut d = vec![0.0; n * n];
            for j in 0..n {
                d[j * n + j] = sigma[j] * sigma[j];
            }
            // S = G G' with G random: positive semi-definite.
            let g: Vec<f64> = (0..n * 3).map(|_| 0.05 * rng.normal()).collect();
            let mut c = d.clone();
            for i in 0..n {
                for j in 0..n {
                    c[i * n + j] += (0..3).map(|k| g[i * 3 + k] * g[j * 3 + k]).sum::<f64>();
                }
            }
            assert!(terms(&mut c, n) <= terms(&mut d, n) + 1e-9);
        }
    }
}
