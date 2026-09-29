//! A synthetic town of any length.
//!
//! The scale study (`10_findings/detection_range_and_scale.txt`, V-S4)
//! needs networks of 10, 50, 100 km and more, shaped like real ones. Real
//! distribution networks follow the streets: a grid of blocks, a few
//! larger mains, loops where streets close a block, dead ends where they
//! do not (cul-de-sacs), and a mix of pipe materials laid over decades.
//! The town is built in that order:
//!
//! 1. Streets on a jittered grid of blocks (about `block` metres apart).
//! 2. Mains along every fourth street each way: always laid, ductile iron.
//! 3. Other streets: enough to reach every block (a random spanning tree),
//!    then each remaining street with probability `loops` - so the town
//!    is partly looped, partly branched.
//! 4. Cul-de-sacs: short dead-end pipes off 15% of the corners.
//! 5. Hydrants: half the streets get a junction part-way along.
//! 6. Reservoirs: one per 50 km (at least one), each feeding the mains
//!    through a trunk main from the west.
//! 7. Everything is scaled so the total pipe length is exactly `length`.
//!
//! Materials (per street; wave speeds varied +/-5-10% pipe by pipe, as
//! real ones are - `02_physics_of_transients/wave_speed.txt`):
//! mains ductile iron 250 mm, a ~1100 m/s; distribution 55% uPVC 110 mm
//! (a ~400), 30% asbestos cement 150 mm (a ~1050), 15% cast iron 150 mm
//! (a ~1200). Demand is spread evenly over every junction.
//!
//! The same seed always gives the same town (P5).

use jk_hydraulics::{HydraulicsError, Network, Node, NodeId, Pipe};
use jk_kernel::random::Rng;
use jk_kernel::{Flow, Head, Length, STANDARD_GRAVITY, WaveSpeed};

/// Every how many streets a main runs, each way.
const MAIN_EVERY: usize = 4;
/// Share of grid corners with a cul-de-sac.
const SPUR_SHARE: f64 = 0.15;
/// Share of streets with a hydrant junction part-way along.
const HYDRANT_SHARE: f64 = 0.5;
/// Town length per reservoir [m].
const LENGTH_PER_RESERVOIR: f64 = 50_000.0;
/// Head at every reservoir [m].
const RESERVOIR_HEAD: f64 = 75.0;
/// Darcy friction factor of every pipe.
const FRICTION: f64 = 0.02;

/// How the town is made.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TownOptions {
    /// Total pipe length. The town is scaled to match it exactly.
    pub length: Length,
    /// The seed: the same seed always gives the same town.
    pub seed: u64,
    /// Typical street block, before scaling. Default 120 m.
    pub block: Length,
    /// Share of the streets beyond those needed to reach every block that
    /// are laid too (0: fully branched, 1: every block closed). Default
    /// 0.35.
    pub loops: f64,
    /// Water drawn per km of pipe, spread over every junction. Default
    /// 1 l/s per km.
    pub demand_per_km: Flow,
}

impl Default for TownOptions {
    fn default() -> Self {
        Self {
            length: Length::new(10_000.0),
            seed: 1,
            block: Length::new(120.0),
            loops: 0.35,
            demand_per_km: Flow::new(1e-3),
        }
    }
}

/// What a pipe is made of.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Material {
    /// Mains and trunk mains.
    DuctileIron,
    /// Older distribution.
    AsbestosCement,
    /// Oldest distribution.
    CastIron,
    /// Newer distribution (plastic).
    Upvc,
}

impl Material {
    /// Short name.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::DuctileIron => "ductile iron",
            Self::AsbestosCement => "asbestos cement",
            Self::CastIron => "cast iron",
            Self::Upvc => "uPVC",
        }
    }

    /// Internal diameter [m] and book wave speed [m/s] of a distribution
    /// pipe (mains are sized separately).
    fn distribution(self) -> (f64, f64) {
        match self {
            Self::DuctileIron => (0.25, 1100.0),
            Self::AsbestosCement => (0.15, 1050.0),
            Self::CastIron => (0.15, 1200.0),
            Self::Upvc => (0.11, 400.0),
        }
    }
}

/// A generated town: the network and how to draw and describe it.
#[derive(Debug, Clone)]
pub struct Town {
    /// The engine's map.
    pub network: Network,
    /// Plan position of every node [m], x east, y north.
    pub positions: Vec<(f64, f64)>,
    /// Name of every node.
    pub node_names: Vec<String>,
    /// Name of every pipe.
    pub pipe_names: Vec<String>,
    /// Material of every pipe.
    pub materials: Vec<Material>,
    /// Whether each pipe is a main (or trunk main).
    pub mains: Vec<bool>,
    /// Every junction: where a logger could be fitted (hydrants, tap
    /// points).
    pub hydrants: Vec<NodeId>,
    /// The reservoirs.
    pub reservoirs: Vec<NodeId>,
}

impl Town {
    /// Total pipe length.
    #[must_use]
    pub fn length(&self) -> Length {
        Length::new(
            self.network
                .pipes()
                .iter()
                .map(|p| p.pipe.length().value())
                .sum(),
        )
    }
}

/// A street before it becomes pipes.
#[derive(Debug, Clone, Copy)]
struct Street {
    from: usize,
    to: usize,
    material: Material,
    main: bool,
}

/// Generates a town.
///
/// # Errors
/// Options that are not finite and positive (`loops` must be in [0, 1],
/// demand may be zero).
///
/// # Panics
/// Only if an internal invariant is broken (a node index out of range) -
/// that would be a bug.
#[allow(clippy::too_many_lines)] // the seven steps of the module docs, in order
pub fn town(options: &TownOptions) -> Result<Town, HydraulicsError> {
    validate(options)?;
    let mut rng = Rng::new(options.seed);
    let b = options.block.value();
    let target = options.length.value();

    // 1. The grid, sized so the streets come to about the target length.
    let k = (target / ((1.0 + options.loops) * b * 1.1)).sqrt();
    let nx = round_count(k * 1.25).max(3);
    let ny = round_count(k / 1.25).max(3);
    let xs = spaced(&mut rng, nx, b);
    let ys = spaced(&mut rng, ny, b);
    let mut positions: Vec<(f64, f64)> = Vec::with_capacity(nx * ny);
    for &y in &ys {
        for &x in &xs {
            positions.push((
                x + b * 0.16 * (rng.uniform() - 0.5),
                y + b * 0.16 * (rng.uniform() - 0.5),
            ));
        }
    }
    let at = |i: usize, j: usize| j * nx + i;

    // 2-3. Mains always; other streets: a spanning tree, then loops.
    let mut mains = Vec::new();
    let mut others = Vec::new();
    for j in 0..ny {
        for i in 0..nx {
            if i + 1 < nx {
                let s = (at(i, j), at(i + 1, j));
                if j % MAIN_EVERY == 0 {
                    mains.push(s);
                } else {
                    others.push(s);
                }
            }
            if j + 1 < ny {
                let s = (at(i, j), at(i, j + 1));
                if i % MAIN_EVERY == 0 {
                    mains.push(s);
                } else {
                    others.push(s);
                }
            }
        }
    }
    let mut sets = DisjointSets::new(nx * ny);
    let mut streets: Vec<Street> = Vec::new();
    for (from, to) in mains {
        sets.join(from, to);
        streets.push(Street {
            from,
            to,
            material: Material::DuctileIron,
            main: true,
        });
    }
    shuffle(&mut rng, &mut others);
    for (from, to) in others {
        let tree = sets.join(from, to);
        let loop_kept = rng.uniform() < options.loops;
        if tree || loop_kept {
            streets.push(Street {
                from,
                to,
                material: distribution_material(&mut rng),
                main: false,
            });
        }
    }

    // 4. Cul-de-sacs.
    for corner in 0..nx * ny {
        if rng.uniform() >= SPUR_SHARE {
            continue;
        }
        let angle = std::f64::consts::TAU * (rng.uniform() * 4.0).floor() / 4.0
            + std::f64::consts::FRAC_PI_4;
        let reach = b * (0.3 + 0.4 * rng.uniform());
        let (x, y) = positions[corner];
        positions.push((x + reach * angle.cos(), y + reach * angle.sin()));
        streets.push(Street {
            from: corner,
            to: positions.len() - 1,
            material: distribution_material(&mut rng),
            main: false,
        });
    }

    // 5. Hydrants part-way along half the streets.
    let mut pieces: Vec<Street> = Vec::with_capacity(streets.len() * 3 / 2);
    for s in streets {
        if rng.uniform() < HYDRANT_SHARE {
            let f = 0.35 + 0.3 * rng.uniform();
            let (a, c) = (positions[s.from], positions[s.to]);
            positions.push((a.0 + f * (c.0 - a.0), a.1 + f * (c.1 - a.1)));
            let h = positions.len() - 1;
            pieces.push(Street { to: h, ..s });
            pieces.push(Street { from: h, ..s });
        } else {
            pieces.push(s);
        }
    }

    // 6. Reservoirs west of the town, each on a trunk main to a main row.
    let reservoirs = round_count((target / LENGTH_PER_RESERVOIR).ceil()).max(1);
    let main_rows: Vec<usize> = (0..ny).step_by(MAIN_EVERY).collect();
    let first_reservoir = positions.len();
    for r in 0..reservoirs {
        let row = main_rows[(r * main_rows.len()) / reservoirs];
        let (x, y) = positions[at(0, row)];
        positions.push((x - 3.0 * b, y + b * 0.5 * to_f64(r % 2)));
        pieces.push(Street {
            from: positions.len() - 1,
            to: at(0, row),
            material: Material::DuctileIron,
            main: true,
        });
    }

    // 7. Scale to the target length.
    let raw: f64 = pieces
        .iter()
        .map(|s| distance(positions[s.from], positions[s.to]))
        .sum();
    let scale = target / raw;
    for p in &mut positions {
        *p = (p.0 * scale, p.1 * scale);
    }

    build(options, &mut rng, &positions, &pieces, first_reservoir)
}

/// Turns the laid-out streets into the engine's network.
fn build(
    options: &TownOptions,
    rng: &mut Rng,
    positions: &[(f64, f64)],
    pieces: &[Street],
    first_reservoir: usize,
) -> Result<Town, HydraulicsError> {
    let reservoir_count = positions.len() - first_reservoir;
    let junctions = first_reservoir;
    let total_demand = options.demand_per_km.value() * options.length.value() / 1000.0;
    let demand = Flow::new(total_demand / to_f64(junctions));
    // Mains and trunks sized for about 1 m/s at the reservoir's share.
    let per_reservoir = total_demand / to_f64(reservoir_count);
    let trunk = ((4.0 * per_reservoir / std::f64::consts::PI).sqrt() / 0.05).ceil() * 0.05;
    let main_diameter = trunk.max(0.25);

    let mut network = Network::new(STANDARD_GRAVITY);
    let mut hydrants = Vec::with_capacity(junctions);
    let mut reservoirs = Vec::with_capacity(reservoir_count);
    let mut node_names = Vec::with_capacity(positions.len());
    for n in 0..positions.len() {
        if n < first_reservoir {
            hydrants.push(network.add_node(Node::Junction { demand }));
            node_names.push(format!("J{n}"));
        } else {
            reservoirs.push(network.add_node(Node::Reservoir {
                head: Head::new(RESERVOIR_HEAD),
            }));
            node_names.push(format!("R{}", n - first_reservoir + 1));
        }
    }
    // Junctions were added first, then reservoirs: ids follow positions.
    let ids: Vec<NodeId> = hydrants.iter().chain(&reservoirs).copied().collect();
    let mut pipe_names = Vec::with_capacity(pieces.len());
    let mut materials = Vec::with_capacity(pieces.len());
    let mut mains = Vec::with_capacity(pieces.len());
    for (k, s) in pieces.iter().enumerate() {
        let (diameter, book) = if s.main {
            (main_diameter, 1100.0)
        } else {
            s.material.distribution()
        };
        let spread = if s.material == Material::Upvc {
            0.1
        } else {
            0.05
        };
        let a = book * (1.0 + spread * (2.0 * rng.uniform() - 1.0));
        let length = distance(positions[s.from], positions[s.to]);
        let pipe = Pipe::new(
            Length::new(length),
            Length::new(diameter),
            WaveSpeed::new(a),
            FRICTION,
        )?;
        network.add_pipe(ids[s.from], ids[s.to], pipe)?;
        pipe_names.push(format!("P{k}"));
        materials.push(s.material);
        mains.push(s.main);
    }
    Ok(Town {
        network,
        positions: positions.to_vec(),
        node_names,
        pipe_names,
        materials,
        mains,
        hydrants,
        reservoirs,
    })
}

fn validate(options: &TownOptions) -> Result<(), HydraulicsError> {
    let invalid = |name, value, requirement| HydraulicsError::InvalidParameter {
        name,
        value,
        requirement,
    };
    for (name, v) in [
        ("length", options.length.value()),
        ("block", options.block.value()),
    ] {
        if !(v.is_finite() && v > 0.0) {
            return Err(invalid(name, v, "finite and > 0"));
        }
    }
    if !(0.0..=1.0).contains(&options.loops) {
        return Err(invalid("loops", options.loops, "in [0, 1]"));
    }
    let q = options.demand_per_km.value();
    if !(q.is_finite() && q >= 0.0) {
        return Err(invalid("demand_per_km", q, "finite and >= 0"));
    }
    Ok(())
}

fn distribution_material(rng: &mut Rng) -> Material {
    let u = rng.uniform();
    if u < 0.55 {
        Material::Upvc
    } else if u < 0.85 {
        Material::AsbestosCement
    } else {
        Material::CastIron
    }
}

/// `n` coordinates, each gap `b` +/-30%.
fn spaced(rng: &mut Rng, n: usize, b: f64) -> Vec<f64> {
    let mut out = Vec::with_capacity(n);
    let mut v = 0.0;
    for _ in 0..n {
        out.push(v);
        v += b * (0.7 + 0.6 * rng.uniform());
    }
    out
}

/// Fisher-Yates shuffle with our own generator (deterministic).
fn shuffle<T>(rng: &mut Rng, items: &mut [T]) {
    for i in (1..items.len()).rev() {
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // in 0..=i
        let j = ((rng.uniform() * to_f64(i + 1)) as usize).min(i);
        items.swap(i, j);
    }
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // small, positive
fn round_count(x: f64) -> usize {
    x.round().max(0.0) as usize
}

#[allow(clippy::cast_precision_loss)] // small counts
fn to_f64(i: usize) -> f64 {
    i as f64
}

/// Union-find: which grid corners the streets laid so far connect.
struct DisjointSets {
    parent: Vec<usize>,
}

impl DisjointSets {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
        }
    }

    fn root(&mut self, mut i: usize) -> usize {
        while self.parent[i] != i {
            self.parent[i] = self.parent[self.parent[i]];
            i = self.parent[i];
        }
        i
    }

    /// Joins the sets of `a` and `b`; false if they were already one.
    fn join(&mut self, a: usize, b: usize) -> bool {
        let (ra, rb) = (self.root(a), self.root(b));
        if ra == rb {
            return false;
        }
        self.parent[ra] = rb;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn small(seed: u64) -> Town {
        town(&TownOptions {
            length: Length::new(5_000.0),
            seed,
            ..TownOptions::default()
        })
        .unwrap()
    }

    #[test]
    fn the_length_is_exactly_the_target() {
        let t = small(7);
        assert!((t.length().value() - 5_000.0).abs() < 1e-6);
    }

    #[test]
    fn the_same_seed_gives_the_same_town() {
        let (a, b) = (small(3), small(3));
        assert_eq!(a.positions, b.positions);
        assert_eq!(a.network.pipes(), b.network.pipes());
        assert_ne!(small(4).positions, a.positions);
    }

    #[test]
    fn every_junction_is_reachable_and_the_town_has_loops_and_dead_ends() {
        let t = small(11);
        let n = t.network.nodes().len();
        let mut sets = DisjointSets::new(n);
        let mut degree = vec![0usize; n];
        for p in t.network.pipes() {
            sets.join(p.from.index(), p.to.index());
            degree[p.from.index()] += 1;
            degree[p.to.index()] += 1;
        }
        let root = sets.root(0);
        assert!((0..n).all(|i| sets.root(i) == root), "one connected town");
        assert!(
            t.network.pipes().len() >= n,
            "more pipes than a tree: loops"
        );
        assert!(degree.contains(&1), "dead ends exist");
        assert_eq!(t.reservoirs.len(), 1);
        assert!(t.mains.iter().any(|&m| m));
    }

    #[test]
    fn big_towns_get_more_reservoirs() {
        let t = town(&TownOptions {
            length: Length::new(120_000.0),
            ..TownOptions::default()
        })
        .unwrap();
        assert_eq!(t.reservoirs.len(), 3);
    }
}
