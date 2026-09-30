//! Would a burst be FELT at the loggers? (Study V-S2, first build.)
//!
//! `docs/02_physics_of_transients/friction_and_damping.txt` section 4 and
//! `joukowsky_equation.txt`. A burst suddenly letting out a flow Q in the
//! middle of a pipe drops the head there by
//!
//! ```text
//!   h0 = (B / 2) Q,        B = a / (g A)   (the pipe's impedance)
//! ```
//!
//! (the two halves of the pipe feed it in parallel), and sends that drop
//! both ways. On its way to a logger the FIRST front - the one localisation
//! times - is multiplied at every node it meets by the junction's
//! transmission coefficient
//!
//! ```text
//!   T = 2 (1 / B_in) / sum over all pipes at the node (1 / B_i)
//! ```
//!
//! (2/3 at a tee of equal pipes, 1 on a straight run, 2 at a dead end - the
//! head doubles - and 0 at a reservoir, which holds its head). The logger
//! at a node feels the head there. It FEELS the burst when that drop is at
//! least `detect_units` times its noise: 6, measured as always caught and
//! timed to 2 samples (tests/detection/analytical/sensitivity.rs; 5 units
//! is caught 96% of the time).
//!
//! Every factor is linear in Q, so one computation per point gives the
//! SMALLEST burst each logger can feel there - and the smallest burst that
//! TWO loggers feel, which is the smallest that can be located.
//!
//! Plastic pipes (a creeping wall, `pipe/viscoelastic.rs`) change the
//! picture in two ways, and they must not be confused (FN-023):
//! - the SHARP front - what the onset picker times - shrinks as
//!   exp(-kappa x) (the wall's `front_damping`; V-A6): with the published
//!   HDPE wall it halves every ~180 m. [`first_fronts`] reports this.
//! - behind it the drop keeps building as the wall creeps, so within a
//!   detection window W after the front the logger feels far more: the
//!   components of frequency ~1/W, which plastic softens (relax) but hardly
//!   damps (the wall's `window_response`). "Felt" - in [`detectability`]
//!   and the placement advisor - uses this, W = `PlacementOptions::window`.
//!
//! Both checked against the full simulator on a plastic estate (tests/
//! localisation/simulation/`plastic_fronts.rs`). Rigid pipes: no change.
//!
//! What this leaves out, all of which make real fronts SMALLER - so these
//! are upper bounds:
//! - bursts that open over a time comparable to the reflections (the front
//!   is then partly cancelled by reflections from reservoirs);
//! - friction: quasi-steady friction shrinks a front by
//!   exp(-f V x / (2 D a)) - for f = 0.02, V = 0.5 m/s, D = 0.15 m,
//!   a = 1000 m/s that is 3% per km, so it is ignored;
//! - unsteady friction (milestone S5);
//! - a valve node (at the end of a pipe) is treated as a dead end.
//!
//! Devices between two pipes pass on T = 2 Bout / (Bin + Bout + r) of a
//! wave, r their small-wave resistance in the steady state - a PRV frozen
//! at its opening, a running pump's droop - and nothing when shut
//! (`graph/devices.rs`, FN-029).

use jk_hydraulics::{Network, Node, NodeId};
use jk_kernel::{Flow, Head, Length, Time};

use super::coverage::PlacementOptions;
use crate::error::LocalisationError;
use crate::graph::{Devices, Routes, TravelGraph};
use crate::locate::Point;

/// What a logger is taken to feel of a burst's wave.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(super) enum Response {
    /// The sharp first front (what is timed).
    Front,
    /// The drop within a window this long after the front (what is felt).
    Window(Time),
}

/// First-front heads at the sensors, per unit of burst flow.
#[derive(Debug, Clone)]
pub(super) struct Fronts {
    sensors: Vec<usize>,
    graph: TravelGraph,
    ends: Vec<(usize, usize)>,
    nominal: Vec<f64>,
    impedance: Vec<f64>,
    /// Per node: the impedances of the pipes meeting there.
    at_node: Vec<Vec<f64>>,
    reservoir: Vec<bool>,
    /// What pumps and valves do to a passing wave.
    devices: Devices,
    /// Per pipe: how much of the wave it keeps per metre travelled - the
    /// plastic wall's damping [1/m] for the chosen [`Response`] (0 for rigid
    /// pipes) - and its length [m].
    damping: Vec<f64>,
    lengths: Vec<f64>,
    routes: Vec<Routes>,
}

impl Fronts {
    pub(super) fn new(network: &Network, sensors: &[NodeId], response: Response) -> Self {
        Self::with_devices(network, sensors, response, Devices::of(network))
    }

    /// As [`Self::new`], with the pumps and valves in the states `devices`
    /// gives.
    pub(super) fn with_devices(
        network: &Network,
        sensors: &[NodeId],
        response: Response,
        devices: Devices,
    ) -> Self {
        let pipes = network.pipes();
        let g = network.gravity();
        let ends: Vec<(usize, usize)> = pipes
            .iter()
            .map(|p| (p.from.index(), p.to.index()))
            .collect();
        let nominal: Vec<f64> = pipes.iter().map(|p| p.pipe.travel_time().value()).collect();
        // Per pipe: damping per metre and the impedance factor (the wall's
        // give softens a plastic pipe for slow changes).
        let wall_effect: Vec<(f64, f64)> = pipes
            .iter()
            .map(|p| {
                p.pipe.wall().map_or((0.0, 1.0), |w| match response {
                    Response::Front => (w.front_damping(p.pipe.wave_speed(), g), 1.0),
                    Response::Window(window) => w.window_response(p.pipe.wave_speed(), g, window),
                })
            })
            .collect();
        let impedance: Vec<f64> = pipes
            .iter()
            .zip(&wall_effect)
            .map(|(p, &(_, relax))| {
                let b = p.pipe.impedance(g);
                b * relax
            })
            .collect();
        let mut at_node = vec![Vec::new(); network.nodes().len()];
        for (i, &(a, b)) in ends.iter().enumerate() {
            at_node[a].push(impedance[i]);
            at_node[b].push(impedance[i]);
        }
        let reservoir = network
            .nodes()
            .iter()
            .map(|n| matches!(n, Node::Reservoir { .. }))
            .collect();
        let damping = wall_effect.iter().map(|&(d, _)| d).collect();
        let lengths = pipes.iter().map(|p| p.pipe.length().value()).collect();
        let graph = TravelGraph::with_devices(network, &devices);
        let routes = sensors
            .iter()
            .map(|s| graph.routes_from(s.index(), &nominal))
            .collect();
        Self {
            sensors: sensors.iter().map(|s| s.index()).collect(),
            graph,
            ends,
            nominal,
            impedance,
            at_node,
            reservoir,
            devices,
            damping,
            lengths,
            routes,
        }
    }

    /// Head drop of the first front at each sensor for a burst of unit flow
    /// (1 m^3/s) `fraction` of the way along `pipe` [m per m^3/s]; 0 where
    /// no wave arrives.
    pub(super) fn per_unit_flow(&self, pipe: usize, fraction: f64) -> Vec<f64> {
        self.sensors
            .iter()
            .zip(&self.routes)
            .map(|(&s, r)| self.at(s, r, pipe, fraction))
            .collect()
    }

    /// As [`Self::per_unit_flow`] for one sensor at node `sensor`, given its
    /// `routes` ([`TravelGraph::routes_from`] with the book wave speeds) -
    /// for sensors not given to [`Self::new`].
    pub(super) fn at(&self, sensor: usize, routes: &Routes, pipe: usize, fraction: f64) -> f64 {
        let transmission = |node: usize, incoming: f64| {
            if self.reservoir[node] {
                0.0
            } else {
                self.devices
                    .transmission(node, incoming, &self.at_node[node])
            }
        };
        let (times, pipe_routes) = (&routes.0, &routes.1);
        let (_, entry) =
            self.graph
                .point_route(sensor, times, self.ends[pipe], fraction, self.nominal[pipe]);
        let Some((end, part)) = entry else {
            return 0.0;
        };
        // Along the burst pipe to `end` (a `part` of it), out at `end`, then
        // along the route to the sensor (listed in the order the wave
        // travels); plastic pipes shrink the front on the way.
        let kept = |p: usize, share: f64| {
            if self.damping[p] > 0.0 {
                (-self.damping[p] * share * self.lengths[p]).exp()
            } else {
                1.0
            }
        };
        let mut head =
            0.5 * self.impedance[pipe] * kept(pipe, part) * transmission(end, self.impedance[pipe]);
        let mut node = end;
        for &p in &pipe_routes[end] {
            let (a, b) = self.ends[p];
            node = if a == node { b } else { a };
            head *= kept(p, 1.0) * transmission(node, self.impedance[p]);
        }
        head
    }
}

/// The head drop of the first front at each of `sensors` for a sudden
/// burst of `burst` at `point` (0 where no wave arrives, e.g. a logger at a
/// reservoir).
///
/// # Errors
/// An unknown sensor node or pipe, or a point off the pipe.
pub fn first_fronts(
    network: &Network,
    sensors: &[NodeId],
    point: Point,
    burst: Flow,
) -> Result<Vec<Head>, LocalisationError> {
    heads(
        network,
        Devices::of(network),
        sensors,
        point,
        burst,
        Response::Front,
    )
}

/// As [`first_fronts`], with the pumps and valves in the states `devices`
/// gives (e.g. those of the hour).
///
/// # Errors
/// As [`first_fronts`].
pub fn first_fronts_with(
    network: &Network,
    devices: &Devices,
    sensors: &[NodeId],
    point: Point,
    burst: Flow,
) -> Result<Vec<Head>, LocalisationError> {
    heads(
        network,
        devices.clone(),
        sensors,
        point,
        burst,
        Response::Front,
    )
}

/// The head drop each of `sensors` FEELS within `window` after the first
/// front, for a sudden burst of `burst` at `point` - what decides whether a
/// logger notices it. Equal to [`first_fronts`] on rigid pipes; larger on
/// plastic ones, where the wall's creep builds the drop up behind a
/// shrinking front.
///
/// # Errors
/// As [`first_fronts`], or a non-positive window.
pub fn felt_heads(
    network: &Network,
    sensors: &[NodeId],
    point: Point,
    burst: Flow,
    window: Time,
) -> Result<Vec<Head>, LocalisationError> {
    crate::error::positive("window", window.value())?;
    heads(
        network,
        Devices::of(network),
        sensors,
        point,
        burst,
        Response::Window(window),
    )
}

fn heads(
    network: &Network,
    devices: Devices,
    sensors: &[NodeId],
    point: Point,
    burst: Flow,
    response: Response,
) -> Result<Vec<Head>, LocalisationError> {
    let pipes = network.pipes();
    let p = point.pipe.index();
    if p >= pipes.len() || sensors.iter().any(|s| s.index() >= network.nodes().len()) {
        return Err(LocalisationError::InvalidInput(
            "unknown pipe or sensor node".into(),
        ));
    }
    let length = pipes[p].pipe.length().value();
    let fraction = point.distance.value() / length;
    if !(0.0..=1.0).contains(&fraction) {
        return Err(LocalisationError::InvalidInput(format!(
            "{} m is off a {length} m pipe",
            point.distance.value()
        )));
    }
    Ok(Fronts::with_devices(network, sensors, response, devices)
        .per_unit_flow(p, fraction)
        .iter()
        .map(|h| Head::new(h * burst.value()))
        .collect())
}

/// One pipe's detectability.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipeDetectability {
    /// The pipe.
    pub pipe: jk_hydraulics::PipeId,
    /// Its length.
    pub length: Length,
    /// Share of its length where the burst is felt by at least one logger.
    pub felt: f64,
    /// Share where at least two loggers feel it (so it can be located).
    pub locatable: f64,
    /// The smallest burst two loggers feel at the pipe's WORST spot: any
    /// burst this big, anywhere on the pipe, can be located.
    pub smallest_locatable: Flow,
}

/// How much of the network a burst of a given size would be felt on.
#[derive(Debug, Clone, PartialEq)]
pub struct Detectability {
    /// The loggers.
    pub sensors: Vec<NodeId>,
    /// The burst size asked about.
    pub burst: Flow,
    /// Share of the pipe length where at least one logger feels it.
    pub felt: f64,
    /// Share where at least two loggers feel it (so it can be located).
    pub locatable: f64,
    /// Per pipe, in network order.
    pub pipes: Vec<PipeDetectability>,
}

/// Which loggers at `sensors` would feel a sudden burst of `burst` - the
/// size in `options.burst`, default 1 l/s - anywhere on `network`, trying
/// points every `options.source_step`.
///
/// # Errors
/// Fewer than two sensors, an unknown or repeated node, or options out of
/// range (the burst size must be given).
pub fn detectability(
    network: &Network,
    sensors: &[NodeId],
    options: &PlacementOptions,
) -> Result<Detectability, LocalisationError> {
    super::coverage::validate(network, sensors, None, options)?;
    let Some(burst) = options.burst else {
        return Err(LocalisationError::InvalidInput(
            "detectability needs a burst size".into(),
        ));
    };
    let fronts = Fronts::new(network, sensors, Response::Window(options.window));
    let threshold = options.detect_units * options.noise.value();
    let step = options.source_step.value();
    let (mut felt_len, mut locatable_len, mut total_len) = (0.0, 0.0, 0.0);
    let mut pipes = Vec::with_capacity(network.pipes().len());
    for (p, link) in network.pipes().iter().enumerate() {
        let Some(pipe) = network.pipe_id(p) else {
            continue;
        };
        let length = link.pipe.length().value();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        // ceil of a positive ratio
        let n = ((length / step).ceil() as usize).max(1);
        let (mut felt, mut locatable, mut worst) = (0usize, 0usize, 0.0_f64);
        for i in 0..n {
            let fraction = (to_f64(i) + 0.5) / to_f64(n);
            // Smallest burst each logger feels here, smallest first.
            let mut smallest: Vec<f64> = fronts
                .per_unit_flow(p, fraction)
                .iter()
                .map(|&h| {
                    if h > 0.0 {
                        threshold / h
                    } else {
                        f64::INFINITY
                    }
                })
                .collect();
            smallest.sort_by(f64::total_cmp);
            if smallest[0] <= burst.value() {
                felt += 1;
            }
            if smallest[1] <= burst.value() {
                locatable += 1;
            }
            worst = worst.max(smallest[1]);
        }
        felt_len += length * to_f64(felt) / to_f64(n);
        locatable_len += length * to_f64(locatable) / to_f64(n);
        total_len += length;
        pipes.push(PipeDetectability {
            pipe,
            length: link.pipe.length(),
            felt: to_f64(felt) / to_f64(n),
            locatable: to_f64(locatable) / to_f64(n),
            smallest_locatable: Flow::new(worst),
        });
    }
    let share = |x: f64| if total_len > 0.0 { x / total_len } else { 0.0 };
    Ok(Detectability {
        sensors: sensors.to_vec(),
        burst,
        felt: share(felt_len),
        locatable: share(locatable_len),
        pipes,
    })
}

#[allow(clippy::cast_precision_loss)] // counts are far below 2^52
fn to_f64(i: usize) -> f64 {
    i as f64
}
