//! The 1-D Saint-Venant solver: finite volumes, HLL flux, explicit steps
//! (`docs/02_physics_of_transients/open_channels.txt`, section 2).
//!
//! Each cell holds its wetted area A and flow Q. A step moves water by
//! the HLL fluxes between cells, then lets gravity (bed slope) and friction
//! act together, semi-implicitly:
//!
//! ```text
//!   Q <- (Q - dt/dx dF + dt g A S0) / (1 + dt g n^2 |Q| / (A R^(4/3)))
//! ```
//!
//! - exact for uniform flow (friction then cancels the slope exactly, step
//!   after step), and stable however shallow the water.

use jk_kernel::{Flow, Length, Time};

use crate::error::{ChannelError, positive};
use crate::section::{Reach, Section};

/// Depth below which a cell counts as dry [m].
const DRY: f64 = 1e-6;

/// What comes in at the upstream end.
#[derive(Debug, Clone, PartialEq)]
pub enum Upstream {
    /// A given inflow over time: (time, flow) points, linear between,
    /// held before the first and after the last.
    Inflow(Vec<(Time, Flow)>),
    /// A given water depth over time at the inlet (a reservoir or dam
    /// outlet): (time, depth) points, linear between.
    Level(Vec<(Time, Length)>),
    /// A closed end.
    Wall,
}

/// What happens at the downstream end.
#[derive(Debug, Clone, PartialEq)]
pub enum Downstream {
    /// The depth held there (a pool, a gate holding a level).
    Depth(Length),
    /// Water leaves as it arrives (zero gradient).
    FreeOutfall,
    /// A closed end.
    Wall,
    /// Water drawn off at a given rate (an intake, a pump station, the
    /// network a canal feeds): (time, flow) points, linear between. Never
    /// more than the end of the channel holds - a dry canal gives nothing.
    Outflow(Vec<(Time, Flow)>),
}

/// The channel at the start.
#[derive(Debug, Clone, PartialEq)]
pub enum Initial {
    /// Uniform flow at normal depth.
    Uniform(Flow),
    /// Still water this deep everywhere.
    Still(Length),
    /// A dam at `at`: still water `upstream` deep before it, `downstream`
    /// deep after it (0: a dry bed).
    Dam {
        /// Where the dam stands, from the upstream end.
        at: Length,
        /// Depth before it.
        upstream: Length,
        /// Depth after it.
        downstream: Length,
    },
    /// Depth and flow in every cell (cell centres: [`centres`]).
    Cells(Vec<(Length, Flow)>),
}

/// Settings of a run.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelOptions {
    /// Time to cover.
    pub duration: Time,
    /// Number of cells along the reach.
    pub cells: usize,
    /// Courant number (< 1).
    pub courant: f64,
    /// Time between recorded states at the gauges.
    pub record_every: Time,
}

/// A run: depth and flow at each gauge over time, and the final state of
/// every cell.
#[derive(Debug, Clone, PartialEq)]
pub struct ChannelRun {
    /// Recorded times.
    pub times: Vec<Time>,
    /// Per gauge: its position, and depth and flow at each recorded time.
    pub gauges: Vec<(Length, Vec<Length>, Vec<Flow>)>,
    /// Depth in every cell at the end.
    pub depth: Vec<Length>,
    /// Flow in every cell at the end.
    pub flow: Vec<Flow>,
    /// Water in the reach at the start and at the end [m^3].
    pub volume: (f64, f64),
    /// Water that came in upstream and went out downstream [m^3].
    pub exchanged: (f64, f64),
    /// Steps taken.
    pub steps: usize,
    /// Flow out of the downstream end at each recorded time: the average
    /// over the interval ending there (the first: over the first interval)
    /// - so its integral is the water that left.
    pub outlet: Vec<Flow>,
}

/// The centre of each of `cells` equal cells along `reach` [m].
#[must_use]
pub fn centres(reach: &Reach, cells: usize) -> Vec<f64> {
    #[allow(clippy::cast_precision_loss)] // cell counts are small
    let dx = reach.length.value() / cells as f64;
    #[allow(clippy::cast_precision_loss)]
    (0..cells).map(|i| (i as f64 + 0.5) * dx).collect()
}

/// Physical flux (A-flux, Q-flux) of a cell state, its velocity and wave
/// celerity.
#[inline]
fn physics(s: &Section, g: f64, a: f64, q: f64) -> (f64, f64, f64, f64) {
    let h = s.depth(a);
    if h <= DRY {
        return (0.0, 0.0, 0.0, 0.0);
    }
    let u = q / a;
    (q, q * u + g * s.pressure_term(h), u, s.celerity(h, g))
}

/// HLL flux between a left and a right state (Toro, ch. 10), with the
/// dry-front speeds u +/- 2c against a dry bed.
#[inline]
fn hll(s: &Section, g: f64, (al, ql): (f64, f64), (ar, qr): (f64, f64)) -> (f64, f64) {
    let (fla, flq, ul, cl) = physics(s, g, al, ql);
    let (fra, frq, ur, cr) = physics(s, g, ar, qr);
    let (dry_l, dry_r) = (s.depth(al) <= DRY, s.depth(ar) <= DRY);
    if dry_l && dry_r {
        return (0.0, 0.0);
    }
    let (sl, sr) = if dry_l {
        (ur - 2.0 * cr, ur + cr)
    } else if dry_r {
        (ul - cl, ul + 2.0 * cl)
    } else {
        ((ul - cl).min(ur - cr), (ul + cl).max(ur + cr))
    };
    if sl >= 0.0 {
        (fla, flq)
    } else if sr <= 0.0 {
        (fra, frq)
    } else {
        let w = 1.0 / (sr - sl);
        (
            (sr * fla - sl * fra + sl * sr * (ar - al)) * w,
            (sr * flq - sl * frq + sl * sr * (qr - ql)) * w,
        )
    }
}

/// A piecewise-linear history at time `t` (held before the first point and
/// after the last).
fn at(points: &[(Time, f64)], t: f64) -> f64 {
    match points.iter().position(|(pt, _)| pt.value() > t) {
        None => points.last().map_or(0.0, |p| p.1),
        Some(0) => points[0].1,
        Some(k) => {
            let ((t0, v0), (t1, v1)) = (points[k - 1], points[k]);
            let w = (t - t0.value()) / (t1.value() - t0.value());
            v0 + w * (v1 - v0)
        }
    }
}

/// A reach in motion: its cells' areas and flows at time `t`, advanced on
/// demand - so it can run side by side with another solver (a pipe network
/// drawing from it), exchanging levels and flows every step.
#[derive(Debug, Clone, PartialEq)]
pub struct ChannelState {
    reach: Reach,
    courant: f64,
    inflow: Option<Vec<(Time, f64)>>,
    level: Option<Vec<(Time, f64)>>,
    drawn: Option<Vec<(Time, f64)>>,
    upstream_wall: bool,
    downstream: Downstream,
    a: Vec<f64>,
    q: Vec<f64>,
    flux: Vec<(f64, f64)>,
    dx: f64,
    t: f64,
    /// Water that came in upstream and went out downstream so far [m^3].
    exchanged: (f64, f64),
    steps: usize,
    /// A flow drawn off now, overriding the downstream history ([`Self::draw`]).
    drawing: Option<f64>,
}

impl ChannelState {
    /// A reach at time 0 in the state `initial`, cut into `cells`, stepped
    /// at Courant number `courant`.
    ///
    /// # Errors
    /// A bad reach, a Courant number outside (0, 1), fewer than two cells,
    /// a history that is empty or not in time order, an initial state of
    /// the wrong length, or a uniform start on a bed with no uniform flow.
    pub fn new(
        reach: &Reach,
        upstream: &Upstream,
        downstream: &Downstream,
        initial: &Initial,
        cells: usize,
        courant: f64,
    ) -> Result<Self, ChannelError> {
        reach.validate()?;
        if !(courant > 0.0 && courant < 1.0) {
            return Err(ChannelError::InvalidInput(
                "the Courant number must be between 0 and 1".into(),
            ));
        }
        if cells < 2 {
            return Err(ChannelError::InvalidInput("at least two cells".into()));
        }
        let history = |points: Vec<(Time, f64)>| -> Result<Vec<(Time, f64)>, ChannelError> {
            if points.is_empty() || points.windows(2).any(|w| w[1].0 <= w[0].0) {
                return Err(ChannelError::InvalidInput(
                    "a history needs points in increasing time".into(),
                ));
            }
            Ok(points)
        };
        let inflow = match upstream {
            Upstream::Inflow(p) => Some(history(p.iter().map(|(t, q)| (*t, q.value())).collect())?),
            _ => None,
        };
        let level = match upstream {
            Upstream::Level(p) => Some(history(p.iter().map(|(t, h)| (*t, h.value())).collect())?),
            _ => None,
        };
        let drawn = match downstream {
            Downstream::Outflow(p) => {
                Some(history(p.iter().map(|(t, q)| (*t, q.value())).collect())?)
            }
            _ => None,
        };
        let s = reach.section;
        let x = centres(reach, cells);
        #[allow(clippy::cast_precision_loss)]
        let dx = reach.length.value() / cells as f64;
        let (a, q): (Vec<f64>, Vec<f64>) = match initial {
            Initial::Uniform(flow) => {
                let h = reach.normal_depth(*flow)?.value();
                (vec![s.area(h); cells], vec![flow.value(); cells])
            }
            Initial::Still(h) => (vec![s.area(h.value()); cells], vec![0.0; cells]),
            Initial::Dam {
                at: dam,
                upstream,
                downstream,
            } => (
                x.iter()
                    .map(|&xi| {
                        s.area(if xi < dam.value() {
                            upstream.value()
                        } else {
                            downstream.value()
                        })
                    })
                    .collect(),
                vec![0.0; cells],
            ),
            Initial::Cells(given) => {
                if given.len() != cells {
                    return Err(ChannelError::InvalidInput(format!(
                        "{} initial cells for {cells} cells",
                        given.len()
                    )));
                }
                (
                    given.iter().map(|(h, _)| s.area(h.value())).collect(),
                    given.iter().map(|(_, f)| f.value()).collect(),
                )
            }
        };
        Ok(Self {
            reach: *reach,
            courant,
            inflow,
            level,
            drawn,
            upstream_wall: matches!(upstream, Upstream::Wall),
            downstream: downstream.clone(),
            a,
            q,
            flux: vec![(0.0, 0.0); cells + 1],
            dx,
            t: 0.0,
            exchanged: (0.0, 0.0),
            steps: 0,
            drawing: None,
        })
    }

    /// The time reached [s].
    #[must_use]
    pub fn time(&self) -> Time {
        Time::new(self.t)
    }

    /// Depth in cell `i`.
    #[must_use]
    pub fn depth(&self, i: usize) -> Length {
        Length::new(self.reach.section.depth(self.a[i]))
    }

    /// Flow in cell `i`.
    #[must_use]
    pub fn flow(&self, i: usize) -> Flow {
        Flow::new(self.q[i])
    }

    /// Number of cells.
    #[must_use]
    pub fn cells(&self) -> usize {
        self.a.len()
    }

    /// Depth at the downstream end (its last cell).
    #[must_use]
    pub fn outlet_depth(&self) -> Length {
        self.depth(self.a.len() - 1)
    }

    /// Water in the reach now [m^3].
    #[must_use]
    pub fn volume(&self) -> f64 {
        self.a.iter().sum::<f64>() * self.dx
    }

    /// Water that came in upstream and went out downstream so far [m^3].
    #[must_use]
    pub fn exchanged(&self) -> (f64, f64) {
        self.exchanged
    }

    /// Steps taken so far.
    #[must_use]
    pub fn steps(&self) -> usize {
        self.steps
    }

    /// Advances to `until`, in steps the fastest wave allows.
    pub fn advance(&mut self, until: Time) {
        let until = until.value();
        while self.t < until - 1e-12 {
            let (s, g) = (self.reach.section, self.reach.gravity.value());
            let fastest = self
                .a
                .iter()
                .zip(&self.q)
                .map(|(&ai, &qi)| {
                    let (_, _, u, c) = physics(&s, g, ai, qi);
                    u.abs() + c
                })
                .fold(0.0, f64::max)
                .max(1e-9);
            let dt = (self.courant * self.dx / fastest).min(until - self.t);
            self.step(dt);
        }
    }

    /// Advances to `until` drawing `flow` off at the downstream end (which
    /// must be [`Downstream::Outflow`]); returns the average flow actually
    /// given - less than asked if the end of the channel runs low.
    pub fn draw(&mut self, until: Time, flow: Flow) -> Flow {
        let (t0, out0) = (self.t, self.exchanged.1);
        self.drawing = Some(flow.value());
        self.advance(until);
        self.drawing = None;
        if self.t > t0 {
            Flow::new((self.exchanged.1 - out0) / (self.t - t0))
        } else {
            Flow::ZERO
        }
    }

    /// One step of length `dt`.
    fn step(&mut self, dt: f64) {
        let (s, g, n, dx, t) = (
            self.reach.section,
            self.reach.gravity.value(),
            self.a.len(),
            self.dx,
            self.t,
        );
        let dry_area = s.area(DRY);
        let (a, q) = (&mut self.a, &mut self.q);
        // Ghost cells.
        let up = if let Some(points) = &self.inflow {
            let qin = at(points, t);
            let critical = if qin > 0.0 {
                self.reach
                    .critical_depth(Flow::new(qin))
                    .map_or(0.0, |h| s.area(h.value()))
            } else {
                0.0
            };
            (a[0].max(critical), qin)
        } else if let Some(points) = &self.level {
            (s.area(at(points, t)), q[0])
        } else {
            debug_assert!(self.upstream_wall);
            (a[0], -q[0])
        };
        let down = match &self.downstream {
            Downstream::Depth(h) => (s.area(h.value()), q[n - 1]),
            Downstream::FreeOutfall | Downstream::Outflow(_) => (a[n - 1], q[n - 1]),
            Downstream::Wall => (a[n - 1], -q[n - 1]),
        };
        let flux = &mut self.flux;
        flux[0] = hll(&s, g, up, (a[0], q[0]));
        for i in 1..n {
            flux[i] = hll(&s, g, (a[i - 1], q[i - 1]), (a[i], q[i]));
        }
        let asked = self
            .drawing
            .or_else(|| self.drawn.as_deref().map(|p| at(p, t)));
        flux[n] = match asked {
            // Drawn off: exactly the asked flow, capped by what the last
            // cell can give this step.
            Some(asked) => {
                let h_last = s.depth(a[n - 1]);
                let available = ((a[n - 1] - dry_area) * dx / dt + flux[n - 1].0).max(0.0) * 0.999;
                let qd = asked.max(0.0).min(available);
                if h_last <= DRY || a[n - 1] <= dry_area {
                    (0.0, 0.0)
                } else {
                    (qd, qd * qd / a[n - 1] + g * s.pressure_term(h_last))
                }
            }
            None => hll(&s, g, (a[n - 1], q[n - 1]), down),
        };
        self.exchanged.0 += dt * flux[0].0;
        self.exchanged.1 += dt * flux[n].0;
        let r = dt / dx;
        for i in 0..n {
            a[i] = (a[i] - r * (flux[i + 1].0 - flux[i].0)).max(0.0);
            let moved = q[i] - r * (flux[i + 1].1 - flux[i].1);
            let h = s.depth(a[i]);
            if a[i] <= dry_area || h <= DRY {
                q[i] = 0.0;
                continue;
            }
            // Gravity along the bed and friction, together.
            let pushed = moved + dt * g * a[i] * self.reach.slope;
            let radius = a[i] / s.perimeter(h);
            let k = g * self.reach.manning * self.reach.manning / (a[i] * radius.powf(4.0 / 3.0));
            q[i] = pushed / (1.0 + dt * k * moved.abs());
        }
        self.t += dt;
        self.steps += 1;
    }
}

/// Runs the reach.
///
/// # Errors
/// A bad reach or options, a history that is empty or not in time order,
/// an initial state of the wrong length, or a uniform start on a bed with
/// no uniform flow.
pub fn simulate(
    reach: &Reach,
    upstream: &Upstream,
    downstream: &Downstream,
    initial: &Initial,
    gauges: &[Length],
    options: &ChannelOptions,
) -> Result<ChannelRun, ChannelError> {
    positive("duration", options.duration.value())?;
    positive("record interval", options.record_every.value())?;
    let mut state = ChannelState::new(
        reach,
        upstream,
        downstream,
        initial,
        options.cells,
        options.courant,
    )?;
    let n = state.cells();
    #[allow(clippy::cast_precision_loss)]
    let dx = reach.length.value() / n as f64;
    let gauge_cells: Vec<usize> = gauges
        .iter()
        .map(|p| {
            #[allow(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                clippy::cast_precision_loss
            )] // clamped, few cells
            let i = (p.value() / dx).floor().clamp(0.0, (n - 1) as f64) as usize;
            i
        })
        .collect();
    let mut run = ChannelRun {
        times: Vec::new(),
        gauges: gauges
            .iter()
            .map(|&p| (p, Vec::new(), Vec::new()))
            .collect(),
        depth: Vec::new(),
        flow: Vec::new(),
        volume: (state.volume(), 0.0),
        exchanged: (0.0, 0.0),
        steps: 0,
        outlet: Vec::new(),
    };
    let record = |run: &mut ChannelRun, state: &ChannelState| {
        run.times.push(state.time());
        for (k, &i) in gauge_cells.iter().enumerate() {
            run.gauges[k].1.push(state.depth(i));
            run.gauges[k].2.push(state.flow(i));
        }
    };
    let end = options.duration.value();
    let every = options.record_every.value();
    record(&mut run, &state);
    let mut out_at_record = 0.0;
    let mut k = 1.0;
    loop {
        let t0 = state.time().value();
        let next = (k * every).min(end);
        state.advance(Time::new(next));
        let out = state.exchanged().1;
        record(&mut run, &state);
        run.outlet
            .push(Flow::new((out - out_at_record) / (next - t0)));
        out_at_record = out;
        if next >= end - 1e-12 {
            break;
        }
        k += 1.0;
    }
    // The first record's outflow: that of the first interval.
    run.outlet.insert(0, run.outlet[0]);
    run.volume.1 = state.volume();
    run.exchanged = state.exchanged();
    run.steps = state.steps();
    run.depth = (0..n).map(|i| state.depth(i)).collect();
    run.flow = (0..n).map(|i| state.flow(i)).collect();
    Ok(run)
}
