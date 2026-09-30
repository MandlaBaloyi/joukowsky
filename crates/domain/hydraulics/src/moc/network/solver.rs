//! The network time loop.
//!
//! Each step:
//!   1. every pipe updates its interior points from the two characteristics
//!      (the same [`Characteristics`] code the single-pipe solver uses);
//!   2. every node gathers the characteristic each of its pipes brings and
//!      applies its boundary: reservoir, junction (with any active burst),
//!      valve, check valve, or pump (after a trip, its speed is advanced
//!      with a predictor-corrector step of the run-down equation);
//!   3. plastic pipes advance each point's wall creep (S6);
//!   4. probes are recorded.
//!
//! A plastic pipe's creep is folded into every characteristic it carries
//! (`pipe/viscoelastic.rs`): interior points and pipe ends then solve with
//! the same code as rigid pipes.
//!
//! All arithmetic is in a fixed order, so the same network and inputs give
//! bit-identical results on the same platform (rule T3).

use jk_kernel::Time;

use crate::HydraulicsError;
use crate::boundary::check_valve;
use crate::boundary::junction::{self, Arrival, PipeEnd};
use crate::boundary::{leak, pump, valve};
use crate::error::positive;
use crate::moc::characteristics::Characteristics;
use crate::moc::grid::MAX_STEPS;
use crate::moc::network::grid::{GridOptions, NetworkGrid};
use crate::moc::network::initial::InitialCondition;
use crate::moc::trace::Trace;
use crate::model::{Burst, Network, Node, NodeId, PipeId};
use crate::pipe::viscoelastic::{Creep, CreepStep};

/// A plastic pipe's wall during a run: the step constants, each grid
/// point's creep, and the head each point's wall settled at (the initial
/// state).
struct Wall {
    step: CreepStep,
    creep: Vec<Creep>,
    settled: Vec<f64>,
}

impl Wall {
    /// The characteristic (c, b) from point `foot` (old head `foot_head`)
    /// arriving at point `i` (old head `head`), with the creep folded in.
    #[inline]
    fn fold(
        &self,
        (foot, foot_head): (usize, f64),
        i: usize,
        c: f64,
        b: f64,
        head: f64,
    ) -> (f64, f64) {
        let foot_rate = self
            .step
            .rate_now(&self.creep[foot], foot_head - self.settled[foot]);
        self.step.fold(
            c,
            b,
            foot_rate,
            &self.creep[i],
            self.settled[i],
            head - self.settled[i],
        )
    }
}

/// A point to record - the simulation's equivalent of a sensor.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Probe {
    /// A node: its head, and the flow LEAVING the network there (demand +
    /// burst outflow at a junction, the valve flow at a valve, the net flow
    /// into a reservoir, 0 at a check valve). At a check valve the head is
    /// the upstream side's. At a pump: the DISCHARGE head (where the
    /// down-surge after a trip is worst) and the flow through the pump.
    Node(NodeId),
    /// A point along a pipe (fraction 0 = its `from` end, 1 = its `to` end):
    /// head and pipe flow at the nearest grid node.
    Pipe {
        /// The pipe.
        pipe: PipeId,
        /// Position along it, 0..=1.
        fraction: f64,
    },
}

/// The result of a network simulation.
#[derive(Debug, Clone, PartialEq)]
pub struct NetworkRun {
    /// The grid used, including every wave-speed adjustment made.
    pub grid: NetworkGrid,
    /// The recorded probes, in request order.
    pub trace: Trace,
    /// Speed ratio N / `N_rated` of every pump, one sample per time step.
    pub pump_speeds: Vec<(NodeId, Vec<f64>)>,
}

#[derive(Clone, Copy)]
enum Resolved {
    Node(usize),
    Pipe(usize, usize),
}

impl Network {
    /// Simulates the network from `initial` for `duration`, recording
    /// `probes`.
    ///
    /// # Errors
    /// Invalid topology, grid options, initial state, duration or probes.
    ///
    /// # Panics
    /// Only if an internal invariant is broken (a check valve without its
    /// two pipes after the topology check passed) - that would be a bug.
    #[allow(clippy::too_many_lines)] // one loop, kept together for clarity
    pub fn simulate(
        &self,
        initial: &InitialCondition,
        duration: Time,
        probes: &[Probe],
        options: &GridOptions,
    ) -> Result<NetworkRun, HydraulicsError> {
        let topo = self.topology()?;
        let grid = NetworkGrid::new(self, options)?;
        let steady = initial.resolve(self, &topo)?;
        let dt = grid.time_step();
        let steps = steps_for(duration, dt)?;
        let resolved = self.resolve_probes(probes, &grid)?;

        // Pipes: coefficients on the adjusted wave speed, steady profiles.
        let g = self.gravity();
        let mut chars = Vec::with_capacity(self.pipes().len());
        let mut h: Vec<Vec<f64>> = Vec::with_capacity(self.pipes().len());
        let mut q: Vec<Vec<f64>> = Vec::with_capacity(self.pipes().len());
        for (p, np) in self.pipes().iter().enumerate() {
            let r = grid.reaches()[p];
            let adjusted = np.pipe.with_wave_speed(grid.wave_speeds()[p])?;
            let c = Characteristics::new(&adjusted, np.pipe.length() / f64::from(r), g);
            let q0 = steady.pipe_flows[p];
            let (h_from, h_to) = steady.pipe_end_heads[p];
            let mut hp: Vec<f64> = (0..=r)
                .map(|i| h_from - f64::from(i) * c.reach_head_loss(q0))
                .collect();
            hp[r as usize] = h_to;
            h.push(hp);
            q.push(vec![q0; r as usize + 1]);
            chars.push(c);
        }
        let mut h_new = h.clone();
        let mut q_new = q.clone();
        let mut walls: Vec<Option<Wall>> = self
            .pipes()
            .iter()
            .enumerate()
            .map(|(p, np)| {
                np.pipe.wall().map(|w| Wall {
                    step: CreepStep::new(w, grid.wave_speeds()[p], dt, g),
                    creep: vec![Creep::default(); h[p].len()],
                    settled: h[p].clone(),
                })
            })
            .collect();

        // Nodes: heads, outflows, device states.
        let mut burst_at: Vec<Option<Burst>> = vec![None; self.nodes().len()];
        for b in self.bursts() {
            burst_at[b.node.0] = Some(*b);
        }
        let mut node_head = steady.node_heads.clone();
        let mut node_outflow: Vec<f64> = self
            .nodes()
            .iter()
            .enumerate()
            .map(|(n, node)| match node {
                Node::Junction { demand } => demand.value(),
                Node::Valve { .. } => steady.valve_steady[n].0,
                Node::Reservoir { .. } => topo[n]
                    .iter()
                    .map(|&(pid, end)| match end {
                        PipeEnd::Downstream => steady.pipe_flows[pid.0],
                        PipeEnd::Upstream => -steady.pipe_flows[pid.0],
                    })
                    .sum(),
                Node::CheckValve => 0.0,
                Node::Pump { .. } => pump_suction_flow(&topo[n], &steady.pipe_flows),
            })
            .collect();
        let mut check_states = steady.check_valves.clone();

        // Pumps: speed ratio, flow, and the recorded speed history. A pump
        // node reports its discharge head.
        let mut pump_alpha = vec![1.0; self.nodes().len()];
        let mut pump_flow = node_outflow.clone();
        let mut pump_speeds: Vec<(NodeId, Vec<f64>)> = Vec::new();
        for (n, node) in self.nodes().iter().enumerate() {
            if matches!(node, Node::Pump { .. }) {
                let discharge = topo[n]
                    .iter()
                    .find(|(_, e)| *e == PipeEnd::Upstream)
                    .map(|(p, _)| p.0)
                    .expect("topology checked: a pump has a discharge pipe");
                node_head[n] = steady.pipe_end_heads[discharge].0;
                let mut history = Vec::with_capacity(steps + 1);
                history.push(1.0);
                pump_speeds.push((NodeId(n), history));
            }
        }
        let (g_si, dt_si) = (g.value(), dt.value());

        let mut trace = Trace::new(dt, probes.len(), steps + 1);
        record(&mut trace, &resolved, &node_head, &node_outflow, &h, &q);
        let mut arrivals: Vec<Arrival> = Vec::new();

        for k in 1..=steps {
            #[allow(clippy::cast_precision_loss)] // k <= MAX_STEPS, exact in f64
            let t = dt * k as f64;

            // 1. Pipe interiors.
            for (p, c) in chars.iter().enumerate() {
                let (hp, qp) = (&h[p], &q[p]);
                for i in 1..hp.len() - 1 {
                    let cp = c.c_plus(hp[i - 1], qp[i - 1]);
                    let cm = c.c_minus(hp[i + 1], qp[i + 1]);
                    (h_new[p][i], q_new[p][i]) = match &walls[p] {
                        None => c.interior(cp, cm),
                        Some(w) => {
                            let (cp, b) = w.fold((i - 1, hp[i - 1]), i, cp, c.impedance(), hp[i]);
                            let (cm, _) = w.fold((i + 1, hp[i + 1]), i, cm, c.impedance(), hp[i]);
                            (f64::midpoint(cp, cm), (cp - cm) / (2.0 * b))
                        }
                    };
                }
            }

            // 2. Nodes.
            for (n, node) in self.nodes().iter().enumerate() {
                arrivals.clear();
                for &(pid, end) in &topo[n] {
                    let (c, hp, qp) = (&chars[pid.0], &h[pid.0], &q[pid.0]);
                    let last = hp.len() - 1;
                    let (characteristic, at, foot) = match end {
                        PipeEnd::Downstream => {
                            (c.c_plus(hp[last - 1], qp[last - 1]), last, last - 1)
                        }
                        PipeEnd::Upstream => (c.c_minus(hp[1], qp[1]), 0, 1),
                    };
                    let (characteristic, impedance) = match &walls[pid.0] {
                        None => (characteristic, c.impedance()),
                        Some(w) => {
                            w.fold((foot, hp[foot]), at, characteristic, c.impedance(), hp[at])
                        }
                    };
                    arrivals.push(Arrival {
                        characteristic,
                        impedance,
                        end,
                    });
                }

                let (head, outflow) = match *node {
                    Node::Reservoir { head } => {
                        let hn = self.source_head(n, head, t);
                        let net_in = arrivals
                            .iter()
                            .map(|a| match a.end {
                                PipeEnd::Downstream => a.flow(hn),
                                PipeEnd::Upstream => -a.flow(hn),
                            })
                            .sum();
                        (hn, net_in)
                    }
                    Node::Junction { demand } => {
                        let d = demand.value();
                        match burst_at[n] {
                            Some(b) => {
                                let (hn, leak_flow) = leak::head(
                                    &arrivals,
                                    d,
                                    b.coefficient_at(t),
                                    b.outside_head.value(),
                                );
                                (hn, d + leak_flow)
                            }
                            None => (junction::head(&arrivals, d), d),
                        }
                    }
                    Node::Valve {
                        downstream_head,
                        closure,
                        ..
                    } => {
                        let a = arrivals[0];
                        let (q0, dh0) = steady.valve_steady[n];
                        let cv = valve::coefficient(closure.opening(t), q0, dh0);
                        let (hn, qv) = valve::downstream(
                            a.characteristic,
                            downstream_head.value(),
                            a.impedance,
                            cv,
                        );
                        (hn, qv)
                    }
                    Node::CheckValve => {
                        let pos = |e: PipeEnd| {
                            arrivals
                                .iter()
                                .position(|a| a.end == e)
                                .expect("topology checked: one pipe each side")
                        };
                        let (i_in, i_out) = (pos(PipeEnd::Downstream), pos(PipeEnd::Upstream));
                        let (a_in, a_out) = (arrivals[i_in], arrivals[i_out]);
                        let s = check_valve::step(
                            check_states[n],
                            a_in.characteristic,
                            a_in.impedance,
                            a_out.characteristic,
                            a_out.impedance,
                        );
                        check_states[n] = s.state;
                        let (p_in, p_out) = (topo[n][i_in].0.0, topo[n][i_out].0.0);
                        let last = h_new[p_in].len() - 1;
                        h_new[p_in][last] = s.head_upstream;
                        q_new[p_in][last] = s.flow;
                        h_new[p_out][0] = s.head_downstream;
                        q_new[p_out][0] = s.flow;
                        node_head[n] = s.head_upstream;
                        node_outflow[n] = 0.0;
                        continue;
                    }
                    Node::Pump { pump, trip } => {
                        let pos = |e: PipeEnd| {
                            arrivals
                                .iter()
                                .position(|a| a.end == e)
                                .expect("topology checked: suction and discharge pipes")
                        };
                        let (i_in, i_out) = (pos(PipeEnd::Downstream), pos(PipeEnd::Upstream));
                        let (a_in, a_out) = (arrivals[i_in], arrivals[i_out]);
                        let solve = |alpha: f64| {
                            pump::step(
                                &pump.curve,
                                alpha,
                                a_in.characteristic,
                                a_in.impedance,
                                a_out.characteristic,
                                a_out.impedance,
                            )
                        };
                        let tripped = trip.is_some_and(|at| t > at);
                        let alpha = if tripped {
                            // Heun: predict with the old torque, correct with
                            // the average of old and predicted.
                            let a_old = pump_alpha[n];
                            let dec_old = pump.deceleration(pump_flow[n], a_old, g_si);
                            let a_pred = (a_old + dt_si * dec_old).max(0.0);
                            let dec_pred = pump.deceleration(solve(a_pred).flow, a_pred, g_si);
                            (a_old + 0.5 * dt_si * (dec_old + dec_pred)).max(0.0)
                        } else {
                            1.0
                        };
                        let s = solve(alpha);
                        pump_alpha[n] = alpha;
                        pump_flow[n] = s.flow;
                        let (p_in, p_out) = (topo[n][i_in].0.0, topo[n][i_out].0.0);
                        let last = h_new[p_in].len() - 1;
                        h_new[p_in][last] = s.head_suction;
                        q_new[p_in][last] = s.flow;
                        h_new[p_out][0] = s.head_discharge;
                        q_new[p_out][0] = s.flow;
                        node_head[n] = s.head_discharge;
                        node_outflow[n] = s.flow;
                        continue;
                    }
                };

                for (a, &(pid, end)) in arrivals.iter().zip(&topo[n]) {
                    let i = match end {
                        PipeEnd::Downstream => h_new[pid.0].len() - 1,
                        PipeEnd::Upstream => 0,
                    };
                    h_new[pid.0][i] = head;
                    q_new[pid.0][i] = a.flow(head);
                }
                node_head[n] = head;
                node_outflow[n] = outflow;
            }

            // 3. Plastic walls creep on, point by point.
            for (p, wall) in walls.iter_mut().enumerate() {
                if let Some(w) = wall {
                    for i in 0..w.creep.len() {
                        let (old, new) = (h[p][i] - w.settled[i], h_new[p][i] - w.settled[i]);
                        w.step.advance(&mut w.creep[i], old, new);
                    }
                }
            }

            std::mem::swap(&mut h, &mut h_new);
            std::mem::swap(&mut q, &mut q_new);
            record(&mut trace, &resolved, &node_head, &node_outflow, &h, &q);
            for (id, history) in &mut pump_speeds {
                history.push(pump_alpha[id.0]);
            }
        }

        Ok(NetworkRun {
            grid,
            trace,
            pump_speeds,
        })
    }

    fn resolve_probes(
        &self,
        probes: &[Probe],
        grid: &NetworkGrid,
    ) -> Result<Vec<Resolved>, HydraulicsError> {
        probes
            .iter()
            .map(|probe| match *probe {
                Probe::Node(id) => {
                    if id.0 < self.nodes().len() {
                        Ok(Resolved::Node(id.0))
                    } else {
                        Err(HydraulicsError::ProbeOutOfRange {
                            node: id.0,
                            last: self.nodes().len().saturating_sub(1),
                        })
                    }
                }
                Probe::Pipe { pipe, fraction } => {
                    if pipe.0 >= self.pipes().len() {
                        return Err(HydraulicsError::InvalidNetwork(format!(
                            "probe on pipe {} which does not exist",
                            pipe.0
                        )));
                    }
                    if !(0.0..=1.0).contains(&fraction) {
                        return Err(HydraulicsError::invalid(
                            "probe fraction",
                            fraction,
                            "between 0 and 1",
                        ));
                    }
                    let r = grid.reaches()[pipe.0];
                    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // in [0, r]
                    let i = (fraction * f64::from(r)).round() as usize;
                    Ok(Resolved::Pipe(pipe.0, i))
                }
            })
            .collect()
    }
}

/// Flow in a pump's suction pipe (the pipe ending at the pump node).
fn pump_suction_flow(touching: &[(PipeId, PipeEnd)], pipe_flows: &[f64]) -> f64 {
    touching
        .iter()
        .find(|(_, e)| *e == PipeEnd::Downstream)
        .map_or(0.0, |(p, _)| pipe_flows[p.0])
}

fn steps_for(duration: Time, dt: Time) -> Result<usize, HydraulicsError> {
    positive("duration", duration.value())?;
    let steps = (duration / dt).ceil();
    #[allow(clippy::cast_precision_loss)] // MAX_STEPS is exact in f64
    if steps > MAX_STEPS as f64 {
        return Err(HydraulicsError::invalid(
            "duration / time step",
            steps,
            "<= 5e7 steps",
        ));
    }
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // bounded above
    Ok(steps as usize)
}

fn record(
    trace: &mut Trace,
    probes: &[Resolved],
    node_head: &[f64],
    node_outflow: &[f64],
    h: &[Vec<f64>],
    q: &[Vec<f64>],
) {
    for (j, probe) in probes.iter().enumerate() {
        match *probe {
            Resolved::Node(n) => trace.push(j, node_head[n], node_outflow[n]),
            Resolved::Pipe(p, i) => trace.push(j, h[p][i], q[p][i]),
        }
    }
}
