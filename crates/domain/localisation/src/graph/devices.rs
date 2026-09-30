//! What a network's devices do to a passing pressure wave (FN-029, gap
//! G-19): pumps, check valves and pressure-reducing valves, as the network
//! stands in its steady state.
//!
//! A device between two pipes is, for a small wave, a linear resistance r
//! between them (`docs/02_physics_of_transients/governing_equations_and_
//! moc.txt`, section 6): a wave arriving along a pipe of impedance Bin
//! passes on as
//!
//! ```text
//!   T = 2 Bout / (Bin + Bout + r)
//! ```
//!
//! - a PRV passing flow, frozen at its opening: r = 2 R Q0 = 2 dH0 / Q0
//!   (dH0 its steady head drop);
//! - a running pump: r = -dH/dQ, the (positive) droop of its curve at its
//!   flow - the pump holds its speed while the wave passes;
//! - an open check valve: r = 0, a straight run;
//! - shut (a check valve or PRV held shut, a pump whose non-return valve is
//!   shut): no wave passes at all - routes through it are cut, as through
//!   reservoirs.
//!
//! The states come from one steady solve of the network. If that cannot be
//! solved (e.g. an end valve without a flow coefficient), every device is
//! taken as a straight run - the behaviour before this module.

use jk_hydraulics::boundary::prv::PrvState;
use jk_hydraulics::joukowsky::junction_transmission;
use jk_hydraulics::{Network, Node, SteadyOptions, SteadySolution};

/// Per node: shut, and the resistance a small wave meets crossing it.
#[derive(Debug, Clone, PartialEq)]
pub struct Devices {
    /// No wave passes (a shut valve, a pump that is not delivering).
    shut: Vec<bool>,
    /// Linear resistance r [s/m^2] (0 for plain nodes).
    resistance: Vec<f64>,
}

impl Devices {
    /// The devices of `network` as its steady state (base demands, every
    /// pump running) leaves them.
    #[must_use]
    pub fn of(network: &Network) -> Self {
        let any = network.nodes().iter().any(|node| {
            matches!(
                node,
                Node::CheckValve | Node::Pump { .. } | Node::PressureReducingValve { .. }
            )
        });
        match network.steady_state(&SteadyOptions::default()) {
            Ok(s) if any => Self::from_steady(network, &s),
            _ => Self::open(network),
        }
    }

    /// Every device open and transparent (a straight run).
    #[must_use]
    pub fn open(network: &Network) -> Self {
        let n = network.nodes().len();
        Self {
            shut: vec![false; n],
            resistance: vec![0.0; n],
        }
    }

    /// The devices as the steady state `s` of `network` leaves them - e.g.
    /// a state from an extended-period run at a given hour
    /// ([`Network::extended_observed`]).
    #[must_use]
    pub fn from_steady(network: &Network, s: &SteadySolution) -> Self {
        let mut d = Self::open(network);
        // The flow leaving a device node: its one downstream pipe.
        let through = |node: usize| {
            network
                .pipes()
                .iter()
                .position(|p| p.from.index() == node)
                .map_or(0.0, |k| s.pipe_flows[k].value())
        };
        for v in s.closed_check_valves.iter().chain(&s.idle_pumps) {
            d.shut[v.index()] = true;
        }
        for (i, node) in network.nodes().iter().enumerate() {
            match node {
                Node::Pump { pump, .. } if !d.shut[i] => {
                    d.resistance[i] = (-pump.curve.slope(through(i), 1.0)).max(0.0);
                }
                Node::PressureReducingValve {
                    setting,
                    open_resistance,
                } => {
                    let state = s.prvs.iter().find(|(v, _)| v.index() == i).map(|p| p.1);
                    let q = through(i);
                    let drop = match state {
                        Some(PrvState::Active) => s.node_heads[i].value() - setting.value(),
                        Some(PrvState::Open) => open_resistance * q * q,
                        _ => f64::NAN,
                    };
                    if q > 0.0 && drop.is_finite() {
                        d.resistance[i] = 2.0 * drop.max(0.0) / q;
                    } else {
                        d.shut[i] = true; // shut, or passing nothing: held shut
                    }
                }
                _ => {}
            }
        }
        d
    }

    /// How many devices are shut.
    #[must_use]
    pub fn shut_count(&self) -> usize {
        self.shut.iter().filter(|s| **s).count()
    }

    /// True if no wave passes node `node`.
    #[must_use]
    pub fn is_shut(&self, node: usize) -> bool {
        self.shut[node]
    }

    /// Fraction of a wave arriving at `node` along a pipe of impedance
    /// `incoming` that goes on, with `all` the impedances of every pipe at
    /// the node (including the incoming one).
    #[must_use]
    pub fn transmission(&self, node: usize, incoming: f64, all: &[f64]) -> f64 {
        if self.shut[node] {
            return 0.0;
        }
        let r = self.resistance[node];
        if r > 0.0 && all.len() == 2 {
            let out = all.iter().sum::<f64>() - incoming;
            2.0 * out / (incoming + out + r)
        } else {
            junction_transmission(incoming, all)
        }
    }
}
