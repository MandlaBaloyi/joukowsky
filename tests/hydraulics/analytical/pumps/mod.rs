//! Pumps: normal running, and the run-down after a power failure.
//!
//! Code in `crates/domain/hydraulics/src/boundary/pump.rs` (the pump) and
//! the steady and network solvers that use it. Every test uses the same
//! pump station, pumping from a sump up a rising main:
//!
//! ```text
//!   sump (10 m) --20 m DN300-- PUMP --2000 m DN300 rising main-- top reservoir
//! ```
//!
//! Pump: shut-off head 80 m; rated 100 l/s at 65 m, 80% efficient,
//! 1450 rpm; run-out 150 l/s at 45 m. Rotating inertia from Thorley's
//! formulas for its rated power (about 80 kW).

mod inertia;
mod operating_point;
mod run_down;
mod trip;

use jk_hydraulics::{Network, Node, NodeId, PipeId, Pump, PumpCurve, thorley_inertia};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::pipe;

/// Rated shaft power [kW]: rho g Q H / eta at the rated point.
#[must_use]
pub fn rated_power_kw() -> f64 {
    998.2 * STANDARD_GRAVITY.value() * 0.10 * 65.0 / 0.8 / 1000.0
}

/// The station pump.
///
/// # Panics
/// Never: the datasheet points are valid.
#[must_use]
pub fn station_pump() -> Pump {
    Pump {
        curve: PumpCurve::from_points(
            Head::new(80.0),
            (Flow::new(0.10), Head::new(65.0)),
            (Flow::new(0.15), Head::new(45.0)),
        )
        .expect("valid datasheet points"),
        rated_flow: Flow::new(0.10),
        rated_efficiency: 0.8,
        rated_speed_rpm: 1450.0,
        inertia: thorley_inertia(rated_power_kw(), 1450.0),
    }
}

/// The pump station: sump at 10 m, top reservoir at `top` m, power failing
/// at `trip` seconds (if any), Darcy friction 0.018. Returns the network,
/// the pump node and the rising main.
#[must_use]
pub fn station(top: f64, trip: Option<f64>) -> (Network, NodeId, PipeId) {
    station_with_friction(top, trip, 0.018)
}

/// [`station`] with a chosen friction factor in both pipes.
///
/// # Panics
/// Never: the network is valid.
#[must_use]
pub fn station_with_friction(
    top: f64,
    trip: Option<f64>,
    friction: f64,
) -> (Network, NodeId, PipeId) {
    let mut net = Network::new(STANDARD_GRAVITY);
    let sump = net.add_node(Node::Reservoir {
        head: Head::new(10.0),
    });
    let pump = net.add_node(Node::Pump {
        pump: station_pump(),
        trip: trip.map(Time::new),
    });
    let high = net.add_node(Node::Reservoir {
        head: Head::new(top),
    });
    net.add_pipe(sump, pump, pipe(20.0, 0.3, 1000.0, friction))
        .unwrap();
    let main = net
        .add_pipe(pump, high, pipe(2000.0, 0.3, 1000.0, friction))
        .unwrap();
    (net, pump, main)
}
