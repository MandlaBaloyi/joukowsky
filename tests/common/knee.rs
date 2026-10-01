//! The pump station with a knee (shared by the hydraulics and pipeline
//! tests): sump 10 m -> pump -> 1000 m up to a knee 50 m high -> 1000 m on
//! to a reservoir at 100 m (ground 95 m), DN300, a = 1000 m/s, f = 0.018.
//! When the pump trips the main keeps moving away from it and the pressure
//! falls lowest at the knee - where the column separates if the pump stops
//! fast (a light flywheel).

use jk_hydraulics::vapour::FreeGas;
use jk_hydraulics::{Network, Node, NodeId, Pump, PumpCurve, thorley_inertia};
use jk_kernel::{Flow, Head, Length, STANDARD_GRAVITY, Time};

use crate::pipe;

/// The knee's ground elevation [m].
pub const KNEE: f64 = 50.0;

/// The station: the pump trips at `trip` s, its inertia `inertia_share` x
/// the Thorley estimate; water boils at `floor` (gauge) if given, carrying
/// `gas`. Returns the network, the pump node and the knee node.
///
/// # Panics
/// Never for the fixed station (a test bug otherwise).
#[must_use]
pub fn station(
    trip: f64,
    inertia_share: f64,
    floor: Option<f64>,
    gas: Option<FreeGas>,
) -> (Network, NodeId, NodeId) {
    let power = 998.2 * STANDARD_GRAVITY.value() * 0.10 * 120.0 / 0.8 / 1000.0;
    let pump = Pump {
        curve: PumpCurve::from_points(
            Head::new(140.0),
            (Flow::new(0.10), Head::new(120.0)),
            (Flow::new(0.15), Head::new(90.0)),
        )
        .expect("a falling curve"),
        rated_flow: Flow::new(0.10),
        rated_efficiency: 0.8,
        rated_speed_rpm: 1450.0,
        inertia: inertia_share * thorley_inertia(power, 1450.0),
    };
    let mut net = Network::new(STANDARD_GRAVITY);
    let sump = net.add_node(Node::Reservoir {
        head: Head::new(10.0),
    });
    let p = net.add_node(Node::Pump {
        pump,
        trip: Some(Time::new(trip)),
    });
    let knee = net.add_node(Node::Junction { demand: Flow::ZERO });
    let top = net.add_node(Node::Reservoir {
        head: Head::new(100.0),
    });
    let main = || pipe(1000.0, 0.3, 1000.0, 0.018);
    net.add_pipe(sump, p, pipe(20.0, 0.3, 1000.0, 0.018))
        .expect("valid");
    net.add_pipe(p, knee, main()).expect("valid");
    net.add_pipe(knee, top, main()).expect("valid");
    net.set_elevation(knee, Length::new(KNEE)).expect("valid");
    net.set_elevation(top, Length::new(95.0)).expect("valid");
    net.set_vapour_floor(floor.map(Head::new)).expect("valid");
    net.set_free_gas(gas).expect("valid");
    (net, p, knee)
}
