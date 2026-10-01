//! Refills through the live monitor (V-S6): the drained main of
//! `tests/common/refill.rs`, its logger sending 1 s chunks. The supply
//! watch runs beside the front detector at the sensor; fronts after supply
//! came back must come out as REFILL warnings - never as bursts.

use jk_application::{Level, SensorSite, Site};
use jk_classification::Context;
use jk_classification::EventClass;
use jk_hydraulics::{Network, Node};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY, Time};
use jk_tests::refill::{A, D, DT, F, H, L, at_the_end, refill_with, vented};
use jk_tests::{check, pipe};

use crate::{live, record};

/// The drained main as the engine knows it: supply reservoir -> 500 m ->
/// closed end, one logger at the closed end.
fn site() -> Site {
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir { head: Head::new(H) });
    let end = net.add_node(Node::Junction { demand: Flow::ZERO });
    net.add_pipe(r, end, pipe(L, D, A, F)).unwrap();
    Site {
        network: net,
        sensors: vec![SensorSite {
            node: end,
            name: "S1 closed end".into(),
            clock: None,
            uncalibrated_clock: Time::new(1e-6),
            range: None,
        }],
        pipe_names: vec!["main".into()],
        context: Context::unknown(),
        schedule: None,
    }
}

#[test]
fn vs6_live_refills_come_out_as_refill_warnings_not_bursts() {
    for mm in [5.0, 20.0, 100.0] {
        println!("\nSupply returns to the drained main; {mm} mm air valve at the far end:");
        let truth = at_the_end(&refill_with(vented(mm), 300.0));
        let issued = live(&site(), &record(&[truth], DT, 21), 1.0, &[], None);
        check("  at least one alert", !issued.is_empty());
        check(
            "  every alert is a REFILL",
            issued.iter().all(|i| i.alert.class == EventClass::Refill),
        );
        check(
            "  at WARNING level",
            issued.iter().all(|i| i.alert.level == Level::Warning),
        );
    }
}
