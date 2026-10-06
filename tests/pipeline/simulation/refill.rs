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
        utc_offset: None,
    }
}

#[test]
fn vs6_live_refills_come_out_as_refill_warnings_not_bursts() {
    // FN-052: with a small air valve the trapped air cushions the refill -
    // the pressure rises gently to about the supply head, no surge - and
    // the engine reports it on the supply-cut alert ("supply RETURNED ...
    // the pipes refill"); a REFILL alert there came only from a rise at the
    // edge of detection, so it is allowed, not required. With larger valves
    // the column slams: then a REFILL warning must report the slam's peak
    // and the column separation that follows.
    for mm in [5.0, 20.0, 100.0] {
        println!(
            "
Supply returns to the drained main; {mm} mm air valve at the far end:"
        );
        let run = refill_with(vented(mm), 300.0);
        let slammed = run.slam.is_some();
        let truth = at_the_end(&run);
        let peak_truth = truth.iter().copied().fold(f64::MIN, f64::max);
        println!(
            "  truth: {}, highest pressure {peak_truth:.1} m",
            if slammed {
                "the column SLAMS into the closed end"
            } else {
                "the air cushions it - no slam"
            }
        );
        let issued = live(&site(), &record(&[truth], DT, 21), 1.0, &[], None);
        let transient: Vec<_> = issued
            .iter()
            .filter(|i| i.alert.class != EventClass::SupplyCut)
            .collect();
        check(
            "  every alert on the refill a REFILL, never a burst",
            transient
                .iter()
                .all(|i| i.alert.class == EventClass::Refill),
        );
        check(
            "  at WARNING level",
            transient.iter().all(|i| i.alert.level == Level::Warning),
        );
        check(
            "  and the empty pipe itself reported as a SUPPLY CUT, revised when it came back",
            issued.iter().any(|i| {
                i.alert.class == EventClass::SupplyCut
                    && i.alert
                        .reasons
                        .iter()
                        .any(|r| r.contains("supply RETURNED"))
            }),
        );
        // The highest pressure any alert reports (a rise, or a peak after).
        let reported = transient
            .iter()
            .flat_map(|i| i.alert.reasons.iter())
            .filter_map(|r| {
                ["ROSE by ", "peaked at "].iter().find_map(|key| {
                    let at = r.find(key)? + key.len();
                    r[at..].split(' ').next()?.parse::<f64>().ok()
                })
            })
            .fold(0.0, f64::max);
        if slammed {
            println!("  the alerts report up to {reported:.1} m");
            check(
                "  the SLAM reported: its peak within 5% of the truth",
                (reported - peak_truth).abs() <= 0.05 * peak_truth,
            );
            check(
                "  and the column separation after it",
                transient.iter().any(|i| {
                    i.alert
                        .reasons
                        .iter()
                        .any(|r| r.contains("COLUMN SEPARATION"))
                }),
            );
        } else {
            check(
                "  no surge invented: nothing reported above twice the supply head",
                reported <= 2.0 * H,
            );
        }
    }
}
