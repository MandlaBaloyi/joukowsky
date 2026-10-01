//! V-A14a: uniform flow at normal depth (Manning).
//!
//! In a long straight canal, flow settles at the depth where friction just
//! balances the pull of the sloping bed - the normal depth `h_n`:
//!
//! ```text
//!   Q = (1/n) A R^(2/3) sqrt(S0)
//! ```
//!
//! Started there, the solver must stay there; started elsewhere, it must
//! go there.

use jk_channels::{ChannelOptions, Downstream, Initial, Upstream, simulate};
use jk_kernel::{Flow, Length, Time};
use jk_tests::{check, check_close};

use crate::canal;

const Q: f64 = 10.0;

#[test]
fn va14_normal_depth_satisfies_manning_and_uniform_flow_stays_uniform() {
    let reach = canal();
    let hn = reach.normal_depth(Flow::new(Q)).unwrap().value();
    let s = reach.section;
    let (a, r) = (s.area(hn), s.area(hn) / s.perimeter(hn));
    println!("Canal 3 m wide, sides 1:1.5, bed 0.5 m/km, n = 0.015, carrying 10 m^3/s:");
    check_close(
        "  normal depth: Manning's flow at it [m^3/s]",
        a * r.powf(2.0 / 3.0) * reach.slope.sqrt() / reach.manning,
        Q,
        1e-12,
    );
    println!(
        "  normal depth {hn:.4} m; critical depth {:.4} m (subcritical: slower than a small wave)",
        reach.critical_depth(Flow::new(Q)).unwrap().value()
    );
    let run = simulate(
        &reach,
        &Upstream::Inflow(vec![(Time::ZERO, Flow::new(Q))]),
        Downstream::FreeOutfall,
        &Initial::Uniform(Flow::new(Q)),
        &[],
        &ChannelOptions {
            duration: Time::new(7200.0),
            cells: 250,
            courant: 0.9,
            record_every: Time::new(600.0),
        },
    )
    .unwrap();
    let worst = run
        .depth
        .iter()
        .map(|h| (h.value() - hn).abs())
        .fold(0.0, f64::max);
    check(
        &format!(
            "  started at normal depth, after 2 h ({} steps) every cell is still there \
             (largest change {worst:.1e} m)",
            run.steps
        ),
        worst < 1e-9,
    );
}

#[test]
fn va14_flow_started_elsewhere_settles_at_normal_depth() {
    let reach = canal();
    let hn = reach.normal_depth(Flow::new(Q)).unwrap().value();
    let run = simulate(
        &reach,
        &Upstream::Inflow(vec![(Time::ZERO, Flow::new(Q))]),
        Downstream::Depth(Length::new(hn)),
        &Initial::Still(Length::new(1.0)),
        &[Length::new(2500.0)],
        &ChannelOptions {
            duration: Time::new(12.0 * 3600.0),
            cells: 250,
            courant: 0.9,
            record_every: Time::new(1800.0),
        },
    )
    .unwrap();
    let mid = &run.gauges[0];
    println!("Still water 1 m deep; 10 m^3/s let in upstream, the outlet held at normal depth:");
    for (t, h) in run.times.iter().zip(&mid.1).step_by(4) {
        println!(
            "  {:5.1} h: depth half way {:.4} m",
            t.value() / 3600.0,
            h.value()
        );
    }
    check_close(
        "  after 12 h, half way down [m]",
        mid.1.last().unwrap().value(),
        hn,
        1e-3,
    );
    check_close(
        "  ... carrying the inflow [m^3/s]",
        mid.2.last().unwrap().value(),
        Q,
        1e-3,
    );
}
