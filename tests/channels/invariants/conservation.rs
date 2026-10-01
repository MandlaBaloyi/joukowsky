//! Water is neither made nor lost (principle P1).
//!
//! The finite-volume form moves water only from one cell to the next, so
//! the water in the reach changes by exactly what crosses its ends - to
//! rounding, whatever the flow does inside.

use jk_channels::{ChannelOptions, Downstream, Initial, Upstream, simulate};
use jk_kernel::{Flow, Length, Time};
use jk_tests::check;

use crate::{canal, flume};

#[test]
fn a_closed_flume_keeps_its_water_through_a_dam_break_and_its_echoes() {
    let reach = flume(500.0, 2.0);
    let run = simulate(
        &reach,
        &Upstream::Wall,
        Downstream::Wall,
        &Initial::Dam {
            at: Length::new(200.0),
            upstream: Length::new(1.5),
            downstream: Length::new(0.2),
        },
        &[],
        &ChannelOptions {
            duration: Time::new(600.0),
            cells: 500,
            courant: 0.9,
            record_every: Time::new(60.0),
        },
    )
    .unwrap();
    let (v0, v1) = run.volume;
    let drift = (v1 - v0).abs() / v0;
    println!(
        "Closed flume, a dam breaks and the surge rings for 10 minutes ({} steps): water at \
         the start {v0:.6} m^3, at the end {v1:.6} m^3",
        run.steps
    );
    check(
        &format!("  the same water (relative change {drift:.1e})"),
        drift < 1e-12,
    );
}

#[test]
fn an_open_canal_changes_its_water_by_exactly_what_crosses_its_ends() {
    let reach = canal();
    let run = simulate(
        &reach,
        &Upstream::Inflow(vec![
            (Time::ZERO, Flow::new(5.0)),
            (Time::new(1800.0), Flow::new(15.0)),
            (Time::new(3600.0), Flow::new(8.0)),
        ]),
        Downstream::FreeOutfall,
        &Initial::Uniform(Flow::new(5.0)),
        &[],
        &ChannelOptions {
            duration: Time::new(3.0 * 3600.0),
            cells: 250,
            courant: 0.9,
            record_every: Time::new(600.0),
        },
    )
    .unwrap();
    let (v0, v1) = run.volume;
    let (inn, out) = run.exchanged;
    let mismatch = ((v1 - v0) - (inn - out)).abs() / inn;
    println!(
        "Canal, inflow rising 5 -> 15 -> 8 m^3/s over 3 h: in {inn:.1} m^3, out {out:.1} m^3, \
         stored {:.1} m^3 more",
        v1 - v0
    );
    check(
        &format!("  stored = in - out (mismatch {mismatch:.1e} of the inflow)"),
        mismatch < 1e-10,
    );
}
