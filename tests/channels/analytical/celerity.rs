//! V-A14c: a small wave on still water travels at sqrt(g h).
//!
//! A 1 cm hump on water 2 m deep, in a flat flume with no friction, splits
//! into two halves running each way at c = sqrt(g h) = 4.43 m/s - minutes
//! to cross a canal, not the milliseconds of a pressure wave in a pipe.

use jk_channels::{ChannelOptions, Downstream, Initial, Upstream, centres, simulate};
use jk_kernel::{Flow, Length, Time};
use jk_tests::check_close;

use crate::flume;

#[test]
fn va14_a_small_wave_travels_at_the_square_root_of_g_h() {
    let reach = flume(2000.0, 5.0);
    let (depth, amp) = (2.0, 0.01);
    let cells = 4000;
    let start: Vec<(Length, Flow)> = centres(&reach, cells)
        .iter()
        .map(|&x| {
            let hump = amp * (-((x - 1000.0) / 10.0).powi(2)).exp();
            (Length::new(depth + hump), Flow::ZERO)
        })
        .collect();
    let run = simulate(
        &reach,
        &Upstream::Wall,
        Downstream::Wall,
        &Initial::Cells(start),
        &[Length::new(1500.0)],
        &ChannelOptions {
            duration: Time::new(140.0),
            cells,
            courant: 0.9,
            record_every: Time::new(0.05),
        },
    )
    .unwrap();
    let (_, depths, _) = &run.gauges[0];
    let (k, peak) = depths
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.value().total_cmp(&b.1.value()))
        .unwrap();
    let arrival = run.times[k].value();
    let c = (reach.gravity.value() * depth).sqrt();
    println!(
        "A 1 cm hump on 2 m of still water; its crest reaches a gauge 500 m away at {arrival:.2} s \
         ({:.2} cm high: each half carries half of it)",
        (peak.value() - depth) * 100.0
    );
    check_close("  speed [m/s] vs sqrt(g h)", 500.0 / arrival, c, 0.005);
}
