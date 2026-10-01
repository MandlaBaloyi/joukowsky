//! V-A14d: Ritter's dam-break onto a dry bed (1892).
//!
//! Still water h0 = 1 m deep behind a dam in a flat, frictionless flume;
//! dry bed beyond it. The dam vanishes at t = 0. With x measured from the
//! dam and c0 = sqrt(g h0):
//!
//! ```text
//!   h = h0                          x <= -c0 t
//!   h = (2 c0 - x/t)^2 / (9 g)     -c0 t < x < 2 c0 t
//!   h = 0                           x >= 2 c0 t   (the front)
//! ```
//!
//! At the dam itself the depth is always 4/9 h0. The front - water racing
//! over a dry bed - is the hardest part for a solver: depths go to zero.

use jk_channels::{ChannelOptions, Downstream, Initial, Upstream, centres, simulate};
use jk_kernel::{Length, Time};
use jk_tests::{check, check_close};

use crate::flume;

#[test]
fn va14_a_dam_break_onto_a_dry_bed_follows_ritter() {
    let reach = flume(2000.0, 1.0);
    let (h0, dam, t) = (1.0, 1000.0, 50.0);
    let g = reach.gravity.value();
    let c0 = (g * h0).sqrt();
    let cells = 4000;
    let run = simulate(
        &reach,
        &Upstream::Wall,
        &Downstream::FreeOutfall,
        &Initial::Dam {
            at: Length::new(dam),
            upstream: Length::new(h0),
            downstream: Length::ZERO,
        },
        &[],
        &ChannelOptions {
            duration: Time::new(t),
            cells,
            courant: 0.9,
            record_every: Time::new(t),
        },
    )
    .unwrap();
    let ritter = |x: f64| {
        let xi = (x - dam) / t;
        if xi <= -c0 {
            h0
        } else if xi >= 2.0 * c0 {
            0.0
        } else {
            (2.0 * c0 - xi).powi(2) / (9.0 * g)
        }
    };
    let xs = centres(&reach, cells);
    let dx = xs[1] - xs[0];
    let error: f64 = xs
        .iter()
        .zip(&run.depth)
        .map(|(&x, h)| (h.value() - ritter(x)).abs() * dx)
        .sum();
    let span = 3.0 * c0 * t;
    println!(
        "A 1 m dam breaks onto a dry bed; after {t} s the wave spans {span:.0} m (front at \
         {:.0} m past the dam):",
        2.0 * c0 * t
    );
    check(
        &format!(
            "  depth profile vs Ritter: mean error {:.2}% of h0 over the wave",
            100.0 * error / (h0 * span)
        ),
        error / (h0 * span) < 0.01,
    );
    let at_dam = xs.iter().position(|&x| x >= dam).unwrap();
    check_close(
        "  depth at the dam: 4/9 h0 [m]",
        f64::midpoint(run.depth[at_dam - 1].value(), run.depth[at_dam].value()),
        4.0 / 9.0 * h0,
        0.01,
    );
    // Where the advancing water is d deep: Ritter puts it at
    // x/t = 2 c0 - sqrt(9 g d).
    let edge = |d: f64| {
        xs.iter()
            .zip(&run.depth)
            .filter(|(_, h)| h.value() > d)
            .map(|(&x, _)| x)
            .fold(0.0, f64::max)
            - dam
    };
    for d in [0.05, 0.01] {
        check_close(
            &format!(
                "  where the advancing water is {:.0} cm deep [m past the dam]",
                d * 100.0
            ),
            edge(d),
            (2.0 * c0 - (9.0 * g * d).sqrt()) * t,
            0.03,
        );
    }
    println!(
        "  (the thinnest edge lags: 1 mm deep at {:.0} m, Ritter {:.0} m - a first-order scheme \
         smears a vanishing depth)",
        edge(0.001),
        (2.0 * c0 - (9.0 * g * 0.001).sqrt()) * t
    );
    check(
        "  and no depth anywhere is negative",
        run.depth.iter().all(|h| h.value() >= 0.0),
    );
}
