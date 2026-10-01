//! V-A14b: the backwater curve (gradually varied flow, M1 profile).
//!
//! A weir or a gate holds the canal's end above normal depth; upstream of
//! it the water surface curves down to normal depth over kilometres. The
//! textbook way to draw that curve is the direct-step method: from the
//! known depth at the end, step the depth down a little at a time and find
//! how far upstream each depth lies,
//!
//! ```text
//!   dx = dE / (S0 - Sf_mean),    E = h + Q^2 / (2 g A^2)
//! ```
//!
//! The solver knows nothing of this: it runs the full equations until the
//! flow stops changing. Its steady profile must lie on the curve.

use jk_channels::{ChannelOptions, Downstream, Initial, Upstream, centres, simulate};
use jk_kernel::{Flow, Length, Time};
use jk_tests::check;

use crate::canal;

const Q: f64 = 10.0;

#[test]
fn va14_a_steady_backwater_profile_lies_on_the_direct_step_curve() {
    let reach = canal();
    let s = reach.section;
    let g = reach.gravity.value();
    let hn = reach.normal_depth(Flow::new(Q)).unwrap().value();
    let h_end = 1.6 * hn;
    let energy = |h: f64| h + Q * Q / (2.0 * g * s.area(h).powi(2));

    // Direct step from the outlet (x = L) upstream.
    let mut curve = vec![(reach.length.value(), h_end)];
    let (mut x, mut h) = (reach.length.value(), h_end);
    let dh = (h_end - hn) / 2000.0;
    while x > 0.0 && h - dh > hn * 1.001 {
        let h2 = h - dh;
        let sf = f64::midpoint(reach.friction_slope(h, Q), reach.friction_slope(h2, Q));
        x -= (energy(h) - energy(h2)) / (reach.slope - sf);
        h = h2;
        curve.push((x, h));
    }

    let cells = 500;
    let run = simulate(
        &reach,
        &Upstream::Inflow(vec![(Time::ZERO, Flow::new(Q))]),
        Downstream::Depth(Length::new(h_end)),
        &Initial::Uniform(Flow::new(Q)),
        &[],
        &ChannelOptions {
            duration: Time::new(24.0 * 3600.0),
            cells,
            courant: 0.9,
            record_every: Time::new(3600.0),
        },
    )
    .unwrap();
    let xs = centres(&reach, cells);
    let solver_at = |x: f64| {
        let k = xs
            .iter()
            .position(|&c| c >= x)
            .unwrap_or(xs.len() - 1)
            .max(1);
        let w = ((x - xs[k - 1]) / (xs[k] - xs[k - 1])).clamp(0.0, 1.0);
        run.depth[k - 1].value() * (1.0 - w) + run.depth[k].value() * w
    };
    println!(
        "Normal depth {hn:.3} m; the outlet held at {h_end:.3} m. Depth along the canal \
         (solver after 24 h vs direct step):"
    );
    let mut worst: f64 = 0.0;
    for target in [4900.0, 4500.0, 4000.0, 3000.0, 2000.0, 1000.0] {
        let Some(&(xc, hc)) = curve
            .iter()
            .min_by(|a, b| (a.0 - target).abs().total_cmp(&(b.0 - target).abs()))
        else {
            continue;
        };
        let hs = solver_at(xc);
        // Error relative to the whole rise the weir makes (h_end - h_n).
        let err = (hs - hc).abs() / (h_end - hn);
        worst = worst.max(err);
        println!(
            "  x = {xc:6.0} m: solver {hs:.4} m, direct step {hc:.4} m (off by {:.1} mm)",
            1000.0 * (hs - hc).abs()
        );
    }
    check(
        &format!(
            "  the solver's profile lies on the curve (worst {:.2}% of the {:.2} m rise at the weir)",
            100.0 * worst,
            h_end - hn
        ),
        worst < 0.005,
    );
}
