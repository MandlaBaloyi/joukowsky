//! V-A16: refilling a pipe after a supply cut - the rigid-column model
//! (`crates/domain/hydraulics/src/refill.rs`; `docs/02_physics_of_transients/
//! refilling_and_air_pockets.txt`).
//!
//! A 500 m DN200 pipe, closed at its far end, is empty but for 50 m of
//! water at the entrance when the supply (30 m of head) returns.
//!
//! ```text
//!   no air in the way:   v^2 = 2 g H (1 - x0/x)                  (frictionless)
//!                        v^2 = (2 g H D / f)(1 - e^(-f (x-x0)/D)) / x   (friction)
//!   sealed air pocket:   the column stops where the supply's work equals
//!                        the work done on the air, W(V) = rho g H A (x - x0)
//!   the slam:            dH = a v / g
//! ```

use jk_hydraulics::boundary::pump::WATER_DENSITY;
use jk_hydraulics::refill::{Air, Refill, Vent, refill};
use jk_kernel::{Head, Length, Pressure, STANDARD_GRAVITY, Time, WaveSpeed};
use jk_tests::{check, check_close};

const L: f64 = 500.0;
const D: f64 = 0.2;
const H: f64 = 30.0;
const X0: f64 = 50.0;
const P_ATM: f64 = 101_325.0;

fn pipe(air: Air, friction: f64) -> Refill {
    Refill {
        length: Length::new(L),
        diameter: Length::new(D),
        friction,
        wave_speed: WaveSpeed::new(1000.0),
        rise: Length::ZERO,
        entrance_loss: 0.0,
        supply: Head::new(H),
        start: Length::new(X0),
        atmosphere: Pressure::new(P_ATM),
        air,
        gravity: STANDARD_GRAVITY,
    }
}

#[test]
fn va16_with_no_air_in_the_way_the_column_races_in_as_theory_says_and_slams() {
    let g = STANDARD_GRAVITY.value();
    for friction in [0.0, 0.02] {
        let run = refill(&pipe(Air::Free, friction), Time::new(200.0), Time::new(0.5)).unwrap();
        let theory = |x: f64| {
            if friction == 0.0 {
                (2.0 * g * H * (1.0 - X0 / x)).sqrt()
            } else {
                (2.0 * g * H * D / friction * (1.0 - (-friction * (x - X0) / D).exp()) / x).sqrt()
            }
        };
        println!("Supply returns (30 m) into an empty 500 m pipe, no air cushion, f = {friction}:");
        let mut worst: f64 = 0.0;
        for (x, v) in run.front.iter().zip(&run.velocity).skip(2) {
            let (x, v) = (x.value(), v.value());
            if x < L {
                worst = worst.max((v - theory(x)).abs() / theory(x));
            }
        }
        check(
            &format!("  column speed along the way vs the closed form (worst {worst:.1e})"),
            worst < 1e-6,
        );
        let slam = run.slam.expect("it reaches the end");
        check_close(
            &format!(
                "  reaches the end at {:.1} s with speed [m/s]",
                slam.at.value()
            ),
            slam.velocity.value(),
            theory(L),
            1e-6,
        );
        check_close(
            "  and slams: a v / g [m]",
            slam.surge.value(),
            1000.0 * slam.velocity.value() / g,
            1e-12,
        );
    }
}

/// Work done on a sealed pocket squeezed from V0 to V (gauge, per the
/// momentum equation's `p - p_atm`) [J].
fn work(v: f64, v0: f64, n: f64) -> f64 {
    let p = P_ATM * (v0 / v).powf(n);
    if (n - 1.0).abs() < 1e-12 {
        P_ATM * v0 * (v0 / v).ln() - P_ATM * (v0 - v)
    } else {
        (p * v - P_ATM * v0) / (n - 1.0) - P_ATM * (v0 - v)
    }
}

#[test]
fn va16_a_sealed_air_pocket_stops_the_column_where_the_energy_says() {
    let g = STANDARD_GRAVITY.value();
    let a = std::f64::consts::PI * 0.25 * D * D;
    let v0 = a * (L - X0);
    for n in [1.4, 1.0] {
        let run = refill(
            &pipe(
                Air::Pocket {
                    polytropic: n,
                    vent: None,
                },
                0.0,
            ),
            Time::new(60.0),
            Time::new(0.001),
        )
        .unwrap();
        // The peak: bisection on W(V(x)) = rho g H A (x - x0).
        let balance = |x: f64| work(a * (L - x), v0, n) - WATER_DENSITY * g * H * a * (x - X0);
        let (mut lo, mut hi) = (X0 + 1e-9, L - 1e-9);
        for _ in 0..200 {
            let mid = f64::midpoint(lo, hi);
            if balance(mid) < 0.0 {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        let x_peak = f64::midpoint(lo, hi);
        let p_peak = P_ATM * (v0 / (a * (L - x_peak))).powf(n);
        let head_peak = (p_peak - P_ATM) / (WATER_DENSITY * g);
        println!(
            "Sealed pocket, n = {n}: the column stops {:.1} m from the end",
            L - x_peak
        );
        check("  it never reaches the end", run.slam.is_none());
        check_close(
            "  peak air pressure vs the energy balance [m of water]",
            run.peak_air_head.value(),
            head_peak,
            1e-4,
        );
        println!(
            "  = {:.1} times the supply's 30 m",
            run.peak_air_head.value() / H
        );
        // The energy books balance all along.
        let scale = WATER_DENSITY * g * H * a * (L - X0);
        let worst = run
            .front
            .iter()
            .zip(&run.velocity)
            .map(|(x, v)| {
                let (x, v) = (x.value(), v.value());
                let kinetic = 0.5 * WATER_DENSITY * a * x * v * v;
                (kinetic + work(a * (L - x), v0, n) - WATER_DENSITY * g * H * a * (x - X0)).abs()
            })
            .fold(0.0, f64::max)
            / scale;
        check(
            &format!("  supply's work = motion + work on the air, all along (worst {worst:.1e})"),
            worst < 1e-6,
        );
    }
}

#[test]
fn va16_the_air_valve_size_decides_between_a_cushion_and_a_slam() {
    let sealed = refill(
        &pipe(
            Air::Pocket {
                polytropic: 1.4,
                vent: None,
            },
            0.02,
        ),
        Time::new(300.0),
        Time::new(0.01),
    )
    .unwrap();
    let free = refill(&pipe(Air::Free, 0.02), Time::new(200.0), Time::new(0.5)).unwrap();
    let free_slam = free.slam.unwrap().surge.value();
    println!(
        "Refilling with friction (f = 0.02), adiabatic air; sealed pocket peak {:.1} m, no \
         air at all slams at {free_slam:.0} m:",
        sealed.peak_air_head.value()
    );
    println!("  air valve   peak air   slam at the end   worst at the end");
    let mut worst_of_all: f64 = 0.0;
    let mut results = Vec::new();
    for mm in [0.5, 2.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0, 100.0] {
        let run = refill(
            &pipe(
                Air::Pocket {
                    polytropic: 1.4,
                    vent: Some(Vent {
                        diameter: Length::new(mm / 1000.0),
                        discharge: 0.6,
                    }),
                },
                0.02,
            ),
            Time::new(300.0),
            Time::new(0.01),
        )
        .unwrap();
        let slam = run.slam.map_or(0.0, |s| s.surge.value());
        let worst = run.peak_air_head.value().max(slam);
        worst_of_all = worst_of_all.max(worst);
        results.push((mm, run.peak_air_head.value(), slam));
        println!(
            "  {mm:6.1} mm   {:7.1} m   {:>12}   {worst:8.1} m",
            run.peak_air_head.value(),
            if slam > 0.0 {
                format!("{slam:.0} m")
            } else {
                "none".to_string()
            }
        );
    }
    check_close(
        "  a pin-hole valve behaves like a sealed pocket [m]",
        results[0].1,
        sealed.peak_air_head.value(),
        0.05,
    );
    check(
        "  a large valve lets the air out and the column slams almost as with no air",
        results.last().is_some_and(|r| r.2 > 0.8 * free_slam),
    );
    check(
        &format!(
            "  the worst valve gives {:.0}x the sealed pocket's peak - letting air out the wrong \
             way is worse than trapping it (as Zhou et al. 2002 found)",
            worst_of_all / sealed.peak_air_head.value()
        ),
        worst_of_all > 2.0 * sealed.peak_air_head.value(),
    );
}
