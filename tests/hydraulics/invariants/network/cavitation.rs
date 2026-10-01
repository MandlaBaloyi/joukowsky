//! Column separation after a pump trip (S7; `docs/02_physics_of_transients/
//! column_separation_and_intrusion.txt` section 5): the pump stops, the
//! rising main keeps moving away from it, and the pressure falls - lowest
//! where the main runs high. The field case: a main climbing 50 m to a
//! knee, then on to the reservoir. With a light flywheel the pump stops
//! fast; in linear theory the knee falls far below what water can hold.
//! With the vapour floor the water boils at the knee, a cavity opens and
//! collapses when the column comes back.
//!
//! Laws checked: no head below its own floor; the cavity opens and closes;
//! the run stays finite. (The exact numbers are V-A17's.)

use jk_hydraulics::vapour::FreeGas;
use jk_hydraulics::{GridOptions, InitialCondition, NetworkRun, Probe, SteadyOptions};
use jk_kernel::{Head, Time};
use jk_tests::check;
use jk_tests::knee::{KNEE, station};

const FLOOR: f64 = -10.0;

/// The knee station (`tests/common/knee.rs`), the pump tripping at 0.01 s
/// with 1/20 of the usual flywheel. Probes: the pump (discharge head), the
/// knee.
fn pump_trip(floor: Option<f64>, gas: Option<FreeGas>) -> NetworkRun {
    let (net, p, knee) = station(0.01, 0.05, floor, gas);
    net.simulate(
        &InitialCondition::Steady(SteadyOptions::default()),
        Time::new(30.0),
        &[Probe::Node(p), Probe::Node(knee)],
        &GridOptions {
            max_wave_speed_adjustment: 0.01,
            min_reaches: 4,
        },
    )
    .unwrap()
}

fn heads(r: &NetworkRun, j: usize) -> Vec<f64> {
    r.trace.head(j).iter().map(|h| h.value()).collect()
}

#[test]
fn after_a_pump_trip_the_column_separates_at_the_knee_of_the_rising_main() {
    let lowest = |h: &[f64]| h.iter().copied().fold(f64::INFINITY, f64::min);
    let highest = |h: &[f64]| h.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let linear = pump_trip(None, None);
    let knee_linear = heads(&linear, 1);
    println!(
        "A pump with 1/20 of the usual flywheel trips; its main climbs to a knee 50 m up. Linear \
         theory: the knee's pressure falls to {:.1} m - water cannot hold that.",
        lowest(&knee_linear) - KNEE
    );
    check(
        "  (linear theory goes below the floor at the knee)",
        lowest(&knee_linear) < KNEE + FLOOR,
    );
    let run = pump_trip(Some(FLOOR), None);
    let knee = heads(&run, 1);
    let volume = &run.cavity_volumes[1];
    let peak = volume.iter().copied().fold(0.0, f64::max);
    let opens = volume.iter().position(|&v| v > 0.0);
    let closes = opens.and_then(|k| volume[k..].iter().position(|&v| v <= 0.0).map(|j| k + j));
    let dt = run.trace.time_step().value();
    #[allow(clippy::cast_precision_loss)]
    let time = |k: Option<usize>| k.map_or(f64::NAN, |k| k as f64 * dt);
    println!(
        "With the vapour floor (-10 m): a cavity opens at the knee at {:.2} s, grows to {:.0} \
         litres and collapses at {:.2} s.",
        time(opens),
        peak * 1e3,
        time(closes)
    );
    check(
        &format!(
            "  the knee's pressure never below the floor (lowest {:.3} m)",
            lowest(&knee) - KNEE
        ),
        lowest(&knee) >= KNEE + FLOOR - 1e-9,
    );
    check(
        &format!(
            "  nor the pump's discharge (lowest {:.3} m)",
            lowest(&heads(&run, 0))
        ),
        lowest(&heads(&run, 0)) >= FLOOR - 1e-9,
    );
    check("  the cavity opens and closes again", closes.is_some());
    check(
        "  every head finite",
        knee.iter().chain(&heads(&run, 0)).all(|h| h.is_finite()),
    );
    println!(
        "  highest pressure at the knee after the collapse: {:.1} m (linear theory: {:.1} m) - \
         the collapse surge, why column separation breaks pipes",
        highest(&knee[closes.unwrap_or(0)..]) - KNEE,
        highest(&knee_linear) - KNEE
    );
}

#[test]
fn free_gas_does_not_take_away_the_doubled_surge_at_the_knee() {
    let highest = |r: &NetworkRun| heads(r, 1).iter().copied().fold(f64::MIN, f64::max) - KNEE;
    let lowest = |r: &NetworkRun| heads(r, 1).iter().copied().fold(f64::MAX, f64::min) - KNEE;
    let linear = highest(&pump_trip(None, None));
    let vapour = highest(&pump_trip(Some(FLOOR), None));
    println!(
        "The pump trip at the knee: how high the pressure goes after the column separates,          against how much free gas the water carries (linear theory: {linear:.1} m)."
    );
    println!("  free gas alpha0      highest   lowest   [m of pressure at the knee]");
    println!(
        "  none (vapour)       {vapour:8.1}   {:6.1}",
        lowest(&pump_trip(Some(FLOOR), None))
    );
    let mut peaks = Vec::new();
    for alpha0 in [1e-7, 1e-6, 1e-5, 1e-4] {
        let run = pump_trip(
            Some(FLOOR),
            Some(FreeGas {
                void_fraction: alpha0,
                at_head: Head::ZERO,
            }),
        );
        println!(
            "  {alpha0:15.0e}     {:8.1}   {:6.1}",
            highest(&run),
            lowest(&run)
        );
        check("    never below the floor", lowest(&run) >= FLOOR - 1e-9);
        peaks.push(highest(&run));
    }
    check(
        "  a trace of gas (1e-7) behaves like the vapour cavity (within 10%)",
        (peaks[0] - vapour).abs() <= 0.1 * vapour,
    );
    let least = peaks.iter().copied().fold(f64::MAX, f64::min);
    check(
        &format!(
            "  at every gas content the collapse still reaches about twice linear theory's worst ({:.2}x at the least)",
            least / linear
        ),
        least > 1.9 * linear,
    );
}
