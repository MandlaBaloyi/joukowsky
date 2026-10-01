//! Classification tests: what kind of transient an event was.
//!
//! Code under test: `crates/domain/classification`. Every scenario runs the
//! whole chain: the hydraulics solver makes the event, a synthetic 128 S/s
//! logger records it (random phase, 2 cm noise), detection finds it, the
//! features are measured, and the Stage A rules give a verdict with
//! reasons. The scenarios include the one the physics cannot decide
//! without outside help - there the classifier must ABSTAIN.
//!
//! Run only these:  `.\tests\run-tests.ps1 -Only classification`

mod simulation;

use jk_classification::{
    Classification, ClassifierConfig, Context, FeatureConfig, classify, extract,
};
use jk_detection::{Detector, DetectorConfig, Series, SupplyConfig, empty_spells};
use jk_kernel::Time;
use jk_tests::sensor::{Logger, Rng, sample};

/// Logger rate [S/s].
pub const RATE: f64 = 128.0;

/// Puts `lead` seconds of the (exact) steady state in front of a simulated
/// history: the simulator starts valve closures and pump trips at t = 0,
/// the detector needs a quiet stretch first to learn the sensor's noise.
#[must_use]
pub fn with_quiet_lead(truth: &[f64], dt: f64, lead: f64) -> Vec<f64> {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let n = (lead / dt).round() as usize;
    std::iter::repeat_n(truth[0], n)
        .chain(truth.iter().copied())
        .collect()
}

/// Records `truth` (every `dt` s) with a 128 S/s logger, detects the first
/// event, measures its features and classifies it with `context` - plus
/// any supply returns the record itself shows (the supply watch: spells of
/// an empty pipe and when they ended). Prints the verdict and its reasons.
///
/// # Panics
/// If no event is detected (a test bug: every scenario has one).
#[must_use]
pub fn verdict(truth: &[f64], dt: f64, context: &Context, seed: u64) -> Classification {
    let mut rng = Rng::new(seed);
    let logger = Logger {
        rate: RATE,
        phase: rng.uniform() / RATE,
        noise: 0.02,
    };
    let samples = sample(truth, dt, &logger, &mut rng);
    let series = Series {
        samples: &samples,
        start: Time::new(logger.phase),
        interval: Time::new(1.0 / RATE),
    };
    let found = Detector::new(DetectorConfig::default())
        .expect("valid config")
        .run(&series)
        .expect("detection runs");
    let returns: Vec<Time> = empty_spells(&series, &SupplyConfig::default())
        .expect("supply watch runs")
        .iter()
        .filter_map(|s| s.until)
        .collect();
    for t in &returns {
        println!(
            "  supply watch: the pipe stood empty, supply returned at t = {:.2} s",
            t.value()
        );
    }
    let context = &context.clone().with_supply_returns(&returns);
    let event = *found
        .events
        .first()
        .expect("the scenario's event must be detected");
    let features = extract(&series, &event, found.noise, &FeatureConfig::default())
        .expect("enough record around the event");
    println!(
        "  first front: {} at t = {:.2} s, from {:.2} m",
        if event.direction == jk_detection::Direction::Rise {
            "RISE"
        } else {
            "DROP"
        },
        event.arrival.value(),
        features.level_before.value()
    );
    let c = classify(
        &features,
        context,
        event.arrival,
        &ClassifierConfig::default(),
    );
    println!(
        "  verdict: {} ({:.0}% confident){}",
        c.class,
        c.confidence * 100.0,
        if c.class == jk_classification::EventClass::Unknown {
            " - ABSTAINS"
        } else {
            ""
        }
    );
    for (class, p) in &c.probabilities {
        println!("    {class:<14} {:>5.1}%", p * 100.0);
    }
    for r in &c.reasons {
        println!("    because: {r}");
    }
    c
}
