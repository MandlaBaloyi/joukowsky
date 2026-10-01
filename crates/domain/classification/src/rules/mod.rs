//! Stage A: transparent rules on physical features
//! (`event_classification.txt` section 3).
//!
//! Each rule adds evidence - a weight in natural-log units - for or against
//! each class, and writes down WHY. The evidence is summed per class and
//! turned into probabilities (softmax, equal priors). If no class reaches
//! the confidence threshold the classifier ABSTAINS: an honest "unknown" is
//! better than a confident mistake (principle P3).
//!
//! - [`evidence`]: the rules and their weights.
//!
//! The weights are physics-informed starting values, not learned ones.
//! Stage B will fit them on large simulated datasets, Stage C on confirmed
//! field events. They are chosen so that the one case the physics cannot
//! separate without outside help - a SLOW-opening burst versus a pump trip
//! when power is not monitored - ends in abstention.

pub mod evidence;

use jk_kernel::{Head, Time};

use crate::classes::{Context, EventClass};
use crate::features::Features;

/// Thresholds of the rules.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClassifierConfig {
    /// Fronts rising faster than this are "sharp" (sudden opening).
    pub sharp_front: Time,
    /// Fronts rising slower than this are "drawn out" (run-down).
    pub slow_front: Time,
    /// A power loss up to this long before the arrival counts.
    pub power_window: Time,
    /// A pipe reading at or below this pressure head before the front was
    /// empty or unpressurised (a drained pipe reads about 0, or below in
    /// a vacuum).
    pub empty_below: Head,
    /// A front this long after supply returned to an empty pipe belongs
    /// to the refill (a long main fills for minutes).
    pub refill_window: Time,
    /// Below this confidence the classifier abstains.
    pub abstain_below: f64,
}

impl Default for ClassifierConfig {
    fn default() -> Self {
        Self {
            sharp_front: Time::new(0.03),
            slow_front: Time::new(0.1),
            power_window: Time::new(3.0),
            empty_below: Head::new(2.0),
            refill_window: Time::new(900.0),
            abstain_below: 0.7,
        }
    }
}

/// The verdict on one event, with its evidence.
#[derive(Debug, Clone, PartialEq)]
pub struct Classification {
    /// The class - `Unknown` if the classifier abstained.
    pub class: EventClass,
    /// Probability of the most likely class.
    pub confidence: f64,
    /// Probability of every candidate class, most likely first.
    pub probabilities: Vec<(EventClass, f64)>,
    /// Plain-language reasons, one per rule that fired.
    pub reasons: Vec<String>,
    /// The features the verdict was based on.
    pub features: Features,
}

/// Classifies an event from its `features`, the outside `context`, and its
/// `arrival` time.
#[must_use]
pub fn classify(
    features: &Features,
    context: &Context,
    arrival: Time,
    config: &ClassifierConfig,
) -> Classification {
    let (scores, reasons) = evidence::weigh(features, context, arrival, config);
    let top = scores
        .iter()
        .map(|(_, s)| *s)
        .fold(f64::NEG_INFINITY, f64::max);
    let total: f64 = scores.iter().map(|(_, s)| (s - top).exp()).sum();
    let mut probabilities: Vec<(EventClass, f64)> = scores
        .iter()
        .map(|(c, s)| (*c, (s - top).exp() / total))
        .collect();
    probabilities.sort_by(|a, b| b.1.total_cmp(&a.1));
    let (best, confidence) = probabilities[0];
    Classification {
        class: if confidence >= config.abstain_below {
            best
        } else {
            EventClass::Unknown
        },
        confidence,
        probabilities,
        reasons,
        features: *features,
    }
}
