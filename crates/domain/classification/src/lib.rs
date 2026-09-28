//! Classification bounded context: WHAT kind of transient an event was.
//!
//! Implements Stage A of `docs/04_detection_and_localisation/
//! event_classification.txt`: a transparent, rule-based classifier on
//! physical features, needing no training data. Every decision carries its
//! probabilities and the plain-language reasons behind it (principle P5:
//! "why did the engine think this was a burst?"), and the classifier may
//! ABSTAIN rather than guess (principle P3).
//!
//! Version 1 classes: burst, valve closure, pump trip - the events the
//! hydraulics simulator can generate, so every rule is validated. Column
//! separation, refilling, pump start, valve opening and sensor faults join
//! when they can be simulated or measured; Stage B (trained on synthetic
//! data) and Stage C (fine-tuned on field labels) follow.
//!
//! Pure domain code (rule R2). The folders:
//! - [`features`]: physical features measured from one sensor's record.
//! - [`classes`]: the event classes, and outside context (power losses).
//! - [`rules`]: the evidence rules and the classifier.

pub mod classes;
mod error;
pub mod features;
pub mod rules;

pub use classes::{Context, EventClass};
pub use error::ClassificationError;
pub use features::{FeatureConfig, Features, extract};
pub use rules::{Classification, ClassifierConfig, classify};
