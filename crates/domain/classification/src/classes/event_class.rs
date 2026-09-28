//! The event classes (`event_classification.txt` section 1).

use std::fmt;

/// What kind of transient an event was.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub enum EventClass {
    /// A pipe failed: a new outflow opened, sending a negative front.
    Burst,
    /// A valve shut (seen upstream of it: a positive front).
    ValveClosure,
    /// A pump lost power and ran down: a drawn-out negative front.
    PumpTrip,
    /// The evidence does not single out one class - the classifier
    /// abstains rather than guess (principle P3).
    Unknown,
}

impl EventClass {
    /// The classes the rules choose between (everything except Unknown).
    pub const CANDIDATES: [Self; 3] = [Self::Burst, Self::ValveClosure, Self::PumpTrip];
}

impl fmt::Display for EventClass {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.pad(match self {
            Self::Burst => "burst",
            Self::ValveClosure => "valve closure",
            Self::PumpTrip => "pump trip",
            Self::Unknown => "unknown",
        })
    }
}
