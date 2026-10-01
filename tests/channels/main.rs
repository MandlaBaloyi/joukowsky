//! Channel tests: canals and rivers, open-channel flow.
//!
//! Code under test: `crates/domain/channels` (1-D Saint-Venant, finite
//! volumes) - `docs/02_physics_of_transients/open_channels.txt`.
//!
//! Run only these:  `.\tests\run-tests.ps1 -Only channels`

mod analytical;
mod invariants;
mod simulation;

use jk_channels::{Reach, Section};
use jk_kernel::{Length, STANDARD_GRAVITY};

/// A concrete-lined irrigation canal: trapezoid 3 m wide at the bottom,
/// sides 1.5 horizontal per 1 vertical, bed falling 0.5 m per km, Manning
/// n = 0.015, 5 km long.
#[must_use]
pub fn canal() -> Reach {
    Reach {
        length: Length::new(5000.0),
        section: Section {
            bottom_width: Length::new(3.0),
            side_slope: 1.5,
        },
        slope: 0.0005,
        manning: 0.015,
        gravity: STANDARD_GRAVITY,
    }
}

/// A flat, frictionless rectangular flume `length` long and `width` wide
/// (for the textbook wave and dam-break solutions).
#[must_use]
pub fn flume(length: f64, width: f64) -> Reach {
    Reach {
        length: Length::new(length),
        section: Section::rectangle(Length::new(width)),
        slope: 0.0,
        manning: 0.0,
        gravity: STANDARD_GRAVITY,
    }
}
