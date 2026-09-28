//! The rules. Weights are in natural-log units: +2 makes a class about
//! seven times more likely, -4 about fifty times less.
//!
//! | rule | physics | burst | valve closure | pump trip |
//! |---|---|---|---|---|
//! | front RISES | a closing valve raises pressure upstream of it | -6 | 0 | -6 |
//! | front DROPS | a new outflow or a slowing pump lowers pressure | 0 | -6 | 0 |
//! | SHARP front | a pipe splits in milliseconds | +2 | 0 | -2 |
//! | DRAWN-OUT front | a pump runs down over a second | -0.5 | 0 | +1.5 |
//! | DEEP drop (>= 30% of the pressure) | a pump stopping collapses the head | -1 | 0 | +1.5 |
//! | SHALLOW drop (<= 15%) | a hole lets out a little water | +1 | 0 | -1 |
//! | power LOST just before | pumps stop when power fails | 0 | 0 | +4 |
//! | power monitored and STABLE | no power loss, no pump trip | 0 | 0 | -4 |
//!
//! Known blind spot, stated: a sensor DOWNSTREAM of a closing valve sees a
//! drop; version 1 would call that a burst or pump trip. Arrival order and
//! signs across several sensors (localisation) will resolve it.

use jk_detection::Direction;
use jk_kernel::Time;

use crate::classes::{Context, EventClass};
use crate::features::Features;
use crate::rules::ClassifierConfig;

/// Evidence per candidate class, and the reasons given.
#[must_use]
pub fn weigh(
    f: &Features,
    context: &Context,
    arrival: Time,
    config: &ClassifierConfig,
) -> (Vec<(EventClass, f64)>, Vec<String>) {
    let (mut burst, mut valve, mut pump) = (0.0, 0.0, 0.0);
    let mut reasons = Vec::new();

    match f.direction {
        Direction::Rise => {
            burst -= 6.0;
            pump -= 6.0;
            reasons.push(format!(
                "pressure ROSE by {:.2} m: a closing valve raises pressure upstream of it; bursts and pump trips lower it",
                f.swing.value()
            ));
        }
        Direction::Drop => {
            valve -= 6.0;
            reasons.push(format!(
                "pressure DROPPED by {:.2} m: a burst or a pump trip, not a closing valve (seen upstream)",
                -f.swing.value()
            ));
        }
    }

    let rise_ms = f.rise_time.value() * 1e3;
    if f.rise_time <= config.sharp_front {
        burst += 2.0;
        pump -= 2.0;
        reasons.push(format!(
            "SHARP front ({rise_ms:.0} ms, 10-90%): a sudden change - a pipe splitting, a valve slamming - not a pump running down"
        ));
    } else if f.rise_time >= config.slow_front {
        burst -= 0.5;
        pump += 1.5;
        reasons.push(format!(
            "DRAWN-OUT front ({rise_ms:.0} ms, 10-90%): a gradual change - a pump running down, a valve closing slowly, or a burst opening slowly"
        ));
    } else {
        reasons.push(format!(
            "front of {rise_ms:.0} ms: neither clearly sharp nor drawn out"
        ));
    }

    if f.direction == Direction::Drop && f.level_before.value() > 0.0 {
        let share = -f.swing.value() / f.level_before.value();
        if share >= 0.30 {
            burst -= 1.0;
            pump += 1.5;
            reasons.push(format!(
                "DEEP drop: {:.0}% of the pressure - a stopping pump collapses the head",
                share * 100.0
            ));
        } else if share <= 0.15 {
            burst += 1.0;
            pump -= 1.0;
            reasons.push(format!(
                "SHALLOW drop: {:.0}% of the pressure - a hole lets out a little water",
                share * 100.0
            ));
        }
    }

    match context.power_lost_before(arrival, config.power_window) {
        Some(true) => {
            pump += 4.0;
            reasons.push("pump power was LOST just before the front arrived".into());
        }
        Some(false) => {
            pump -= 4.0;
            reasons.push("pump power is monitored and did NOT fail: not a pump trip".into());
        }
        None => reasons.push("pump power is not monitored: no outside evidence".into()),
    }

    (
        vec![
            (EventClass::Burst, burst),
            (EventClass::ValveClosure, valve),
            (EventClass::PumpTrip, pump),
        ],
        reasons,
    )
}
