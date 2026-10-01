//! The rules. Weights are in natural-log units: +2 makes a class about
//! seven times more likely, -4 about fifty times less.
//!
//! | rule | physics | burst | valve closure | pump trip | refill |
//! |---|---|---|---|---|---|
//! | front RISES | a closing valve raises pressure upstream of it; returning water fills a pipe | -6 | 0 | -6 | 0 |
//! | front DROPS | a new outflow or a slowing pump lowers pressure | 0 | -6 | 0 | -6 |
//! | SHARP front | a pipe splits in milliseconds | +2 | 0 | -2 | 0 |
//! | DRAWN-OUT front | a pump runs down over a second | -0.5 | 0 | +1.5 | 0 |
//! | DEEP drop (>= 30% of the pressure) | a pump stopping collapses the head | -1 | 0 | +1.5 | 0 |
//! | SHALLOW drop (<= 15%) | a hole lets out a little water | +1 | 0 | -1 | 0 |
//! | EMPTY before (<= 2 m) | a drained pipe: nothing flowing to stop or let out | -3 | -3 | -3 | +5 |
//! | supply RETURNED (<= 15 min) before | the pipe is still filling; its column bounces and slams | 0 | 0 | 0 | +5, and no -6 for a drop |
//! | otherwise UNDER PRESSURE before | a full pipe is not refilling | 0 | 0 | 0 | -5 |
//! | power LOST just before | pumps stop when power fails | 0 | 0 | +4 | 0 |
//! | power monitored and STABLE | no power loss, no pump trip | 0 | 0 | -4 | 0 |
//!
//! Known blind spots, stated:
//! - a sensor DOWNSTREAM of a closing valve sees a drop; version 1 would
//!   call that a burst or pump trip. Arrival order and signs across several
//!   sensors (localisation) will resolve it.
//! - a refill against trapped air is a slow ramp the front detector lets
//!   pass; only the supply watch (`jk_detection::empty_spells`, passed in
//!   as `Context::supply_returns`) ties its later fronts to the refill.
//!   Without it they read as other classes.
//! - a burst DURING a refill (refill surges do burst pipes) is weighed
//!   against the refill; a sharp drop then is not certain either way.
//! - a sensor right behind a valve discharging to air also reads ~0 before
//!   the valve shuts; supply records (an interruption) would separate it.
//! - a vapour cavity collapsing (column separation, not simulated yet) also
//!   rises from the vapour floor; the drop that came seconds before tells
//!   it apart from a drained pipe.

use jk_detection::Direction;
use jk_kernel::Time;

use crate::classes::{Context, EventClass};
use crate::features::Features;
use crate::rules::ClassifierConfig;

/// Evidence per candidate class, and the reasons given.
#[must_use]
#[allow(clippy::too_many_lines)] // the rule table, read top to bottom
pub fn weigh(
    f: &Features,
    context: &Context,
    arrival: Time,
    config: &ClassifierConfig,
) -> (Vec<(EventClass, f64)>, Vec<String>) {
    let (mut burst, mut valve, mut pump, mut refill) = (0.0, 0.0, 0.0, 0.0);
    let mut reasons = Vec::new();
    let refilling = context.supply_returned_before(arrival, config.refill_window);

    match f.direction {
        Direction::Rise => {
            burst -= 6.0;
            pump -= 6.0;
            reasons.push(format!(
                "pressure ROSE by {:.2} m: a closing valve raises pressure upstream of it, returning water fills a pipe; bursts and pump trips lower it",
                f.swing.value()
            ));
        }
        Direction::Drop => {
            valve -= 6.0;
            if refilling.is_none() {
                refill -= 6.0;
            }
            reasons.push(format!(
                "pressure DROPPED by {:.2} m: a burst or a pump trip, not a closing valve (seen upstream){}",
                -f.swing.value(),
                if refilling.is_some() {
                    " - or, in a refilling pipe, its column bouncing back off the air ahead"
                } else {
                    " or a refill"
                }
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

    if f.level_before <= config.empty_below {
        burst -= 3.0;
        valve -= 3.0;
        pump -= 3.0;
        refill += 5.0;
        reasons.push(format!(
            "the pipe was EMPTY before: {:.2} m of pressure - supply returning to a drained pipe; with nothing flowing there was nothing to stop or let out",
            // Rounded as shown, so a reading a hair below 0 is "0.00", not "-0.00".
            (f.level_before.value() * 100.0).round() / 100.0 + 0.0
        ));
    } else if let Some(lag) = refilling {
        refill += 5.0;
        reasons.push(format!(
            "supply RETURNED {:.0} s before to a pipe that had stood empty: it is still filling - \
             the air ahead squeezed, the column bouncing on it or slamming into a closed end",
            lag.value()
        ));
    } else {
        refill -= 5.0;
        reasons.push(format!(
            "the pipe was under pressure before ({:.1} m): not refilling",
            f.level_before.value()
        ));
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
            (EventClass::Refill, refill),
        ],
        reasons,
    )
}
