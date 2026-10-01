//! Evidence from outside the pressure record.
//!
//! `event_classification.txt` section 1, "Other cues": a pump trip comes
//! with a power event at the same time. In South Africa power losses are
//! often KNOWN - pump controller logs, VSD alarms, the load-shedding
//! schedule (`data_source_requirements.txt` section 4) - and that is the
//! strongest single clue there is.
//!
//! Three states matter, and are kept apart:
//!   - power is monitored and failed near the event: strong for a pump trip;
//!   - power is monitored and did NOT fail: evidence against a pump trip;
//!   - power is not monitored: no evidence either way.
//!
//! Supply returns: when water came back to a pipe that had stood empty -
//! from the sensors' own records (`jk_detection::empty_spells`) or a
//! utility's notice. A pipe refills for minutes after; its surges then
//! belong to the refill.

use jk_kernel::Time;

/// What is known about the site around the event.
#[derive(Debug, Clone, PartialEq, Default)]
pub struct Context {
    /// Whether pump power is monitored at all.
    pub power_monitored: bool,
    /// Times at which pump power was lost (meaningful only if monitored).
    pub power_losses: Vec<Time>,
    /// Times at which supply returned to a pipe that had stood empty.
    pub supply_returns: Vec<Time>,
}

impl Context {
    /// Nothing is known about the site.
    #[must_use]
    pub fn unknown() -> Self {
        Self::default()
    }

    /// Power is monitored and was lost at `times`.
    #[must_use]
    pub fn power_lost_at(times: &[Time]) -> Self {
        Self {
            power_monitored: true,
            power_losses: times.to_vec(),
            supply_returns: Vec::new(),
        }
    }

    /// Power is monitored and stayed on.
    #[must_use]
    pub fn power_stable() -> Self {
        Self {
            power_monitored: true,
            power_losses: Vec::new(),
            supply_returns: Vec::new(),
        }
    }

    /// The same, knowing also that supply returned at `times`.
    #[must_use]
    pub fn with_supply_returns(mut self, times: &[Time]) -> Self {
        self.supply_returns.extend_from_slice(times);
        self
    }

    /// How long before `time` supply last returned, if within `window`.
    #[must_use]
    pub fn supply_returned_before(&self, time: Time, window: Time) -> Option<Time> {
        self.supply_returns
            .iter()
            .map(|&back| time - back)
            .filter(|lag| (0.0..=window.value()).contains(&lag.value()))
            .min_by(|a, b| a.value().total_cmp(&b.value()))
    }

    /// Some(true) if power was lost within `tolerance` before `time` (a
    /// pump's wave arrives after its trip), Some(false) if monitored and
    /// not, None if not monitored.
    #[must_use]
    pub fn power_lost_before(&self, time: Time, tolerance: Time) -> Option<bool> {
        self.power_monitored.then(|| {
            self.power_losses.iter().any(|&loss| {
                let lead = (time - loss).value();
                (-0.05..=tolerance.value()).contains(&lead)
            })
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn three_states_of_knowledge() {
        let t = Time::new(10.0);
        let tol = Time::new(2.0);
        assert_eq!(Context::unknown().power_lost_before(t, tol), None);
        assert_eq!(
            Context::power_stable().power_lost_before(t, tol),
            Some(false)
        );
        assert_eq!(
            Context::power_lost_at(&[Time::new(9.2)]).power_lost_before(t, tol),
            Some(true)
        );
        assert_eq!(
            Context::power_lost_at(&[Time::new(3.0)]).power_lost_before(t, tol),
            Some(false),
            "a loss long before is a different event"
        );
    }

    #[test]
    fn the_latest_supply_return_within_the_window_counts() {
        let c = Context::unknown().with_supply_returns(&[Time::new(10.0), Time::new(100.0)]);
        let window = Time::new(60.0);
        assert_eq!(
            c.supply_returned_before(Time::new(130.0), window),
            Some(Time::new(30.0))
        );
        assert_eq!(c.supply_returned_before(Time::new(90.0), window), None);
        assert_eq!(c.supply_returned_before(Time::new(5.0), window), None);
    }
}
