//! Ports: what the pipeline needs from the outside world, as traits.
//!
//! The pipeline hands each alert to an [`AlertSink`] without knowing where
//! it goes. An SMS gateway, an email service, a dashboard feed or a
//! database each become an ADAPTER implementing this trait, in
//! `crates/adapters/` - swapping one never touches the engine.
//!
//! Later ports: where sensor records come from (a streaming source), where
//! events and labels are stored, where the network model is loaded from.

use crate::alert::Alert;

/// Somewhere alerts go.
pub trait AlertSink {
    /// Delivers one alert.
    ///
    /// # Errors
    /// Why delivery failed (network down, gateway refused...). The pipeline
    /// reports it; it never drops an alert silently.
    fn send(&mut self, alert: &Alert) -> Result<(), String>;
}

/// A sink that keeps every alert in memory - for tests and for the command
/// line tool.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct CollectingSink {
    /// Alerts received, in order.
    pub alerts: Vec<Alert>,
}

impl AlertSink for CollectingSink {
    fn send(&mut self, alert: &Alert) -> Result<(), String> {
        self.alerts.push(alert.clone());
        Ok(())
    }
}
