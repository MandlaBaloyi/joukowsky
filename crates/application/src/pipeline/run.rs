//! The use case over whole records: one window of records in, alerts out.
//!
//! It is the live [`Monitor`] fed each record whole, then told no more
//! data will come - so the window and the live paths cannot disagree.
//! Nothing is decided before every record is in (no waiting limit).

use jk_kernel::Time;
use jk_telemetry::Chunk;

use crate::alert::Alert;
use crate::error::ApplicationError;
use crate::pipeline::{Monitor, PipelineConfig, Record, Site};
use crate::ports::AlertSink;

/// Processes one window of `records` from `site`: detect at every sensor,
/// fuse, locate, classify, and hand one alert per event to `sink`. Returns
/// the alerts too.
///
/// # Errors
/// Records that do not match the site, refused data, or an alert that
/// could not be delivered.
pub fn process(
    site: &Site,
    records: &[Record<'_>],
    config: &PipelineConfig,
    sink: &mut dyn AlertSink,
) -> Result<Vec<Alert>, ApplicationError> {
    validate(site, records)?;
    let whole = PipelineConfig {
        max_wait: Time::new(f64::INFINITY),
        ..*config
    };
    let mut monitor = Monitor::new(site.clone(), whole)?;
    let mut alerts = Vec::new();
    for r in records {
        let chunk = Chunk {
            start: r.series.start,
            interval: r.series.interval,
            samples: r.series.samples,
        };
        alerts.extend(monitor.ingest(r.sensor, chunk, sink)?);
    }
    alerts.extend(monitor.finish(sink)?);
    Ok(alerts)
}

fn validate(site: &Site, records: &[Record<'_>]) -> Result<(), ApplicationError> {
    let bad = |why: String| Err(ApplicationError::InvalidInput(why));
    for (i, r) in records.iter().enumerate() {
        if r.sensor >= site.sensors.len() {
            return bad(format!(
                "record {i} is for sensor {} which does not exist",
                r.sensor
            ));
        }
        if records[..i].iter().any(|o| o.sensor == r.sensor) {
            return bad(format!(
                "two records for sensor {}",
                site.sensors[r.sensor].name
            ));
        }
    }
    Ok(())
}
