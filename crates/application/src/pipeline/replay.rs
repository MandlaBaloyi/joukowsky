//! The use case over a site's recordings: each logger's runs (with gaps
//! between them) fed to the live [`Monitor`] in time order, in pieces, as
//! the site would have sent them - the same engine that would run live.

use jk_health::Spectrum;
use jk_kernel::Time;
use jk_telemetry::Chunk;

use crate::alert::Alert;
use crate::error::ApplicationError;
use crate::pipeline::{Monitor, PipelineConfig, Site};
use crate::ports::AlertSink;

/// One logger's recording: its sensor in the site, its sampling interval,
/// and its continuous runs (start [s, UTC], heads [m]; NaN where it had no
/// valid value - never filled, P6).
#[derive(Debug, Clone, PartialEq)]
pub struct Feed {
    /// Its index in the site's sensors.
    pub sensor: usize,
    /// Seconds between samples.
    pub interval: Time,
    /// (start, heads) of each continuous run.
    pub runs: Vec<(Time, Vec<f64>)>,
}

/// What a replay gave.
#[derive(Debug, Clone)]
pub struct Replay {
    /// Every alert and revision as it went out, with the data time then
    /// (the latest sample received from any logger) [s].
    pub alerts: Vec<(Alert, f64)>,
    /// Each sensor's fatigue loading over the whole replay.
    pub loading: Vec<Spectrum>,
    /// The time the data covers: first and last sample [s].
    pub span: (f64, f64),
}

impl Replay {
    /// The latest revision of each alert, in order of first issue.
    #[must_use]
    pub fn latest(&self) -> Vec<&Alert> {
        let mut out: Vec<&Alert> = Vec::new();
        for (a, _) in &self.alerts {
            match out.iter_mut().find(|b| b.id == a.id) {
                Some(b) => {
                    if a.revision >= b.revision {
                        *b = a;
                    }
                }
                None => out.push(a),
            }
        }
        out
    }
}

/// Replays `feeds` from `site` through the live engine, `chunk` seconds of
/// each logger at a time, the loggers taking turns in time order.
///
/// # Errors
/// A feed for a sensor the site does not have, a bad chunk length,
/// refused data, or an alert that could not be delivered.
pub fn replay(
    site: &Site,
    config: &PipelineConfig,
    feeds: &[Feed],
    chunk: Time,
    sink: &mut dyn AlertSink,
) -> Result<Replay, ApplicationError> {
    if !(chunk.value() > 0.0 && chunk.value().is_finite()) {
        return Err(ApplicationError::InvalidInput(format!(
            "chunk of {} s: must be positive",
            chunk.value()
        )));
    }
    if let Some(f) = feeds.iter().find(|f| f.sensor >= site.sensors.len()) {
        return Err(ApplicationError::InvalidInput(format!(
            "a feed for sensor {} which the site does not have",
            f.sensor
        )));
    }
    // Every piece of every run: (start, feed, run, from, to).
    let mut pieces: Vec<(f64, usize, usize, usize, usize)> = Vec::new();
    let mut span = (f64::INFINITY, f64::NEG_INFINITY);
    for (i, f) in feeds.iter().enumerate() {
        let dt = f.interval.value();
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)] // >= 1
        let per = ((chunk.value() / dt).round() as usize).max(1);
        for (r, (start, heads)) in f.runs.iter().enumerate() {
            let mut from = 0;
            while from < heads.len() {
                let to = (from + per).min(heads.len());
                #[allow(clippy::cast_precision_loss)] // sample counts are small
                let t = start.value() + from as f64 * dt;
                pieces.push((t, i, r, from, to));
                from = to;
            }
            #[allow(clippy::cast_precision_loss)]
            let end = start.value() + heads.len().saturating_sub(1) as f64 * dt;
            span = (span.0.min(start.value()), span.1.max(end));
        }
    }
    pieces.sort_by(|a, b| a.0.total_cmp(&b.0).then(a.1.cmp(&b.1)));
    let mut monitor = Monitor::new(site.clone(), *config)?;
    let mut alerts = Vec::new();
    for (t, i, r, from, to) in pieces {
        let f = &feeds[i];
        let piece = Chunk {
            start: Time::new(t),
            interval: f.interval,
            samples: &f.runs[r].1[from..to],
        };
        for a in monitor.ingest(f.sensor, piece, sink)? {
            let data_time = monitor.lead().map_or(t, Time::value);
            alerts.push((a, data_time));
        }
    }
    let loading = monitor.loading();
    let end = monitor.lead().map_or(span.1, Time::value);
    for a in monitor.finish(sink)? {
        alerts.push((a, end));
    }
    Ok(Replay {
        alerts,
        loading,
        span,
    })
}
