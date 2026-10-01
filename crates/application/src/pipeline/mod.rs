//! The event pipeline: sensor data in, alerts out.
//!
//! - [`fusion`]: which detections at different sensors are one event.
//! - [`Monitor`]: the LIVE use case - chunks of data in as they arrive, each
//!   alert out once, as soon as the data allows; revised if late data
//!   changes it.
//! - [`process`](process()): the same over a window of whole records.
//! - `assess` (private): one event's alert - classify, locate,
//!   corroborate, grade.

mod assess;
pub mod fusion;
mod monitor;
mod run;

pub use monitor::Monitor;
pub use run::process;

use jk_classification::{ClassifierConfig, Context, FeatureConfig};
use jk_detection::{DetectorConfig, Series, SupplyConfig};
use jk_hydraulics::steady::extended::PeriodOptions;
use jk_hydraulics::{Network, NodeId};
use jk_kernel::{Head, Time};
use jk_localisation::{ClockEstimate, LocaliseOptions};

/// One sensor installed on the network.
#[derive(Debug, Clone, PartialEq)]
pub struct SensorSite {
    /// The node it sits on.
    pub node: NodeId,
    /// A name people recognise ("S1 gatehouse").
    pub name: String,
    /// Its estimated clock offset, if calibrated.
    pub clock: Option<ClockEstimate>,
    /// Clock uncertainty assumed when it is not calibrated (e.g. 1 us for
    /// GNSS, 40 ms for NTP over cellular).
    pub uncalibrated_clock: Time,
    /// The logger's measuring range as gauge pressure head (lowest,
    /// highest), if known - e.g. 0 to 163 m for a 0-16 bar logger. A
    /// reading held at an end of it may be clipped (principle P2).
    pub range: Option<(Head, Head)>,
}

/// Everything the pipeline knows about a site.
#[derive(Debug, Clone, PartialEq)]
pub struct Site {
    /// The hydraulic model of the network.
    pub network: Network,
    /// The installed sensors.
    pub sensors: Vec<SensorSite>,
    /// Names for the pipes, indexed like the network's pipes (optional).
    pub pipe_names: Vec<String>,
    /// Outside context: pump power.
    pub context: Context,
    /// How the site runs over a day, if known: then each event is located
    /// with the pumps and valves as they stand at its hour.
    pub schedule: Option<Schedule>,
}

impl Site {
    /// Local time minus UTC [s], if the site's schedule gives it.
    #[must_use]
    pub fn utc_offset(&self) -> Option<f64> {
        self.schedule.as_ref().map(|s| s.utc_offset.value())
    }
}

/// How a site runs over a day: which pumps and valves stand where at each
/// hour follows from it (gap G-19).
#[derive(Debug, Clone, PartialEq)]
pub struct Schedule {
    /// The day's operation - demand patterns, controls, power cuts - for
    /// [`Network::extended`], its t = 0 at local midnight. Its duration is
    /// not used: the monitor runs two days and takes the second.
    pub period: PeriodOptions,
    /// Local time minus UTC (South Africa: +2 h). Event times are UTC.
    pub utc_offset: Time,
}

/// One sensor's record for the window being processed.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Record<'a> {
    /// Index of the sensor in [`Site::sensors`].
    pub sensor: usize,
    /// Its samples (times on its own clock).
    pub series: Series<'a>,
}

/// Settings of every stage.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipelineConfig {
    /// Detection at each sensor.
    pub detector: DetectorConfig,
    /// Supply watch at each sensor: when its pipe stood empty and when the
    /// supply returned (refills).
    pub supply: SupplyConfig,
    /// Feature windows for classification.
    pub features: FeatureConfig,
    /// Classification rules.
    pub classifier: ClassifierConfig,
    /// Localisation.
    pub localise: LocaliseOptions,
    /// Fusion: extra allowance on travel times between sensors for wave
    /// speeds being wrong (0.1 = 10%).
    pub speed_margin: f64,
    /// Fusion: timing allowance in standard deviations.
    pub gate_sigmas: f64,
    /// A burst becomes an Alarm only at or above this confidence.
    pub alarm_confidence: f64,
    /// Above this timing misfit the location is not trusted.
    pub max_misfit: f64,
    /// Live only: how long (in data time, behind the sensor that has sent
    /// the most) to wait for a lagging sensor's data before deciding an
    /// event without it. Its late data then REVISES the alert.
    pub max_wait: Time,
    /// After an event that took the pressure to or below atmospheric, how
    /// long to keep watching for what follows - the collapse when separated
    /// columns rejoin - before revising its alert with the peak.
    pub follow_up: Time,
    /// Fatigue counting at each sensor: swings smaller than this are not
    /// counted (logger noise is about 2 cm) ...
    pub fatigue_gate: Head,
    /// ... and counted swings are kept in range bins this wide.
    pub fatigue_bin: Head,
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            detector: DetectorConfig::default(),
            supply: SupplyConfig::default(),
            features: FeatureConfig::default(),
            classifier: ClassifierConfig::default(),
            localise: LocaliseOptions::default(),
            speed_margin: 0.1,
            gate_sigmas: 4.0,
            alarm_confidence: 0.9,
            max_misfit: 3.0,
            max_wait: Time::new(10.0),
            follow_up: Time::new(30.0),
            fatigue_gate: Head::new(0.5),
            fatigue_bin: Head::new(0.1),
        }
    }
}
