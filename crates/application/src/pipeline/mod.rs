//! The event pipeline: records in, alerts out.
//!
//! - [`fusion`]: which detections at different sensors are one event.
//! - [`process`](process()): the use case - detect, fuse, locate, classify, alert.

pub mod fusion;
mod run;

pub use run::process;

use jk_classification::{ClassifierConfig, Context, FeatureConfig};
use jk_detection::{DetectorConfig, Series};
use jk_hydraulics::{Network, NodeId};
use jk_kernel::Time;
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
}

impl Default for PipelineConfig {
    fn default() -> Self {
        Self {
            detector: DetectorConfig::default(),
            features: FeatureConfig::default(),
            classifier: ClassifierConfig::default(),
            localise: LocaliseOptions::default(),
            speed_margin: 0.1,
            gate_sigmas: 4.0,
            alarm_confidence: 0.9,
            max_misfit: 3.0,
        }
    }
}
