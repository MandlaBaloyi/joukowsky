//! The noise unit the CUSUM works in: the sensor noise when quiet, raised
//! while the network rings.
//!
//! While ringing, the largest recent swing (around the median of the last
//! settle window) counts as `ring_units` noise units - below the CUSUM
//! drift, so the ringing stays silent while a new event about three times
//! its size still fires. The floor is re-measured every quarter settle
//! window and falls by at most half per window: ringing beats, and one
//! calm stretch does not mean it has died away. It returns to the sensor
//! noise when the window looks like plain noise again (`settle_band`).

use crate::detector::DetectorConfig;
use crate::noise::median;

/// While ringing, the most the floor may fall per settle window.
const RELEASE_PER_WINDOW: f64 = 0.5;

#[derive(Debug, Clone, PartialEq)]
pub(super) struct Floor {
    sigma: f64,
    band: f64,
    ring_units: f64,
    settle: usize,
    stride: usize,
    release: f64,
    /// The noise unit in use.
    pub(super) scale: f64,
    /// Next sample at which to re-measure while ringing.
    pub(super) next_check: usize,
}

impl Floor {
    pub(super) fn new(sigma: f64, settle: usize, config: &DetectorConfig) -> Self {
        Self {
            sigma,
            band: config.settle_band * sigma,
            ring_units: config.ring_units,
            settle,
            stride: (settle / 4).max(1),
            release: RELEASE_PER_WINDOW.powf(1.0 / 4.0),
            scale: sigma,
            next_check: usize::MAX,
        }
    }

    /// Re-measures on `window`, the settle window ending before sample
    /// `end`. Returns the window's median and whether it was quiet. With
    /// `may_fall` false the floor can only rise.
    pub(super) fn measure(&mut self, window: &[f64], end: usize, may_fall: bool) -> (f64, bool) {
        let centre = median(&mut window.to_vec());
        let spread = window
            .iter()
            .map(|v| (v - centre).abs())
            .fold(0.0, f64::max);
        if spread <= self.band {
            self.scale = self.sigma;
            self.next_check = usize::MAX;
            return (centre, true);
        }
        let lowest = if may_fall {
            self.scale * self.release
        } else {
            self.scale
        };
        self.scale = self.sigma.max(spread / self.ring_units).max(lowest);
        // (Never before a full window exists.)
        self.next_check = (end + self.stride).max(self.settle);
        (centre, false)
    }
}
