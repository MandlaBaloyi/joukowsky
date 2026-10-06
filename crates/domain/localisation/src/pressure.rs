//! WHERE a slow leak is, from pressure sensors read every few minutes
//! (Tier 0) and the network model - no wave needed
//! (`docs/04_detection_and_localisation/slow_leaks_and_water_balance.txt`,
//! "Locating from pressure sensors").
//!
//! A leak draws water through the pipes to it, so the pressure falls most
//! near it. [`Signatures::compute`] plays a small leak (`probe`) at each
//! candidate junction in turn, in the network as it stands at some hour,
//! and keeps how far each sensor's pressure falls: one SIGNATURE per
//! candidate. [`locate_from_pressures`] takes a measured change of the
//! sensors' pressures - nights after a leak began minus nights before -
//! and fits every signature to it,
//!
//! ```text
//!   change_s = c * signature_s + b      (weighted least squares, w_s = 1 / noise_s^2)
//! ```
//!
//! c scales the probe to the leak's size, b takes up what changed
//! everywhere alike (demand, the season). The response is near linear only
//! for leaks small beside the flows: friction grows as the flow squared, so
//! a large leak is over-sized (18% for a leak 1.5 times the line's flow in
//! the unit test) - the size is a first estimate. The candidates whose signatures fit best are where
//! the leak most likely is; c x probe estimates its flow.
//!
//! On L-Town (`BattLeDIM` 2020, `tests/localisation/datasets/
//! ltown_pressure.rs`) 782 signatures take one to three seconds; with the
//! true leak dates the best candidate is within 300 m of 18 of the 33
//! leaks, median 288 m, where a junction picked at random is 922 m away;
//! from the night watch's own alarms, within 300 m of a leak that truly
//! changed in 17 of 19, median 153 m (FN-053).

use jk_hydraulics::{Network, Node, NodeId, SteadyOptions};
use jk_kernel::Flow;

use crate::LocalisationError;

/// How each sensor's pressure falls for a probe leak at each candidate.
#[derive(Debug, Clone, PartialEq)]
pub struct Signatures {
    candidates: Vec<NodeId>,
    sensors: Vec<NodeId>,
    /// `drops[i][s]`: head change at sensor s [m] for the probe at
    /// candidate i (negative: the pressure falls).
    drops: Vec<Vec<f64>>,
    probe: Flow,
}

/// One candidate's fit to a measured change.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PressureMatch {
    /// The candidate junction.
    pub node: NodeId,
    /// Weighted squared misfit left after the fit (smaller is better).
    pub misfit: f64,
    /// The leak's flow the fit implies (scale x probe).
    pub flow: Flow,
}

impl Signatures {
    /// Plays `probe` at each of `candidates` (junctions) in `network` as
    /// given - its demands at the hour wanted - and records each sensor's
    /// head change.
    ///
    /// # Errors
    /// No sensors or candidates, a candidate that is not a junction, a
    /// probe that is not a positive flow, or a steady state that cannot be
    /// solved.
    pub fn compute(
        network: &Network,
        sensors: &[NodeId],
        candidates: &[NodeId],
        probe: Flow,
        steady: &SteadyOptions,
    ) -> Result<Self, LocalisationError> {
        if sensors.is_empty() || candidates.is_empty() {
            return Err(LocalisationError::InvalidInput(
                "signatures need sensors and candidate junctions".into(),
            ));
        }
        if !(probe.value().is_finite() && probe.value() > 0.0) {
            return Err(LocalisationError::invalid(
                "probe",
                probe.value(),
                "a positive flow",
            ));
        }
        let solve = |net: &Network| {
            net.steady_state(steady)
                .map_err(|e| LocalisationError::InvalidInput(e.to_string()))
        };
        let base = solve(network)?;
        let mut drops = Vec::with_capacity(candidates.len());
        for &c in candidates {
            let Some(Node::Junction { demand }) = network.nodes().get(c.index()).copied() else {
                return Err(LocalisationError::InvalidInput(format!(
                    "candidate node {} is not a junction",
                    c.index()
                )));
            };
            let mut leaky = network.clone();
            leaky
                .set_demand(c, demand + probe)
                .map_err(|e| LocalisationError::InvalidInput(e.to_string()))?;
            let s = solve(&leaky)?;
            drops.push(
                sensors
                    .iter()
                    .map(|&k| s.node_heads[k.index()].value() - base.node_heads[k.index()].value())
                    .collect(),
            );
        }
        Ok(Self {
            candidates: candidates.to_vec(),
            sensors: sensors.to_vec(),
            drops,
            probe,
        })
    }

    /// The candidate junctions, in the order of their signatures.
    #[must_use]
    pub fn candidates(&self) -> &[NodeId] {
        &self.candidates
    }

    /// The sensors, in the order of each signature's entries.
    #[must_use]
    pub fn sensors(&self) -> &[NodeId] {
        &self.sensors
    }
}

/// Fits every signature to `change` (one entry per sensor, the measured
/// head change [m]) with weights 1 / `noise`^2 (each sensor's
/// night-to-night noise [m]); returns the candidates that fit a leak (a
/// fall: c > 0), best first.
///
/// # Errors
/// `change` or `noise` not one per sensor, or a noise that is not positive.
pub fn locate_from_pressures(
    signatures: &Signatures,
    change: &[f64],
    noise: &[f64],
) -> Result<Vec<PressureMatch>, LocalisationError> {
    let n = signatures.sensors.len();
    if change.len() != n || noise.len() != n {
        return Err(LocalisationError::InvalidInput(format!(
            "{n} sensors: give one change and one noise for each"
        )));
    }
    if noise.iter().any(|x| !(x.is_finite() && *x > 0.0)) {
        return Err(LocalisationError::InvalidInput(
            "every sensor's noise must be positive".into(),
        ));
    }
    let w: Vec<f64> = noise.iter().map(|x| 1.0 / (x * x)).collect();
    let sw: f64 = w.iter().sum();
    let md = change.iter().zip(&w).map(|(y, w)| y * w).sum::<f64>() / sw;
    let mut out: Vec<PressureMatch> = signatures
        .drops
        .iter()
        .zip(&signatures.candidates)
        .filter_map(|(sig, &node)| {
            let ms = sig.iter().zip(&w).map(|(x, w)| x * w).sum::<f64>() / sw;
            let (mut sxy, mut sxx) = (0.0, 0.0);
            for ((x, y), w) in sig.iter().zip(change).zip(&w) {
                sxy += w * (x - ms) * (y - md);
                sxx += w * (x - ms) * (x - ms);
            }
            let c = if sxx > 0.0 { sxy / sxx } else { 0.0 };
            if c <= 0.0 {
                return None; // the change is not this candidate's leak
            }
            let b = md - c * ms;
            let misfit = sig
                .iter()
                .zip(change)
                .zip(&w)
                .map(|((x, y), w)| w * (y - c * x - b).powi(2))
                .sum();
            Some(PressureMatch {
                node,
                misfit,
                flow: signatures.probe * c,
            })
        })
        .collect();
    out.sort_by(|a, b| a.misfit.total_cmp(&b.misfit));
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use jk_hydraulics::Pipe;
    use jk_kernel::{Head, Length, STANDARD_GRAVITY, WaveSpeed};

    /// A reservoir feeding a line of four junctions, 300 m apart.
    fn line() -> (Network, Vec<NodeId>) {
        let mut net = Network::new(STANDARD_GRAVITY);
        let mut prev = net.add_node(Node::Reservoir {
            head: Head::new(50.0),
        });
        let mut js = Vec::new();
        for _ in 0..4 {
            let j = net.add_node(Node::Junction {
                demand: Flow::new(0.002),
            });
            let pipe = Pipe::new(
                Length::new(300.0),
                Length::new(0.15),
                WaveSpeed::new(1000.0),
                0.02,
            )
            .unwrap();
            net.add_pipe(prev, j, pipe).unwrap();
            js.push(j);
            prev = j;
        }
        (net, js)
    }

    #[test]
    fn a_leak_is_found_where_it_was_played_and_sized() {
        let (net, js) = line();
        // A sensor at every junction: the fit has two free numbers (size and
        // a common shift), so it needs more sensors than that to tell
        // candidates apart.
        let sensors = js.clone();
        let sig = Signatures::compute(
            &net,
            &sensors,
            &js,
            Flow::new(0.001),
            &SteadyOptions::default(),
        )
        .unwrap();
        // The "measured" change: a 3 l/s leak at J3 (index 2), seen as the
        // model sees it, plus 0.1 m off everywhere (the season).
        let mut leaky = net.clone();
        leaky.set_demand(js[2], Flow::new(0.005)).unwrap();
        let base = net.steady_state(&SteadyOptions::default()).unwrap();
        let after = leaky.steady_state(&SteadyOptions::default()).unwrap();
        let change: Vec<f64> = sensors
            .iter()
            .map(|s| after.node_heads[s.index()].value() - base.node_heads[s.index()].value() - 0.1)
            .collect();
        let best = locate_from_pressures(&sig, &change, &[0.02; 4]).unwrap();
        assert_eq!(best[0].node, js[2]);
        // Friction grows as the flow squared, so a leak large beside the
        // flows lowers the pressure more than in proportion: scaling the
        // 1 l/s probe gives 3.55 l/s for this 3 l/s leak (18% high).
        let f = best[0].flow.value();
        assert!((0.003..0.003 * 1.25).contains(&f), "{f}");
    }
}
