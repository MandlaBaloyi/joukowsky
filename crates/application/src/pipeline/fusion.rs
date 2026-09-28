//! Fusion: which detections at different sensors belong to ONE event.
//!
//! `docs/04_detection_and_localisation/event_detection.txt`, Layer 4: "an
//! event is confirmed when arrivals at multiple sensors are physically
//! consistent: time differences <= travel times along the network (a
//! coincidence gate derived from the graph and wave speeds). Physically
//! impossible combinations = separate events or a sensor fault."
//!
//! Two arrivals from one source can never be further apart in time than a
//! wave takes to travel between the two sensors. So detections i and j may
//! be one event only if
//!
//! ```text
//!   |t_i - t_j| <= tau_ij (1 + margin) + k sqrt(sigma_i^2 + sigma_j^2)
//! ```
//!
//! (margin for wrong wave speeds, k standard deviations for timing). The
//! grouping is greedy in time order: each detection joins the latest group
//! it is consistent with ALL members of and that has no detection from its
//! own sensor yet; otherwise it starts a new group.

/// One detection, ready for fusion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Candidate {
    /// Index of the sensor.
    pub sensor: usize,
    /// Arrival time on the site clock (clock offset removed) [s].
    pub time: f64,
    /// One-sigma timing uncertainty [s].
    pub sigma: f64,
}

/// Groups `candidates` (any order) into events, given the fastest wave
/// travel time between every pair of sensors, `travel[i][j]` (infinite if no
/// wave path joins them). Returns groups of indices into `candidates`,
/// each sorted by time, groups in order of their first arrival.
#[must_use]
pub fn group(
    candidates: &[Candidate],
    travel: &[Vec<f64>],
    margin: f64,
    sigmas: f64,
) -> Vec<Vec<usize>> {
    let mut order: Vec<usize> = (0..candidates.len()).collect();
    order.sort_by(|&a, &b| candidates[a].time.total_cmp(&candidates[b].time));
    let consistent = |a: &Candidate, b: &Candidate| {
        let tau = travel[a.sensor][b.sensor];
        tau.is_finite()
            && (a.time - b.time).abs() <= tau * (1.0 + margin) + sigmas * a.sigma.hypot(b.sigma)
    };
    let mut groups: Vec<Vec<usize>> = Vec::new();
    for &i in &order {
        let c = &candidates[i];
        let home = groups.iter_mut().rev().find(|g| {
            g.iter()
                .all(|&m| candidates[m].sensor != c.sensor && consistent(&candidates[m], c))
        });
        match home {
            Some(g) => g.push(i),
            None => groups.push(vec![i]),
        }
    }
    groups
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Three sensors on a line, 0.5 s of wave travel apart (0 - 1 - 2).
    fn line() -> Vec<Vec<f64>> {
        vec![
            vec![0.0, 0.5, 1.0],
            vec![0.5, 0.0, 0.5],
            vec![1.0, 0.5, 0.0],
        ]
    }

    fn c(sensor: usize, time: f64) -> Candidate {
        Candidate {
            sensor,
            time,
            sigma: 1e-3,
        }
    }

    #[test]
    fn one_event_seen_everywhere_is_one_group() {
        let g = group(&[c(2, 10.8), c(0, 10.2), c(1, 10.3)], &line(), 0.1, 4.0);
        assert_eq!(g, vec![vec![1, 2, 0]]);
    }

    #[test]
    fn two_events_far_apart_in_time_are_two_groups() {
        let g = group(
            &[c(0, 10.0), c(1, 10.5), c(0, 20.0), c(1, 20.4)],
            &line(),
            0.1,
            4.0,
        );
        assert_eq!(g, vec![vec![0, 1], vec![2, 3]]);
    }

    #[test]
    fn arrivals_too_far_apart_for_the_pipes_are_not_one_event() {
        // Sensors 0 and 2 are 1.0 s apart by wave: 3 s apart cannot be one source.
        let g = group(&[c(0, 10.0), c(2, 13.0)], &line(), 0.1, 4.0);
        assert_eq!(g.len(), 2);
    }
}
