//! Control valves between two pipes (`docs/02_physics_of_transients/
//! governing_equations_and_moc.txt`, section 6): pressure-reducing (PRV),
//! pressure-sustaining (PSV), flow-control (FCV) and throttle (TCV) valves,
//! as EPANET defines them.
//!
//! In a steady solve a valve is ACTIVE (doing its job), OPEN (fully open, a
//! friction loss) or CLOSED (no flow):
//! - PRV holds the head just DOWNSTREAM at its setting while the head
//!   upstream allows (South African zones everywhere - less pressure, less
//!   leakage);
//! - PSV holds the head just UPSTREAM at least at its setting (keeps a high
//!   zone or a reservoir inlet from being drained);
//! - FCV lets through at most its setting's flow (a bulk supply point, a
//!   purchase limit);
//! - TCV is a fixed partial opening: always OPEN, a loss R Q|Q| both ways.
//!
//! PRV, PSV and FCV pass water one way only. In a transient every valve is
//! FROZEN at its steady opening: a pilot-operated valve needs seconds to
//! move, a wave crosses a site in well under one. Frozen, it is an orifice
//! between its two pipes (one-way unless a TCV).

use jk_kernel::{Flow, Head};

/// What a control valve does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ValveMode {
    /// Pressure-reducing: holds the downstream head at `setting`.
    Prv {
        /// Head held just downstream [m].
        setting: Head,
    },
    /// Pressure-sustaining: holds the upstream head at least at `setting`.
    Psv {
        /// Head kept just upstream [m].
        setting: Head,
    },
    /// Flow-control: passes at most `flow`.
    Fcv {
        /// Largest flow passed [m^3/s].
        flow: Flow,
    },
    /// Throttle: a fixed loss, the valve's `open_resistance`.
    Tcv,
}

impl ValveMode {
    /// Short name, as EPANET writes it.
    #[must_use]
    pub fn name(&self) -> &'static str {
        match self {
            Self::Prv { .. } => "PRV",
            Self::Psv { .. } => "PSV",
            Self::Fcv { .. } => "FCV",
            Self::Tcv => "TCV",
        }
    }

    /// True if the valve passes water one way only (all but the TCV).
    #[must_use]
    pub fn one_way(&self) -> bool {
        !matches!(self, Self::Tcv)
    }
}

/// What a control valve is doing in a steady state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValveState {
    /// Doing its job: holding its head or its flow.
    Active,
    /// Fully open: a plain friction loss.
    Open,
    /// Shut: no flow.
    Closed,
}

/// Heads on both sides of the valve and the flow through it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ValveStep {
    /// Head at the end of the upstream pipe [m].
    pub head_upstream: f64,
    /// Head at the start of the downstream pipe [m].
    pub head_downstream: f64,
    /// Flow through the valve [m^3/s] (never negative if one-way).
    pub flow: f64,
}

/// One time step of a frozen valve: an orifice with head loss `r` Q|Q|
/// (`None`: shut), between the upstream C+ (`H = cp - bu Q`) and the
/// downstream C- (`H = cm + bd Q`):
///
/// ```text
///   r Q|Q| + (bu + bd) Q - (cp - cm) = 0      (Q >= 0 if one_way)
/// ```
#[must_use]
pub fn step(r: Option<f64>, one_way: bool, cp: f64, bu: f64, cm: f64, bd: f64) -> ValveStep {
    let drive = cp - cm;
    let flow = match r {
        Some(_) if one_way && drive <= 0.0 => 0.0,
        Some(r) => {
            let b = bu + bd;
            let (sign, d) = (drive.signum(), drive.abs());
            if r > 0.0 {
                // The stable root, written to avoid cancellation.
                sign * 2.0 * d / (b + (b * b + 4.0 * r * d).sqrt())
            } else {
                drive / b
            }
        }
        None => 0.0,
    };
    ValveStep {
        head_upstream: cp - bu * flow,
        head_downstream: cm + bd * flow,
        flow,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn orifice_balances_both_characteristics() {
        let (r, cp, bu, cm, bd) = (12_000.0, 90.0, 1400.0, 40.0, 1500.0);
        let s = step(Some(r), true, cp, bu, cm, bd);
        let loss = s.head_upstream - s.head_downstream;
        assert!((loss - r * s.flow * s.flow).abs() < 1e-9);
        assert!(s.flow > 0.0);
    }

    #[test]
    #[allow(clippy::float_cmp)] // exactly zero by construction
    fn one_way_never_backwards_and_shut_is_shut() {
        let back = step(Some(1000.0), true, 40.0, 1400.0, 90.0, 1500.0);
        assert_eq!(back.flow, 0.0);
        assert_eq!((back.head_upstream, back.head_downstream), (40.0, 90.0));
        let shut = step(None, true, 90.0, 1400.0, 40.0, 1500.0);
        assert_eq!(shut.flow, 0.0);
    }

    #[test]
    fn a_throttle_passes_both_ways() {
        let (r, bu, bd) = (5000.0, 1400.0, 1500.0);
        let s = step(Some(r), false, 40.0, bu, 90.0, bd);
        assert!(s.flow < 0.0);
        let loss = s.head_upstream - s.head_downstream;
        assert!((loss - r * s.flow * s.flow.abs()).abs() < 1e-9);
    }
}
