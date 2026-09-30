//! Pressure-reducing valve (PRV): holds the head just downstream at its
//! setting while the head upstream allows (`docs/02_physics_of_transients/
//! governing_equations_and_moc.txt`, section 6).
//!
//! In a steady solve it is ACTIVE (downstream head = setting), OPEN (fully
//! open, a friction loss) or CLOSED (no flow; it also stops reverse flow).
//! In a transient it is FROZEN at its steady opening: a pilot-operated PRV
//! needs seconds to move, a wave crosses a site in well under one. Frozen,
//! it is an orifice between its two pipes that never passes flow
//! backwards.

/// What a PRV is doing in a steady state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PrvState {
    /// Holding the downstream head at its setting.
    Active,
    /// Fully open: the head upstream is below the setting.
    Open,
    /// Shut: the flow would run backwards (the head downstream is above
    /// what it would hold).
    Closed,
}

/// Heads on both sides of the valve and the flow through it.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PrvStep {
    /// Head at the end of the upstream pipe [m].
    pub head_upstream: f64,
    /// Head at the start of the downstream pipe [m].
    pub head_downstream: f64,
    /// Flow through the valve [m^3/s], never negative.
    pub flow: f64,
}

/// One time step of a frozen PRV: an orifice with head loss `r` Q^2
/// (`None`: shut), between the upstream C+ (`H = cp - bu Q`) and the
/// downstream C- (`H = cm + bd Q`):
///
/// ```text
///   r Q^2 + (bu + bd) Q - (cp - cm) = 0,   Q >= 0
/// ```
#[must_use]
pub fn step(r: Option<f64>, cp: f64, bu: f64, cm: f64, bd: f64) -> PrvStep {
    let drive = cp - cm;
    let flow = match r {
        Some(r) if drive > 0.0 => {
            let b = bu + bd;
            if r > 0.0 {
                // The stable root, written to avoid cancellation.
                2.0 * drive / (b + (b * b + 4.0 * r * drive).sqrt())
            } else {
                drive / b
            }
        }
        _ => 0.0,
    };
    PrvStep {
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
        let s = step(Some(r), cp, bu, cm, bd);
        let loss = s.head_upstream - s.head_downstream;
        assert!((loss - r * s.flow * s.flow).abs() < 1e-9);
        assert!(s.flow > 0.0);
    }

    #[test]
    #[allow(clippy::float_cmp)] // exactly zero by construction
    fn never_backwards_and_shut_is_shut() {
        let back = step(Some(1000.0), 40.0, 1400.0, 90.0, 1500.0);
        assert_eq!(back.flow, 0.0);
        assert_eq!((back.head_upstream, back.head_downstream), (40.0, 90.0));
        let shut = step(None, 90.0, 1400.0, 40.0, 1500.0);
        assert_eq!(shut.flow, 0.0);
    }
}
