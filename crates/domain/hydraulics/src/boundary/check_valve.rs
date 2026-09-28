//! Check valve (non-return valve) between two pipes: a one-way door.
//!
//! Open: the two pipes are joined with no loss and share one head.
//! Closed: each side is a dead end. Transitions each time step:
//!   - Open -> Closed when the flow would reverse (Q < 0);
//!   - Closed -> Open when the upstream side pushes harder than the
//!     downstream side (`C_P` upstream > `C_M` downstream).
//!
//! This is the IDEAL check valve: it shuts the instant flow reverses. Real
//! discs have inertia and shut late, after reverse flow has built up - the
//! "slam" that follows a pump trip (`docs/06_engine_architecture/
//! outages_scale_and_errors.txt` 1b). The dynamic slam model comes with the
//! pump boundary; the ideal valve is its lower bound and a test reference.

/// Whether the valve is passing flow.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CheckValveState {
    /// Passing forward flow.
    Open,
    /// Shut: no flow.
    Closed,
}

/// Heads on both sides, the flow through, and the new state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CheckValveStep {
    /// Head at the last node of the upstream pipe [m].
    pub head_upstream: f64,
    /// Head at the first node of the downstream pipe [m].
    pub head_downstream: f64,
    /// Flow through the valve (>= 0) [m^3/s].
    pub flow: f64,
    /// State after this step.
    pub state: CheckValveState,
}

/// One time step of an ideal check valve between an upstream pipe (bringing
/// `C_P`, impedance `b_up`) and a downstream pipe (bringing `C_M`,
/// impedance `b_down`).
#[must_use]
pub fn step(
    state: CheckValveState,
    c_plus_up: f64,
    b_up: f64,
    c_minus_down: f64,
    b_down: f64,
) -> CheckValveStep {
    let open = || {
        let h = (c_plus_up / b_up + c_minus_down / b_down) / (1.0 / b_up + 1.0 / b_down);
        (h, (c_plus_up - h) / b_up)
    };
    let closed = CheckValveStep {
        head_upstream: c_plus_up,
        head_downstream: c_minus_down,
        flow: 0.0,
        state: CheckValveState::Closed,
    };
    let opened = |h: f64, q: f64| CheckValveStep {
        head_upstream: h,
        head_downstream: h,
        flow: q,
        state: CheckValveState::Open,
    };
    match state {
        CheckValveState::Open => {
            let (h, q) = open();
            if q < 0.0 { closed } else { opened(h, q) }
        }
        CheckValveState::Closed => {
            if c_plus_up > c_minus_down {
                let (h, q) = open();
                opened(h, q)
            } else {
                closed
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const B: f64 = 5000.0;

    #[test]
    fn open_valve_passes_forward_flow_like_a_plain_joint() {
        // Steady forward flow q0 at head h0 on both sides.
        let (h0, q0) = (60.0, 0.03);
        let s = step(CheckValveState::Open, h0 + B * q0, B, h0 - B * q0, B);
        assert_eq!(s.state, CheckValveState::Open);
        assert!((s.flow - q0).abs() < 1e-15);
        assert!((s.head_upstream - h0).abs() < 1e-12);
    }

    #[test]
    fn reversing_flow_shuts_it_and_each_side_becomes_a_dead_end() {
        let s = step(CheckValveState::Open, 40.0, B, 70.0, B);
        assert_eq!(s.state, CheckValveState::Closed);
        assert_eq!(
            (s.head_upstream, s.head_downstream, s.flow),
            (40.0, 70.0, 0.0)
        );
    }

    #[test]
    fn stays_shut_until_upstream_pushes_harder_then_reopens() {
        let s = step(CheckValveState::Closed, 69.0, B, 70.0, B);
        assert_eq!(s.state, CheckValveState::Closed);
        let s = step(CheckValveState::Closed, 71.0, B, 70.0, B);
        assert_eq!(s.state, CheckValveState::Open);
        assert!(s.flow > 0.0);
    }
}
