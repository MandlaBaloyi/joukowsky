//! Junction: any number of pipes meeting at one node, with an optional
//! fixed outflow (a demand, or a burst of known size).
//!
//! `docs/02_physics_of_transients/governing_equations_and_moc.txt` section 2:
//! all pipes share one head at the node, and the flows balance. Each pipe
//! brings one characteristic to the node:
//!   - a pipe whose DOWNSTREAM end is here brings `C_P`:  Q = (`C_P` - H) / B,
//!     flowing INTO the node;
//!   - a pipe whose UPSTREAM end is here brings `C_M`:    Q = (H - `C_M`) / B,
//!     flowing OUT of the node.
//!
//! Continuity, sum(in) - sum(out) = outflow, then gives the head directly:
//!
//! ```text
//!   H = ( sum(C / B) - outflow ) / sum(1 / B)
//! ```
//!
//! Minor losses at the junction are neglected (docs: to add when a network
//! needs them). A pressure-dependent outflow (leak, burst) is in `leak.rs`.

/// Which end of a pipe sits at the node - decides which characteristic
/// arrives and which way positive flow points.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PipeEnd {
    /// The pipe's downstream end (its last node) is here; C+ arrives.
    Downstream,
    /// The pipe's upstream end (its first node) is here; C- arrives.
    Upstream,
}

/// One pipe's contribution at a junction node.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Arrival {
    /// `C_P` for a downstream end, `C_M` for an upstream end [m].
    pub characteristic: f64,
    /// The pipe's characteristic impedance B [s/m^2].
    pub impedance: f64,
    /// Which end of the pipe is at the node.
    pub end: PipeEnd,
}

impl Arrival {
    /// The flow in this pipe at the node once the node head is known,
    /// positive in the pipe's own direction (upstream -> downstream).
    #[must_use]
    #[inline]
    pub fn flow(&self, head: f64) -> f64 {
        match self.end {
            PipeEnd::Downstream => (self.characteristic - head) / self.impedance,
            PipeEnd::Upstream => (head - self.characteristic) / self.impedance,
        }
    }
}

/// The two sums every junction-type node needs: sum(C / B) and sum(1 / B).
#[must_use]
#[inline]
pub(crate) fn sums(arrivals: &[Arrival]) -> (f64, f64) {
    arrivals.iter().fold((0.0, 0.0), |(c, s), a| {
        (c + a.characteristic / a.impedance, s + 1.0 / a.impedance)
    })
}

/// Head at a junction with a fixed `outflow` leaving the network there
/// (0 for a plain junction). Use [`Arrival::flow`] for each pipe's flow.
///
/// With one pipe and no outflow this is a dead end; with two pipes it is a
/// series connection (e.g. a change of pipe material).
///
/// # Panics
/// In debug builds, if `arrivals` is empty.
#[must_use]
pub fn head(arrivals: &[Arrival], outflow: f64) -> f64 {
    debug_assert!(!arrivals.is_empty(), "a junction needs at least one pipe");
    let (c, s) = sums(arrivals);
    (c - outflow) / s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn arrival(characteristic: f64, impedance: f64, end: PipeEnd) -> Arrival {
        Arrival {
            characteristic,
            impedance,
            end,
        }
    }

    #[test]
    fn flows_balance_and_every_pipe_satisfies_its_characteristic() {
        let arrivals = [
            arrival(120.0, 5000.0, PipeEnd::Downstream),
            arrival(95.0, 3000.0, PipeEnd::Upstream),
            arrival(101.0, 12000.0, PipeEnd::Upstream),
        ];
        let outflow = 0.002;
        let h = head(&arrivals, outflow);
        let inflow = arrivals[0].flow(h);
        let outflows = arrivals[1].flow(h) + arrivals[2].flow(h);
        assert!((inflow - outflows - outflow).abs() < 1e-15, "continuity");
        for a in &arrivals {
            let q = a.flow(h);
            let back = match a.end {
                PipeEnd::Downstream => a.characteristic - a.impedance * q,
                PipeEnd::Upstream => a.characteristic + a.impedance * q,
            };
            assert!((back - h).abs() < 1e-12, "characteristic violated");
        }
    }

    #[test]
    fn a_single_pipe_without_outflow_is_a_dead_end() {
        let a = [arrival(150.0, 5000.0, PipeEnd::Downstream)];
        let h = head(&a, 0.0);
        assert!((h - 150.0).abs() < 1e-12);
        assert!(a[0].flow(h).abs() < 1e-15);
    }
}
