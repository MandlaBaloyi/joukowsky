//! Vapour cavities on the network grid: the discrete vapour cavity model
//! (DVCM; Wylie & Streeter 1993, review Bergant, Simpson & Tijsseling
//! 2006; `docs/02_physics_of_transients/column_separation_and_intrusion.txt`
//! section 5).
//!
//! Where a point's head would fall below its vapour floor (the floor as a
//! gauge head plus the pipe's elevation there), the water boils: the head
//! is held AT the floor, the two characteristics then give two different
//! flows - into the point from upstream (Qu, from C+) and out of it
//! downstream (Qd, from C-) - and the difference fills a vapour cavity:
//!
//! ```text
//!   H  = floor
//!   Qu = (C_P - H) / B,   Qd = (H - C_M) / B
//!   V  = V_old + dt (Qd - Qu)          (psi = 1: the new flows)
//! ```
//!
//! While V > 0 the cavity stays; when the water returns and V would fall
//! to 0 or below, it collapses: the point is solved as usual again, and the
//! columns meeting there make the collapse surge. At a node the same
//! balance uses every pipe's characteristic and the node's own outflow
//! (demand, leak, valve) at the floor head.
//!
//! A two-sided device (pump, check valve, control valve) can hold a cavity
//! on either side ([`two_sided`]): that side's head is then fixed at the
//! floor, which to the device is a characteristic with impedance 0, so
//! every device law is used unchanged. After a pump trip this is where the
//! column tears away from the pump.
//!
//! The cavities are an exact water budget: water missing from the columns
//! is the cavity volume. psi = 1 [VERIFY against psi = 0.5 on Bergant &
//! Simpson's laboratory runs when that data is approved].
//!
//! GAS cavities (DGCM, [`crate::vapour::FreeGas`]): with a gas constant
//! `c3` = alpha0 Vreach (h0 - floor) > 0 at a point, its gas volume obeys
//! `Vg (H - H_floor) = c3` at all times, and the same balance holds:
//!
//! ```text
//!   c3 / y = Vg_old + dt (Qd - Qu),   y = H - H_floor > 0
//!   Qd - Qu = (2 H - C_P - C_M) / B
//!   =>  s y^2 + k y - c3 = 0,  s = 2 dt / B,  k = Vg_old + dt (2 H_floor - C_P - C_M) / B
//! ```
//!
//! one positive root, solved without cancellation. The head never reaches
//! the floor; as c3 -> 0 the gas cavity becomes the vapour cavity.

use crate::boundary::junction::{Arrival, PipeEnd};

/// One grid point inside a pipe, from the characteristics arriving at it
/// (`c_plus` = `C_P`, `c_minus` = `C_M`, impedance `b`), its piezometric
/// `floor` and its cavity `volume` before the step. Returns (head, flow on
/// the upstream side, flow on the downstream side, cavity volume).
#[must_use]
#[inline]
pub fn point(
    c_plus: f64,
    c_minus: f64,
    b: f64,
    floor: f64,
    volume: f64,
    dt: f64,
) -> (f64, f64, f64, f64) {
    point_gas(c_plus, c_minus, b, floor, (volume, 0.0), dt)
}

/// As [`point`] with free gas: `gas` = (volume before the step, gas
/// constant c3). With c3 = 0 it is the vapour cavity of [`point`].
#[must_use]
#[inline]
pub fn point_gas(
    c_plus: f64,
    c_minus: f64,
    b: f64,
    floor: f64,
    (volume, c3): (f64, f64),
    dt: f64,
) -> (f64, f64, f64, f64) {
    if c3 > 0.0 {
        let s = 2.0 * dt / b;
        let k = volume + dt * (2.0 * floor - c_plus - c_minus) / b;
        let y = positive_root(s, k, c3);
        let head = floor + y;
        return (head, (c_plus - head) / b, (head - c_minus) / b, c3 / y);
    }
    let head = f64::midpoint(c_plus, c_minus);
    if volume > 0.0 || head < floor {
        let upstream = (c_plus - floor) / b;
        let downstream = (floor - c_minus) / b;
        let v = volume + dt * (downstream - upstream);
        if v > 0.0 {
            return (floor, upstream, downstream, v);
        }
    }
    let q = (c_plus - c_minus) / (2.0 * b);
    (head, q, q, 0.0)
}

/// The positive root y of s y^2 + k y - c = 0 (s, c > 0), in the form
/// without cancellation.
#[inline]
fn positive_root(s: f64, k: f64, c: f64) -> f64 {
    let d = (k * k + 4.0 * s * c).sqrt();
    if k > 0.0 {
        2.0 * c / (k + d)
    } else {
        (d - k) / (2.0 * s)
    }
}

/// A node with free gas (gas constant `c3` > 0, gas volume `volume` before
/// the step): finds y = H - floor > 0 with c3 / y = `volume` + dt (outflow
/// minus inflow) at H, the outflow law `outflow_at` being any increasing
/// function (demand, leak, valve). Returns (head, outflow, gas volume).
#[must_use]
pub fn node_gas(
    arrivals: &[Arrival],
    floor: f64,
    (volume, c3): (f64, f64),
    dt: f64,
    outflow_at: impl Fn(f64) -> f64,
) -> (f64, f64, f64) {
    let inflow = |h: f64| -> f64 {
        arrivals
            .iter()
            .map(|a| match a.end {
                PipeEnd::Downstream => a.flow(h),
                PipeEnd::Upstream => -a.flow(h),
            })
            .sum()
    };
    // Increasing in y, from -infinity at y -> 0.
    let f = |y: f64| volume + dt * (outflow_at(floor + y) - inflow(floor + y)) - c3 / y;
    let mut hi = 1.0;
    while f(hi) < 0.0 && hi < 1e7 {
        hi *= 2.0;
    }
    let mut lo = hi;
    while f(lo) > 0.0 && lo > 1e-300 {
        lo *= 0.5;
    }
    for _ in 0..200 {
        let mid = f64::midpoint(lo, hi);
        if mid <= lo || mid >= hi {
            break;
        }
        if f(mid) < 0.0 {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let y = f64::midpoint(lo, hi);
    (floor + y, outflow_at(floor + y), c3 / y)
}

/// A node, given its usual solution (`head`, `outflow`) and how its own
/// outflow depends on its head (`outflow_at`): if it cavitates, the head
/// is held at `floor` and the cavity takes up the imbalance. Returns
/// (head, outflow, cavity volume).
#[must_use]
pub fn node(
    arrivals: &[Arrival],
    (head, outflow): (f64, f64),
    floor: f64,
    volume: f64,
    dt: f64,
    outflow_at: impl Fn(f64) -> f64,
) -> (f64, f64, f64) {
    if volume > 0.0 || head < floor {
        let inflow: f64 = arrivals
            .iter()
            .map(|a| match a.end {
                PipeEnd::Downstream => a.flow(floor),
                PipeEnd::Upstream => -a.flow(floor),
            })
            .sum();
        let out = outflow_at(floor);
        let v = volume + dt * (out - inflow);
        if v > 0.0 {
            return (floor, out, v);
        }
    }
    (head, outflow, 0.0)
}

/// What a two-sided device's step gives with cavities: heads and pipe
/// flows on each side, the device's own result, and each side's cavity.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TwoSided<T> {
    /// Head on the upstream (inlet) side.
    pub head_upstream: f64,
    /// Head on the downstream (outlet) side.
    pub head_downstream: f64,
    /// Flow arriving in the upstream pipe.
    pub flow_upstream: f64,
    /// Flow leaving into the downstream pipe.
    pub flow_downstream: f64,
    /// The device's own result at the heads found.
    pub device: T,
    /// Cavity volume on the upstream side.
    pub volume_upstream: f64,
    /// Cavity volume on the downstream side.
    pub volume_downstream: f64,
}

/// A two-sided device between an upstream pipe (bringing `C_P`, impedance
/// `bu`) and a downstream pipe (bringing `C_M`, impedance `bd`), with
/// cavities `volumes` (upstream, downstream) before the step. `solve`
/// steps the device for given characteristics and returns (upstream head,
/// downstream head, flow through it, its own result). A side whose head
/// would fall below `floor` - or whose cavity is still open - is held at
/// the floor (impedance 0) and its cavity takes up the difference between
/// the pipe's flow and the device's; a cavity the water fills collapses
/// and the side is solved as usual again. A few passes settle which sides
/// cavitate.
#[must_use]
pub fn two_sided<T: Copy>(
    (c_plus, bu): (f64, f64),
    (c_minus, bd): (f64, f64),
    floor: f64,
    (volume_up, volume_down): (f64, f64),
    dt: f64,
    solve: impl Fn(f64, f64, f64, f64) -> (f64, f64, f64, T),
) -> TwoSided<T> {
    let (mut up, mut down) = (volume_up > 0.0, volume_down > 0.0);
    let mut pass = 0;
    loop {
        let (cu, bu_side) = if up { (floor, 0.0) } else { (c_plus, bu) };
        let (cd, bd_side) = if down { (floor, 0.0) } else { (c_minus, bd) };
        let (hu, hd, q, device) = solve(cu, bu_side, cd, bd_side);
        let (hu, hd) = (if up { floor } else { hu }, if down { floor } else { hd });
        let flow_upstream = (c_plus - hu) / bu;
        let flow_downstream = (hd - c_minus) / bd;
        let vu = if up {
            volume_up + dt * (q - flow_upstream)
        } else {
            0.0
        };
        let vd = if down {
            volume_down + dt * (flow_downstream - q)
        } else {
            0.0
        };
        let next_up = if up { vu > 0.0 } else { hu < floor };
        let next_down = if down { vd > 0.0 } else { hd < floor };
        pass += 1;
        if (next_up == up && next_down == down) || pass == 4 {
            return TwoSided {
                head_upstream: hu,
                head_downstream: hd,
                flow_upstream: if up { flow_upstream } else { q },
                flow_downstream: if down { flow_downstream } else { q },
                device,
                volume_upstream: vu.max(0.0),
                volume_downstream: vd.max(0.0),
            };
        }
        (up, down) = (next_up, next_down);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A shut device: nothing passes, each side a dead end.
    fn shut(cu: f64, _bu: f64, cd: f64, _bd: f64) -> (f64, f64, f64, ()) {
        (cu, cd, 0.0, ())
    }

    #[test]
    fn a_column_pulling_away_from_a_shut_device_opens_a_cavity_behind_it() {
        // Downstream: C_M = -40 m would pull the outlet side to -40 m.
        let (b, dt) = (100.0, 1e-3);
        let s = two_sided((50.0, b), (-40.0, b), -10.0, (0.0, 0.0), dt, shut);
        assert!(
            (s.head_upstream - 50.0).abs() < 1e-12,
            "inlet side untouched"
        );
        assert!(
            (s.head_downstream + 10.0).abs() < 1e-12,
            "held at the floor"
        );
        let q_out = (-10.0 + 40.0) / b;
        assert!((s.flow_downstream - q_out).abs() < 1e-15);
        assert!((s.volume_downstream - dt * q_out).abs() < 1e-18);
        assert!(s.volume_upstream.abs() < 1e-18);
    }

    #[test]
    fn the_cavity_collapses_when_the_column_comes_back() {
        // C_M = +60 m: the column pushes back into a 1e-6 m^3 cavity.
        let s = two_sided((50.0, 100.0), (60.0, 100.0), -10.0, (0.0, 1e-6), 1e-3, shut);
        assert!(s.volume_downstream.abs() < 1e-18);
        assert!((s.head_downstream - 60.0).abs() < 1e-12, "a dead end again");
    }

    #[test]
    fn above_the_floor_nothing_changes() {
        let (h, qu, qd, v) = point(60.0, 40.0, 100.0, -10.0, 0.0, 1e-3);
        assert!((h - 50.0).abs() < 1e-12);
        assert!((qu - 0.1).abs() < 1e-12 && (qu - qd).abs() < 1e-18 && v.abs() < 1e-18);
    }

    #[test]
    fn below_the_floor_a_cavity_opens_and_takes_the_difference() {
        // Usual head would be -30 m; held at -10 m instead.
        let (b, dt) = (100.0, 1e-3);
        let (cp, cm) = (-20.0, -40.0);
        let (h, qu, qd, v) = point(cp, cm, b, -10.0, 0.0, dt);
        assert!((h + 10.0).abs() < 1e-12);
        assert!((qu - (cp + 10.0) / b).abs() < 1e-15, "C+ holds");
        assert!((qd - (-10.0 - cm) / b).abs() < 1e-15, "C- holds");
        assert!((v - dt * (qd - qu)).abs() < 1e-18 && v > 0.0);
    }

    #[test]
    fn gas_obeys_its_law_and_both_characteristics() {
        let (b, dt, floor, c3) = (100.0, 1e-3, -10.0, 1e-4);
        let (cp, cm) = (-20.0, -40.0);
        let v_old = c3 / 5.0;
        let (h, qu, qd, v) = point_gas(cp, cm, b, floor, (v_old, c3), dt);
        assert!(h > floor, "never reaches the floor");
        assert!((v * (h - floor) - c3).abs() < 1e-15, "gas law");
        assert!((v - v_old - dt * (qd - qu)).abs() < 1e-15, "water budget");
        assert!((h - (cp - b * qu)).abs() < 1e-12 && (h - (cm + b * qd)).abs() < 1e-12);
    }

    #[test]
    fn a_trace_of_gas_is_a_vapour_cavity() {
        let (b, dt, floor) = (100.0, 1e-3, -10.0);
        let vapour = point(-20.0, -40.0, b, floor, 0.0, dt);
        let gas = point_gas(-20.0, -40.0, b, floor, (1e-15, 1e-15), dt);
        assert!((vapour.0 - gas.0).abs() < 1e-9);
        assert!((vapour.3 - gas.3).abs() < 1e-12);
    }

    #[test]
    fn node_gas_balances_its_flows() {
        use crate::boundary::junction::PipeEnd;
        let arrivals = [Arrival {
            characteristic: -40.0,
            impedance: 100.0,
            end: PipeEnd::Downstream,
        }];
        let (dt, floor, c3, v_old) = (1e-3, -10.0, 1e-4, 1e-5);
        let (h, out, v) = node_gas(&arrivals, floor, (v_old, c3), dt, |_| 0.002);
        let inflow = arrivals[0].flow(h);
        assert!((v * (h - floor) - c3).abs() < 1e-12 * c3.max(1.0));
        assert!((v - v_old - dt * (out - inflow)).abs() < 1e-12);
    }

    #[test]
    fn a_cavity_the_water_fills_collapses() {
        // Columns closing in fast enough to fill the 1e-6 m^3 cavity.
        let (h, qu, qd, v) = point(80.0, 60.0, 100.0, -10.0, 1e-6, 1e-3);
        assert!(v.abs() < 1e-18);
        assert!((h - 70.0).abs() < 1e-12 && (qu - qd).abs() < 1e-18);
    }
}
