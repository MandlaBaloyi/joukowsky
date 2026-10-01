//! Small oscillations in the frequency domain: the transfer-matrix model
//! (solver S9; Chaudhry, Applied Hydraulic Transients, ch. 12; Wylie &
//! Streeter ch. 12; `docs/02_physics_of_transients/
//! governing_equations_and_moc.txt` section 7).
//!
//! For small oscillations at angular frequency w, head h and flow q along a
//! pipe vary as e^{iwt}, and the linearised water-hammer equations give,
//! per unit length, a series impedance z = R' + iw/(gA) (friction R' =
//! f|Q0|/(gDA^2), linearised about the steady flow Q0, and the water's
//! inertia) and a shunt admittance y = iw gA/a^2 (the pipe's and water's
//! compliance). Over a pipe of length L, with gamma = sqrt(z y) and
//! Zc = sqrt(z/y):
//!
//! ```text
//!   h_down = cosh(gL) h_up - Zc sinh(gL) q_up
//!   q_down = -sinh(gL)/Zc h_up + cosh(gL) q_up
//!   =>  q_up   = (cosh(gL) h_up - h_down) / (Zc sinh(gL))
//!       q_down = (h_up - cosh(gL) h_down) / (Zc sinh(gL))
//! ```
//!
//! At every junction the flows balance; reservoirs hold their head (one
//! may be DRIVEN with a unit oscillation). A linear system in the complex
//! node heads, solved exactly. A method independent of the
//! method of characteristics - no time steps, no grid - so the two
//! checking each other closes the circle of a simulator tested only
//! against itself (gap G-01).

use std::ops::{Add, Div, Mul, Neg, Sub};

use crate::HydraulicsError;
use crate::model::{Network, Node, NodeId};

/// A complex number (no external crate: TD-005).
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Complex {
    /// Real part.
    pub re: f64,
    /// Imaginary part.
    pub im: f64,
}

impl Complex {
    /// re + i im.
    #[must_use]
    pub const fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    /// |z|.
    #[must_use]
    pub fn abs(self) -> f64 {
        self.re.hypot(self.im)
    }

    /// The angle of z [rad].
    #[must_use]
    pub fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }

    fn sqrt(self) -> Self {
        let r = self.abs().sqrt();
        let t = self.arg() / 2.0;
        Self::new(r * t.cos(), r * t.sin())
    }

    fn cosh(self) -> Self {
        Self::new(
            self.re.cosh() * self.im.cos(),
            self.re.sinh() * self.im.sin(),
        )
    }

    fn sinh(self) -> Self {
        Self::new(
            self.re.sinh() * self.im.cos(),
            self.re.cosh() * self.im.sin(),
        )
    }
}

impl Add for Complex {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.re + o.re, self.im + o.im)
    }
}

impl Sub for Complex {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.re - o.re, self.im - o.im)
    }
}

impl Mul for Complex {
    type Output = Self;
    fn mul(self, o: Self) -> Self {
        Self::new(
            self.re * o.re - self.im * o.im,
            self.re * o.im + self.im * o.re,
        )
    }
}

impl Div for Complex {
    type Output = Self;
    fn div(self, o: Self) -> Self {
        let d = o.re * o.re + o.im * o.im;
        Self::new(
            (self.re * o.re + self.im * o.im) / d,
            (self.im * o.re - self.re * o.im) / d,
        )
    }
}

impl Neg for Complex {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.re, -self.im)
    }
}

/// The complex head amplitude at every node when reservoir `driven`
/// oscillates with unit amplitude at `frequency` [Hz] and every other
/// reservoir holds still. `wave_speeds` (per pipe, [m/s]) default to the
/// pipes' own; `steady_flows` (per pipe, [m^3/s]) linearise friction about
/// them - none: oscillation about rest, where friction is of second order.
///
/// # Errors
/// A driven node that is not a reservoir, a node kind the model does not
/// cover yet (valves, pumps, check and control valves), a frequency that
/// is not positive, or a network that leaves the heads undetermined.
pub fn response(
    network: &Network,
    driven: NodeId,
    frequency: f64,
    wave_speeds: Option<&[f64]>,
    steady_flows: Option<&[f64]>,
) -> Result<Vec<Complex>, HydraulicsError> {
    if !(frequency > 0.0 && frequency.is_finite()) {
        return Err(HydraulicsError::invalid(
            "frequency",
            frequency,
            "positive and finite",
        ));
    }
    let nodes = network.nodes();
    if !matches!(nodes.get(driven.index()), Some(Node::Reservoir { .. })) {
        return Err(HydraulicsError::InvalidNetwork(
            "the driven node must be a reservoir".into(),
        ));
    }
    if let Some((n, _)) = nodes
        .iter()
        .enumerate()
        .find(|(_, n)| !matches!(n, Node::Reservoir { .. } | Node::Junction { .. }))
    {
        return Err(HydraulicsError::InvalidNetwork(format!(
            "node {n} is a device the frequency model does not cover yet"
        )));
    }
    let g = network.gravity().value();
    let w = std::f64::consts::TAU * frequency;
    let n = nodes.len();
    // Nodal admittance system: A h = b.
    let mut a = vec![vec![Complex::default(); n]; n];
    let mut b = vec![Complex::default(); n];
    for (p, np) in network.pipes().iter().enumerate() {
        let pipe = &np.pipe;
        let area = pipe.area().value();
        let c = wave_speeds.map_or(pipe.wave_speed().value(), |s| s[p]);
        let r = steady_flows.map_or(0.0, |q| {
            pipe.friction_factor() * q[p].abs() / (g * pipe.diameter().value() * area * area)
        });
        let z = Complex::new(r, w / (g * area));
        let y = Complex::new(0.0, w * g * area / (c * c));
        let gamma_l = (z * y).sqrt() * Complex::new(pipe.length().value(), 0.0);
        let zc = (z / y).sqrt();
        let (ch, sh) = (gamma_l.cosh(), gamma_l.sinh());
        let d = zc * sh;
        let (u, v) = (np.from.index(), np.to.index());
        // Flow leaving u into the pipe, and arriving at v from it.
        //   q_up   = (ch h_u - h_v) / d       (leaves u)
        //   q_down = (h_u - ch h_v) / d       (arrives at v)
        a[u][u] = a[u][u] + ch / d;
        a[u][v] = a[u][v] - Complex::new(1.0, 0.0) / d;
        a[v][u] = a[v][u] - Complex::new(1.0, 0.0) / d;
        a[v][v] = a[v][v] + ch / d;
    }
    // Reservoirs: fixed heads (1 at the driven one, 0 elsewhere).
    for (k, node) in nodes.iter().enumerate() {
        if matches!(node, Node::Reservoir { .. }) {
            a[k] = vec![Complex::default(); n];
            a[k][k] = Complex::new(1.0, 0.0);
            b[k] = Complex::new(if k == driven.index() { 1.0 } else { 0.0 }, 0.0);
        }
    }
    solve(a, b)
}

/// Gaussian elimination with partial pivoting.
fn solve(mut a: Vec<Vec<Complex>>, mut b: Vec<Complex>) -> Result<Vec<Complex>, HydraulicsError> {
    let n = b.len();
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))
            .expect("non-empty");
        if a[pivot][col].abs() < 1e-300 {
            return Err(HydraulicsError::InvalidNetwork(
                "the oscillating heads are undetermined (a resonance, or a node cut off)".into(),
            ));
        }
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..n {
            let f = a[row][col] / a[col][col];
            if f.abs() == 0.0 {
                continue;
            }
            let (top, bottom) = a.split_at_mut(row);
            for (x, p) in bottom[0][col..].iter_mut().zip(&top[col][col..]) {
                *x = *x - f * *p;
            }
            let sub = f * b[col];
            b[row] = b[row] - sub;
        }
    }
    let mut x = vec![Complex::default(); n];
    for row in (0..n).rev() {
        let mut s = b[row];
        for k in row + 1..n {
            s = s - a[row][k] * x[k];
        }
        x[row] = s / a[row][row];
    }
    Ok(x)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Pipe;
    use jk_kernel::{Flow, Head, Length, STANDARD_GRAVITY, WaveSpeed};

    #[test]
    fn a_dead_end_answers_one_over_cos() {
        // Reservoir -> 1000 m -> dead end: h_end / h_res = 1 / cos(wL/a).
        let mut net = Network::new(STANDARD_GRAVITY);
        let r = net.add_node(Node::Reservoir {
            head: Head::new(50.0),
        });
        let e = net.add_node(Node::Junction { demand: Flow::ZERO });
        let pipe = Pipe::new(
            Length::new(1000.0),
            Length::new(0.2),
            WaveSpeed::new(1000.0),
            0.02,
        )
        .unwrap();
        net.add_pipe(r, e, pipe).unwrap();
        for f in [0.1, 0.13, 0.4] {
            let h = response(&net, r, f, None, None).unwrap();
            let expected = 1.0 / (std::f64::consts::TAU * f).cos();
            assert!((h[e.index()].re - expected).abs() < 1e-9 * expected.abs());
            assert!(h[e.index()].im.abs() < 1e-9);
        }
    }
}
