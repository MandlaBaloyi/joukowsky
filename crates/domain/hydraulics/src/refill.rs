//! Refilling a pipe after a supply cut: a rigid water column driven by
//! the returning supply, compressing (and venting) the air ahead of it
//! (`docs/02_physics_of_transients/refilling_and_air_pockets.txt`; R-5,
//! gap G-14).
//!
//! ```text
//!   dx/dt = v
//!   x dv/dt = g (H - z_f) - p_a / rho - (1 + K) v^2 / 2 - f x v|v| / (2 D)
//!   dp/dt = n p ( (dm/dt)/m - (dV/dt)/V ),   V = A (L - x),   dm/dt = -air out
//! ```
//!
//! Liou & Hunt (1996), Zhou, Hicks & Steffler (2002). If the column
//! reaches the closed end it SLAMS: Joukowsky, dH = a v / g.

use jk_kernel::{Acceleration, Head, Length, Pressure, Time, Velocity, WaveSpeed};

use crate::HydraulicsError;
use crate::boundary::pump::WATER_DENSITY;
use crate::error::positive;

/// Ratio of specific heats of air.
const GAMMA: f64 = 1.4;
/// Gas constant of air [J/(kg K)].
const R_AIR: f64 = 287.05;
/// Air temperature for the vent's flow [K] (20 C) [VERIFY on site].
const T_AIR: f64 = 293.15;
/// Longest integration step [s].
const MAX_STEP: f64 = 1e-3;

/// An air valve the pocket vents through: an orifice.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vent {
    /// Orifice diameter.
    pub diameter: Length,
    /// Discharge coefficient (about 0.6 for a sharp orifice [VERIFY per
    /// valve]).
    pub discharge: f64,
}

/// The air ahead of the column.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Air {
    /// The air leaves freely (a large opening): no cushion at all.
    Free,
    /// A pocket at atmospheric pressure at the start, sealed (`vent`
    /// None) or venting through an air valve; compressed and expanded
    /// polytropically (n = 1 isothermal ... 1.4 adiabatic).
    Pocket {
        /// Polytropic exponent n.
        polytropic: f64,
        /// The air valve, if any.
        vent: Option<Vent>,
    },
}

/// A pipe refilling after a supply cut.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Refill {
    /// Pipe length, from the supply's entrance to the closed end.
    pub length: Length,
    /// Inner diameter.
    pub diameter: Length,
    /// Darcy friction factor.
    pub friction: f64,
    /// Wave speed (for the slam at the end).
    pub wave_speed: WaveSpeed,
    /// Elevation of the closed end above the entrance (negative: falling).
    pub rise: Length,
    /// Entrance loss coefficient K.
    pub entrance_loss: f64,
    /// Supply head, piezometric, above the entrance.
    pub supply: Head,
    /// Length of the column already in the pipe at the start (> 0).
    pub start: Length,
    /// Atmospheric pressure, absolute (about 101 kPa at the coast, 83-86
    /// kPa on the Highveld).
    pub atmosphere: Pressure,
    /// The air ahead.
    pub air: Air,
    /// Gravity.
    pub gravity: Acceleration,
}

/// The column reaching the closed end.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Slam {
    /// When.
    pub at: Time,
    /// The column's velocity then.
    pub velocity: Velocity,
    /// The surge it makes: Joukowsky, a v / g.
    pub surge: Head,
}

/// A refilling run.
#[derive(Debug, Clone, PartialEq)]
pub struct RefillRun {
    /// Recorded times.
    pub times: Vec<Time>,
    /// Column length (front position) at each.
    pub front: Vec<Length>,
    /// Column velocity at each.
    pub velocity: Vec<Velocity>,
    /// Air pressure ahead of the column, gauge, as metres of water, at each.
    pub air_head: Vec<Head>,
    /// The highest air pressure reached (gauge, m of water).
    pub peak_air_head: Head,
    /// The slam, if the column reached the end.
    pub slam: Option<Slam>,
}

/// State: front position x, velocity v, absolute air pressure p, air mass m.
#[derive(Debug, Clone, Copy)]
struct State {
    x: f64,
    v: f64,
    p: f64,
    m: f64,
}

impl Refill {
    fn area(&self) -> f64 {
        std::f64::consts::PI * 0.25 * self.diameter.value().powi(2)
    }

    /// Air mass flow OUT through the vent [kg/s] (negative: in) at absolute
    /// pressure `p`: isentropic orifice flow, choked when the pressure ratio
    /// passes the critical one.
    fn vented(&self, p: f64) -> f64 {
        let Air::Pocket {
            vent: Some(vent), ..
        } = self.air
        else {
            return 0.0;
        };
        let pa = self.atmosphere.value();
        let area = std::f64::consts::PI * 0.25 * vent.diameter.value().powi(2);
        let (high, low, sign) = if p >= pa { (p, pa, 1.0) } else { (pa, p, -1.0) };
        let ratio = low / high;
        let critical = (2.0 / (GAMMA + 1.0)).powf(GAMMA / (GAMMA - 1.0));
        let flux = if ratio <= critical {
            high * (GAMMA / (R_AIR * T_AIR)).sqrt()
                * (2.0 / (GAMMA + 1.0)).powf((GAMMA + 1.0) / (2.0 * (GAMMA - 1.0)))
        } else {
            high * (2.0 * GAMMA / ((GAMMA - 1.0) * R_AIR * T_AIR)
                * (ratio.powf(2.0 / GAMMA) - ratio.powf((GAMMA + 1.0) / GAMMA)))
            .sqrt()
        };
        sign * vent.discharge * area * flux
    }

    /// The rates of change of the state.
    fn rates(&self, s: State) -> State {
        let (g, a) = (self.gravity.value(), self.area());
        let length = self.length.value();
        let gauge = match self.air {
            Air::Free => 0.0,
            Air::Pocket { .. } => s.p - self.atmosphere.value(),
        };
        let z_front = self.rise.value() * s.x / length;
        let dv = (g * (self.supply.value() - z_front)
            - gauge / WATER_DENSITY
            - (1.0 + self.entrance_loss) * s.v * s.v / 2.0
            - self.friction * s.x * s.v * s.v.abs() / (2.0 * self.diameter.value()))
            / s.x;
        let (dp, dm) = match self.air {
            Air::Free => (0.0, 0.0),
            Air::Pocket { polytropic, .. } => {
                let volume = a * (length - s.x);
                let dm = -self.vented(s.p);
                let dvol = -a * s.v;
                (polytropic * s.p * (dm / s.m - dvol / volume), dm)
            }
        };
        State {
            x: s.v,
            v: dv,
            p: dp,
            m: dm,
        }
    }
}

/// Runs a refill for `duration`, recording every `record_every` - or until
/// the column slams into the end.
///
/// # Errors
/// A dimension, head or time that is not positive, a start column not
/// inside the pipe, or a polytropic exponent outside [1, 1.4].
#[allow(clippy::too_many_lines)] // one integration loop with its records
pub fn refill(
    r: &Refill,
    duration: Time,
    record_every: Time,
) -> Result<RefillRun, HydraulicsError> {
    positive("pipe length", r.length.value())?;
    positive("diameter", r.diameter.value())?;
    positive("wave speed", r.wave_speed.value())?;
    positive("atmospheric pressure", r.atmosphere.value())?;
    positive("duration", duration.value())?;
    positive("record interval", record_every.value())?;
    positive("gravity", r.gravity.value())?;
    let length = r.length.value();
    if !(r.start.value() > 0.0 && r.start.value() < length) {
        return Err(HydraulicsError::invalid(
            "refill start column",
            r.start.value(),
            "between 0 and the pipe length",
        ));
    }
    if let Air::Pocket { polytropic, vent } = r.air {
        if !(1.0..=GAMMA).contains(&polytropic) {
            return Err(HydraulicsError::invalid(
                "polytropic exponent",
                polytropic,
                "between 1 (isothermal) and 1.4 (adiabatic)",
            ));
        }
        if let Some(v) = vent {
            positive("vent diameter", v.diameter.value())?;
            positive("vent discharge coefficient", v.discharge)?;
        }
    }
    let (g, a) = (r.gravity.value(), r.area());
    let p_atm = r.atmosphere.value();
    let volume0 = a * (length - r.start.value());
    let mut s = State {
        x: r.start.value(),
        v: 0.0,
        p: p_atm,
        m: p_atm * volume0 / (R_AIR * T_AIR),
    };
    let head = |p: f64| (p - p_atm) / (WATER_DENSITY * g);
    let mut run = RefillRun {
        times: Vec::new(),
        front: Vec::new(),
        velocity: Vec::new(),
        air_head: Vec::new(),
        peak_air_head: Head::ZERO,
        slam: None,
    };
    let record = |run: &mut RefillRun, t: f64, s: &State| {
        run.times.push(Time::new(t));
        run.front.push(Length::new(s.x));
        run.velocity.push(Velocity::new(s.v));
        run.air_head.push(Head::new(match r.air {
            Air::Free => 0.0,
            Air::Pocket { .. } => head(s.p),
        }));
    };
    let (end, every) = (duration.value(), record_every.value());
    let mut t = 0.0;
    let mut next = 0.0;
    let mut peak: f64 = 0.0;
    while t < end - 1e-12 {
        if t >= next - 1e-12 {
            record(&mut run, t, &s);
            next += every;
        }
        // Step: the air volume (and the column) may change by 1% at most.
        let volume = a * (length - s.x);
        let speed = s.v.abs().max(1e-9);
        let mut dt = MAX_STEP
            .min(0.01 * volume / (a * speed))
            .min(0.01 * s.x / speed)
            .min(next - t)
            .min(end - t)
            .max(1e-12);
        // Land on the end if this step would reach it.
        let hits = matches!(r.air, Air::Free) || matches!(r.air, Air::Pocket { vent: Some(_), .. });
        if hits && s.v > 0.0 && s.x + s.v * dt >= length {
            dt = (length - s.x) / s.v;
        }
        let k1 = r.rates(s);
        let at = |k: &State, h: f64| State {
            x: s.x + h * k.x,
            v: s.v + h * k.v,
            p: s.p + h * k.p,
            m: s.m + h * k.m,
        };
        let k2 = r.rates(at(&k1, 0.5 * dt));
        let k3 = r.rates(at(&k2, 0.5 * dt));
        let k4 = r.rates(at(&k3, dt));
        let w = |f: fn(&State) -> f64| (f(&k1) + 2.0 * f(&k2) + 2.0 * f(&k3) + f(&k4)) / 6.0;
        s = State {
            x: s.x + dt * w(|k| k.x),
            v: s.v + dt * w(|k| k.v),
            p: (s.p + dt * w(|k| k.p)).max(1.0),
            m: (s.m + dt * w(|k| k.m)).max(0.0),
        };
        t += dt;
        if let Air::Pocket { .. } = r.air {
            peak = peak.max(head(s.p));
        }
        if s.x >= length * (1.0 - 1e-9) {
            run.slam = Some(Slam {
                at: Time::new(t),
                velocity: Velocity::new(s.v),
                surge: Head::new(r.wave_speed.value() * s.v / g),
            });
            break;
        }
        if s.x <= 1e-6 {
            break; // the air pushed the column back out
        }
    }
    record(&mut run, t, &s);
    run.peak_air_head = Head::new(peak);
    Ok(run)
}

#[cfg(test)]
mod tests {
    use super::*;
    use jk_kernel::STANDARD_GRAVITY;

    fn pipe(air: Air) -> Refill {
        Refill {
            length: Length::new(500.0),
            diameter: Length::new(0.2),
            friction: 0.0,
            wave_speed: WaveSpeed::new(1000.0),
            rise: Length::ZERO,
            entrance_loss: 0.0,
            supply: Head::new(30.0),
            start: Length::new(50.0),
            atmosphere: Pressure::new(101_325.0),
            air,
            gravity: STANDARD_GRAVITY,
        }
    }

    #[test]
    fn a_free_column_slams_and_a_sealed_pocket_holds() {
        let free = refill(&pipe(Air::Free), Time::new(100.0), Time::new(1.0)).unwrap();
        assert!(free.slam.is_some());
        let sealed = refill(
            &pipe(Air::Pocket {
                polytropic: 1.2,
                vent: None,
            }),
            Time::new(30.0),
            Time::new(0.1),
        )
        .unwrap();
        assert!(sealed.slam.is_none());
        assert!(sealed.peak_air_head.value() > 30.0);
    }

    #[test]
    fn a_vent_lets_air_out() {
        let r = pipe(Air::Pocket {
            polytropic: 1.4,
            vent: Some(Vent {
                diameter: Length::new(0.05),
                discharge: 0.6,
            }),
        });
        assert!(r.vented(150_000.0) > 0.0);
        assert!(r.vented(80_000.0) < 0.0);
        assert!(r.vented(101_325.0).abs() < 1e-12);
    }
}
