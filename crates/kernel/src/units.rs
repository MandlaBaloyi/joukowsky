//! SI physical quantities as zero-cost newtypes over `f64`.
//!
//! Conventions follow `docs/README.txt`: H head [m], p pressure [Pa],
//! Q flow [m^3/s], V mean velocity [m/s], a wave speed [m/s].
//!
//! Each quantity supports addition and subtraction with itself, scaling by
//! a plain `f64`, and division by itself (giving a dimensionless `f64`).
//! Products and quotients between different quantities exist only where
//! the physics needs them, so every one of them is a deliberate choice.
//!
//! Wave speed and flow velocity are separate types on purpose: a pressure
//! wave travels at ~1000 m/s while the water moves at ~1 m/s, and mixing
//! them up is exactly the kind of error these types exist to prevent.

use std::fmt;
use std::ops::{Add, AddAssign, Div, Mul, Neg, Sub, SubAssign};

macro_rules! quantity {
    ($(#[$doc:meta])* $name:ident, $unit:literal) => {
        $(#[$doc])*
        #[derive(Debug, Clone, Copy, PartialEq, PartialOrd, Default)]
        pub struct $name(f64);

        impl $name {
            /// Zero of this quantity.
            pub const ZERO: Self = Self(0.0);
            /// SI unit symbol.
            pub const UNIT: &'static str = $unit;

            /// Wraps a value given in SI units.
            #[must_use]
            pub const fn new(value: f64) -> Self {
                Self(value)
            }

            /// The value in SI units.
            #[must_use]
            pub const fn value(self) -> f64 {
                self.0
            }

            /// Absolute value.
            #[must_use]
            pub fn abs(self) -> Self {
                Self(self.0.abs())
            }

            /// True unless the value is NaN or infinite.
            #[must_use]
            pub fn is_finite(self) -> bool {
                self.0.is_finite()
            }

            /// The larger of two values.
            #[must_use]
            pub fn max(self, other: Self) -> Self {
                Self(self.0.max(other.0))
            }

            /// The smaller of two values.
            #[must_use]
            pub fn min(self, other: Self) -> Self {
                Self(self.0.min(other.0))
            }
        }

        impl Add for $name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self {
                Self(self.0 + rhs.0)
            }
        }

        impl Sub for $name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self {
                Self(self.0 - rhs.0)
            }
        }

        impl AddAssign for $name {
            fn add_assign(&mut self, rhs: Self) {
                self.0 += rhs.0;
            }
        }

        impl SubAssign for $name {
            fn sub_assign(&mut self, rhs: Self) {
                self.0 -= rhs.0;
            }
        }

        impl Neg for $name {
            type Output = Self;
            fn neg(self) -> Self {
                Self(-self.0)
            }
        }

        impl Mul<f64> for $name {
            type Output = Self;
            fn mul(self, rhs: f64) -> Self {
                Self(self.0 * rhs)
            }
        }

        impl Mul<$name> for f64 {
            type Output = $name;
            fn mul(self, rhs: $name) -> $name {
                $name(self * rhs.0)
            }
        }

        impl Div<f64> for $name {
            type Output = Self;
            fn div(self, rhs: f64) -> Self {
                Self(self.0 / rhs)
            }
        }

        impl Div for $name {
            type Output = f64;
            fn div(self, rhs: Self) -> f64 {
                self.0 / rhs.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                fmt::Display::fmt(&self.0, f)?;
                write!(f, " {}", $unit)
            }
        }
    };
}

quantity!(
    /// Length or distance along a pipe [m].
    Length, "m"
);
quantity!(
    /// Cross-sectional area [m^2].
    Area, "m^2"
);
quantity!(
    /// Time or duration [s]. Simulation time, not wall-clock time.
    Time, "s"
);
quantity!(
    /// Mean flow velocity of the water in a pipe [m/s].
    Velocity, "m/s"
);
quantity!(
    /// Speed of a pressure wave along a pipe [m/s]. Not a flow velocity.
    WaveSpeed, "m/s"
);
quantity!(
    /// Acceleration, used for gravity [m/s^2].
    Acceleration, "m/s^2"
);
quantity!(
    /// Piezometric head [m of water].
    Head, "m"
);
quantity!(
    /// Pressure [Pa].
    Pressure, "Pa"
);
quantity!(
    /// Volumetric flow rate [m^3/s].
    Flow, "m^3/s"
);
quantity!(
    /// Mass density [kg/m^3].
    Density, "kg/m^3"
);

/// Standard gravity, 9.80665 m/s^2 (CGPM 1901). Local gravity in South
/// Africa differs by about 0.3%; code that cares takes `g` as an input.
pub const STANDARD_GRAVITY: Acceleration = Acceleration(9.806_65);

const PASCALS_PER_BAR: f64 = 1.0e5;

impl Area {
    /// Area of a circle with the given internal diameter.
    #[must_use]
    pub fn of_circle(diameter: Length) -> Self {
        Self(std::f64::consts::PI * 0.25 * diameter.0 * diameter.0)
    }
}

impl Pressure {
    /// Pressure from a value in bar (1 bar = 100 kPa).
    #[must_use]
    pub fn from_bar(bar: f64) -> Self {
        Self(bar * PASCALS_PER_BAR)
    }

    /// Value in bar.
    #[must_use]
    pub fn bar(self) -> f64 {
        self.0 / PASCALS_PER_BAR
    }

    /// Pressure exerted by a column of liquid: p = rho * g * H.
    #[must_use]
    pub fn from_head(head: Head, density: Density, gravity: Acceleration) -> Self {
        Self(density.0 * gravity.0 * head.0)
    }
}

impl Head {
    /// Height of liquid column equivalent to a pressure: H = p / (rho * g).
    #[must_use]
    pub fn from_pressure(pressure: Pressure, density: Density, gravity: Acceleration) -> Self {
        Self(pressure.0 / (density.0 * gravity.0))
    }
}

impl Mul for Length {
    type Output = Area;
    fn mul(self, rhs: Self) -> Area {
        Area(self.0 * rhs.0)
    }
}

/// Travel time of a wave over a distance: t = L / a.
impl Div<WaveSpeed> for Length {
    type Output = Time;
    fn div(self, rhs: WaveSpeed) -> Time {
        Time(self.0 / rhs.0)
    }
}

/// Distance a wave covers in a time: L = a * t.
impl Mul<Time> for WaveSpeed {
    type Output = Length;
    fn mul(self, rhs: Time) -> Length {
        Length(self.0 * rhs.0)
    }
}

/// Distance a wave covers in a time: L = t * a.
impl Mul<WaveSpeed> for Time {
    type Output = Length;
    fn mul(self, rhs: WaveSpeed) -> Length {
        Length(self.0 * rhs.0)
    }
}

/// Mean velocity: V = Q / A.
impl Div<Area> for Flow {
    type Output = Velocity;
    fn div(self, rhs: Area) -> Velocity {
        Velocity(self.0 / rhs.0)
    }
}

/// Flow from mean velocity: Q = V * A.
impl Mul<Area> for Velocity {
    type Output = Flow;
    fn mul(self, rhs: Area) -> Flow {
        Flow(self.0 * rhs.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: f64, b: f64, rel: f64) -> bool {
        (a - b).abs() <= rel * a.abs().max(b.abs())
    }

    #[test]
    fn same_type_arithmetic() {
        let h = Head::new(10.0) + Head::new(2.5) - Head::new(0.5);
        assert_eq!(h, Head::new(12.0));
        assert_eq!(-h, Head::new(-12.0));
        assert_eq!(h * 2.0, Head::new(24.0));
        assert_eq!(2.0 * h, Head::new(24.0));
        assert_eq!(h / 4.0, Head::new(3.0));
        assert!((Head::new(3.0) / Head::new(12.0) - 0.25).abs() < f64::EPSILON);
    }

    #[test]
    fn wave_travel_time_and_distance_round_trip() {
        let l = Length::new(1000.0);
        let a = WaveSpeed::new(1250.0);
        let t = l / a;
        assert_eq!(t, Time::new(0.8));
        assert_eq!(a * t, l);
        assert_eq!(t * a, l);
    }

    #[test]
    fn circle_area_dn150() {
        // DN150 AC main from docs/02_physics_of_transients/joukowsky_equation.txt:
        // A = pi * 0.075^2 = 0.01767 m^2.
        let a = Area::of_circle(Length::new(0.15));
        assert!(close(a.value(), 0.017_671_458_676_442_587, 1e-15));
    }

    #[test]
    fn velocity_flow_area() {
        let area = Area::new(0.5);
        let v = Flow::new(0.25) / area;
        assert_eq!(v, Velocity::new(0.5));
        assert_eq!(v * area, Flow::new(0.25));
    }

    #[test]
    fn head_pressure_round_trip() {
        let rho = Density::new(998.0);
        let h = Head::new(127.0);
        let p = Pressure::from_head(h, rho, STANDARD_GRAVITY);
        assert!(close(
            Head::from_pressure(p, rho, STANDARD_GRAVITY).value(),
            127.0,
            1e-15
        ));
        // Docs convention: 1 bar ~= 10.2 m of water head.
        let one_bar = Head::from_pressure(Pressure::from_bar(1.0), rho, STANDARD_GRAVITY);
        assert!(close(one_bar.value(), 10.2, 0.01));
    }

    #[test]
    fn display_carries_unit() {
        assert_eq!(format!("{:.1}", Head::new(12.345)), "12.3 m");
        assert_eq!(format!("{}", Flow::new(0.5)), "0.5 m^3/s");
    }
}
