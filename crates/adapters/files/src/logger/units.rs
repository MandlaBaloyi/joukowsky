//! Pressure units, to metres of water head.
//!
//! The engine works in head H [m] (`docs` convention). A gauge pressure p
//! is a pressure head p / (rho g) [m]; with rho = 998.2 kg/m^3 (water at
//! 20 C) and g = 9.81 m/s^2, 1 bar = 10.212 m. (The ground elevation of the
//! logger would add a constant; transients are read from changes and
//! timing, so it does not matter for them - the reader notes it.)

/// A pressure unit a logger may export in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PressureUnit {
    /// Bar (10^5 Pa).
    Bar,
    /// Millibar (100 Pa).
    Millibar,
    /// Kilopascal.
    Kilopascal,
    /// Megapascal.
    Megapascal,
    /// Pascal.
    Pascal,
    /// Pounds per square inch.
    Psi,
    /// Metres of water: already a head.
    MetreHead,
}

impl PressureUnit {
    /// The unit named in a column header, if it names exactly one: looks in
    /// brackets first (`Pressure (bar)`, `P1 [kPa]`), then at the last word
    /// (`pressure_bar`, `head m`).
    #[must_use]
    pub fn from_header(header: &str) -> Option<Self> {
        let h = header.to_ascii_lowercase();
        let bracketed = h.find(['(', '[']).and_then(|i| {
            h[i + 1..]
                .find([')', ']'])
                .map(|j| h[i + 1..i + 1 + j].trim().to_string())
        });
        if let Some(b) = bracketed.as_deref().and_then(Self::parse) {
            return Some(b);
        }
        let last = h
            .rsplit([' ', '_', '-', '/'])
            .find(|w| !w.is_empty())
            .unwrap_or("");
        Self::parse(last)
    }

    /// A unit's name, as written (any case).
    #[must_use]
    pub fn parse(s: &str) -> Option<Self> {
        let compact: String = s
            .to_ascii_lowercase()
            .chars()
            .filter(|c| !c.is_whitespace() && *c != '.')
            .collect();
        Some(match compact.as_str() {
            "bar" | "barg" => Self::Bar,
            "mbar" | "mbarg" => Self::Millibar,
            "kpa" | "kpag" => Self::Kilopascal,
            "mpa" => Self::Megapascal,
            "pa" => Self::Pascal,
            "psi" | "psig" => Self::Psi,
            "m" | "mh2o" | "mwc" | "mwg" | "mhead" | "head" => Self::MetreHead,
            _ => return None,
        })
    }

    /// Name for notes.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Bar => "bar",
            Self::Millibar => "mbar",
            Self::Kilopascal => "kPa",
            Self::Megapascal => "MPa",
            Self::Pascal => "Pa",
            Self::Psi => "psi",
            Self::MetreHead => "m of water",
        }
    }

    /// Metres of head in one of this unit, for water of `density` kg/m^3
    /// under gravity `g`.
    #[must_use]
    pub fn metres_head(self, density: f64, g: f64) -> f64 {
        let pascals = match self {
            Self::Bar => 1e5,
            Self::Millibar => 100.0,
            Self::Kilopascal => 1e3,
            Self::Megapascal => 1e6,
            Self::Pascal => 1.0,
            Self::Psi => 6_894.757_293_168,
            Self::MetreHead => return 1.0,
        };
        pascals / (density * g)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn units_from_headers() {
        for (h, u) in [
            ("Pressure (bar)", PressureUnit::Bar),
            ("P1 [kPa]", PressureUnit::Kilopascal),
            ("pressure_bar", PressureUnit::Bar),
            ("Head m", PressureUnit::MetreHead),
            ("P (m H2O)", PressureUnit::MetreHead),
            ("Channel 2 [psi]", PressureUnit::Psi),
            ("p_mbar", PressureUnit::Millibar),
        ] {
            assert_eq!(PressureUnit::from_header(h), Some(u), "{h}");
        }
        for h in ["Pressure", "Channel 1", "Value", "Temperature (C)"] {
            assert_eq!(PressureUnit::from_header(h), None, "{h}");
        }
    }

    #[test]
    fn one_bar_is_about_ten_metres() {
        let m = PressureUnit::Bar.metres_head(998.2, 9.81);
        assert!((m - 10.212).abs() < 1e-3, "{m}");
        assert!((PressureUnit::MetreHead.metres_head(998.2, 9.81) - 1.0).abs() < 1e-15);
    }
}
