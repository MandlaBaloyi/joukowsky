//! EPANET's units, converted to SI.
//!
//! The flow unit chosen in `[OPTIONS] UNITS` also fixes every other unit
//! (EPANET 2.2 manual, "Units of Measurement"):
//!
//! ```text
//!   US customary (CFS, GPM, MGD, IMGD, AFD): length, elevation, head in
//!     feet; pipe diameter in inches; Darcy-Weisbach roughness in 1e-3 ft.
//!   SI (LPS, LPM, MLD, CMH, CMD, CMS): metres; diameter in millimetres;
//!     Darcy-Weisbach roughness in millimetres.
//! ```
//!
//! With no UNITS option EPANET assumes GPM (US customary).

/// Metres per foot (exact).
pub const FOOT: f64 = 0.3048;
/// Metres per inch (exact).
pub const INCH: f64 = 0.0254;

/// The flow unit of a file.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlowUnits {
    /// Cubic feet per second.
    Cfs,
    /// US gallons per minute (EPANET's default).
    Gpm,
    /// Million US gallons per day.
    Mgd,
    /// Million imperial gallons per day.
    Imgd,
    /// Acre-feet per day.
    Afd,
    /// Litres per second.
    Lps,
    /// Litres per minute.
    Lpm,
    /// Megalitres per day.
    Mld,
    /// Cubic metres per hour.
    Cmh,
    /// Cubic metres per day.
    Cmd,
    /// Cubic metres per second (EPANET 2.3).
    Cms,
}

impl FlowUnits {
    /// Reads the name as written in files (any case).
    #[must_use]
    pub fn parse(s: &str) -> Option<Self> {
        Some(match s.to_ascii_uppercase().as_str() {
            "CFS" => Self::Cfs,
            "GPM" => Self::Gpm,
            "MGD" => Self::Mgd,
            "IMGD" => Self::Imgd,
            "AFD" => Self::Afd,
            "LPS" => Self::Lps,
            "LPM" => Self::Lpm,
            "MLD" => Self::Mld,
            "CMH" => Self::Cmh,
            "CMD" => Self::Cmd,
            "CMS" => Self::Cms,
            _ => return None,
        })
    }

    /// Name as written in files.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::Cfs => "CFS",
            Self::Gpm => "GPM",
            Self::Mgd => "MGD",
            Self::Imgd => "IMGD",
            Self::Afd => "AFD",
            Self::Lps => "LPS",
            Self::Lpm => "LPM",
            Self::Mld => "MLD",
            Self::Cmh => "CMH",
            Self::Cmd => "CMD",
            Self::Cms => "CMS",
        }
    }

    /// Whether the file uses US customary units.
    #[must_use]
    pub fn us(self) -> bool {
        matches!(
            self,
            Self::Cfs | Self::Gpm | Self::Mgd | Self::Imgd | Self::Afd
        )
    }

    /// Cubic metres per second in one unit of this flow.
    #[must_use]
    pub fn to_m3s(self) -> f64 {
        const DAY: f64 = 86_400.0;
        const US_GALLON: f64 = 3.785_411_784e-3; // m^3, exact
        const IMP_GALLON: f64 = 4.546_09e-3; // m^3, exact
        const ACRE_FOOT: f64 = 1_233.481_837_547_52; // m^3, exact
        match self {
            Self::Cfs => FOOT * FOOT * FOOT,
            Self::Gpm => US_GALLON / 60.0,
            Self::Mgd => 1e6 * US_GALLON / DAY,
            Self::Imgd => 1e6 * IMP_GALLON / DAY,
            Self::Afd => ACRE_FOOT / DAY,
            Self::Lps => 1e-3,
            Self::Lpm => 1e-3 / 60.0,
            Self::Mld => 1e3 / DAY,
            Self::Cmh => 1.0 / 3600.0,
            Self::Cmd => 1.0 / DAY,
            Self::Cms => 1.0,
        }
    }

    /// Metres in one unit of length, elevation or head.
    #[must_use]
    pub fn length(self) -> f64 {
        if self.us() { FOOT } else { 1.0 }
    }

    /// Metres of water head in one unit of pressure (valve settings): psi
    /// with EPANET's 0.4333 psi per foot of water, or metres.
    #[must_use]
    pub fn pressure(self) -> f64 {
        if self.us() { FOOT / 0.4333 } else { 1.0 }
    }

    /// Metres in one unit of pipe diameter.
    #[must_use]
    pub fn diameter(self) -> f64 {
        if self.us() { INCH } else { 1e-3 }
    }

    /// Metres in one unit of Darcy-Weisbach roughness.
    #[must_use]
    pub fn roughness(self) -> f64 {
        if self.us() { 1e-3 * FOOT } else { 1e-3 }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn conversions_match_the_definitions() {
        // 1 GPM = 6.309e-5 m^3/s; 1 MGD = 0.04381 m^3/s; 1 CFS = 0.02832.
        assert!((FlowUnits::Gpm.to_m3s() - 6.309_020e-5).abs() < 1e-10);
        assert!((FlowUnits::Mgd.to_m3s() - 0.043_812_6).abs() < 1e-7);
        assert!((FlowUnits::Cfs.to_m3s() - 0.028_316_85).abs() < 1e-8);
        assert!((FlowUnits::Mld.to_m3s() - 0.011_574_07).abs() < 1e-8);
        assert!(FlowUnits::Gpm.us() && !FlowUnits::Lps.us());
        assert!((FlowUnits::Gpm.diameter() - 0.0254).abs() < 1e-15);
        assert_eq!(FlowUnits::parse("lps"), Some(FlowUnits::Lps));
    }
}
