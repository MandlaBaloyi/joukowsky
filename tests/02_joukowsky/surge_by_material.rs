//! Section 4(a): stopping water flowing at 1 m/s instantly, per pipe
//! material. The docs round their answers, so the tolerance is 2%.

use jk_hydraulics::joukowsky;
use jk_kernel::{Acceleration, Density, Pressure, Velocity, WaveSpeed};
use jk_tests::{G_DOCS, check_close};

/// (material, wave speed a [m/s], head rise the docs state [m])
const TABLE: [(&str, f64, f64); 4] = [
    ("steel / ductile iron / cast iron", 1250.0, 127.0),
    ("asbestos-cement", 1000.0, 102.0),
    ("uPVC", 370.0, 38.0),
    ("HDPE", 300.0, 31.0),
];

#[test]
fn head_rise_when_1_m_per_s_is_stopped() {
    println!("dH = -(a / g) dV with dV = -1 m/s:");
    for (material, a, docs_rise) in TABLE {
        let rise = joukowsky::head_change(
            Velocity::new(-1.0),
            WaveSpeed::new(a),
            Acceleration::new(G_DOCS),
        );
        check_close(
            &format!("{material} (a = {a}) [m]"),
            rise.value(),
            docs_rise,
            0.02,
        );
    }
}

/// The same surge in pressure form: dp = -rho a dV.
#[test]
fn pressure_and_head_forms_agree() {
    println!("rho a dV versus rho g (a/g dV), steel main:");
    let rho = Density::new(998.0);
    let g = Acceleration::new(G_DOCS);
    let (dv, a) = (Velocity::new(-1.0), WaveSpeed::new(1250.0));
    let dp = joukowsky::pressure_change(dv, a, rho);
    let dh = joukowsky::head_change(dv, a, g);
    check_close(
        "pressure rise [bar]",
        dp.bar(),
        Pressure::from_head(dh, rho, g).bar(),
        1e-12,
    );
    check_close("pressure rise vs docs ~12.5 bar", dp.bar(), 12.5, 0.01);
}
