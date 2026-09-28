//! Conversions the docs rely on, checked against their stated numbers.

use jk_kernel::{
    Acceleration, Area, Density, Flow, Head, Length, Pressure, STANDARD_GRAVITY, Time, Velocity,
    WaveSpeed,
};
use jk_tests::{G_DOCS, check_close};

/// docs/README.txt: "1 bar = 100 kPa ~= 10.2 m of water head".
#[test]
fn one_bar_is_about_10_2_metres_of_water() {
    println!("1 bar expressed as head of water (rho = 998 kg/m^3):");
    let head = Head::from_pressure(
        Pressure::from_bar(1.0),
        Density::new(998.0),
        STANDARD_GRAVITY,
    );
    check_close("head of 1 bar [m]", head.value(), 10.2, 0.01);
}

/// `joukowsky_equation.txt` section 5: DN150, A = pi * 0.075^2 = 0.01767 m^2.
#[test]
fn dn150_area() {
    println!("Cross-section of a DN150 pipe:");
    let area = Area::of_circle(Length::new(0.15));
    check_close("area [m^2]", area.value(), 0.017_67, 1e-3);
}

/// Pressure and head are two views of the same thing: p = rho g H.
#[test]
fn head_and_pressure_round_trip() {
    println!("127 m of head -> pressure -> back to head:");
    let rho = Density::new(998.0);
    let g = Acceleration::new(G_DOCS);
    let p = Pressure::from_head(Head::new(127.0), rho, g);
    check_close("pressure [bar]", p.bar(), 12.43, 1e-3);
    check_close(
        "head again [m]",
        Head::from_pressure(p, rho, g).value(),
        127.0,
        1e-15,
    );
}

/// `joukowsky_equation.txt` 4(b): L = 1000 m, a = 1250 m/s -> 2L/a = 1.6 s.
#[test]
fn wave_travel_time() {
    println!("Pressure wave crossing 1000 m of steel main at 1250 m/s:");
    let one_way = Length::new(1000.0) / WaveSpeed::new(1250.0);
    check_close("one way L/a [s]", one_way.value(), 0.8, 1e-15);
    check_close("round trip 2L/a [s]", (one_way * 2.0).value(), 1.6, 1e-15);
    check_close(
        "distance back from a * t [m]",
        (WaveSpeed::new(1250.0) * Time::new(0.8)).value(),
        1000.0,
        1e-15,
    );
}

/// Flow, velocity and area: Q = V A.
#[test]
fn flow_velocity_area() {
    println!("1 m/s in a DN300 pipe:");
    let area = Area::of_circle(Length::new(0.3));
    let q: Flow = Velocity::new(1.0) * area;
    check_close("flow [l/s]", q.value() * 1000.0, 70.686, 1e-4);
    check_close("velocity back [m/s]", (q / area).value(), 1.0, 1e-15);
}
