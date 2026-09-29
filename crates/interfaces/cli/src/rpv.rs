//! `jk rpv`: the reservoir -> pipe -> valve transient (milestone S1).

use jk_hydraulics::{ReservoirPipeValve, ValveClosure, joukowsky};
use jk_kernel::{Head, Length, STANDARD_GRAVITY, Time, Velocity, WaveSpeed};

pub const USAGE: &str = "
    jk rpv [OPTIONS]      reservoir -> pipe -> valve transient

  OPTIONS (SI units, defaults in brackets):
    --length M            pipe length [1000]
    --diameter M          internal diameter [0.3]
    --wave-speed M/S      pressure wave speed [1000]
    --velocity M/S        steady flow velocity [1.0]
    --friction F          Darcy friction factor [0.0]
    --reservoir-head M    upstream reservoir head [100]
    --downstream-head M   head below the valve [50]
    --reaches N           MOC reaches [20]
    --closure S           closure time, 0 = instantaneous [0]
    --exponent X          closure curve tau = (1 - t/Tc)^X [1]
    --duration S          simulated time [12]
    --csv                 print the valve and mid-pipe histories as CSV
";

pub struct Args {
    length: f64,
    diameter: f64,
    wave_speed: f64,
    velocity: f64,
    friction: f64,
    reservoir_head: f64,
    downstream_head: f64,
    reaches: u32,
    closure: f64,
    exponent: f64,
    duration: f64,
    csv: bool,
}

impl Default for Args {
    fn default() -> Self {
        Self {
            length: 1000.0,
            diameter: 0.3,
            wave_speed: 1000.0,
            velocity: 1.0,
            friction: 0.0,
            reservoir_head: 100.0,
            downstream_head: 50.0,
            reaches: 20,
            closure: 0.0,
            exponent: 1.0,
            duration: 12.0,
            csv: false,
        }
    }
}

pub fn parse(mut it: impl Iterator<Item = String>) -> Result<Args, String> {
    let mut a = Args::default();
    while let Some(flag) = it.next() {
        if flag == "--csv" {
            a.csv = true;
            continue;
        }
        let value = it.next().ok_or_else(|| format!("{flag} needs a value"))?;
        let num = || {
            value
                .parse::<f64>()
                .map_err(|_| format!("{flag}: '{value}' is not a number"))
        };
        match flag.as_str() {
            "--length" => a.length = num()?,
            "--diameter" => a.diameter = num()?,
            "--wave-speed" => a.wave_speed = num()?,
            "--velocity" => a.velocity = num()?,
            "--friction" => a.friction = num()?,
            "--reservoir-head" => a.reservoir_head = num()?,
            "--downstream-head" => a.downstream_head = num()?,
            "--reaches" => {
                a.reaches = value
                    .parse()
                    .map_err(|_| format!("--reaches: '{value}' is not a whole number"))?;
            }
            "--closure" => a.closure = num()?,
            "--exponent" => a.exponent = num()?,
            "--duration" => a.duration = num()?,
            _ => return Err(format!("unknown option {flag}")),
        }
    }
    Ok(a)
}

pub fn run(a: &Args) -> Result<(), String> {
    let pipe = jk_hydraulics::Pipe::new(
        Length::new(a.length),
        Length::new(a.diameter),
        WaveSpeed::new(a.wave_speed),
        a.friction,
    )
    .map_err(|e| e.to_string())?;
    let closure = if a.closure == 0.0 {
        ValveClosure::Instantaneous
    } else {
        ValveClosure::Power {
            duration: Time::new(a.closure),
            exponent: a.exponent,
        }
    };
    let system = ReservoirPipeValve {
        reservoir_head: Head::new(a.reservoir_head),
        initial_flow: Velocity::new(a.velocity) * pipe.area(),
        pipe,
        downstream_head: Head::new(a.downstream_head),
        closure,
        reaches: a.reaches,
        gravity: STANDARD_GRAVITY,
    };
    let valve = system.valve_node();
    let mid = system
        .grid()
        .map_err(|e| e.to_string())?
        .node_at(pipe.length() * 0.5);
    let trace = system
        .simulate(Time::new(a.duration), &[valve, mid])
        .map_err(|e| e.to_string())?;

    if a.csv {
        println!("time_s,head_valve_m,flow_valve_m3s,head_mid_m,flow_mid_m3s");
        for k in 0..trace.len() {
            println!(
                "{:.6},{:.6},{:.9},{:.6},{:.9}",
                trace.time(k).value(),
                trace.head(0)[k].value(),
                trace.flow(0)[k].value(),
                trace.head(1)[k].value(),
                trace.flow(1)[k].value(),
            );
        }
        return Ok(());
    }

    let steady = system.steady_valve_head();
    let jouk = joukowsky::head_change(
        Velocity::new(-a.velocity),
        pipe.wave_speed(),
        STANDARD_GRAVITY,
    );
    let rapid = a.closure <= pipe.round_trip_time().value();
    let max = trace.max_head(0);
    let min = trace.min_head(0);
    println!("Reservoir -> pipe -> valve");
    println!(
        "  pipe          L = {}, D = {}, a = {}",
        pipe.length(),
        pipe.diameter(),
        pipe.wave_speed()
    );
    println!(
        "  timing        2L/a = {:.3}, period 4L/a = {:.3}",
        pipe.round_trip_time(),
        pipe.round_trip_time() * 2.0
    );
    println!(
        "  grid          {} reaches, dt = {:.5}",
        a.reaches,
        trace.time_step()
    );
    println!(
        "  closure       {}",
        if a.closure == 0.0 {
            "instantaneous".to_string()
        } else {
            format!("{} s ({})", a.closure, if rapid { "rapid" } else { "slow" })
        }
    );
    println!("At the valve");
    println!("  steady head   {steady:.3}");
    println!("  max head      {max:.3}  (surge {:.3})", max - steady);
    println!("  min head      {min:.3}");
    println!(
        "  Joukowsky dH  {jouk:.3}{}",
        if rapid {
            ""
        } else {
            "  (upper bound; closure is slow)"
        }
    );
    if min.value() < -8.0 {
        println!(
            "  WARNING: head below ~-8 m gauge reaches the vapour floor; cavitation is not modelled yet (milestone S7)."
        );
    }
    Ok(())
}
