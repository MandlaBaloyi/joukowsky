//! The demo sites, with a drawing position for every node.
//!
//! Positions are metres on a plan, laid out so each pipe is drawn about as
//! long as it is. (Real network files - EPANET `.inp` - carry their own
//! coordinates; the readers will use them.)

use jk_hydraulics::boundary::leak;
use jk_hydraulics::{
    Burst, BurstOpening, GridOptions, InitialCondition, Network, Node, NodeId, Pipe, Probe,
    SteadyOptions,
};
use jk_kernel::{Area, Flow, Head, Length, STANDARD_GRAVITY, Time, WaveSpeed};

/// How to draw a network: a plan position and a name per node, a name per
/// pipe (both in network order).
#[derive(Debug, Clone)]
pub struct Drawing {
    /// Plan position of each node [m].
    pub positions: Vec<(f64, f64)>,
    /// Name of each node.
    pub node_names: Vec<String>,
    /// Name of each pipe.
    pub pipe_names: Vec<String>,
}

/// A network the engine knows, and how to draw it.
#[derive(Debug, Clone)]
pub struct DemoSite {
    /// The engine's map of the network.
    pub network: Network,
    /// How to draw it.
    pub drawing: Drawing,
}

/// Where and when a simulated burst really happened.
#[derive(Debug, Clone, Copy)]
pub struct TrueBurst {
    /// Pipe index on the engine's map.
    pub pipe: usize,
    /// Distance from the pipe's start [m].
    pub distance: f64,
    /// When the hole opened [s].
    pub time: f64,
}

/// A simulated event: the site, its loggers, and what they would record.
#[derive(Debug, Clone)]
pub struct BurstDemo {
    /// The site as the engine knows it (no holes on the map).
    pub site: DemoSite,
    /// The logger nodes, in record order.
    pub loggers: Vec<NodeId>,
    /// True head history at each logger, one value every `dt`.
    pub truths: Vec<Vec<f64>>,
    /// Simulation time step [s].
    pub dt: f64,
    /// What really happened.
    pub bursts: Vec<TrueBurst>,
}

fn pipe(length: f64, diameter: f64, wave_speed: f64) -> Result<Pipe, String> {
    Pipe::new(
        Length::new(length),
        Length::new(diameter),
        WaveSpeed::new(wave_speed),
        0.02,
    )
    .map_err(|e| e.to_string())
}

fn hole(node: NodeId, start: f64) -> Burst {
    Burst {
        node,
        coefficient: leak::coefficient(0.6, Area::of_circle(Length::new(0.010)), STANDARD_GRAVITY),
        outside_head: Head::ZERO,
        start: Time::new(start),
        opening: BurstOpening::Instantaneous,
    }
}

fn names(list: &[&str]) -> Vec<String> {
    list.iter().map(|s| (*s).to_string()).collect()
}

/// The estate with consumers drawing 15 l/s at S2 and S3, a 10 mm hole
/// opening 200 m along the south branch at 3 s and another 150 m along the
/// north branch at 30 s; 40 s simulated (as in the pipeline tests).
///
/// ```text
///   R --400-- S1 --300-- T --500-- S2
///                        |
///                       600
///                        |
///                        S3
/// ```
///
/// # Errors
/// Only if the network or simulation is invalid (a bug).
pub fn flowing_estate_two_bursts() -> Result<BurstDemo, String> {
    let main = |l| pipe(l, 0.15, 1000.0);
    let j = |q| Node::Junction {
        demand: Flow::new(q),
    };
    let err = |e: jk_hydraulics::HydraulicsError| e.to_string();

    // The real network: the two branches split where the holes are.
    let mut net = Network::new(STANDARD_GRAVITY);
    let r = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let s1 = net.add_node(j(0.0));
    let tee = net.add_node(j(0.0));
    let s2 = net.add_node(j(0.015));
    let s3 = net.add_node(j(0.015));
    let b1 = net.add_node(j(0.0));
    let b2 = net.add_node(j(0.0));
    net.add_pipe(r, s1, main(400.0)?).map_err(err)?;
    net.add_pipe(s1, tee, main(300.0)?).map_err(err)?;
    net.add_pipe(tee, b2, main(150.0)?).map_err(err)?;
    net.add_pipe(b2, s2, main(350.0)?).map_err(err)?;
    net.add_pipe(tee, b1, main(200.0)?).map_err(err)?;
    net.add_pipe(b1, s3, main(400.0)?).map_err(err)?;
    net.add_burst(hole(b1, 3.0)).map_err(err)?;
    net.add_burst(hole(b2, 30.0)).map_err(err)?;
    let run = net
        .simulate(
            &InitialCondition::Steady(SteadyOptions::default()),
            Time::new(40.0),
            &[s1, s2, s3].map(Probe::Node),
            &GridOptions {
                max_wave_speed_adjustment: 0.01,
                min_reaches: 100,
            },
        )
        .map_err(err)?;
    let truths = (0..3)
        .map(|i| run.trace.head(i).iter().map(|h| h.value()).collect())
        .collect();

    // The engine's map: the same pipes, no holes.
    let mut m = Network::new(STANDARD_GRAVITY);
    let r = m.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let s1 = m.add_node(j(0.0));
    let tee = m.add_node(j(0.0));
    let s2 = m.add_node(j(0.015));
    let s3 = m.add_node(j(0.015));
    m.add_pipe(r, s1, main(400.0)?).map_err(err)?;
    m.add_pipe(s1, tee, main(300.0)?).map_err(err)?;
    m.add_pipe(tee, s2, main(500.0)?).map_err(err)?;
    m.add_pipe(tee, s3, main(600.0)?).map_err(err)?;
    Ok(BurstDemo {
        site: DemoSite {
            network: m,
            drawing: Drawing {
                positions: vec![
                    (0.0, 0.0),
                    (400.0, 0.0),
                    (700.0, 0.0),
                    (1200.0, 0.0),
                    (700.0, 600.0),
                ],
                node_names: names(&[
                    "R (municipal supply)",
                    "S1 gatehouse",
                    "T (tee)",
                    "S2 north block",
                    "S3 south block",
                ]),
                pipe_names: names(&[
                    "R-S1 supply main",
                    "S1-T spine",
                    "T-S2 north branch",
                    "T-S3 south branch",
                ]),
            },
        },
        loggers: vec![s1, s2, s3],
        truths,
        dt: run.trace.time_step().value(),
        bursts: vec![
            TrueBurst {
                pipe: 3,
                distance: 200.0,
                time: 3.0,
            },
            TrueBurst {
                pipe: 2,
                distance: 150.0,
                time: 30.0,
            },
        ],
    })
}

/// A commercial campus: the municipal connection feeds a ring main (ductile
/// iron, a = 1000 m/s) with plastic branches (uPVC, a = 400 m/s) to the
/// buildings; 2.31 km of pipe. Returns the site and the nodes where a
/// logger could go (all but the reservoir).
///
/// # Errors
/// Only if the network is invalid (a bug).
pub fn campus() -> Result<(DemoSite, Vec<NodeId>), String> {
    let iron = |l| pipe(l, 0.15, 1000.0);
    let upvc = |l| pipe(l, 0.1, 400.0);
    let err = |e: jk_hydraulics::HydraulicsError| e.to_string();
    let mut net = Network::new(STANDARD_GRAVITY);
    let j = || Node::Junction { demand: Flow::ZERO };
    let r = net.add_node(Node::Reservoir {
        head: Head::new(70.0),
    });
    let node_names = [
        "R (municipal)",
        "M entrance",
        "A",
        "B",
        "C",
        "m1 gatehouse",
        "a1 block A1",
        "a2 block A2",
        "b1 block B1",
        "b2 block B2",
        "c1 block C1",
        "c2 block C2",
    ];
    let n: Vec<NodeId> = node_names[1..].iter().map(|_| net.add_node(j())).collect();
    let (m, a, b, c) = (n[0], n[1], n[2], n[3]);
    let links = [
        (r, m, iron(150.0)?, "R-M connection"),
        (m, a, iron(300.0)?, "M-A ring"),
        (a, b, iron(250.0)?, "A-B ring"),
        (b, c, iron(300.0)?, "B-C ring"),
        (c, m, iron(250.0)?, "C-M ring"),
        (m, n[4], upvc(100.0)?, "M-m1"),
        (a, n[5], upvc(120.0)?, "A-a1"),
        (a, n[6], upvc(180.0)?, "A-a2"),
        (b, n[7], upvc(200.0)?, "B-b1"),
        (b, n[8], upvc(90.0)?, "B-b2"),
        (c, n[9], upvc(150.0)?, "C-c1"),
        (c, n[10], upvc(220.0)?, "C-c2"),
    ];
    let mut pipe_names = Vec::new();
    for (x, y, p, name) in links {
        net.add_pipe(x, y, p).map_err(err)?;
        pipe_names.push(name.to_string());
    }
    let site = DemoSite {
        network: net,
        drawing: Drawing {
            positions: vec![
                (-150.0, 0.0),
                (0.0, 0.0),
                (300.0, 0.0),
                (300.0, 250.0),
                (0.0, 250.0),
                (0.0, -100.0),
                (385.0, -85.0),
                (480.0, 0.0),
                (500.0, 250.0),
                (300.0, 340.0),
                (-150.0, 250.0),
                (0.0, 470.0),
            ],
            node_names: names(&node_names),
            pipe_names,
        },
    };
    Ok((site, n))
}
