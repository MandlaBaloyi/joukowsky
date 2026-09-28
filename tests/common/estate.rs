//! The estate network shared by several test programs.

use crate::pipe;
use jk_hydraulics::{Network, Node, NodeId, PipeId};
use jk_kernel::{Flow, Head, STANDARD_GRAVITY};

/// Wave speed used throughout [m/s].
pub const A: f64 = 1000.0;

/// A small estate network, resting at 60 m:
///
/// ```text
///   R --400 m-- S1 --300 m-- T --500 m-- S2 (dead end)
///                            |
///                          600 m
///                            |
///                            S3 (dead end)
/// ```
pub struct Estate {
    /// The network.
    pub net: Network,
    /// Sensor nodes S1, S2, S3.
    pub sensors: [NodeId; 3],
    /// Pipes: R-S1, S1-T, T-S2, T-S3.
    pub pipes: [PipeId; 4],
    /// The tee.
    pub tee: NodeId,
}

/// Builds the estate network (with an optional junction splitting T-S3 at
/// `burst_at` metres from T, for simulating a burst there).
///
/// # Panics
/// Never: the network is valid.
#[must_use]
pub fn estate(burst_at: Option<f64>) -> (Estate, Option<NodeId>) {
    let main = |l| pipe(l, 0.15, A, 0.02);
    let mut net = Network::new(STANDARD_GRAVITY);
    let j = || Node::Junction { demand: Flow::ZERO };
    let r = net.add_node(Node::Reservoir {
        head: Head::new(60.0),
    });
    let s1 = net.add_node(j());
    let tee = net.add_node(j());
    let s2 = net.add_node(j());
    let s3 = net.add_node(j());
    let p0 = net.add_pipe(r, s1, main(400.0)).unwrap();
    let p1 = net.add_pipe(s1, tee, main(300.0)).unwrap();
    let p2 = net.add_pipe(tee, s2, main(500.0)).unwrap();
    let (p3, burst) = match burst_at {
        None => (net.add_pipe(tee, s3, main(600.0)).unwrap(), None),
        Some(d) => {
            let b = net.add_node(j());
            let p3 = net.add_pipe(tee, b, main(d)).unwrap();
            net.add_pipe(b, s3, main(600.0 - d)).unwrap();
            (p3, Some(b))
        }
    };
    (
        Estate {
            net,
            sensors: [s1, s2, s3],
            pipes: [p0, p1, p2, p3],
            tee,
        },
        burst,
    )
}
