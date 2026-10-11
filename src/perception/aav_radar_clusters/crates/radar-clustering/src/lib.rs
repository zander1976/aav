//! DBSCAN for the radar's points, and a box round each cluster.
//!
//! Purpose: group the radar's points into objects, with the same clusters and boxes as the
//! 2025-26 C++ node (`dbscan` in `aav_object_detection`). No ROS, so `cargo test` runs it
//! on any laptop. [`cloud`] reads and writes the points in a `sensor_msgs/PointCloud2`.
//!
//! DBSCAN, in short:
//! - two points are neighbours when they are at most `radius` apart. Every point is its own
//!   neighbour;
//! - a core point has at least `min_points` neighbours;
//! - a cluster grows from a core point: its neighbours join, and so do the neighbours of
//!   every core point that joins;
//! - a point that no cluster reaches is noise.

pub mod cloud;

use std::{error::Error, fmt, num::NonZeroUsize};

/// A distance, in metres.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd)]
pub struct Metres(pub f64);

/// One point of the radar's cloud.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Point {
    /// x, y and z in metres, in the cloud's frame (radar_link: x forward, y left, z up).
    pub position: [f32; 3],
    /// The return's strength, as the radar reports it; 0 if the cloud has none.
    pub intensity: f32,
    /// The radial (Doppler) speed in m/s, as the radar reports it; 0 if the cloud has none.
    pub velocity: f32,
}

/// How close points must be, and how many, to make a cluster.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Settings {
    radius: Metres,
    min_points: NonZeroUsize,
}

/// Why [`Settings::new`] refused its values.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SettingsError {
    /// The radius must be a distance above zero.
    Radius(f64),
    /// A cluster needs at least one point.
    NoPoints,
}

impl fmt::Display for SettingsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Radius(r) => write!(f, "the radius must be more than 0 m, not {r}"),
            Self::NoPoints => write!(f, "a cluster needs at least 1 point"),
        }
    }
}

impl Error for SettingsError {}

impl Settings {
    /// `radius`: points this close are neighbours. `min_points`: the neighbours a core
    /// point needs, itself included (DBSCAN's minPts).
    pub fn new(radius: Metres, min_points: usize) -> Result<Self, SettingsError> {
        if !(radius.0.is_finite() && radius.0 > 0.0) {
            return Err(SettingsError::Radius(radius.0));
        }
        let min_points = NonZeroUsize::new(min_points).ok_or(SettingsError::NoPoints)?;
        Ok(Self { radius, min_points })
    }

    /// Points this close are neighbours.
    pub fn radius(&self) -> Metres {
        self.radius
    }

    /// The neighbours a core point needs, itself included.
    pub fn min_points(&self) -> usize {
        self.min_points.get()
    }
}

/// One cluster: its points, and the space they take.
#[derive(Clone, Debug, PartialEq)]
pub struct Cluster {
    /// The points in it: their indices in the points given to [`cluster`], in order.
    pub members: Vec<usize>,
    /// The smallest and largest x, y and z of those points.
    pub bounds: Bounds,
}

/// Groups `points` into clusters, in the order they were found: the cluster with the
/// first core point comes first. Points in no cluster are noise.
///
/// A border point (a neighbour of a core point, but not core itself) that two clusters
/// reach goes where the C++ node put it: to the later cluster, if that cluster's first
/// core point is its neighbour; otherwise to the first cluster that reached it.
pub fn cluster(points: &[Point], settings: &Settings) -> Vec<Cluster> {
    let reach = settings.radius.0 * settings.radius.0;
    let neighbours = |i: usize| -> Vec<usize> {
        let here = &points[i].position;
        points
            .iter()
            .enumerate()
            .filter(|(_, p)| distance_squared(here, &p.position) <= reach)
            .map(|(j, _)| j)
            .collect()
    };
    let is_core = |around: &[usize]| around.len() >= settings.min_points.get();

    let mut label: Vec<Option<usize>> = vec![None; points.len()];
    let mut found = 0;
    for seed in 0..points.len() {
        if label[seed].is_some() {
            continue;
        }
        let mut queue = neighbours(seed);
        if !is_core(&queue) {
            continue; // noise, unless a cluster found later reaches it
        }
        let id = found;
        found += 1;
        // Every neighbour of the first core point joins, even one an earlier cluster
        // took: the C++ node did this, so the clusters match it.
        for &j in &queue {
            label[j] = Some(id);
        }
        queue.retain(|&j| j != seed);
        let mut next = 0;
        while let Some(&p) = queue.get(next) {
            next += 1;
            let around = neighbours(p);
            if is_core(&around) {
                for j in around {
                    if label[j].is_none() {
                        label[j] = Some(id);
                        queue.push(j);
                    }
                }
            }
        }
    }

    let mut members = vec![Vec::new(); found];
    for (i, id) in label.into_iter().enumerate() {
        if let Some(id) = id {
            members[id].push(i);
        }
    }
    members
        .into_iter()
        .map(|members| {
            let bounds = Bounds::of(members.iter().map(|&i| &points[i].position))
                .expect("a cluster holds at least its first core point");
            Cluster { members, bounds }
        })
        .collect()
}

/// The squared distance between two positions. Each difference is an f32 and the sum an
/// f64, added in the C++ node's order, so a pair right at the radius goes the same way.
fn distance_squared(a: &[f32; 3], b: &[f32; 3]) -> f64 {
    a.iter()
        .zip(b)
        .map(|(p, q)| f64::from(p - q))
        .map(|d| d * d)
        .sum()
}

/// The space some points take: the smallest and the largest x, y and z (m).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub min: [f32; 3],
    pub max: [f32; 3],
}

/// A box as RViz's CUBE marker draws it: its centre, and its size along each axis (m).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Cuboid {
    pub centre: [f64; 3],
    pub size: [f64; 3],
}

/// How much bigger than its points a cluster's box is, on every side, as in the C++
/// node: so a flat cluster, or one point, still shows.
pub const BOX_MARGIN: Metres = Metres(0.1);

impl Bounds {
    /// The bounds of `positions`; `None` if there are none.
    pub fn of<'a>(positions: impl IntoIterator<Item = &'a [f32; 3]>) -> Option<Self> {
        let mut positions = positions.into_iter();
        let first = *positions.next()?;
        Some(positions.fold(
            Self {
                min: first,
                max: first,
            },
            |b, p| Self {
                min: [0, 1, 2].map(|i| b.min[i].min(p[i])),
                max: [0, 1, 2].map(|i| b.max[i].max(p[i])),
            },
        ))
    }

    /// The box round these bounds, `margin` bigger on every side. The sums are the C++
    /// node's, in its order, so the numbers match it to the last bit.
    pub fn padded(&self, margin: Metres) -> Cuboid {
        let size = [0, 1, 2].map(|i| f64::from(self.max[i] - self.min[i]) + 2.0 * margin.0);
        let centre = [0, 1, 2].map(|i| f64::from(self.min[i]) + size[i] / 2.0 - margin.0);
        Cuboid { centre, size }
    }
}

/// A colour, 8 bits a channel.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rgb {
    pub r: u8,
    pub g: u8,
    pub b: u8,
}

impl Rgb {
    /// Packed as PCL and RViz read a point's `rgb` field: 0x00RRGGBB.
    pub fn packed(self) -> u32 {
        u32::from(self.r) << 16 | u32::from(self.g) << 8 | u32::from(self.b)
    }

    /// Each channel from 0 to 1, as a marker's colour takes it.
    pub fn unit(self) -> [f32; 3] {
        [self.r, self.g, self.b].map(|c| f32::from(c) / 255.0)
    }
}

/// The clusters' colours: Tableau's ten, which are easy to tell apart. After ten
/// clusters they repeat.
const PALETTE: [Rgb; 10] = [
    Rgb {
        r: 0x1f,
        g: 0x77,
        b: 0xb4,
    }, // blue
    Rgb {
        r: 0xff,
        g: 0x7f,
        b: 0x0e,
    }, // orange
    Rgb {
        r: 0x2c,
        g: 0xa0,
        b: 0x2c,
    }, // green
    Rgb {
        r: 0xd6,
        g: 0x27,
        b: 0x28,
    }, // red
    Rgb {
        r: 0x94,
        g: 0x67,
        b: 0xbd,
    }, // purple
    Rgb {
        r: 0x8c,
        g: 0x56,
        b: 0x4b,
    }, // brown
    Rgb {
        r: 0xe3,
        g: 0x77,
        b: 0xc2,
    }, // pink
    Rgb {
        r: 0x7f,
        g: 0x7f,
        b: 0x7f,
    }, // grey
    Rgb {
        r: 0xbc,
        g: 0xbd,
        b: 0x22,
    }, // olive
    Rgb {
        r: 0x17,
        g: 0xbe,
        b: 0xcf,
    }, // cyan
];

/// The colour of the cluster at `index` (0 is the first), for its points and its box.
pub fn colour(index: usize) -> Rgb {
    PALETTE[index % PALETTE.len()]
}

#[cfg(test)]
mod tests {
    use super::*;

    fn at(x: f32, y: f32, z: f32) -> Point {
        Point {
            position: [x, y, z],
            ..Point::default()
        }
    }

    fn settings(radius: f64, min_points: usize) -> Settings {
        Settings::new(Metres(radius), min_points).expect("valid test settings")
    }

    fn members(clusters: &[Cluster]) -> Vec<Vec<usize>> {
        clusters.iter().map(|c| c.members.clone()).collect()
    }

    /// `count` points on a 0.1 m grid line from (x, y), along y.
    fn blob(x: f32, y: f32, count: usize) -> Vec<Point> {
        (0..count).map(|i| at(x, y + 0.1 * i as f32, 0.0)).collect()
    }

    #[test]
    fn two_blobs_make_two_clusters_and_the_noise_is_dropped() {
        let mut points = blob(2.0, 1.0, 6); // 0 to 5
        points.push(at(4.0, -3.0, 0.0)); // 6: alone
        points.extend(blob(6.0, -1.0, 6)); // 7 to 12
        points.push(at(8.0, 3.0, 1.0)); // 13: alone
        let clusters = cluster(&points, &settings(0.5, 3));
        assert_eq!(
            members(&clusters),
            [(0..6).collect::<Vec<_>>(), (7..13).collect()]
        );
    }

    #[test]
    fn points_far_apart_are_all_noise() {
        let points: Vec<_> = (0..10).map(|i| at(i as f32, 0.0, 0.0)).collect();
        assert!(cluster(&points, &settings(0.8, 2)).is_empty());
        assert!(cluster(&[], &settings(0.8, 1)).is_empty());
    }

    #[test]
    fn a_chain_of_core_points_is_one_cluster() {
        // 0.5 m apart: each inner point has 3 neighbours (itself and two), each end 2.
        let points: Vec<_> = (0..8).map(|i| at(0.5 * i as f32, 0.0, 0.0)).collect();
        let clusters = cluster(&points, &settings(0.6, 3));
        assert_eq!(members(&clusters), [(0..8).collect::<Vec<_>>()]);
    }

    #[test]
    fn min_points_counts_the_point_itself() {
        let pair = [at(1.0, 0.0, 0.0), at(1.3, 0.0, 0.0)];
        assert_eq!(members(&cluster(&pair, &settings(0.5, 2))), [[0, 1]]);
        assert!(cluster(&pair, &settings(0.5, 3)).is_empty());
        // With 1, every point is a cluster of its own, unless it has neighbours.
        let apart = [at(0.0, 0.0, 0.0), at(5.0, 0.0, 0.0)];
        assert_eq!(members(&cluster(&apart, &settings(0.5, 1))), [[0], [1]]);
    }

    #[test]
    fn the_radius_is_a_distance_not_a_squared_one() {
        // The C++ node compared eps with the squared distance: its 0.8 joined these two.
        let pair = [at(0.0, 0.0, 0.0), at(0.85, 0.0, 0.0)];
        assert!(cluster(&pair, &settings(0.8, 2)).is_empty());
        assert_eq!(cluster(&pair, &settings(0.9, 2)).len(), 1);
        // A point exactly at the radius is a neighbour.
        let edge = [at(0.0, 0.0, 0.0), at(0.5, 0.0, 0.0)];
        assert_eq!(cluster(&edge, &settings(0.5, 2)).len(), 1);
    }

    #[test]
    fn distance_is_three_dimensional() {
        let stacked = [at(1.0, 0.0, 0.0), at(1.0, 0.0, 1.0)];
        assert!(cluster(&stacked, &settings(0.8, 2)).is_empty());
    }

    #[test]
    fn clusters_come_in_the_order_of_their_first_core_point() {
        let mut points = blob(6.0, 0.0, 4); // 0 to 3: found first
        points.extend(blob(1.0, 0.0, 4)); // 4 to 7
        let clusters = cluster(&points, &settings(0.25, 3));
        assert_eq!(members(&clusters), [vec![0, 1, 2, 3], vec![4, 5, 6, 7]]);
    }

    /// Two clusters on the x axis, 0.3 m apart within each, and between them a border
    /// point at 1.85 m: 0.95 m from A's last point and from B's first, so it is a
    /// neighbour of both, but has only 3 neighbours itself (radius 1, min_points 4).
    fn bridge(b_first: bool) -> Vec<Point> {
        let a = [0.0, 0.3, 0.6, 0.9];
        let b = if b_first {
            [2.8, 3.1, 3.4, 3.7] // B starts from 2.8, the border point's neighbour
        } else {
            [3.1, 3.4, 3.7, 2.8] // B starts from 3.1, which can't reach the border point
        };
        let xs = a.iter().chain(&[1.85]).chain(&b);
        xs.map(|&x| at(x, 0.0, 0.0)).collect()
    }

    #[test]
    fn a_border_point_goes_to_the_later_cluster_when_its_first_core_point_reaches_it() {
        // A reaches the border point (4) first; B's first core point takes it, as in C++.
        let clusters = cluster(&bridge(true), &settings(1.0, 4));
        assert_eq!(members(&clusters), [vec![0, 1, 2, 3], vec![4, 5, 6, 7, 8]]);
    }

    #[test]
    fn a_border_point_stays_with_the_first_cluster_otherwise() {
        // B reaches the border point only through a later core point: A keeps it.
        let clusters = cluster(&bridge(false), &settings(1.0, 4));
        assert_eq!(members(&clusters), [vec![0, 1, 2, 3, 4], vec![5, 6, 7, 8]]);
    }

    #[test]
    fn points_that_are_not_numbers_are_noise() {
        let mut points = blob(1.0, 1.0, 5);
        points.push(at(f32::NAN, 1.0, 0.0));
        points.push(at(f32::INFINITY, 1.0, 0.0));
        let clusters = cluster(&points, &settings(0.5, 3));
        assert_eq!(members(&clusters), [vec![0, 1, 2, 3, 4]]);
    }

    #[test]
    fn duplicate_points_count_as_neighbours() {
        let points = [at(1.0, 1.0, 0.0); 3];
        assert_eq!(members(&cluster(&points, &settings(0.1, 3))), [[0, 1, 2]]);
    }

    #[test]
    fn bounds_and_box() {
        let points = [at(1.0, 2.0, 3.0), at(2.0, 4.0, 3.5), at(1.5, 3.0, 3.25)];
        let clusters = cluster(&points, &settings(3.0, 3));
        let bounds = clusters[0].bounds;
        assert_eq!(bounds.min, [1.0, 2.0, 3.0]);
        assert_eq!(bounds.max, [2.0, 4.0, 3.5]);
        let cube = bounds.padded(BOX_MARGIN);
        for (got, want) in cube.size.iter().zip([1.2, 2.2, 0.7]) {
            assert!((got - want).abs() < 1e-12, "size {got}, not {want}");
        }
        for (got, want) in cube.centre.iter().zip([1.5, 3.0, 3.25]) {
            assert!((got - want).abs() < 1e-12, "centre {got}, not {want}");
        }
    }

    #[test]
    fn the_box_sums_are_the_cpp_nodes() {
        // The C++: scale = max - min + 0.2; position = min + scale / 2 - 0.1 (f32 min and
        // max, f64 sums).
        let (min, max) = (-1.234_567_f32, 0.765_432_f32);
        let cube = Bounds {
            min: [min; 3],
            max: [max; 3],
        }
        .padded(BOX_MARGIN);
        let scale = f64::from(max - min) + 0.2;
        assert_eq!(cube.size[0].to_bits(), scale.to_bits());
        let position = f64::from(min) + scale / 2.0 - 0.1;
        assert_eq!(cube.centre[0].to_bits(), position.to_bits());
    }

    #[test]
    fn bad_settings_are_refused() {
        for r in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let refused = Settings::new(Metres(r), 5);
            assert!(
                matches!(refused, Err(SettingsError::Radius(_))),
                "radius {r}"
            );
        }
        assert_eq!(Settings::new(Metres(0.8), 0), Err(SettingsError::NoPoints));
        let ok = settings(0.8, 15);
        assert_eq!((ok.radius(), ok.min_points()), (Metres(0.8), 15));
    }

    #[test]
    fn colours_pack_as_rviz_reads_them_and_repeat_after_ten() {
        assert_eq!(
            Rgb {
                r: 0x12,
                g: 0x34,
                b: 0x56
            }
            .packed(),
            0x0012_3456
        );
        assert_eq!(colour(0), colour(10));
        assert_ne!(colour(0), colour(1));
        assert_eq!(
            Rgb {
                r: 255,
                g: 0,
                b: 51
            }
            .unit(),
            [1.0, 0.0, 0.2]
        );
    }
}
