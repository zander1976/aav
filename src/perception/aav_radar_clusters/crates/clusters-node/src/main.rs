//! radar_clusters: DBSCAN groups the radar's points into objects, with a box round each.
//!
//! Subscribes  /radar/points         sensor_msgs/PointCloud2         best effort, as the
//!                                                                    radar publishes it
//! Publishes   /radar/clusters       sensor_msgs/PointCloud2         the clustered points,
//!             coloured by cluster, with its number in a `cluster` field; noise left out
//!             /radar/cluster_boxes  visualization_msgs/MarkerArray  a box round each
//!             cluster, for RViz, after a DELETEALL that clears the last cloud's boxes
//! Parameters  eps (m): points this close are neighbours; minimum_points: the neighbours a
//!             point needs, itself included, to start or grow a cluster
//!
//! Both outputs keep the input's header, so they carry the radar's time and frame. The
//! maths is in the `radar-clustering` crate, with no ROS. Rewritten from the 2025-26 C++
//! node (`dbscan` in `aav_object_detection`); where it differs, on purpose:
//! - eps is a distance. The C++ compared it with the squared distance, so its 0.8 was
//!   0.89 m;
//! - noise points are left out. The C++ sent one point at (0, 0, 0) for each;
//! - an empty cloud clears the clusters and boxes. The C++ sent the last ones again;
//! - the boxes are cleared each time, so a box goes when its object does;
//! - intensity comes through (the C++ lost it to a typo), and the clusters are published
//!   reliable, as the contract says (the C++: best effort);
//! - it stops cleanly on Ctrl-C.

use std::{sync::Arc, time::Duration};

use anyhow::{Context as _, Result};
use radar_clustering::{
    cloud::{self, CloudView, Field, CLUSTER_FIELDS, CLUSTER_POINT_STEP},
    cluster, colour, Cluster, Metres, Point, Settings, BOX_MARGIN,
};
use rclrs::*;
use ros_env::geometry_msgs::msg::{Point as Position, Pose, Vector3};
use ros_env::sensor_msgs::msg::{PointCloud2, PointField};
use ros_env::std_msgs::msg::{ColorRGBA, Header};
use ros_env::visualization_msgs::msg::{Marker, MarkerArray};

/// The boxes' namespace in RViz: the node's name.
const NAMESPACE: &str = "radar_clusters";
/// How see-through the boxes are: 0.2, as the C++ drew them.
const BOX_ALPHA: f32 = 0.2;

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("radar_clusters")?;

    let params = Params::declare(&node)?;
    let (eps, minimum_points) = (params.eps.get(), params.minimum_points.get());
    let min_points = usize::try_from(minimum_points)
        .with_context(|| format!("minimum_points {minimum_points} is not a count"))?;
    let settings = Settings::new(Metres(eps), min_points)
        .with_context(|| format!("eps {eps} m, minimum_points {minimum_points}"))?;
    log_info!(
        node.logger(),
        "Clustering /radar/points: eps {eps} m, minimum_points {minimum_points}"
    );

    let out = Output {
        clusters: node.create_publisher("/radar/clusters")?,
        boxes: node.create_publisher("/radar/cluster_boxes")?,
    };
    let logger = node.logger().clone();
    let _subscription =
        node.create_subscription("/radar/points".best_effort(), move |radar: PointCloud2| {
            if let Err(e) = out.publish(&radar, &settings) {
                log_warn!(logger.throttle(Duration::from_secs(1)), "{e:#}");
            }
        })?;

    // Ctrl-C, or launch stopping us: stop spinning, and exit cleanly.
    let commands = Arc::clone(executor.commands());
    ctrlc::set_handler(move || commands.halt_spinning())?;
    executor.spin(SpinOptions::default()).first_error()?;
    Ok(())
}

/// The node's parameters; car.launch.py and bench.launch.py set them through
/// radar_clusters.launch.py.
///
/// Keep this alive as long as the node: rclrs undeclares a parameter when its handle is
/// dropped, and then `ros2 param` can't see it. Read-only, because they are read once.
struct Params {
    eps: ReadOnlyParameter<f64>,
    minimum_points: ReadOnlyParameter<i64>,
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        Ok(Self {
            eps: node.declare_parameter("eps").default(0.8).read_only()?, // m
            minimum_points: node
                .declare_parameter("minimum_points")
                .default(15)
                .read_only()?,
        })
    }
}

/// The two publishers.
struct Output {
    clusters: Publisher<PointCloud2>,
    boxes: Publisher<MarkerArray>,
}

impl Output {
    /// Clusters one radar cloud, and publishes its clusters and their boxes.
    fn publish(&self, radar: &PointCloud2, settings: &Settings) -> Result<()> {
        let view = CloudView {
            fields: radar.fields.iter().map(field).collect(),
            width: radar.width,
            height: radar.height,
            point_step: radar.point_step,
            row_step: radar.row_step,
            is_bigendian: radar.is_bigendian,
            data: &radar.data,
        };
        let points = cloud::read_points(&view).context("reading /radar/points")?;
        let clusters = cluster(&points, settings);
        self.clusters
            .publish(clusters_cloud(&radar.header, &points, &clusters))?;
        self.boxes.publish(boxes(&radar.header, &clusters))?;
        Ok(())
    }
}

fn field(f: &PointField) -> Field<'_> {
    Field {
        name: &f.name,
        offset: f.offset,
        datatype: f.datatype,
        count: f.count,
    }
}

/// The clustered points, one row, in the radar's frame and time.
fn clusters_cloud(header: &Header, points: &[Point], clusters: &[Cluster]) -> PointCloud2 {
    let data = cloud::write_clusters(points, clusters);
    let width = u32::try_from(data.len()).unwrap_or(u32::MAX) / CLUSTER_POINT_STEP;
    PointCloud2 {
        header: header.clone(),
        height: 1,
        width,
        fields: CLUSTER_FIELDS
            .iter()
            .map(|f| PointField {
                name: f.name.to_string(),
                offset: f.offset,
                datatype: f.datatype,
                count: f.count,
            })
            .collect(),
        is_bigendian: false,
        point_step: CLUSTER_POINT_STEP,
        row_step: CLUSTER_POINT_STEP.saturating_mul(width),
        data,
        is_dense: true,
    }
}

/// A DELETEALL for this node's boxes, then a box round each cluster: id 1 for the first,
/// as the clusters cloud numbers them.
fn boxes(header: &Header, clusters: &[Cluster]) -> MarkerArray {
    let clear = Marker {
        header: header.clone(),
        ns: NAMESPACE.into(),
        action: Marker::DELETEALL,
        ..Marker::default()
    };
    let cubes = clusters.iter().enumerate().map(|(index, c)| {
        let cube = c.bounds.padded(BOX_MARGIN);
        let ([x, y, z], [sx, sy, sz]) = (cube.centre, cube.size);
        let [r, g, b] = colour(index).unit();
        Marker {
            header: header.clone(),
            ns: NAMESPACE.into(),
            id: i32::try_from(index + 1).unwrap_or(i32::MAX),
            type_: Marker::CUBE,
            action: Marker::ADD,
            pose: Pose {
                position: Position { x, y, z },
                ..Pose::default()
            },
            scale: Vector3 {
                x: sx,
                y: sy,
                z: sz,
            },
            color: ColorRGBA {
                r,
                g,
                b,
                a: BOX_ALPHA,
            },
            ..Marker::default()
        }
    });
    MarkerArray {
        markers: std::iter::once(clear).chain(cubes).collect(),
    }
}
