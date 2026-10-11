//! imu_node: the Yahboom 10-axis IMU as ROS topics, in Rust.
//!
//! Rewritten from the 2025-26 C++ driver (`yahboom_imu_cpp`, removed: PROVENANCE.md), with
//! its node name, parameters, topics, units and covariances: `aav_sensors/config/imu.yaml`
//! sets them.
//!
//! Publishes   /imu/data_raw  sensor_msgs/Imu            gyro and accel, no orientation
//!             /imu/mag       sensor_msgs/MagneticField  raw counts, y negated (README)
//! Parameters  port, baud_rate (0: a pseudo-terminal), frame_id, gyro_stddev (rad/s),
//!             accel_stddev (m/s^2)
//!
//! A thread reads the port and publishes; the main thread spins the node, so `ros2 param`
//! works. Where it differs from the C++ driver, on purpose:
//! - it exits with an error when the port goes away, instead of spinning silently, so a
//!   launch `respawn` can bring it back;
//! - any baud rate the port supports works (the C++ fell back to 9600 for unknown rates);
//!   0 leaves the speed alone, for a pseudo-terminal in a test: macOS refuses to set a
//!   speed on one, and serialport skips that call for 0;
//! - it stops cleanly on Ctrl-C.

use std::{
    io::ErrorKind,
    sync::{
        atomic::{AtomicBool, Ordering},
        Arc,
    },
    thread,
    time::Duration,
};

use anyhow::{bail, Context as _, Result};
use rclrs::*;
use ros_env::builtin_interfaces::msg::Time;
use ros_env::sensor_msgs::msg::{Imu, MagneticField};
use yahboom_protocol::{Decoder, Event, Sample};

/// How long a read waits before checking whether the node is stopping.
const READ_TIMEOUT: Duration = Duration::from_millis(100);

/// The publishers, and what every message carries, fixed at start-up.
struct Output {
    imu: Publisher<Imu>,
    mag: Publisher<MagneticField>,
    frame_id: String,
    gyro_var: f64,
    accel_var: f64,
}

impl Output {
    /// Publishes one sample, filled in field by field as the C++ driver fills it.
    fn publish(&self, s: &Sample, stamp: Time) -> Result<(), RclrsError> {
        let mut imu = Imu::default();
        imu.header.stamp = stamp.clone();
        imu.header.frame_id = self.frame_id.clone();
        let a = &mut imu.linear_acceleration;
        [a.x, a.y, a.z] = s.accel;
        let w = &mut imu.angular_velocity;
        [w.x, w.y, w.z] = s.gyro;
        // No orientation here: imu_filter_madgwick adds it. -1 says so (REP-145).
        imu.orientation_covariance[0] = -1.0;
        // robot_localization reads an all-zero covariance as a perfect sensor.
        for i in [0, 4, 8] {
            imu.angular_velocity_covariance[i] = self.gyro_var;
            imu.linear_acceleration_covariance[i] = self.accel_var;
        }
        self.imu.publish(imu)?;

        let mut mag = MagneticField::default();
        mag.header.stamp = stamp;
        mag.header.frame_id = self.frame_id.clone();
        let m = &mut mag.magnetic_field;
        [m.x, m.y, m.z] = s.mag;
        self.mag.publish(mag)
    }
}

fn main() -> Result<()> {
    let context = Context::default_from_env()?;
    let mut executor = context.create_basic_executor();
    let node = executor.create_node("imu_node")?;

    let params = Params::declare(&node)?;
    let (port, baud_rate) = (params.port.get(), params.baud_rate.get());
    let (gyro_stddev, accel_stddev) = (params.gyro_stddev.get(), params.accel_stddev.get());

    let baud = u32::try_from(baud_rate)
        .with_context(|| format!("baud_rate {baud_rate} is not a baud rate"))?;
    let mut serial = serialport::new(&*port, baud)
        .timeout(READ_TIMEOUT)
        .open()
        .with_context(|| format!("Failed to open: {port}"))?;
    log_info!(node.logger(), "Opened {port} at {baud} baud");

    let out = Output {
        imu: node.create_publisher("/imu/data_raw")?,
        mag: node.create_publisher("/imu/mag")?,
        frame_id: params.frame_id.get().to_string(),
        gyro_var: gyro_stddev * gyro_stddev,
        accel_var: accel_stddev * accel_stddev,
    };

    // Ctrl-C, or launch stopping us: halt the executor, and the reader stops with it.
    let running = Arc::new(AtomicBool::new(true));
    let commands = Arc::clone(executor.commands());
    {
        let running = Arc::clone(&running);
        let commands = Arc::clone(&commands);
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
            commands.halt_spinning();
        })?;
    }

    let reader = {
        let node = Arc::clone(&node);
        let running = Arc::clone(&running);
        thread::spawn(move || {
            let result = read_loop(&node, serial.as_mut(), &out, &running);
            if let Err(e) = &result {
                log_error!(node.logger(), "{e:#}");
            }
            // Whatever ended the loop ends the node.
            running.store(false, Ordering::SeqCst);
            commands.halt_spinning();
            result
        })
    };

    executor.spin(SpinOptions::default()).first_error()?;
    running.store(false, Ordering::SeqCst);
    match reader.join() {
        Ok(result) => result,
        Err(_) => bail!("the reader thread panicked"),
    }
}

/// The C++ driver's parameters and defaults; config/imu.yaml sets them on the car.
///
/// Keep this alive as long as the node: rclrs undeclares a parameter when its handle is
/// dropped, and then `ros2 param` can't see it. Read-only, because they are read once.
struct Params {
    port: ReadOnlyParameter<Arc<str>>,
    baud_rate: ReadOnlyParameter<i64>,
    frame_id: ReadOnlyParameter<Arc<str>>,
    gyro_stddev: ReadOnlyParameter<f64>,
    accel_stddev: ReadOnlyParameter<f64>,
}

impl Params {
    fn declare(node: &Node) -> Result<Self> {
        fn param<T: ParameterVariant + 'static>(
            node: &Node,
            name: &str,
            default: T,
        ) -> Result<ReadOnlyParameter<T>> {
            Ok(node.declare_parameter(name).default(default).read_only()?)
        }
        Ok(Self {
            port: param(node, "port", "/dev/aav/imu".into())?, // the udev name
            baud_rate: param(node, "baud_rate", 115_200)?,
            frame_id: param(node, "frame_id", "imu_link".into())?,
            // 1-sigma noise, published as variances. Starting guesses: measure them.
            gyro_stddev: param(node, "gyro_stddev", 0.02)?, // rad/s
            accel_stddev: param(node, "accel_stddev", 0.2)?, // m/s^2
        })
    }
}

/// Reads the port until the node stops or the port fails; publishes each sample.
fn read_loop(
    node: &Node,
    serial: &mut dyn serialport::SerialPort,
    out: &Output,
    running: &AtomicBool,
) -> Result<()> {
    let clock = node.get_clock();
    let mut decoder = Decoder::default();
    let mut buf = [0u8; 256];
    while running.load(Ordering::SeqCst) {
        let n = match serial.read(&mut buf) {
            Ok(0) => bail!("the IMU's port closed"),
            Ok(n) => n,
            Err(e) if matches!(e.kind(), ErrorKind::TimedOut | ErrorKind::Interrupted) => continue,
            Err(e) => return Err(e).context("reading the IMU's port"),
        };
        for &byte in &buf[..n] {
            match decoder.push(byte) {
                Some(Event::Sample(s)) => {
                    let stamp = clock.now().to_ros_msg().unwrap_or_default();
                    out.publish(&s, stamp)?;
                }
                Some(Event::BadChecksum) => {
                    log_warn!(
                        node.logger().throttle(Duration::from_secs(1)),
                        "Checksum mismatch"
                    )
                }
                None => {}
            }
        }
    }
    Ok(())
}
