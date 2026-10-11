//! The Yahboom 10-axis IMU's serial protocol: frames, checksums and units.
//!
//! Purpose: turn the IMU's byte stream into one [`Sample`] per set of readings, with the
//! same numbers the 2025-26 C++ driver (`yahboom_imu_cpp`) published: `tests/cpp_vectors.rs`
//! checks them. No ROS and no I/O, so it is tested on any laptop with `cargo test`.
//!
//! The IMU repeats a set of 11-byte frames: acceleration (0x51), angular rate (0x52),
//! angles (0x53) and magnetic field (0x54). A frame is the header 0x55, its type, four
//! little-endian `i16` values, and a checksum: the low byte of the sum of the first ten
//! bytes. The angle frame closes a set, so that is when a sample comes out.
#![no_std]

use core::f64::consts::PI;

/// Every frame starts with this byte.
pub const HEADER: u8 = 0x55;
/// The header, the type, 8 data bytes and the checksum.
pub const FRAME_LEN: usize = 11;

/// Frame type: acceleration, ±16 g.
pub const ACCEL: u8 = 0x51;
/// Frame type: angular rate, ±2000 °/s.
pub const GYRO: u8 = 0x52;
/// Frame type: angles. Not used, but it closes a set.
pub const ANGLE: u8 = 0x53;
/// Frame type: magnetic field.
pub const MAG: u8 = 0x54;

/// One set of readings, converted exactly as the C++ driver converted them.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Sample {
    /// Linear acceleration, m/s², in the IMU's axes.
    pub accel: [f64; 3],
    /// Angular rate, rad/s.
    pub gyro: [f64; 3],
    /// Magnetic field in raw counts with y negated, not tesla. The C++ driver publishes
    /// it this way, which is why `imu_filter_madgwick` runs with `use_mag: false`.
    pub mag: [f64; 3],
}

/// What a byte can complete.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Event {
    /// An angle frame closed a set: publish it.
    Sample(Sample),
    /// A frame failed its checksum and was dropped.
    BadChecksum,
}

/// Splits the byte stream into frames, and keeps the latest reading of each sensor.
#[derive(Clone, Debug, Default)]
pub struct Decoder {
    frame: [u8; FRAME_LEN],
    len: usize,
    latest: Sample,
}

impl Decoder {
    /// Feeds one byte from the serial port. Returns an event when the byte completes a
    /// frame that matters.
    pub fn push(&mut self, byte: u8) -> Option<Event> {
        if self.len == 0 && byte != HEADER {
            return None; // between frames: wait for a header
        }
        self.frame[self.len] = byte;
        self.len += 1;
        if self.len < FRAME_LEN {
            return None;
        }
        self.len = 0;
        if checksum(&self.frame[..FRAME_LEN - 1]) != self.frame[FRAME_LEN - 1] {
            self.resync();
            return Some(Event::BadChecksum);
        }
        self.apply()
    }

    /// After a bad frame, start again at the next header inside it. The C++ driver skips
    /// all 11 bytes instead, so a lost byte costs it two frames; here it costs one.
    fn resync(&mut self) {
        if let Some(i) = self.frame[1..].iter().position(|&b| b == HEADER) {
            let start = i + 1;
            self.frame.copy_within(start.., 0);
            self.len = FRAME_LEN - start;
        }
    }

    /// Converts a checked frame. The arithmetic is written in the C++ driver's order, so
    /// the results are the same to the last bit (g = 9.8, as there).
    fn apply(&mut self) -> Option<Event> {
        let f = &self.frame;
        let raw = |i: usize| f64::from(i16::from_le_bytes([f[2 + 2 * i], f[3 + 2 * i]]));
        match f[1] {
            ACCEL => self.latest.accel = [0, 1, 2].map(|i| raw(i) / 32768.0 * 16.0 * 9.8),
            GYRO => self.latest.gyro = [0, 1, 2].map(|i| raw(i) / 32768.0 * 2000.0 * PI / 180.0),
            MAG => self.latest.mag = [raw(0), -raw(1), raw(2)],
            ANGLE => return Some(Event::Sample(self.latest)),
            _ => {} // other frame types (quaternion, pressure...) are ignored, as in C++
        }
        None
    }
}

/// The low byte of the sum of `bytes`.
pub fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0, |sum, &b| sum.wrapping_add(b))
}

/// Builds a valid frame, as the IMU would send it. For tests, and for faking the IMU.
pub fn frame(kind: u8, values: [i16; 4]) -> [u8; FRAME_LEN] {
    let mut f = [0; FRAME_LEN];
    f[0] = HEADER;
    f[1] = kind;
    for (i, v) in values.iter().enumerate() {
        f[2 + 2 * i..4 + 2 * i].copy_from_slice(&v.to_le_bytes());
    }
    f[FRAME_LEN - 1] = checksum(&f[..FRAME_LEN - 1]);
    f
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use std::vec::Vec;

    /// Feeds bytes and collects the events.
    fn run(decoder: &mut Decoder, bytes: &[u8]) -> Vec<Event> {
        bytes.iter().filter_map(|&b| decoder.push(b)).collect()
    }

    /// One full set: +1 g on z, 100 °/s about z, a field of (100, 200, -300) counts.
    fn one_set() -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend(frame(ACCEL, [0, 0, 2048, 0]));
        bytes.extend(frame(GYRO, [0, 0, 1638, 0]));
        bytes.extend(frame(MAG, [100, 200, -300, 0]));
        bytes.extend(frame(ANGLE, [0, 0, 0, 0]));
        bytes
    }

    fn sample(events: &[Event]) -> Sample {
        match events {
            [Event::Sample(s)] => *s,
            other => panic!("expected one sample, got {other:?}"),
        }
    }

    #[test]
    fn a_set_becomes_one_sample_in_si_units() {
        let s = sample(&run(&mut Decoder::default(), &one_set()));
        assert_eq!(s.accel, [0.0, 0.0, 9.8]); // 2048 counts is exactly 1 g
        assert!((s.gyro[2] - 1638.0 / 32768.0 * 2000.0_f64.to_radians()).abs() < 1e-12);
        assert_eq!(s.mag, [100.0, -200.0, -300.0]); // y negated, as in the C++ driver
    }

    #[test]
    fn values_match_the_cpp_formulas_bit_for_bit() {
        // The C++ driver: r / 32768.0 * 16.0 * 9.8 and r / 32768.0 * 2000.0 * M_PI / 180.0.
        let mut d = Decoder::default();
        for r in [i16::MIN, -12345, -1, 1, 777, 2048, i16::MAX] {
            let mut bytes = Vec::new();
            bytes.extend(frame(ACCEL, [r, 0, 0, 0]));
            bytes.extend(frame(GYRO, [r, 0, 0, 0]));
            bytes.extend(frame(ANGLE, [0; 4]));
            let s = sample(&run(&mut d, &bytes));
            let r = f64::from(r);
            assert_eq!(s.accel[0].to_bits(), (r / 32768.0 * 16.0 * 9.8).to_bits());
            assert_eq!(
                s.gyro[0].to_bits(),
                (r / 32768.0 * 2000.0 * PI / 180.0).to_bits()
            );
        }
    }

    #[test]
    fn only_the_angle_frame_publishes() {
        let set = one_set();
        let events = run(&mut Decoder::default(), &set[..3 * FRAME_LEN]);
        assert!(events.is_empty());
    }

    #[test]
    fn a_bad_checksum_drops_one_frame() {
        let mut bytes = one_set();
        bytes[FRAME_LEN - 1] ^= 0xff; // break the acceleration frame's checksum
        let events = run(&mut Decoder::default(), &bytes);
        assert_eq!(events[0], Event::BadChecksum);
        let s = sample(&events[1..]);
        assert_eq!(s.accel, [0.0; 3]); // that reading was lost; the others came through
        assert_eq!(s.mag, [100.0, -200.0, -300.0]);
    }

    #[test]
    fn noise_before_the_first_header_is_skipped() {
        let mut bytes = std::vec![0x00, 0x13, 0xff, 0x51];
        bytes.extend(one_set());
        assert_eq!(
            sample(&run(&mut Decoder::default(), &bytes)).accel,
            [0.0, 0.0, 9.8]
        );
    }

    #[test]
    fn a_lost_byte_costs_one_frame() {
        let mut bytes = one_set();
        bytes.remove(3); // a byte of the acceleration frame never arrives
        let events = run(&mut Decoder::default(), &bytes);
        // The broken frame swallows the gyro frame's header; the decoder finds it inside.
        let s = sample(&events[1..]);
        assert_eq!(events[0], Event::BadChecksum);
        assert!(s.gyro[2] > 1.7); // the gyro frame right after it still counts
    }

    #[test]
    fn unknown_frame_types_are_ignored() {
        let mut bytes = std::vec::Vec::from(frame(0x59, [1, 2, 3, 4])); // quaternion
        bytes.extend(one_set());
        assert_eq!(run(&mut Decoder::default(), &bytes).len(), 1);
    }

    #[test]
    fn the_checksum_is_the_low_byte_of_the_sum() {
        assert_eq!(checksum(&[0x55, 0x51, 0xff, 0xff]), 0xa4);
    }
}
