//! The CAN frames of firmware/can/aav.dbc.
//!
//! Keep in step with src/backends/aav_spine/aav_spine/can_codec.py:
//! tests/vectors.rs (generated from the Python codec) checks both put the same
//! bytes on the bus. Signals are little-endian; the last byte of each frame is
//! a CRC-8/SAE-J1850 over the ID (low byte, high byte) and the other bytes.

pub const ESTOP_CMD_ID: u16 = 0x010;
pub const DRIVE_CMD_ID: u16 = 0x100;
pub const VEHICLE_FEEDBACK_ID: u16 = 0x180;
pub const VEHICLE_STATUS_ID: u16 = 0x181;
pub const WHEEL_SPEED_L_ID: u16 = 0x200;
pub const WHEEL_SPEED_R_ID: u16 = 0x201;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameError {
    WrongLength,
    BadChecksum,
    UnknownId,
}

/// Who drives the actuators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Mode {
    /// A person drives; the actuators are idle.
    Manual = 0,
    /// The RC transmitter drives.
    Rc = 1,
    /// The computer drives, through DRIVE_CMD.
    Computer = 2,
}

impl Mode {
    fn from_raw(raw: u64) -> Mode {
        match raw {
            1 => Mode::Rc,
            2 => Mode::Computer,
            _ => Mode::Manual,
        }
    }
}

/// Software e-stop request (0x010).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EstopCmd {
    pub estop_request: bool,
    pub counter: u8,
}

/// Drive command (0x100). Speeds in m/s, angles in rad (positive left).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DriveCmd {
    pub target_speed: f32,
    pub steering_angle: f32,
    /// m/s^2; 0 means the firmware default.
    pub accel_limit: f32,
    /// rad/s; 0 means the firmware default.
    pub steer_rate_limit: f32,
    pub armed: bool,
    pub counter: u8,
}

/// Measured motion (0x180).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleFeedback {
    pub measured_speed: f32,
    pub measured_steering_angle: f32,
    pub wheel_ticks: u16,
    pub counter: u8,
}

/// Spine state (0x181). Throttle and brake in percent.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleStatus {
    pub mode: Mode,
    pub armed: bool,
    pub estop_active: bool,
    pub throttle: f32,
    pub brake: f32,
    pub faults: u16,
    pub counter: u8,
}

/// Which front wheel a wheel node sits on. Each has its own frame ID.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wheel {
    Left,
    Right,
}

impl Wheel {
    /// The ID of this wheel's WHEEL_SPEED frame.
    pub const fn id(self) -> u16 {
        match self {
            Wheel::Left => WHEEL_SPEED_L_ID,
            Wheel::Right => WHEEL_SPEED_R_ID,
        }
    }

    /// The wheel a WHEEL_SPEED frame ID belongs to, if it is one.
    pub const fn from_id(id: u16) -> Option<Wheel> {
        match id {
            WHEEL_SPEED_L_ID => Some(Wheel::Left),
            WHEEL_SPEED_R_ID => Some(Wheel::Right),
            _ => None,
        }
    }
}

/// Wheel speed (0x200 left, 0x201 right), from a wheel node at 100 Hz.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct WheelSpeed {
    /// Which wheel: it picks the frame ID, so it is not in the data bytes.
    pub wheel: Wheel,
    /// m/s of the tyre over the ground. A magnitude unless `direction_known`.
    pub wheel_speed: f32,
    /// Cumulative sensor pulses; wraps at 65536.
    pub pulse_count: u16,
    /// True when the sign of `wheel_speed` is measured, not assumed.
    pub direction_known: bool,
    pub counter: u8,
}

/// Any frame the spine can receive.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Incoming {
    Estop(EstopCmd),
    Drive(DriveCmd),
}

/// Decode a frame addressed to the spine.
pub fn decode_incoming(id: u16, data: &[u8]) -> Result<Incoming, FrameError> {
    match id {
        ESTOP_CMD_ID => EstopCmd::decode(data).map(Incoming::Estop),
        DRIVE_CMD_ID => DriveCmd::decode(data).map(Incoming::Drive),
        _ => Err(FrameError::UnknownId),
    }
}

fn crc8_update(mut crc: u8, byte: u8) -> u8 {
    crc ^= byte;
    for _ in 0..8 {
        crc = if crc & 0x80 != 0 {
            (crc << 1) ^ 0x1D
        } else {
            crc << 1
        };
    }
    crc
}

/// CRC-8/SAE-J1850: poly 0x1D, init 0xFF, not reflected, xorout 0xFF.
pub fn crc8_sae_j1850(bytes: &[u8]) -> u8 {
    bytes.iter().fold(0xFF, |crc, &b| crc8_update(crc, b)) ^ 0xFF
}

/// CRC of a frame: the ID (low byte, high byte), then the body.
pub fn frame_crc(id: u16, body: &[u8]) -> u8 {
    let crc = crc8_update(crc8_update(0xFF, (id & 0xFF) as u8), (id >> 8) as u8);
    body.iter().fold(crc, |crc, &b| crc8_update(crc, b)) ^ 0xFF
}

fn put(word: &mut u64, start: u32, len: u32, raw: u64) {
    let mask = (1u64 << len) - 1;
    *word |= (raw & mask) << start;
}

fn get(word: u64, start: u32, len: u32) -> u64 {
    (word >> start) & ((1u64 << len) - 1)
}

fn get_signed(word: u64, start: u32, len: u32) -> i64 {
    let raw = get(word, start, len) as i64;
    if raw & (1 << (len - 1)) != 0 {
        raw - (1 << len)
    } else {
        raw
    }
}

/// Round half away from zero, like the Python codec. NaN becomes 0.
fn round_raw(x: f32) -> i64 {
    if x.is_nan() {
        0
    } else if x >= 0.0 {
        (x + 0.5) as i64
    } else {
        (x - 0.5) as i64
    }
}

fn raw_signed(value: f32, per_unit: f32, bits: u32) -> u64 {
    let hi = (1i64 << (bits - 1)) - 1;
    round_raw(value * per_unit).clamp(-hi - 1, hi) as u64
}

fn raw_unsigned(value: f32, per_unit: f32, bits: u32) -> u64 {
    let hi = (1i64 << bits) - 1;
    round_raw(value * per_unit).clamp(0, hi) as u64
}

fn finish<const N: usize>(id: u16, word: u64) -> [u8; N] {
    let all = word.to_le_bytes();
    let mut out = [0u8; N];
    out[..N - 1].copy_from_slice(&all[..N - 1]);
    out[N - 1] = frame_crc(id, &out[..N - 1]);
    out
}

fn check<const N: usize>(id: u16, data: &[u8]) -> Result<u64, FrameError> {
    if data.len() != N {
        return Err(FrameError::WrongLength);
    }
    if frame_crc(id, &data[..N - 1]) != data[N - 1] {
        return Err(FrameError::BadChecksum);
    }
    let mut all = [0u8; 8];
    all[..N].copy_from_slice(data);
    Ok(u64::from_le_bytes(all))
}

impl EstopCmd {
    pub fn encode(&self) -> [u8; 2] {
        let mut w = 0;
        put(&mut w, 0, 1, self.estop_request as u64);
        put(&mut w, 4, 4, self.counter as u64);
        finish(ESTOP_CMD_ID, w)
    }

    pub fn decode(data: &[u8]) -> Result<Self, FrameError> {
        let w = check::<2>(ESTOP_CMD_ID, data)?;
        Ok(EstopCmd {
            estop_request: get(w, 0, 1) != 0,
            counter: get(w, 4, 4) as u8,
        })
    }
}

impl DriveCmd {
    pub fn encode(&self) -> [u8; 8] {
        let mut w = 0;
        put(&mut w, 0, 16, raw_signed(self.target_speed, 1000.0, 16));
        put(&mut w, 16, 16, raw_signed(self.steering_angle, 10000.0, 16));
        put(&mut w, 32, 8, raw_unsigned(self.accel_limit, 20.0, 8));
        put(&mut w, 40, 8, raw_unsigned(self.steer_rate_limit, 50.0, 8));
        put(&mut w, 48, 1, self.armed as u64);
        put(&mut w, 52, 4, self.counter as u64);
        finish(DRIVE_CMD_ID, w)
    }

    pub fn decode(data: &[u8]) -> Result<Self, FrameError> {
        let w = check::<8>(DRIVE_CMD_ID, data)?;
        Ok(DriveCmd {
            target_speed: get_signed(w, 0, 16) as f32 / 1000.0,
            steering_angle: get_signed(w, 16, 16) as f32 / 10000.0,
            accel_limit: get(w, 32, 8) as f32 / 20.0,
            steer_rate_limit: get(w, 40, 8) as f32 / 50.0,
            armed: get(w, 48, 1) != 0,
            counter: get(w, 52, 4) as u8,
        })
    }
}

impl VehicleFeedback {
    pub fn encode(&self) -> [u8; 8] {
        let mut w = 0;
        put(&mut w, 0, 16, raw_signed(self.measured_speed, 1000.0, 16));
        put(
            &mut w,
            16,
            16,
            raw_signed(self.measured_steering_angle, 10000.0, 16),
        );
        put(&mut w, 32, 16, self.wheel_ticks as u64);
        put(&mut w, 52, 4, self.counter as u64);
        finish(VEHICLE_FEEDBACK_ID, w)
    }

    pub fn decode(data: &[u8]) -> Result<Self, FrameError> {
        let w = check::<8>(VEHICLE_FEEDBACK_ID, data)?;
        Ok(VehicleFeedback {
            measured_speed: get_signed(w, 0, 16) as f32 / 1000.0,
            measured_steering_angle: get_signed(w, 16, 16) as f32 / 10000.0,
            wheel_ticks: get(w, 32, 16) as u16,
            counter: get(w, 52, 4) as u8,
        })
    }
}

impl VehicleStatus {
    pub fn encode(&self) -> [u8; 8] {
        let mut w = 0;
        put(&mut w, 0, 2, self.mode as u64);
        put(&mut w, 2, 1, self.armed as u64);
        put(&mut w, 3, 1, self.estop_active as u64);
        put(&mut w, 8, 8, raw_unsigned(self.throttle, 2.0, 8));
        put(&mut w, 16, 8, raw_unsigned(self.brake, 2.0, 8));
        put(&mut w, 24, 16, self.faults as u64);
        put(&mut w, 52, 4, self.counter as u64);
        finish(VEHICLE_STATUS_ID, w)
    }

    pub fn decode(data: &[u8]) -> Result<Self, FrameError> {
        let w = check::<8>(VEHICLE_STATUS_ID, data)?;
        Ok(VehicleStatus {
            mode: Mode::from_raw(get(w, 0, 2)),
            armed: get(w, 2, 1) != 0,
            estop_active: get(w, 3, 1) != 0,
            throttle: get(w, 8, 8) as f32 / 2.0,
            brake: get(w, 16, 8) as f32 / 2.0,
            faults: get(w, 24, 16) as u16,
            counter: get(w, 52, 4) as u8,
        })
    }
}

impl WheelSpeed {
    pub fn encode(&self) -> [u8; 8] {
        let mut w = 0;
        put(&mut w, 0, 16, raw_signed(self.wheel_speed, 1000.0, 16));
        put(&mut w, 16, 16, self.pulse_count as u64);
        put(&mut w, 32, 1, self.direction_known as u64);
        put(&mut w, 52, 4, self.counter as u64);
        finish(self.wheel.id(), w)
    }

    /// Decode the frame of `wheel`. The ID is in the checksum, so a left frame decoded
    /// as a right one fails it.
    pub fn decode(wheel: Wheel, data: &[u8]) -> Result<Self, FrameError> {
        let w = check::<8>(wheel.id(), data)?;
        Ok(WheelSpeed {
            wheel,
            wheel_speed: get_signed(w, 0, 16) as f32 / 1000.0,
            pulse_count: get(w, 16, 16) as u16,
            direction_known: get(w, 32, 1) != 0,
            counter: get(w, 52, 4) as u8,
        })
    }
}

/// Frames advanced since the previous counter: 1 is normal, 0 a repeat,
/// more than 1 means frames were lost.
pub fn counter_step(previous: Option<u8>, current: u8) -> u8 {
    match previous {
        None => 1,
        Some(p) => current.wrapping_sub(p) & 0x0F,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_check_value() {
        // The catalogue check value of CRC-8/SAE-J1850.
        assert_eq!(crc8_sae_j1850(b"123456789"), 0x4B);
    }

    #[test]
    fn wrong_id_fails_checksum() {
        let frame = DriveCmd {
            target_speed: 1.0,
            steering_angle: 0.0,
            accel_limit: 0.0,
            steer_rate_limit: 0.0,
            armed: true,
            counter: 1,
        }
        .encode();
        assert_eq!(VehicleStatus::decode(&frame), Err(FrameError::BadChecksum));
    }

    #[test]
    fn corrupted_byte_is_rejected() {
        let mut frame = EstopCmd {
            estop_request: true,
            counter: 2,
        }
        .encode();
        frame[0] ^= 0x01;
        assert_eq!(EstopCmd::decode(&frame), Err(FrameError::BadChecksum));
    }

    #[test]
    fn wrong_length_is_rejected() {
        assert_eq!(DriveCmd::decode(&[0u8; 7]), Err(FrameError::WrongLength));
    }

    #[test]
    fn nan_encodes_as_zero() {
        let a = DriveCmd {
            target_speed: f32::NAN,
            steering_angle: f32::NAN,
            accel_limit: 0.0,
            steer_rate_limit: 0.0,
            armed: false,
            counter: 0,
        };
        let back = DriveCmd::decode(&a.encode()).unwrap();
        assert_eq!(back.target_speed, 0.0);
        assert_eq!(back.steering_angle, 0.0);
    }

    #[test]
    fn wheel_ids_round_trip() {
        for wheel in [Wheel::Left, Wheel::Right] {
            assert_eq!(Wheel::from_id(wheel.id()), Some(wheel));
        }
        assert_eq!(Wheel::from_id(DRIVE_CMD_ID), None);
    }

    #[test]
    fn left_wheel_frame_is_not_a_right_one() {
        let frame = WheelSpeed {
            wheel: Wheel::Left,
            wheel_speed: 1.0,
            pulse_count: 7,
            direction_known: false,
            counter: 1,
        }
        .encode();
        assert_eq!(
            WheelSpeed::decode(Wheel::Right, &frame),
            Err(FrameError::BadChecksum)
        );
    }

    #[test]
    fn counter_wraps() {
        assert_eq!(counter_step(Some(15), 0), 1);
        assert_eq!(counter_step(Some(3), 3), 0);
        assert_eq!(counter_step(Some(3), 6), 3);
        assert_eq!(counter_step(None, 9), 1);
    }
}
