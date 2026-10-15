"""
Pack and unpack the AAV CAN frames defined in firmware/can/aav.dbc.

No ROS imports, so it runs and tests anywhere. The Rust firmware
(firmware/spine/crates/spine-core/src/protocol.rs) implements the same frames.
Both are checked against firmware/can/test_vectors.json, and
test/test_can_codec.py checks this table against the DBC.

Wire rules, for every frame:
- Classic CAN, 11-bit IDs, signals little-endian (Intel), physical = raw * scale.
- The last byte is a CRC-8/SAE-J1850 over the ID (low byte, then high byte)
  followed by every other data byte. A frame sent with the wrong ID fails it.
- A 4-bit rolling counter goes up by one per frame of that ID.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Optional

ESTOP_CMD_ID = 0x010
DRIVE_CMD_ID = 0x100
VEHICLE_FEEDBACK_ID = 0x180
VEHICLE_STATUS_ID = 0x181
WHEEL_SPEED_L_ID = 0x200
WHEEL_SPEED_R_ID = 0x201
WHEEL_SPEED_IDS = (WHEEL_SPEED_L_ID, WHEEL_SPEED_R_ID)

MODE_MANUAL = 0
MODE_RC = 1
MODE_COMPUTER = 2


class FrameError(ValueError):
    """A received frame has the wrong length, an unknown ID or a bad CRC."""


@dataclass(frozen=True)
class Signal:
    start: int            # bit position of the least significant bit
    length: int           # bits
    signed: bool = False
    scale: float = 1.0    # physical units per bit


@dataclass(frozen=True)
class FrameLayout:
    name: str
    dlc: int
    signals: Dict[str, Signal]


def _wheel_layout(name: str) -> FrameLayout:
    """WHEEL_SPEED_L and WHEEL_SPEED_R share one layout."""
    return FrameLayout(name, 8, {
        'wheel_speed': Signal(0, 16, True, 0.001),    # m/s at the tyre
        'pulse_count': Signal(16, 16),                # wraps at 65536
        'direction_known': Signal(32, 1),             # 0 = wheel_speed is a magnitude
        'counter': Signal(52, 4),
        'checksum': Signal(56, 8),
    })


FRAMES: Dict[int, FrameLayout] = {
    ESTOP_CMD_ID: FrameLayout('ESTOP_CMD', 2, {
        'estop_request': Signal(0, 1),
        'counter': Signal(4, 4),
        'checksum': Signal(8, 8),
    }),
    DRIVE_CMD_ID: FrameLayout('DRIVE_CMD', 8, {
        'target_speed': Signal(0, 16, True, 0.001),         # m/s
        'steering_angle': Signal(16, 16, True, 0.0001),     # rad, + left
        'accel_limit': Signal(32, 8, False, 0.05),          # m/s^2, 0 = firmware default
        'steer_rate_limit': Signal(40, 8, False, 0.02),     # rad/s, 0 = firmware default
        'armed': Signal(48, 1),
        'counter': Signal(52, 4),
        'checksum': Signal(56, 8),
    }),
    VEHICLE_FEEDBACK_ID: FrameLayout('VEHICLE_FEEDBACK', 8, {
        'measured_speed': Signal(0, 16, True, 0.001),             # m/s
        'measured_steering_angle': Signal(16, 16, True, 0.0001),  # rad, + left
        'wheel_ticks': Signal(32, 16),                            # wraps at 65536
        'counter': Signal(52, 4),
        'checksum': Signal(56, 8),
    }),
    VEHICLE_STATUS_ID: FrameLayout('VEHICLE_STATUS', 8, {
        'mode': Signal(0, 2),                         # MODE_*
        'armed': Signal(2, 1),
        'estop_active': Signal(3, 1),
        'throttle': Signal(8, 8, False, 0.5),         # % of full throttle
        'brake': Signal(16, 8, False, 0.5),           # % of full brake
        'faults': Signal(24, 16),                     # VehicleStatus.FAULT_* bits
        'counter': Signal(52, 4),
        'checksum': Signal(56, 8),
    }),
    WHEEL_SPEED_L_ID: _wheel_layout('WHEEL_SPEED_L'),
    WHEEL_SPEED_R_ID: _wheel_layout('WHEEL_SPEED_R'),
}


def crc8_sae_j1850(data: bytes) -> int:
    """CRC-8/SAE-J1850: poly 0x1D, init 0xFF, not reflected, xorout 0xFF."""
    crc = 0xFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1D) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc ^ 0xFF


def frame_crc(can_id: int, body: bytes) -> int:
    """CRC of a frame: the 11-bit ID (low byte, high byte), then the body."""
    return crc8_sae_j1850(bytes([can_id & 0xFF, (can_id >> 8) & 0xFF]) + body)


def _per_unit(sig: Signal) -> int:
    """Bits per physical unit (1000 for 0.001). Exact, so Python and Rust agree."""
    return round(1.0 / sig.scale)


def _to_raw(value: float, sig: Signal) -> int:
    if isinstance(value, bool):
        value = int(value)
    if not math.isfinite(value):
        raise ValueError('signal value must be finite')
    x = value * _per_unit(sig)
    raw = int(x + 0.5) if x >= 0 else int(x - 0.5)  # half away from zero, like the firmware
    if sig.signed:
        lo, hi = -(1 << (sig.length - 1)), (1 << (sig.length - 1)) - 1
    else:
        lo, hi = 0, (1 << sig.length) - 1
    return max(lo, min(hi, raw))  # saturate rather than wrap


def _from_raw(word: int, sig: Signal) -> float:
    raw = (word >> sig.start) & ((1 << sig.length) - 1)
    if sig.signed and raw & (1 << (sig.length - 1)):
        raw -= 1 << sig.length
    per = _per_unit(sig)
    return raw / per if per != 1 else raw


def pack(can_id: int, values: Dict[str, float]) -> bytes:
    """Build a frame's data bytes from physical values; missing signals are 0."""
    layout = FRAMES[can_id]
    unknown = set(values) - set(layout.signals) - {'checksum'}
    if unknown:
        raise KeyError(f'{layout.name} has no signal(s) {sorted(unknown)}')
    word = 0
    for name, sig in layout.signals.items():
        if name == 'checksum':
            continue
        raw = _to_raw(values.get(name, 0), sig)
        word |= (raw & ((1 << sig.length) - 1)) << sig.start
    body = word.to_bytes(layout.dlc, 'little')[:-1]
    return body + bytes([frame_crc(can_id, body)])


def unpack(can_id: int, data: bytes) -> Dict[str, float]:
    """Check and decode a frame. Raises FrameError if it is not valid."""
    layout = FRAMES.get(can_id)
    if layout is None:
        raise FrameError(f'unknown CAN id 0x{can_id:03X}')
    data = bytes(data)
    if len(data) != layout.dlc:
        raise FrameError(f'{layout.name}: expected {layout.dlc} bytes, got {len(data)}')
    if frame_crc(can_id, data[:-1]) != data[-1]:
        raise FrameError(f'{layout.name}: bad checksum')
    word = int.from_bytes(data, 'little')
    return {name: _from_raw(word, sig)
            for name, sig in layout.signals.items() if name != 'checksum'}


def counter_step(previous: Optional[int], current: int) -> int:
    """Frames advanced since the last one: 1 is normal, 0 a repeat, >1 lost frames."""
    if previous is None:
        return 1
    return (current - previous) % 16
