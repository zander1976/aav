"""
When to stop for a stop sign, from the detector's boxes. No ROS imports.

A sign counts once its box is tall enough (close) and its score good enough. Then the
lock goes on for `hold_s` seconds: twist_mux holds the autonomous drivers, and ego stops
the car. After that the lock goes off, and signs are ignored for `ignore_s` seconds, so
the car can drive past the one it stopped for.

    CLEAR --a close sign--> STOPPING --hold_s--> PASSING --ignore_s--> CLEAR
              (lock on)                  (lock off, signs ignored)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Tuple

from aav_yolo.boxes import normal_label

CLEAR, STOPPING, PASSING = 'clear', 'stopping', 'passing'


@dataclass
class StopRule:
    """Thresholds, from the watcher's parameters."""

    # The bench camera's, and Gazebo's (one lens): about 4.7 m away. A sign 2 m right of the
    # lane is still in the picture then, with the car 0.7 m off its line and turned 5
    # degrees, as Nav2 on GPS leaves it. At 72 px it had often left the picture (Gazebo).
    min_height_px: float = 55.0
    min_score: float = 0.5
    hold_s: float = 4.0           # about 1 s to stop, then a 3 s wait
    ignore_s: float = 10.0


class StopSignWatcher:

    def __init__(self, rule: StopRule):
        self.rule = rule
        self.state = CLEAR
        self.since: Optional[float] = None

    def close_sign(self, boxes: Iterable[Tuple[str, float, float]]) -> bool:
        """Report whether any (label, score, height_px) box is a stop sign close enough."""
        return any(normal_label(label) == 'stop sign' and score >= self.rule.min_score
                   and height >= self.rule.min_height_px for label, score, height in boxes)

    def update(self, boxes: Iterable[Tuple[str, float, float]], now: float) -> bool:
        """Take the newest boxes; return True while the car must stay stopped."""
        if self.state == CLEAR and self.close_sign(boxes):
            self.state, self.since = STOPPING, now
        elif self.state == STOPPING and now - self.since >= self.rule.hold_s:
            self.state, self.since = PASSING, now
        elif self.state == PASSING and now - self.since >= self.rule.ignore_s:
            self.state, self.since = CLEAR, None
        return self.state == STOPPING

    def tick(self, now: float) -> bool:
        """Advance the timers without new boxes (the detector can go quiet)."""
        return self.update((), now)
