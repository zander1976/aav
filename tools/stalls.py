"""
Late messages and stalls: the numbers `pixi run timing-test` prints (tools/timing_test.py).

No ROS here, so the tests run anywhere (tools/tests/test_stalls.py).

The input is each topic's arrival times (s, one clock for all, such as time.monotonic()).

  period     a topic's usual wait between messages: the median gap
  late       how much longer than that one gap took: gap - period. A 10 Hz camera's
             0.25 s gap is 0.15 s late; a 100 Hz IMU's 0.25 s gap is 0.24 s late
  quiet      while a late topic said nothing it should have: from one period after its
             last message until its next one
  stall      several topics quiet at the same time (their quiet times overlap): one
             cause, such as the messaging or the whole computer pausing, not one sensor

Also two readers for the computer's own load, from `ps` and `top` (macOS and Linux).
"""

from dataclasses import dataclass, field
import math
import re
import statistics

LATE = 0.1          # s: late enough to count
VERY_LATE = 0.5     # s: the sensor watchdog's own limit for most inputs


@dataclass
class Gap:
    """One late gap of one topic."""

    topic: str
    start: float        # the message before it arrived (s)
    end: float          # the next one arrived (s)
    period: float       # the topic's usual gap (s)

    @property
    def length(self):
        """Return the gap (s)."""
        return self.end - self.start

    @property
    def late(self):
        """Return how much longer than usual it was (s)."""
        return self.length - self.period

    @property
    def quiet_from(self):
        """Return when the missing message was due (s)."""
        return self.start + self.period


@dataclass
class Row:
    """One topic over the window."""

    topic: str
    count: int = 0
    hz: float = 0.0
    period: float = math.nan
    worst_gap: float = math.nan
    worst_late: float = math.nan
    late: int = 0           # gaps more than LATE late
    very_late: int = 0      # gaps more than VERY_LATE late


@dataclass
class Stall:
    """Topics quiet at the same time."""

    start: float
    end: float
    topics: list = field(default_factory=list)      # sorted names

    @property
    def length(self):
        """Return how long the stall lasted (s)."""
        return self.end - self.start


def period(times):
    """Return a topic's usual gap: the median gap (s); nan with fewer than 2 messages."""
    gaps = [b - a for a, b in zip(times, times[1:])]
    return statistics.median(gaps) if gaps else math.nan


def late_gaps(topic, times, over=LATE):
    """Return the topic's gaps more than `over` s longer than its period."""
    usual = period(times)
    if math.isnan(usual):
        return []
    return [Gap(topic, a, b, usual) for a, b in zip(times, times[1:]) if b - a - usual > over]


def summary(arrivals, window):
    """
    Return one Row per topic, in name order.

    arrivals: {topic: [arrival times, rising]}; window: the measuring time (s).
    """
    if window <= 0:
        raise ValueError(f'the window must be longer than 0 s, not {window}')
    rows = []
    for topic, times in sorted(arrivals.items()):
        row = Row(topic, count=len(times), hz=len(times) / window, period=period(times))
        if len(times) > 1:
            row.worst_gap = max(b - a for a, b in zip(times, times[1:]))
            row.worst_late = row.worst_gap - row.period
            gaps = late_gaps(topic, times)
            row.late = len(gaps)
            row.very_late = sum(1 for g in gaps if g.late > VERY_LATE)
        rows.append(row)
    return rows


def stalls(gaps, least=2):
    """
    Return the times when at least `least` different topics were quiet together.

    Quiet times that overlap join one stall (a chain: A overlaps B, B overlaps C), which
    lasts from the first one's start to the last one's end.
    """
    found = []
    current = None
    for gap in sorted(gaps, key=lambda g: g.quiet_from):
        if current is not None and gap.quiet_from <= current.end:
            current.end = max(current.end, gap.end)
            if gap.topic not in current.topics:
                current.topics.append(gap.topic)
            continue
        current = Stall(gap.quiet_from, gap.end, [gap.topic])
        found.append(current)
    for stall in found:
        stall.topics.sort()
    return [s for s in found if len(s.topics) >= least]


def ps_seconds(text):
    """Return a CPU time from ps (`time`: [D-]H:M:S, M:S.cc) in seconds; None if empty."""
    text = text.strip()
    if not text:
        return None
    days, _, clock = text.rpartition('-')
    total = 0.0
    for part in clock.split(':'):
        total = total * 60.0 + float(part)
    return total + (float(days) * 86400.0 if days else 0.0)


def group_cpu(ps_output, pgid):
    """Return the CPU seconds of a process group, from `ps -A -o pgid=,time=`."""
    total = 0.0
    for line in ps_output.splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[0] == str(pgid):
            total += ps_seconds(parts[1]) or 0.0
    return total


def top_idle(top_output):
    """Return the last idle percentage in `top`'s output (macOS: 'CPU usage: ... % idle')."""
    found = re.findall(r'([\d.]+)%\s*idle', top_output) or re.findall(
        r'([\d.]+)\s*id\b', top_output)         # Linux: '%Cpu(s): ... 95.0 id'
    return float(found[-1]) if found else None


def mean(values):
    """Return the mean, or nan for none."""
    values = [v for v in values if v is not None]
    return sum(values) / len(values) if values else math.nan
