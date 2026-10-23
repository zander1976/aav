"""stalls: late messages and topics quiet together, as `pixi run timing-test` counts them."""

import math
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import pytest  # noqa: E402
import stalls as s  # noqa: E402


def steady(hz, seconds, start=0.0, holes=()):
    """Return arrival times at `hz`, without those inside any (from, to) hole."""
    times = [start + i / hz for i in range(int(seconds * hz) + 1)]
    return [t for t in times if not any(a < t < b for a, b in holes)]


def test_a_steady_topic_is_never_late():
    (row,) = s.summary({'/imu': steady(100, 10)}, 10.0)
    assert row.hz == pytest.approx(100.1)
    assert row.period == pytest.approx(0.01)
    assert row.worst_late == pytest.approx(0.0, abs=1e-9)
    assert row.late == row.very_late == 0


def test_lateness_is_counted_against_the_topics_own_rate():
    # The same 0.25 s gap: 0.24 s late for a 100 Hz IMU, 0.15 s for a 10 Hz camera.
    imu = steady(100, 10, holes=[(5.0, 5.25)])
    camera = steady(10, 10, holes=[(5.0, 5.25)])
    rows = {r.topic: r for r in s.summary({'/imu': imu, '/camera': camera}, 10.0)}
    assert rows['/imu'].worst_late == pytest.approx(0.24)
    assert rows['/camera'].worst_gap == pytest.approx(0.3)     # 5.0 to 5.3
    assert rows['/camera'].worst_late == pytest.approx(0.2)
    assert rows['/imu'].late == rows['/camera'].late == 1
    assert rows['/imu'].very_late == 0


def test_a_long_hole_is_very_late():
    (row,) = s.summary({'/gps': steady(10, 20, holes=[(3.0, 3.45), (9.0, 9.95)])}, 20.0)
    assert row.late == 2
    assert row.very_late == 1
    assert row.worst_gap == pytest.approx(1.0)


def test_a_topic_with_one_message_has_no_gaps():
    never, once = s.summary({'/once': [4.0], '/never': []}, 10.0)
    assert (never.count, once.count) == (0, 1)
    assert math.isnan(once.worst_gap) and once.late == 0
    assert s.late_gaps('/never', []) == []


def test_the_window_must_be_positive():
    with pytest.raises(ValueError):
        s.summary({}, 0.0)


def test_topics_quiet_together_are_one_stall():
    hole = [(20.0, 20.7)]
    gaps = (s.late_gaps('/imu', steady(100, 60, holes=hole))
            + s.late_gaps('/wheels', steady(50, 60, holes=[(20.1, 20.8)]))
            + s.late_gaps('/ekf', steady(30, 60, holes=hole))
            + s.late_gaps('/gps', steady(10, 60, holes=[(40.0, 40.6)])))
    (stall,) = s.stalls(gaps, least=2)
    assert stall.topics == ['/ekf', '/imu', '/wheels']
    assert stall.start == pytest.approx(20.01, abs=0.02)
    assert stall.end == pytest.approx(20.8, abs=0.03)
    # The GPS's own hole, alone, is late but no stall; with least=1 it is one too.
    assert len(s.stalls(gaps, least=1)) == 2
    assert s.stalls(gaps, least=4) == []


def test_a_chain_of_overlaps_is_one_stall():
    gaps = [s.Gap('/a', 0.0, 1.0, 0.0), s.Gap('/b', 0.9, 2.0, 0.0), s.Gap('/c', 1.9, 3.0, 0.0)]
    (stall,) = s.stalls(gaps, least=3)
    assert (stall.start, stall.end) == (0.0, 3.0)
    assert stall.length == 3.0


def test_one_topic_late_twice_in_a_row_is_not_a_stall():
    gaps = [s.Gap('/a', 0.0, 1.0, 0.1), s.Gap('/a', 1.0, 2.0, 0.1)]
    assert s.stalls(gaps, least=2) == []


@pytest.mark.parametrize('text, seconds', [
    ('0:01.50', 1.5),             # macOS: M:SS.cc
    ('12:03.25', 723.25),
    ('1:02:03', 3723.0),          # Linux: HH:MM:SS
    ('2-01:00:00', 2 * 86400 + 3600.0),
    ('', None),
])
def test_ps_times(text, seconds):
    assert s.ps_seconds(text) == seconds


def test_a_process_groups_cpu_is_its_members_sum():
    ps = '  501   0:01.50\n  777   1:00.00\n  501   0:02.50\n garbage\n'
    assert s.group_cpu(ps, 501) == pytest.approx(4.0)
    assert s.group_cpu(ps, 999) == 0.0


def test_tops_idle_share():
    mac = ('Processes: 700 total\nCPU usage: 9.0% user, 5.0% sys, 86.0% idle\n\n'
           'Processes: 700 total\nCPU usage: 20.5% user, 10.0% sys, 69.5% idle\n')
    assert s.top_idle(mac) == 69.5
    linux = '%Cpu(s):  3.1 us,  1.0 sy,  0.0 ni, 95.4 id,  0.0 wa'
    assert s.top_idle(linux) == 95.4
    assert s.top_idle('nothing') is None


def test_mean_skips_missing_samples():
    assert s.mean([1.0, None, 3.0]) == 2.0
    assert math.isnan(s.mean([]))
