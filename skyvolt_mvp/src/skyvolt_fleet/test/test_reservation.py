import pytest

from skyvolt_fleet.reservation import ReservationTable, Reservation


def test_overlapping_reservations_rejected():
    t = ReservationTable()
    assert t.admit(Reservation("r0", segment_id=1, t_enter=0, t_exit=2)) is None
    c = t.admit(Reservation("r1", segment_id=1, t_enter=1, t_exit=3))
    assert c is not None
    assert c.existing.robot_id == "r0"


def test_disjoint_reservations_accepted():
    t = ReservationTable()
    assert t.admit(Reservation("r0", segment_id=1, t_enter=0, t_exit=2)) is None
    assert t.admit(Reservation("r1", segment_id=1, t_enter=2.5, t_exit=4)) is None


def test_same_robot_can_re_reserve_same_segment():
    """A robot's two passes through same segment shouldn't conflict with itself."""
    t = ReservationTable()
    assert t.admit(Reservation("r0", segment_id=1, t_enter=0, t_exit=2)) is None
    assert t.admit(Reservation("r0", segment_id=1, t_enter=1, t_exit=3)) is None


def test_admit_route_rolls_back_on_partial_conflict():
    t = ReservationTable()
    t.admit(Reservation("r0", segment_id=2, t_enter=5, t_exit=7))
    route = [
        Reservation("r1", segment_id=1, t_enter=0, t_exit=2),
        Reservation("r1", segment_id=2, t_enter=4, t_exit=8),  # conflicts
    ]
    conflicts = t.admit_route(route)
    assert conflicts and conflicts[0].existing.robot_id == "r0"
    # Segment 1 should NOT have been left admitted
    assert all(r.robot_id != "r1" for r in t._by_segment.get(1, []))


def test_release_robot_clears_only_that_robots_entries():
    t = ReservationTable()
    t.admit(Reservation("r0", 1, 0, 2))
    t.admit(Reservation("r1", 1, 3, 5))
    t.admit(Reservation("r0", 2, 0, 2))
    assert t.release_robot("r0") == 2
    assert t.num_reservations() == 1
