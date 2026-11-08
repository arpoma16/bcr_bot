import math

import pytest


@pytest.mark.parametrize('yaw', [0, math.pi / 2, math.pi, -math.pi / 2, 2.86234])
def test_staging(yaw):
    from cart_geometry import staging_pose
    assert staging_pose((12.95, -33.59, yaw), 1.8) == pytest.approx(
        (12.95 - 1.8 * math.cos(yaw), -33.59 - 1.8 * math.sin(yaw), yaw))


@pytest.mark.parametrize('direction,expected', [
    (1, (0.62, 1.37, -0.52, 0.52)), (-1, (-1.37, -0.62, -0.52, 0.52))])
def test_leading_strip_without_lateral_clearance(direction, expected):
    from cart_geometry import leading_strip, strip_blocked
    strip = leading_strip(direction, .62, .52, .75)
    assert strip == pytest.approx(expected)
    assert strip_blocked([(direction * .8, .51)], (0., 0., 0.), strip)
    assert not strip_blocked([(direction * .8, .54), (0., .48)], (0., 0., 0.), strip)
    assert not strip_blocked([(-direction * .8, 0.)], (0., 0., 0.), strip)


def test_sweep_in_rotated_offset_cart_frame():
    from cart_geometry import leading_strip, strip_blocked
    strip = leading_strip(1, .62, .52, .75)
    assert strip_blocked([(.1, 1.)], (.1, .2, math.pi / 2), strip)
    assert not strip_blocked([(.65, 1.)], (.1, .2, math.pi / 2), strip)


@pytest.mark.parametrize('pose,expected', [
    ((0., .009, 0.), True), ((1., .011, 0.), False),
    ((1., 0., math.radians(1.1)), False),
    ((0., .01, math.radians(1.)), False),
    ((float('nan'), 0., 0.), False)])
def test_deviation_and_combined_corner_envelope(pose, expected):
    from cart_geometry import deviation_ok
    assert deviation_ok(pose, (0., 0., 0.), .01, math.radians(1.), .62, .52, .02) == expected


def test_rotated_cart_sweeps_side_wedge_in_actual_motion_direction():
    from cart_geometry import compose, motion_sweep_blocked
    local = (0., 0., .05)
    obstacle = compose(local, (.2, -.54, 0.))[:2]
    assert motion_sweep_blocked([obstacle], local, .62, .52, 1, .75)


@pytest.mark.parametrize('direction', [-1, 1])
def test_aligned_extrusion_is_only_leading_rectangle(direction):
    from cart_geometry import motion_sweep_blocked
    assert motion_sweep_blocked([(direction * .8, .5)], (0., 0., 0.), .62, .52, direction, .75)
    assert not motion_sweep_blocked([(direction * .8, .54), (0., .48)],
                                    (0., 0., 0.), .62, .52, direction, .75)


def test_placement_target_compensates_translation_and_rotation():
    from cart_geometry import compose, placement_robot_pose
    cart = (12.95, -33.59, 2.86234)
    offset = (.03, -.08, .008)
    assert compose(placement_robot_pose(cart, offset), offset) == pytest.approx(cart)
