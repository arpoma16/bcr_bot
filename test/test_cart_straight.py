from concurrent.futures import Future
import json
import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest


def completed(value):
    future = Future()
    future.set_result(value)
    return future


@pytest.fixture
def rig(handler_module, monkeypatch):
    m = handler_module
    node = object.__new__(m.CartHandler)
    clock = NS(now=10.)
    monkeypatch.setattr(m.time, 'monotonic', lambda: clock.now)
    monkeypatch.setattr(m.time, 'sleep', lambda dt: setattr(clock, 'now', clock.now + dt))
    node.approach_speed = node.backout_speed = .15
    node.straight_lateral_limit, node.straight_yaw_limit = .03, math.radians(2.)
    node.straight_envelope, node.straight_lookahead = .04, .75
    node.straight_end_margin, node.straight_cart_body_margin = .05, .03
    node.place_stop_short_limit, node.stop_short_brake = .10, .02
    node.straight_block_wait, node.straight_clear_hold = 0., 1.     # hold disabled: fail at once
    node.straight_data_timeout, node.straight_footprint_padding = .3, .05
    node.server_timeout, node.staging_distance = 1., 1.8
    node.place_tolerance_xy, node.place_tolerance_yaw = .15, math.radians(3.)
    node.place_carried_yaw_tolerance = math.radians(3.)
    node.released_leg_margin = .10
    node.leg_model = m.cart_legs.leg_model(.58, .48)
    node.lateral_tolerance, node.yaw_tolerance = .04, math.radians(3.)
    node.place_staging_along_tolerance, node.place_staging_lateral_tolerance = .05, .03
    node.place_staging_compensation_limit = .10
    node.lifted, node._motion_fault = True, False
    node.global_frame, node.base_frame = 'map', 'base_link'
    pose = [0., 0., 0.]
    node._robot_pose = lambda: tuple(pose)
    node._motion_pose = lambda: tuple(pose)
    node._fresh_stamp = lambda stamp: True
    scan = NS(header=NS(stamp=0, frame_id='laser'), ranges=[math.inf] * 360,
              angle_increment=math.pi / 180, angle_min=-math.pi, range_min=.05, range_max=30.)
    node._scan = scan, clock.now
    transform = NS(transform=NS(translation=NS(x=0., y=0.),
                                rotation=NS(x=0., y=0., z=0., w=1.)))
    node.tf_buffer = NS(lookup_transform=Mock(return_value=transform))
    logger = NS(error=Mock(), info=Mock(), debug=Mock())
    node.get_logger = lambda: logger
    events, released, action_handles = [], [], []
    node._footprints = {}

    def lease(request):
        if not request.data:
            released.append(True)
        clock.now += .001
        edges = ([1.14, 1.14, 1.34, 1.34] if not request.data and node.lifted
                 else [.74, .74, 1., 1.])
        for name in ('local_costmap', 'global_costmap'):
            node._footprints[name] = (edges, clock.now)
        return completed(NS(success=True, message=json.dumps({
            'carrying': node.lifted, 'measured': True, 'cart_pose': [0., 0., 0.],
            'half_x': .62 if node.lifted else .45, 'half_y': .52 if node.lifted else .32,
            'robot_half_x': .45, 'robot_half_y': .32})))

    node.straight_footprint = NS(wait_for_service=Mock(return_value=True), call_async=Mock(side_effect=lease))
    state = NS(after_send=None, complete_after_send=True)

    def client(name):
        def send(goal):
            events.append((name, goal.target.x))
            terminal = Future()

            def result():
                if state.after_send:
                    state.after_send()
                    if state.complete_after_send:
                        terminal.set_result(NS(status=4, result=NS(error_code=0)))
                else:
                    pose[0] += goal.target.x
                    terminal.set_result(NS(status=4, result=NS(error_code=0)))
                return terminal

            def cancel():
                if not terminal.done():
                    terminal.set_result(NS(status=5, result=NS(error_code=0)))
                return completed(NS())

            handle = NS(accepted=True, get_result_async=Mock(side_effect=result),
                        cancel_goal_async=Mock(side_effect=cancel))
            action_handles.append(handle)
            return completed(handle)
        return NS(wait_for_server=Mock(return_value=True), send_goal_async=Mock(side_effect=send))

    node.drive, node.backup = client('drive'), client('backup')
    return NS(m=m, node=node, handle=NS(is_cancel_requested=False), events=events,
              released=released, action_handles=action_handles, pose=pose, scan=scan,
              clock=clock, state=state, logger=logger)


@pytest.mark.parametrize('distance,name', [(1.8, 'drive'), (-1.8, 'backup')])
def test_straight_action_clients_and_release(rig, distance, name):
    rig.node._straight(distance, 0., rig.handle, True)
    assert rig.events == [(name, distance)]
    assert rig.released == [True]
    assert not rig.node._motion_fault


@pytest.mark.parametrize('failure', ['obstacle', 'lateral', 'yaw', 'scan_stale', 'lift', 'cancel'])
def test_active_motion_is_canceled_on_guard_failure(rig, failure):
    def inject():
        if failure == 'obstacle':
            rig.scan.ranges[180] = .8
        elif failure == 'lateral':
            rig.pose[1] = .04
        elif failure == 'yaw':
            rig.pose[2] = math.radians(2.5)
        elif failure == 'scan_stale':
            rig.node._scan = rig.scan, rig.clock.now - 1.
        elif failure == 'lift':
            rig.node.lifted = False
        else:
            rig.handle.is_cancel_requested = True
    rig.state.after_send = inject
    with pytest.raises((rig.m.Failed, rig.m.Canceled)):
        rig.node._straight(1.8, 0., rig.handle, True)
    rig.action_handles[0].cancel_goal_async.assert_called_once()
    assert rig.released == [True]


# The leading sweep covers what is left of the segment plus straight_end_margin (0.05 m), not a
# fixed 0.75 m: carts are left a few cm from a wall, and the wall is not in the way of the motion.
# The carried cart's leading edge is at 0.62 m from the robot (half_x), straight ahead is scan[180].

def test_obstacle_beyond_the_end_point_does_not_block(rig):
    rig.scan.ranges[180] = .62 + .30 + .15          # wall 15 cm past where the cart will stop
    rig.node._straight(.3, 0., rig.handle, True)
    assert rig.events == [('drive', .3)]


def test_obstacle_inside_the_remaining_motion_blocks(rig):
    rig.scan.ranges[180] = .62 + .20                # 20 cm ahead of the edge, motion is 30 cm
    with pytest.raises(rig.m.Failed, match='leading sweep'):
        rig.node._straight(.3, 0., rig.handle, True)
    assert rig.events == []                         # refused before moving


def test_obstacle_inside_the_end_margin_blocks(rig):
    rig.scan.ranges[180] = .62 + .30 + .03          # 3 cm past the end point: within the 5 cm margin
    with pytest.raises(rig.m.Failed, match='leading sweep'):
        rig.node._straight(.3, 0., rig.handle, True)


@pytest.mark.parametrize('margin', [0., .02, math.nan, 1.])
def test_invalid_end_margin_is_rejected(rig, margin):
    rig.node.straight_end_margin = margin
    with pytest.raises(rig.m.Failed, match='safety parameters'):
        rig.node._straight(.3, 0., rig.handle, True)


# The carried cart's own legs sit at the edge of its rectangle (0.4 mm inside half_x in the sim):
# returns within straight_cart_body_margin (0.03 m) of that rectangle are the cart, not obstacles.

def test_leg_return_just_outside_the_cart_rectangle_does_not_block(rig):
    rig.scan.ranges[180] = .62 + .009               # 9 mm past the edge, like the leg seen at -0.629
    rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.events == [('drive', 1.8)]


def test_obstacle_just_past_the_body_margin_blocks(rig):
    rig.scan.ranges[180] = .62 + .04                # 4 cm past the edge: beyond the 3 cm margin
    with pytest.raises(rig.m.Failed, match='leading sweep'):
        rig.node._straight(1.8, 0., rig.handle, True)


def test_blocking_point_is_reported_in_the_cart_frame(rig):
    rig.scan.ranges[180] = .80
    with pytest.raises(rig.m.Failed) as error:
        rig.node._straight(1.8, 0., rig.handle, True)
    assert 'cart-frame (+0.800, +0.000) m' in str(error.value)
    assert 'travelled 0.000 m' in str(error.value)


def test_body_margin_does_not_shorten_the_end_of_the_sweep(rig):
    # Same reach as without the margin: motion 0.3 m + 0.05 m end margin from the true edge.
    rig.scan.ranges[180] = .62 + .34                # 0.34 m past the edge: inside 0.30 + 0.05
    with pytest.raises(rig.m.Failed, match='leading sweep'):
        rig.node._straight(.3, 0., rig.handle, True)
    rig.scan.ranges[180] = .62 + .36                # 0.36 m past the edge: outside
    rig.node._straight(.3, 0., rig.handle, True)


@pytest.mark.parametrize('margin', [-.01, .06, math.nan])
def test_invalid_body_margin_is_rejected(rig, margin):
    rig.node.straight_cart_body_margin = margin
    with pytest.raises(rig.m.Failed, match='safety parameters'):
        rig.node._straight(.3, 0., rig.handle, True)


# --- stop short of an obstacle beyond the target (place_cart's advance) ---------------------------

def _creep(rig, step=.01):
    """Once the goal is sent the robot advances `step` per guard cycle and the obstacle (scan[180])
    gets closer by the same amount, as it does in the robot frame."""
    def motion_pose():
        if rig.events:
            rig.pose[0] += step
            rig.scan.ranges[180] -= step
            rig.node._scan = rig.scan, rig.clock.now
        return tuple(rig.pose)
    rig.node._motion_pose = motion_pose
    rig.state.after_send = lambda: None
    rig.state.complete_after_send = False


def test_advance_stops_short_of_a_leg_just_beyond_the_target(rig):
    _creep(rig)
    rig.scan.ranges[180] = .62 + .9 + .02           # 2 cm past where the cart's edge would end
    rig.node._straight(.9, 0., rig.handle, True, stop_short_within=.10)
    assert .80 < rig.pose[0] < .90                  # stopped short of the 0.9 m target
    rig.action_handles[0].cancel_goal_async.assert_called_once()
    assert not rig.node._motion_fault
    assert .05 <= rig.scan.ranges[180] - .62 <= .09  # ~5 cm kept from the leg (plus braking slack)


def test_same_leg_fails_without_stop_short(rig):
    _creep(rig)
    rig.scan.ranges[180] = .62 + .9 + .02
    with pytest.raises(rig.m.Failed, match='leading sweep'):
        rig.node._straight(.9, 0., rig.handle, True)


def test_stop_short_refuses_when_the_shortfall_is_too_big(rig):
    _creep(rig)
    rig.scan.ranges[180] = .62 + .5                 # the leg is 0.5 m ahead of the edge, target 0.9 m
    with pytest.raises(rig.m.Failed, match='leading sweep'):
        rig.node._straight(.9, 0., rig.handle, True, stop_short_within=.10)
    assert rig.events == []                         # refused before moving


def test_stop_short_refuses_an_obstacle_already_inside_the_margin(rig):
    _creep(rig)
    rig.scan.ranges[180] = .62 + .04                # 4 cm from the edge: less than the 5 cm margin
    with pytest.raises(rig.m.Failed, match='leading sweep'):
        rig.node._straight(.9, 0., rig.handle, True, stop_short_within=.10)
    assert rig.events == []


def test_stop_short_only_applies_to_forward_segments(rig):
    with pytest.raises(rig.m.Failed, match='stop-short'):
        rig.node._straight(-.9, 0., rig.handle, True, stop_short_within=.10)


# --- an occupied sweep stops the motion, holds up to straight_block_wait, then resumes ----------------

def _person(rig, blocked):
    """`blocked(now)`: a person stands 0.3 m ahead of the cart's edge. The scan stays fresh."""
    base = rig.node._motion_pose

    def motion_pose():
        rig.node._scan = rig.scan, rig.clock.now
        rig.scan.ranges[180] = .62 + .3 if blocked(rig.clock.now) else math.inf
        return base()
    rig.node._motion_pose = motion_pose


def test_a_person_walking_by_pauses_the_segment_and_it_resumes(rig):
    rig.node.straight_block_wait = 20.
    seen = {}

    def step_in():
        rig.pose[0] += .5                       # 0.5 m done when the person steps in
        seen['t'] = rig.clock.now
        rig.state.after_send = None             # the resumed goal runs to the end
    rig.state.after_send, rig.state.complete_after_send = step_in, False
    _person(rig, lambda now: 't' in seen and now < seen['t'] + 3.)      # gone after 3 s
    rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.events == [('drive', 1.8), ('drive', pytest.approx(1.3))]    # then what was left
    assert rig.pose[0] == pytest.approx(1.8)
    assert 4. <= rig.clock.now - seen['t'] < 10.                            # 3 s away + 1 s clear
    rig.action_handles[0].cancel_goal_async.assert_called_once()           # stopped at once
    rig.action_handles[1].cancel_goal_async.assert_not_called()
    assert rig.released == [True] and not rig.node._motion_fault


def test_a_person_who_stays_fails_the_segment_after_the_hold(rig):
    rig.node.straight_block_wait = 20.
    seen = {}
    rig.state.after_send = lambda: seen.setdefault('t', rig.clock.now)
    rig.state.complete_after_send = False
    _person(rig, lambda now: 't' in seen)
    with pytest.raises(rig.m.Failed, match='still blocked after 20 s'):
        rig.node._straight(1.8, 0., rig.handle, True)
    assert 20. <= rig.clock.now - seen['t'] < 22.
    rig.action_handles[0].cancel_goal_async.assert_called_once()
    assert rig.released == [True] and not rig.node._motion_fault


def test_the_path_must_stay_clear_for_the_whole_hold_time(rig):
    rig.node.straight_block_wait = 20.
    seen = {}

    def step_in():
        seen['t'] = rig.clock.now
        rig.state.after_send = None
    rig.state.after_send, rig.state.complete_after_send = step_in, False
    # there for 2 s, gone 0.5 s (less than the 1 s hold: not enough), back until 4 s, gone for good
    _person(rig, lambda now: 't' in seen and (now - seen['t'] < 2. or 2.5 <= now - seen['t'] < 4.))
    rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.clock.now - seen['t'] >= 5.
    assert len(rig.events) == 2


def test_cancel_is_honoured_during_the_hold(rig):
    rig.node.straight_block_wait = 20.
    seen = {}
    rig.state.after_send = lambda: seen.setdefault('t', rig.clock.now)
    rig.state.complete_after_send = False
    _person(rig, lambda now: 't' in seen)
    waiting = rig.node._motion_pose

    def with_cancel():
        if 't' in seen and rig.clock.now - seen['t'] >= 2.:
            rig.handle.is_cancel_requested = True
        return waiting()
    rig.node._motion_pose = with_cancel
    with pytest.raises(rig.m.Canceled):
        rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.clock.now - seen['t'] < 5.


def test_blocked_before_starting_waits_then_sends_the_whole_segment(rig):
    rig.node.straight_block_wait = 20.
    t0 = rig.clock.now
    _person(rig, lambda now: now - t0 < 2.)
    rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.events == [('drive', 1.8)]
    assert rig.clock.now - t0 >= 3.


@pytest.mark.parametrize('attr,value', [('straight_block_wait', 61.), ('straight_block_wait', -1.),
                                        ('straight_block_wait', math.nan), ('straight_clear_hold', .1),
                                        ('straight_clear_hold', 6.)])
def test_invalid_hold_parameters_are_rejected(rig, attr, value):
    setattr(rig.node, attr, value)
    with pytest.raises(rig.m.Failed, match='safety parameters'):
        rig.node._straight(.3, 0., rig.handle, True)


def _set_measured_cart_pose(rig, pose):
    original = rig.node.straight_footprint.call_async.side_effect

    def measured(request):
        response = original(request).result()
        data = json.loads(response.message)
        data['cart_pose'] = list(pose)
        response.message = json.dumps(data)
        return completed(response)

    rig.node.straight_footprint.call_async.side_effect = measured


def test_initial_two_degree_cart_offset_without_drift_passes(rig):
    _set_measured_cart_pose(rig, (0., 0., math.radians(2.)))
    rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.events == [('drive', 1.8)]


def _drive_along_heading(rig, heading, forward, sideways=0.):
    """Robot starts with `heading` and moves `forward` along it (plus `sideways` in its frame)."""
    def inject():
        c, s = math.cos(heading), math.sin(heading)
        rig.pose[0] = forward * c - sideways * s
        rig.pose[1] = forward * s + sideways * c
    rig.pose[2] = heading
    rig.state.after_send = inject


@pytest.mark.parametrize('heading_deg', [1.1, 2.0, -2.0])
def test_constant_start_heading_error_is_not_lateral_drift(rig, heading_deg):
    # Measured on the real robot: 1.1 deg start error over 1.8 m gave 36 mm against the cart axis
    # with 0.000 deg of yaw drift. The robot drove perfectly straight; nothing drifted.
    _drive_along_heading(rig, math.radians(heading_deg), -1.8)
    rig.node._straight(-1.8, 0., rig.handle, True)
    logs = [c.args[0] for c in rig.logger.info.call_args_list]
    assert f'initial_yaw={heading_deg:+.3f}°' in next(m for m in logs if m.startswith('straight-motion start'))
    maxima = next(m for m in logs if m.startswith('straight-motion maxima'))
    assert 'lateral=0.000m' in maxima and 'yaw=0.000°' in maxima


def test_real_lateral_drift_is_still_caught_with_a_start_heading_error(rig):
    _drive_along_heading(rig, math.radians(2.), 1.2, sideways=.04)
    with pytest.raises(rig.m.Failed) as caught:
        rig.node._straight(1.8, 0., rig.handle, True)
    assert 'exceeded=lateral' in str(caught.value)


def _back_out_between_legs(rig, close_ray, released_cart, close_range=.5):
    """Robot backs out 1.8 m of a cart left 1.5 m behind its start, ending 0.3 m past its centre.

    There, the rear-right leg is 0.556 m from the laser at -120.3 deg: below the real laser's
    range_min (0.55 m, measured on the robot), so the return is reported as closer than that.
    """
    rig.node.lifted = False
    rig.scan.range_min = .55

    def inject():
        rig.pose[0] = -1.8
        rig.scan.ranges[close_ray] = close_range

    rig.state.after_send = inject
    rig.node._straight(-1.8, 0., rig.handle, False, released_cart=released_cart)


def test_returns_of_the_legs_of_the_cart_just_put_down_are_not_an_invalid_scan(rig):
    _back_out_between_legs(rig, close_ray=60, released_cart=(-1.5, 0., 0.))   # -120 deg: that leg
    assert rig.events == [('backup', -1.8)]
    logs = [c.args[0] for c in rig.logger.info.call_args_list]
    assert any('accepted as legs of the cart just put down' in m for m in logs)


def test_close_return_without_a_released_cart_is_still_an_invalid_scan(rig):
    with pytest.raises(rig.m.Failed) as caught:
        _back_out_between_legs(rig, close_ray=60, released_cart=None)
    assert 'invalid scan' in str(caught.value)


def test_close_return_not_pointing_at_a_leg_is_still_an_invalid_scan(rig):
    with pytest.raises(rig.m.Failed) as caught:
        _back_out_between_legs(rig, close_ray=150, released_cart=(-1.5, 0., 0.))   # -30 deg: no leg
    assert 'invalid scan' in str(caught.value)


def test_nan_is_never_explained_by_the_legs(rig):
    with pytest.raises(rig.m.Failed) as caught:
        _back_out_between_legs(rig, close_ray=60, released_cart=(-1.5, 0., 0.), close_range=math.nan)
    assert 'invalid scan' in str(caught.value)


def test_a_leg_return_explains_only_when_the_leg_is_where_the_ray_points(rig):
    node, scan = rig.node, rig.scan
    scan.range_min = .55
    cart_in_base = (.3, 0., 0.)        # rear-right leg at (-0.28, -0.48): bearing -120.3 deg
    scan.ranges[60] = .5
    assert node._close_returns_are_cart_legs(scan, (0., 0., 0.), cart_in_base)
    scan.ranges[60] = math.inf
    scan.ranges[61] = .5               # one degree off: 1 cm from the leg centre, still a leg
    assert node._close_returns_are_cart_legs(scan, (0., 0., 0.), cart_in_base)
    scan.ranges[61] = math.inf
    scan.ranges[90] = .5               # -90 deg: nothing there
    assert not node._close_returns_are_cart_legs(scan, (0., 0., 0.), cart_in_base)
    scan.ranges[60] = .5               # a leg return does not excuse the other one
    assert not node._close_returns_are_cart_legs(scan, (0., 0., 0.), cart_in_base)


@pytest.mark.parametrize('carried_yaw_deg', [0., .75, -2.5])
def test_place_accepts_a_cart_left_within_the_pick_tolerance(rig, carried_yaw_deg):
    _set_measured_cart_pose(rig, (0., 0., math.radians(carried_yaw_deg)))
    rig.node._place_target((5., 0., 0.), rig.handle)


def test_place_rejects_a_cart_beyond_the_carried_yaw_tolerance(rig):
    _set_measured_cart_pose(rig, (0., 0., math.radians(3.5)))
    with pytest.raises(rig.m.Failed) as caught:
        rig.node._place_target((5., 0., 0.), rig.handle)
    assert 'within 25 cm / 3.0 deg' in str(caught.value) and '3.50 deg' in str(caught.value)


def test_moderate_lateral_and_yaw_drift_passes(rig):
    def inject():
        rig.pose[0] = 1.8
        rig.pose[1] = .02
        rig.pose[2] = math.radians(1.5)

    rig.state.after_send = inject
    rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.events == [('drive', 1.8)]
    logs = [c.args[0] for c in rig.logger.info.call_args_list]
    maxima = [m for m in logs if m.startswith('straight-motion maxima')]
    assert len(maxima) == 1
    for field in ('lateral=0.020m', 'yaw=1.500°', 'envelope=', 'limit=', 'travelled=1.800m'):
        assert field in maxima[0]
    # Measurement lines: start context and a signed drift trace.
    assert sum(m.startswith('straight-motion start') for m in logs) == 1
    assert 'initial_yaw=' in next(m for m in logs if m.startswith('straight-motion start'))
    trace = [m for m in logs if m.startswith('straight-motion trace')]
    assert len(trace) == 1 and '1.8m:+20mm/+1.50°' in trace[0]


@pytest.mark.parametrize('lateral,yaw,exceeded', [
    (.04, 0., 'lateral'),
    (0., math.radians(2.5), 'yaw'),
])
def test_excessive_drift_reports_values(rig, lateral, yaw, exceeded):
    def inject():
        rig.pose[0], rig.pose[1], rig.pose[2] = .6, lateral, yaw

    rig.state.after_send = inject
    with pytest.raises(rig.m.Failed) as caught:
        rig.node._straight(1.8, 0., rig.handle, True)
    message = str(caught.value)
    assert f'exceeded={exceeded}' in message
    for field in ('lateral=', 'yaw=', 'envelope=', 'limit=', 'travelled='):
        assert field in message
    logs = [c.args[0] for c in rig.logger.info.call_args_list]
    assert sum(m.startswith('straight-motion maxima') for m in logs) == 1


@pytest.mark.parametrize('cart_pose', [
    (0., .041, 0.),
    (0., 0., math.radians(3.1)),
])
def test_initial_cart_offset_outside_alignment_tolerance_refuses_before_motion(rig, cart_pose):
    _set_measured_cart_pose(rig, cart_pose)
    with pytest.raises(rig.m.Failed, match='initial cart alignment'):
        rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.events == []


def test_straight_motion_parameter_ceilings_are_usable(rig):
    rig.node.straight_lateral_limit = .05
    rig.node.straight_yaw_limit = math.radians(3.)
    rig.node.straight_envelope = .06
    rig.node._straight(1.8, 0., rig.handle, True)


@pytest.mark.parametrize('name,value', [
    ('straight_lateral_limit', .0501),
    ('straight_yaw_limit', math.radians(3.01)),
    ('straight_envelope', .0601),
])
def test_straight_motion_parameters_above_ceilings_are_rejected(rig, name, value):
    setattr(rig.node, name, value)
    with pytest.raises(rig.m.Failed, match='invalid straight-motion safety parameters'):
        rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.events == []


@pytest.mark.parametrize('failure', ['stale_scan', 'blind_scan', 'invalid_scan', 'no_tf', 'fast'])
def test_guard_refuses_to_start_without_proof(rig, failure):
    if failure == 'stale_scan':
        rig.node._scan = rig.scan, 0.
    elif failure == 'blind_scan':
        rig.scan.ranges = rig.scan.ranges[:180]
    elif failure == 'invalid_scan':
        rig.scan.ranges[0] = math.nan
    elif failure == 'no_tf':
        rig.node.tf_buffer.lookup_transform.side_effect = rig.m.TransformException('missing')
    else:
        rig.node.approach_speed = .3
    with pytest.raises(rig.m.Failed):
        rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.events == []


def test_place_with_mock_action_clients(rig):
    node = rig.node
    node._align_place = lambda *args: rig.events.append(('align', 0.))
    node.lift_down = object()

    def lower(*args):
        rig.events.append(('lift_down', 0.))
        node.lifted = False
    node._call = lower
    pose = NS(position=NS(x=1.8, y=0.), orientation=NS(x=0., y=0., z=0., w=1.))
    req = NS(use_dock_id=False, dock_pose=NS(header=NS(frame_id='map'), pose=pose))
    node._do_place(req, rig.handle, Mock())
    assert rig.events == [('align', 0.), ('drive', 1.8), ('lift_down', 0.), ('backup', -1.8)]
    assert rig.pose == [0., 0., 0.]
    assert rig.released == [True, True, True]


def test_pick_finishes_with_lift_and_staging_backout(rig):
    node = rig.node
    node.lifted = False
    node._settle_localization = Mock()
    node._detect_cart = lambda cart: cart
    node.get_logger = lambda: NS(info=Mock(), error=Mock(), debug=Mock())
    node.pre_entry_distance, node.lateral_tolerance, node.yaw_tolerance = 1.3, .04, .06
    node.pick_lateral_tolerance = .02
    node.max_corrections = 3
    rig.pose[0] = -1.
    node._drive = lambda distance, *args: (rig.events.append(('entry', distance)),
                                          rig.pose.__setitem__(0, rig.pose[0] + distance))
    node.lift_up = object()

    def lift(*args):
        rig.events.append(('lift_up', 0.))
        node.lifted = True
    node._call = lift
    pose = NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))
    req = NS(use_dock_id=False, navigate_to_staging_pose=False,
             dock_pose=NS(header=NS(frame_id='map'), pose=pose))
    node._do_pick(req, rig.handle, Mock())
    assert rig.events == [('entry', 1.), ('lift_up', 0.), ('backup', -1.8)]
    assert rig.pose == [-1.8, 0., 0.]


def test_lost_lease_cancels_motion(rig):
    rig.state.after_send = lambda: None
    rig.state.complete_after_send = False
    original = rig.node.straight_footprint.call_async.side_effect
    count = [0]

    def lease(request):
        count[0] += request.data
        if request.data and count[0] > 1:
            return Future()
        return original(request)
    rig.node.straight_footprint.call_async.side_effect = lease
    # Keep sensor receipt fresh so that this test specifically exercises the lease.
    rig.node.straight_data_timeout = .3
    rig.node._scan = rig.scan, 10.15
    with pytest.raises(rig.m.Failed, match='footprint lease lost'):
        rig.node._straight(1.8, 0., rig.handle, True)
    rig.action_handles[0].cancel_goal_async.assert_called_once()


def test_unconfirmed_cancellation_latches_fault(rig):
    rig.state.complete_after_send = False

    def inject():
        rig.pose[1] = .04
        rig.action_handles[0].cancel_goal_async.side_effect = lambda: completed(NS())
    rig.state.after_send = inject
    with pytest.raises(rig.m.Failed):
        rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.node._motion_fault


def test_unknown_release_never_reports_success(rig):
    original = rig.node.straight_footprint.call_async.side_effect
    rig.node.straight_footprint.call_async.side_effect = (
        lambda request: original(request) if request.data else Future())
    with pytest.raises(rig.m.Failed, match='restoration unconfirmed'):
        rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.node._motion_fault


def test_restoration_requires_costmap_receipt(rig):
    original = rig.node.straight_footprint.call_async.side_effect

    def no_costmap_receipt(request):
        saved = dict(rig.node._footprints)
        response = original(request)
        if not request.data:
            rig.node._footprints = saved
        return response
    rig.node.straight_footprint.call_async.side_effect = no_costmap_receipt
    with pytest.raises(rig.m.Failed, match='costmaps did not restore'):
        rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.node._motion_fault


def test_place_compensates_cart_offset(rig):
    original = rig.node.straight_footprint.call_async.side_effect

    def offset_geometry(request):
        future = original(request)
        response = future.result()
        data = json.loads(response.message)
        data['cart_pose'] = [0., .008, 0.] if rig.node.lifted else [0., 0., 0.]
        response.message = json.dumps(data)
        return completed(response)
    rig.node.straight_footprint.call_async.side_effect = offset_geometry

    def align(target, handle):
        from cart_geometry import staging_pose
        rig.pose[:] = staging_pose(target)
        rig.events.append(('align', tuple(rig.pose)))
    rig.node._align_place = align
    rig.node.lift_down = object()
    actual_drop = []

    def lower(*args):
        actual_drop.append((rig.pose[0], rig.pose[1] + .008))
        rig.node.lifted = False
    rig.node._call = lower
    pose = NS(position=NS(x=1.8, y=0.), orientation=NS(x=0., y=0., z=0., w=1.))
    req = NS(use_dock_id=False, dock_pose=NS(header=NS(frame_id='map'), pose=pose))
    rig.node._do_place(req, rig.handle, Mock())
    assert actual_drop == [(1.8, 0.)]
    assert rig.pose == [0., -.008, 0.]


def test_large_cart_offset_refuses_placement_before_moving(rig):
    original = rig.node.straight_footprint.call_async.side_effect

    def offset_geometry(request):
        response = original(request).result()
        data = json.loads(response.message)
        data['cart_pose'] = [0., .12, 0.]
        response.message = json.dumps(data)
        return completed(response)
    rig.node.straight_footprint.call_async.side_effect = offset_geometry
    with pytest.raises(rig.m.Failed, match='staging compensation limit'):
        rig.node._place_target((1.8, 0., 0.), rig.handle)
    assert rig.events == []


def test_excessive_measured_cart_yaw_refuses_motion(rig):
    original = rig.node.straight_footprint.call_async.side_effect

    def rotated_cart(request):
        response = original(request).result()
        data = json.loads(response.message)
        data['cart_pose'][2] = math.radians(3.1)
        response.message = json.dumps(data)
        return completed(response)
    rig.node.straight_footprint.call_async.side_effect = rotated_cart
    with pytest.raises(rig.m.Failed, match='initial cart alignment'):
        rig.node._straight(1.8, 0., rig.handle, True)
    assert not rig.events


@pytest.mark.parametrize('carrying', [True, False])
def test_reset_fault_confirms_normal_state_from_adapter_and_costmaps(rig, carrying):
    import threading
    node = rig.node
    node.lifted, node._motion_fault = carrying, True
    node._busy, node._lift_lock = threading.Lock(), threading.Lock()
    node._lift_state_seen, node._lift_state_revision = True, 1
    node._pending_goal, node._pending_result = None, None
    response = node._reset_fault(NS(), NS())
    assert response.success
    assert not node._motion_fault
    assert rig.released == [True]


@pytest.mark.parametrize('fault', ['reduced', 'global_missing', 'lift_mismatch', 'lift_unknown',
                                  'lift_changed', 'busy', 'pending_motion', 'canceled_result'])
def test_reset_fault_retains_latch_without_confirmed_normal_state(rig, fault):
    import threading
    node = rig.node
    node._motion_fault = True
    node._busy, node._lift_lock = threading.Lock(), threading.Lock()
    node._lift_state_seen, node._lift_state_revision = True, 1
    node._pending_goal, node._pending_result = None, None
    original = node.straight_footprint.call_async.side_effect

    def abnormal(request):
        response = original(request).result()
        if fault == 'reduced':
            node._footprints['local_costmap'] = ([.74, .74, 1., 1.], rig.clock.now)
        elif fault == 'global_missing':
            node._footprints.pop('global_costmap', None)
        elif fault == 'lift_mismatch':
            data = json.loads(response.message)
            data['carrying'] = False
            response.message = json.dumps(data)
        elif fault == 'lift_changed':
            node._lift_state_revision += 1
        return completed(response)
    node.straight_footprint.call_async.side_effect = abnormal
    if fault == 'lift_unknown':
        node._lift_state_seen = False
    elif fault == 'busy':
        node._busy.acquire()
    elif fault == 'pending_motion':
        node._pending_goal = Future()
    elif fault == 'canceled_result':
        node._pending_goal = completed(NS(accepted=True))
        node._pending_result = Future()
        node._pending_result.cancel()
    response = node._reset_fault(NS(), NS())
    assert not response.success
    assert response.message
    assert node._motion_fault
    if fault == 'busy':
        node._busy.release()


def test_place_accepts_eight_cm_staging_compensation(rig):
    original = rig.node.straight_footprint.call_async.side_effect

    def measured(request):
        response = original(request).result()
        data = json.loads(response.message)
        data['cart_pose'] = [0., .08, 0.]
        response.message = json.dumps(data)
        return completed(response)
    rig.node.straight_footprint.call_async.side_effect = measured
    assert rig.node._place_target((1.8, 0., 0.), rig.handle) == (1.8, -.08, 0.)
