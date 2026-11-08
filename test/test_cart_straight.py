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
    node.straight_data_timeout, node.straight_footprint_padding = .3, .05
    node.server_timeout, node.staging_distance = 1., 1.8
    node.place_tolerance_xy, node.place_tolerance_yaw = .15, math.radians(3.)
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


def test_moderate_lateral_and_yaw_drift_passes(rig):
    def inject():
        rig.pose[0] = 1.8
        rig.pose[1] = .02
        rig.pose[2] = math.radians(1.5)

    rig.state.after_send = inject
    rig.node._straight(1.8, 0., rig.handle, True)
    assert rig.events == [('drive', 1.8)]
    rig.logger.info.assert_called_once()
    diagnostic = rig.logger.info.call_args.args[0]
    for field in ('lateral=0.020m', 'yaw=1.500°', 'envelope=', 'limit=', 'travelled=1.800m'):
        assert field in diagnostic


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
    rig.logger.info.assert_called_once()


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
