import math
import threading
from concurrent.futures import Future
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest


@pytest.mark.parametrize('error,accepted', [(.10, True), (.20, False)])
def test_placement_map_tolerance(handler_module, error, accepted):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node.place_tolerance_xy, node.place_tolerance_yaw = .15, math.radians(3.)
    node._placement_geometry = (0., 0., 0.)
    node._robot_pose = lambda: (error, 0., 0.)
    if accepted:
        node._verify_placement((0., 0., 0.))
    else:
        with pytest.raises(m.Failed, match='outside destination tolerance'):
            node._verify_placement((0., 0., 0.))


def test_pick_accepts_existing_alignment_tolerances(handler_module):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node.lifted, node.global_frame = False, 'map'
    node.staging_distance, node.pre_entry_distance = 1.8, 1.3
    node.lateral_tolerance, node.pick_lateral_tolerance = .04, .04
    node.yaw_tolerance = math.radians(3.)
    node.max_corrections = 0
    # Predicted lateral offset at the centre is 3 cm, heading error is 2 deg.
    node._robot_pose = lambda: (-1., .03 - math.sin(math.radians(2.)), math.radians(2.))
    node._detect_cart = lambda cart: cart
    node._settle_localization = Mock()
    node.get_logger = lambda: NS(info=Mock())
    node._drive, node._call, node._straight, node._confirm_lift = Mock(), Mock(), Mock(), Mock()
    node.lift_up = object()
    pose = NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))
    request = NS(use_dock_id=False, navigate_to_staging_pose=False,
                 dock_pose=NS(header=NS(frame_id='map'), pose=pose))
    handle = NS(is_cancel_requested=False)
    node._do_pick(request, handle, Mock())
    node._drive.assert_called_once_with(1., handle, 'drive under the cart')
    assert node._straight.call_args.args[0] == -1.8


def test_pick_does_not_enter_with_the_lateral_offset_that_collided_in_the_simulation(handler_module):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node.lifted, node.global_frame = False, 'map'
    node.staging_distance, node.pre_entry_distance = 1.8, 1.3
    node.lateral_tolerance, node.pick_lateral_tolerance = .04, .02
    node.yaw_tolerance = math.radians(3.)
    node.max_corrections = 0
    # P2: 2.8 cm predicted at the cart centre. Nav2 aborted the drive under the cart with
    # COLLISION_AHEAD (723): only 4-9 cm per side remain between the legs and the padded footprint.
    node._robot_pose = lambda: (-1., -.028, 0.)
    node._detect_cart = lambda cart: cart
    node._settle_localization = Mock()
    node.get_logger = lambda: NS(info=Mock())
    node._drive, node._call, node._straight, node._confirm_lift = Mock(), Mock(), Mock(), Mock()
    pose = NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))
    request = NS(use_dock_id=False, navigate_to_staging_pose=False,
                 dock_pose=NS(header=NS(frame_id='map'), pose=pose))
    with pytest.raises(m.Failed, match='could not align'):
        node._do_pick(request, NS(is_cancel_requested=False), Mock())
    node._drive.assert_not_called()


def test_pick_demands_the_strict_lateral_only_near_the_cart(handler_module):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node.lifted, node.global_frame = False, 'map'
    node.staging_distance, node.pre_entry_distance = 1.8, 1.3
    node.lateral_tolerance, node.pick_lateral_tolerance = .04, .02
    node.yaw_tolerance = math.radians(3.)
    node.max_corrections = 0
    pose = [-1.95, -.03, 0.]          # far: 3 cm is fine for the coarse check (+-0.7 deg spin noise)

    def drive(distance, handle, what):
        if what == 'approach to pre-entry':
            pose[0] += distance
            pose[1] = -.012           # near: 1.2 cm passes the strict check
        calls.append((round(distance, 2), what))
    calls = []
    node._robot_pose = lambda: tuple(pose)
    node._detect_cart = lambda cart: cart
    node._settle_localization = Mock()
    node.get_logger = lambda: NS(info=Mock())
    node._drive, node._call, node._straight, node._confirm_lift = drive, Mock(), Mock(), Mock()
    node.lift_up = object()
    cart_pose = NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))
    request = NS(use_dock_id=False, navigate_to_staging_pose=False,
                 dock_pose=NS(header=NS(frame_id='map'), pose=cart_pose))
    node._do_pick(request, NS(is_cancel_requested=False), Mock())
    assert calls == [(.65, 'approach to pre-entry'), (1.3, 'drive under the cart')]


def _cart_in_robot_frame(along, lateral, yaw_err):
    """The detection (cart pose in the robot frame) of a robot standing at these coordinates in
    the cart frame."""
    theta = -yaw_err
    return (-(along * math.cos(theta) - lateral * math.sin(theta)),
            -(along * math.sin(theta) + lateral * math.cos(theta)), theta)


def test_cart_relative_pose_from_the_scan_matches_the_pose_based_one(handler_module):
    m = handler_module
    node = object.__new__(m.CartHandler)
    cart = (3., -2., math.radians(164.))
    robot = (3. + 1.9 * math.cos(cart[2]) - .03 * math.sin(cart[2]),
             -2. + 1.9 * math.sin(cart[2]) + .03 * math.cos(cart[2]), cart[2] + math.radians(1.5))
    node._robot_pose = lambda: robot
    from_poses = node._cart_relative(cart)
    # The same geometry as the scan reports it (cart in the robot frame): no AMCL involved.
    node._cart_in_base = _cart_in_robot_frame(*from_poses)
    node._robot_pose = Mock(side_effect=AssertionError('AMCL must not be read'))
    assert node._cart_relative(cart) == pytest.approx(from_poses)
    assert node._cart_in_base is None          # consumed: one detection, one use


def test_pick_correction_is_made_relative_to_the_cart_axis(handler_module):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node.lifted, node.global_frame = False, 'map'
    node.staging_distance, node.pre_entry_distance = 1.8, 1.3
    node.lateral_tolerance, node.pick_lateral_tolerance = .04, .02
    node.yaw_tolerance = math.radians(3.)
    node.max_corrections = 3
    node.correction_lookahead, node.min_correction_clearance = .5, 1.3
    seen = iter([(-2.0, .05, 0.),        # far, 5 cm off the axis: correct
                 (-2.0, .004, 0.),       # far, fine: approach
                 (-1.3, .004, 0.)])      # near, fine: drive under the cart
    calls = []

    def detect(cart):
        node._cart_in_base = _cart_in_robot_frame(*next(seen))
        return cart
    node._detect_cart = detect
    node._robot_pose = Mock(side_effect=AssertionError('AMCL must not be read'))
    node._settle_localization = Mock()
    node._spin_by = lambda delta, handle: calls.append(('spin', round(delta, 4)))
    node._drive = lambda distance, handle, what: calls.append(('drive', round(distance, 4)))
    node._call, node._straight, node._confirm_lift = Mock(), Mock(), Mock()
    node.lift_up = object()
    node.get_logger = lambda: NS(info=Mock())
    pose = NS(position=NS(x=0., y=0.), orientation=NS(x=0., y=0., z=0., w=1.))
    request = NS(use_dock_id=False, navigate_to_staging_pose=False,
                 dock_pose=NS(header=NS(frame_id='map'), pose=pose))
    node._do_pick(request, NS(is_cancel_requested=False), Mock())
    bearing = math.atan2(-.05, .5)
    assert calls == [('spin', round(bearing, 4)), ('drive', round(math.hypot(.5, .05), 4)),
                     ('spin', round(-bearing, 4)),          # back onto the axis
                     ('drive', .7), ('drive', 1.3)]


@pytest.mark.parametrize('normal', [True, False])
def test_reset_fault_only_with_normal_footprints(handler_module, normal):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node._busy, node._lift_lock = threading.Lock(), threading.Lock()
    node._lift_state_seen, node._lift_state_revision = True, 1
    node.lifted, node._motion_fault = True, True
    node._pending_goal, node._pending_result = None, None
    node.straight_footprint = NS(wait_for_service=Mock(return_value=True))
    node.server_timeout = 1.
    if normal:
        node._restore_footprint = Mock()
    else:
        node._restore_footprint = Mock(side_effect=m.Failed(m.DOCK.FAILED_TO_CONTROL,
                                                          'costmaps did not restore the full footprint'))
    response = node._reset_fault(NS(), NS())
    assert response.success == normal
    assert node._motion_fault == (not normal)
    assert response.message
    node._restore_footprint.assert_called_once()


@pytest.mark.parametrize('offset', [(.04, .02), (-.04, -.02)])
def test_staging_map_tolerances_do_not_change_odom_drift(handler_module, offset):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node.staging_distance = 1.8
    node.place_staging_along_tolerance, node.place_staging_lateral_tolerance = .05, .03
    node.straight_lateral_limit = .03
    node._robot_pose = lambda: (offset[0], offset[1], 0.)
    node._spin_to, node._drive, node._settle_localization = Mock(), Mock(), Mock()
    node.get_logger = lambda: NS(info=Mock())
    node._align_place((1.8, 0., 0.), NS())
    node._drive.assert_not_called()
    assert node.straight_lateral_limit == .03


def test_staging_alignment_settles_localization_before_measuring_and_after_the_spin(handler_module):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node.staging_distance = 1.8
    node.place_staging_along_tolerance, node.place_staging_lateral_tolerance = .05, .03
    order = []
    node._settle_localization = Mock(side_effect=lambda handle: order.append('settle'))
    node._spin_to = Mock(side_effect=lambda yaw, handle: order.append('spin'))
    node._drive = Mock(side_effect=lambda *a: order.append('drive'))
    node._robot_pose = lambda: (order.append('measure'), (.01, .01, 0.))[1]
    node.get_logger = lambda: NS(info=Mock())
    node._align_place((1.8, 0., 0.), NS())
    # Settle first (AMCL lags after Nav2 stops), measure, final spin, settle again (the spin moves
    # AMCL's estimate), and only then the final measurement the staging check uses.
    assert order == ['settle', 'measure', 'spin', 'settle', 'measure']


def test_failed_staging_alignment_reports_the_measured_error(handler_module):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node.staging_distance = 1.8
    node.place_staging_along_tolerance, node.place_staging_lateral_tolerance = .05, .03
    # Cart at (1.8, 0) -> staging at (0, 0). The robot never gets closer: 8 cm along, 6 cm lateral.
    node._robot_pose = lambda: (.08, -.06, 0.)
    node._spin_to, node._drive, node._settle_localization = Mock(), Mock(), Mock()
    logger = NS(info=Mock())
    node.get_logger = lambda: logger
    with pytest.raises(m.Failed) as caught:
        node._align_place((1.8, 0., 0.), NS())
    message = str(caught.value)
    assert 'left the staging position' in message
    assert 'along' in message and 'lateral' in message and 'limits 0.050 m along / 0.030 m lateral' in message
    assert logger.info.call_count == m.CartHandler.PLACE_ALIGN_ROUNDS


def test_staging_alignment_corrects_again_when_the_final_spin_shifts_the_pose(handler_module):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node.staging_distance = 1.8
    node.place_staging_along_tolerance, node.place_staging_lateral_tolerance = .05, .03
    # Cart at (1.8, 0) -> staging (0, 0). Measured OK facing the staging point, but the final
    # spin leaves the robot 6 cm off laterally (the P2 failure); the second round corrects it.
    poses = iter([(.0, -.01, 0.), (.0, .06, 0.), (.0, .0, 0.), (.0, .0, 0.)])
    node._robot_pose = lambda: next(poses)
    node._settle_localization, node._drive = Mock(), Mock()
    node._spin_to = Mock()
    node.get_logger = lambda: NS(info=Mock())
    node._align_place((1.8, 0., 0.), NS())
    assert node._drive.call_count == 1
    # spin to the cart, spin toward staging, then spin to the cart again; accepted only facing it
    assert node._spin_to.call_count == 3


def _axial_node(m, poses):
    node = object.__new__(m.CartHandler)
    node.staging_distance = 1.8
    node.place_staging_along_tolerance, node.place_staging_lateral_tolerance = .08, .05
    node.yaw_tolerance = math.radians(3.)
    poses = iter(poses)
    node._robot_pose = lambda: next(poses)
    node._settle_localization, node._drive, node._spin_to = Mock(), Mock(), Mock()
    node._straight = Mock()
    node.get_logger = lambda: NS(info=Mock())
    return node


@pytest.mark.parametrize('along', [.12, -.12])
def test_along_error_facing_the_axis_is_corrected_without_turning_around(handler_module, along):
    m = handler_module
    # Cart at (1.8, 0) -> staging (0, 0). Facing the cart, on the line, 12 cm off along it: one short
    # straight move (back if ahead of the staging, forward if short of it), no spin, no Nav2 drive.
    node = _axial_node(m, [(along, .01, 0.), (.0, .01, 0.)])
    node._align_place((1.8, 0., 0.), NS())
    node._straight.assert_called_once()
    assert node._straight.call_args.args[0] == pytest.approx(-along)
    assert node._straight.call_args.args[3] is True            # carrying: the cart footprint is leased
    node._spin_to.assert_not_called()
    node._drive.assert_not_called()


def test_axial_correction_needs_to_be_facing_the_axis(handler_module):
    m = handler_module
    # 10 deg off the axis: a straight move would not follow the line, so the usual spin + drive.
    node = _axial_node(m, [(.12, .0, math.radians(10.)), (.0, .0, 0.), (.0, .0, 0.)])
    node._align_place((1.8, 0., 0.), NS())
    node._straight.assert_not_called()
    assert node._drive.call_count == 1 and node._spin_to.call_count >= 1


def test_lateral_error_still_needs_the_spin_and_drive(handler_module):
    m = handler_module
    node = _axial_node(m, [(.0, .09, 0.), (.0, .0, 0.), (.0, .0, 0.)])
    node._align_place((1.8, 0., 0.), NS())
    node._straight.assert_not_called()
    assert node._drive.call_count == 1


def test_error_inside_the_new_tolerances_is_accepted_without_moving(handler_module):
    m = handler_module
    # 7 cm along / 4 cm lateral: inside the AMCL-noise-sized tolerances (used to chase it with spins).
    node = _axial_node(m, [(.07, .04, 0.), (.07, .04, 0.)])
    node._align_place((1.8, 0., 0.), NS())
    node._straight.assert_not_called()
    node._drive.assert_not_called()


@pytest.mark.parametrize('error,accepted', [(math.radians(2.), True), (math.radians(4.), False)])
def test_placement_yaw_tolerance(handler_module, error, accepted):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node.place_tolerance_xy, node.place_tolerance_yaw = .15, math.radians(3.)
    node._placement_geometry = (0., 0., 0.)
    node._robot_pose = lambda: (0., 0., error)
    if accepted:
        node._verify_placement((0., 0., 0.))
    else:
        with pytest.raises(m.Failed):
            node._verify_placement((0., 0., 0.))


@pytest.mark.parametrize('pending', ['acceptance', 'result'])
def test_reset_refuses_pending_regular_nav2_action(handler_module, pending):
    m = handler_module
    node = object.__new__(m.CartHandler)
    node._busy, node._lift_lock = threading.Lock(), threading.Lock()
    node._lift_state_seen, node._lift_state_revision = True, 1
    node.lifted, node._motion_fault = True, True
    node._pending_goal, node._pending_result = None, None
    node.server_timeout = 1.
    node.straight_footprint = NS(wait_for_service=Mock(return_value=True))
    node._restore_footprint = Mock()
    send, terminal = Future(), Future()
    action_handle = NS(accepted=True, get_result_async=Mock(return_value=terminal),
                       cancel_goal_async=Mock())
    if pending == 'result':
        send.set_result(action_handle)
        node._wait = Mock(side_effect=[True, False])
    else:
        node._wait = Mock(return_value=False)
    client = NS(wait_for_server=Mock(return_value=True), send_goal_async=Mock(return_value=send))
    with pytest.raises(m.Failed):
        node._run(client, NS(), NS(), 1., m.DOCK.FAILED_TO_CONTROL, 'alignment')
    response = node._reset_fault(NS(), NS())
    assert not response.success, response.message
    assert node._motion_fault
    node._restore_footprint.assert_not_called()
    # Once the same pending action terminates, the normal-state handshake can clear.
    if not send.done():
        send.set_result(action_handle)
    terminal.set_result(NS(status=5))
    response = node._reset_fault(NS(), NS())
    assert response.success
    assert not node._motion_fault
