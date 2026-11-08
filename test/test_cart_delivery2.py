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
    node.lateral_tolerance, node.yaw_tolerance = .04, math.radians(3.)
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
    node.straight_lateral_limit = .01
    node._robot_pose = lambda: (offset[0], offset[1], 0.)
    node._spin_to, node._drive = Mock(), Mock()
    node._align_place((1.8, 0., 0.), NS())
    node._drive.assert_not_called()
    assert node.straight_lateral_limit == .01


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
