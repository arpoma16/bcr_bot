import math
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest


def test_place_sequence(handler_module):
    module = handler_module
    node = object.__new__(module.CartHandler)
    node.lifted = True
    node.global_frame = 'map'
    node.staging_distance = 1.8
    node.place_stop_short_limit = .10
    node.lift_down = object()
    calls, released = [], []
    node._align_place = Mock(side_effect=lambda *args: calls.append('align'))
    node._straight = Mock(side_effect=lambda distance, *args, **kw: (
        calls.append(distance), released.append(kw.get('released_cart'))))
    node._call = Mock(side_effect=lambda *args: calls.append('lower'))
    node._confirm_lift = Mock()
    node._place_target = lambda cart, handle: cart
    node._placement_geometry = (0.01, -0.02, 0.03)     # cart pose in the robot frame (set by _place_target)
    node._verify_placement = Mock()
    pose = NS(position=NS(x=1.8, y=0.0), orientation=NS(x=0., y=0., z=0., w=1.))
    req = NS(use_dock_id=False, dock_pose=NS(header=NS(frame_id='map'), pose=pose))
    node._do_place(req, NS(is_cancel_requested=False), Mock())
    assert calls == ['align', 1.8, 'lower', -1.8]
    # Only the back-out, made without the cart, is told where the cart it leaves stands.
    assert released == [None, (0.01, -0.02, 0.03)]
    node._confirm_lift.assert_called_once()


def test_place_checks_the_carried_cart_against_its_destination_axis(handler_module):
    module = handler_module
    node = object.__new__(module.CartHandler)
    node.lifted = True
    node.global_frame = 'map'
    node.staging_distance = 1.8
    node.place_stop_short_limit = .10
    node.lift_down = object()
    axes = []
    node._align_place = Mock()
    node._straight = Mock(side_effect=lambda distance, axis, *args, **kw: axes.append(axis))
    node._call, node._confirm_lift, node._verify_placement = Mock(), Mock(), Mock()
    # Robot target yaw is the cart yaw minus the yaw the cart was lifted with (here 2 deg).
    node._place_target = lambda cart, handle: (cart[0], cart[1], cart[2] - math.radians(2.))
    node._placement_geometry = (0., 0., math.radians(2.))
    q = (0., 0., math.sin(math.radians(5.)), math.cos(math.radians(5.)))      # destination yaw 10 deg
    pose = NS(position=NS(x=1.8, y=0.0), orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))
    req = NS(use_dock_id=False, dock_pose=NS(header=NS(frame_id='map'), pose=pose))
    node._do_place(req, NS(is_cancel_requested=False), Mock())
    # With the carried cart: the destination axis. Without it (back-out): the robot's own target.
    assert axes == [pytest.approx(math.radians(10.)), pytest.approx(math.radians(8.))]


def test_place_requires_cart(handler_module):
    module = handler_module
    node = object.__new__(module.CartHandler)
    node.lifted = False
    import pytest
    with pytest.raises(module.Failed) as error:
        node._do_place(NS(), NS(), Mock())
    assert error.value.code == module.DOCK.DOCK_NOT_VALID


def test_drop_still_lowers_here_and_backs_out(handler_module):
    import threading
    module = handler_module
    node = object.__new__(module.CartHandler)
    node._busy = threading.Lock()
    node.lifted = True
    node.lift_down, node.backup = object(), object()
    node.backout_distance, node.backout_speed = 1.3, .15
    calls = []
    node._call = Mock(side_effect=lambda *args: calls.append('lower'))
    node._run = Mock(side_effect=lambda client, goal, *args: calls.append(goal.target.x))
    node.get_logger = lambda: NS(info=Mock(), warn=Mock(), error=Mock())
    handle = NS(request=NS(max_undocking_time=40.), succeed=Mock())
    result = node._drop(handle)
    assert result.success
    assert calls == ['lower', -1.3]
    assert node._run.call_args.args[3] == 40.
    handle.succeed.assert_called_once()


# --- per-station staging distance (dock_type "staging_distance=<m>") -----------------------------

def _staging_node(module):
    node = object.__new__(module.CartHandler)
    node.staging_distance, node.min_staging_distance = 1.8, 1.08
    node.place_stop_short_limit = .10
    node._request_staging = None
    node.get_logger = lambda: NS(info=Mock(), warn=Mock(), error=Mock())
    return node


def test_staging_defaults_to_the_parameter_without_dock_type(handler_module):
    node = _staging_node(handler_module)
    node._use_request_staging(NS(dock_type=''))
    assert node._staging() == 1.8
    node._use_request_staging(NS())            # a request with no dock_type at all
    assert node._staging() == 1.8


def test_staging_distance_comes_from_the_request(handler_module):
    node = _staging_node(handler_module)
    node._use_request_staging(NS(dock_type='staging_distance=1.300'))
    assert node._staging() == pytest.approx(1.3)


@pytest.mark.parametrize('text', ['staging_distance=nan', 'staging_distance=abc', 'something=1.5',
                                  '1.5', 'staging_distance=inf'])
def test_malformed_dock_type_is_rejected(handler_module, text):
    node = _staging_node(handler_module)
    with pytest.raises(handler_module.Failed) as error:
        node._use_request_staging(NS(dock_type=text))
    assert error.value.code == handler_module.DOCK.DOCK_NOT_VALID


def test_staging_closer_than_the_rear_legs_is_rejected(handler_module):
    node = _staging_node(handler_module)
    with pytest.raises(handler_module.Failed) as error:
        node._use_request_staging(NS(dock_type='staging_distance=0.82'))
    assert error.value.code == handler_module.DOCK.DOCK_NOT_VALID
    assert 'min_staging_distance' in str(error.value)
    node._use_request_staging(NS(dock_type='staging_distance=1.08'))      # the limit itself is fine
    assert node._staging() == pytest.approx(1.08)


def test_place_reverses_the_stations_own_distance(handler_module):
    module = handler_module
    node = _staging_node(module)
    node.lifted = True
    node.global_frame = 'map'
    node.lift_down = object()
    node._use_request_staging(NS(dock_type='staging_distance=1.300'))
    moves = []
    node._align_place = Mock()
    node._straight = Mock(side_effect=lambda distance, *args, **kw: moves.append(distance))
    node._call, node._confirm_lift, node._verify_placement = Mock(), Mock(), Mock()
    node._place_target = lambda cart, handle: cart
    node._placement_geometry = (0., 0., 0.)
    pose = NS(position=NS(x=1.8, y=0.0), orientation=NS(x=0., y=0., z=0., w=1.))
    req = NS(use_dock_id=False, dock_pose=NS(header=NS(frame_id='map'), pose=pose))
    node._do_place(req, NS(is_cancel_requested=False), Mock())
    assert moves == [pytest.approx(1.3), pytest.approx(-1.3)]


def test_request_staging_does_not_leak_into_the_next_request(handler_module):
    import threading
    node = _staging_node(handler_module)
    node._busy = threading.Lock()
    seen = []
    handle = NS(request=NS(dock_type='staging_distance=1.300'), succeed=Mock(), abort=Mock())
    node._dock_action(handle, lambda *args: seen.append(node._staging()), 'pick_cart')
    node._dock_action(NS(request=NS(dock_type=''), succeed=Mock(), abort=Mock()),
                      lambda *args: seen.append(node._staging()), 'pick_cart')
    assert seen == [pytest.approx(1.3), 1.8]
