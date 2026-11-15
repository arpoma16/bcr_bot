import json
from types import SimpleNamespace as NS
from unittest.mock import Mock


def test_lease_expires_and_restores_cart_footprint(adapter_module, monkeypatch):
    m = adapter_module
    node = object.__new__(m.CartCarryAdapter)
    clock = NS(now=1.)
    monkeypatch.setattr(m.time, 'monotonic', lambda: clock.now)
    node.carrying, node._cart_measured = True, True
    node.cart_pose = (.01, -.01, 0.)
    node.robot_footprint = m._rectangle(.45, .32)
    node.cart_footprint = m._rectangle(.62, .52)
    node.straight_lease_timeout = .6
    node._straight_until = 0.
    node.footprint_pubs = [Mock(), Mock()]
    response = node._straight_footprint(NS(data=True), NS())
    assert response.success
    assert json.loads(response.message)['cart_pose'] == [.01, -.01, 0.]
    assert node.footprint_pubs[0].publish.call_args.args[0].points[0].x == .45
    clock.now += .7
    node._expire_straight()
    assert node._straight_until == 0.
    assert node.footprint_pubs[0].publish.call_args.args[0].points[0].x == .63
    node._straight_footprint(NS(data=True), NS())
    node._straight_footprint(NS(data=False), NS())
    assert node._straight_until == 0.


def make_node(m, carrying=False, box_mode=False):
    node = object.__new__(m.CartCarryAdapter)
    node.carrying, node.box_mode = carrying, box_mode
    node._cart_measured = False
    node.cart_pose = (0., 0., 0.)
    node.robot_footprint = m._rectangle(.45, .32)
    node.cart_footprint = m._rectangle(.62, .52)
    node.filter_half_x, node.filter_half_y = .67, .57
    node.box_filter_margin = .05
    node._straight_until = 0.
    node.footprint_pubs = [Mock(), Mock()]
    node.scan_pub = Mock()
    node.get_logger = Mock()
    return node


def published_half_x(node):
    return max(p.x for p in node.footprint_pubs[0].publish.call_args.args[0].points)


def test_box_mode_keeps_the_robot_footprint_while_the_platform_is_up(adapter_module):
    box = make_node(adapter_module, carrying=True, box_mode=True)
    box._publish_footprint()
    assert published_half_x(box) == .45
    cart = make_node(adapter_module, carrying=True, box_mode=False)
    cart._publish_footprint()
    assert published_half_x(cart) == .62


def test_lifting_in_box_mode_does_not_look_for_cart_legs(adapter_module):
    node = make_node(adapter_module, box_mode=True)
    node._measure_cart = Mock()
    node._lift_state_cb(NS(data=True))
    node._measure_cart.assert_not_called()
    assert node.carrying and node.cart_pose == (0., 0., 0.) and not node._cart_measured
    assert published_half_x(node) == .45


def test_lifting_without_box_mode_still_measures_the_cart(adapter_module):
    node = make_node(adapter_module)
    node._measure_cart = Mock(return_value=(.01, 0., 0.))
    node._lift_state_cb(NS(data=True))
    node._measure_cart.assert_called_once()
    assert published_half_x(node) == .63


def test_box_mode_clears_itself_when_the_platform_goes_down(adapter_module):
    node = make_node(adapter_module, carrying=True, box_mode=True)
    node._lift_state_cb(NS(data=False))
    assert not node.carrying and not node.box_mode
    assert published_half_x(node) == .45


def test_box_mode_service_arms_only_with_the_platform_down(adapter_module):
    node = make_node(adapter_module)
    assert node._carry_box_mode(NS(data=True), NS()).success and node.box_mode
    node = make_node(adapter_module, carrying=True)
    response = node._carry_box_mode(NS(data=True), NS())
    assert not response.success and 'up' in response.message and not node.box_mode


def test_box_mode_cannot_be_dropped_under_a_load(adapter_module):
    node = make_node(adapter_module, carrying=True, box_mode=True)
    response = node._carry_box_mode(NS(data=False), NS())
    assert not response.success and node.box_mode
    node = make_node(adapter_module, box_mode=True)
    assert node._carry_box_mode(NS(data=False), NS()).success and not node.box_mode


def _scan(*ranges):
    return NS(header=NS(frame_id='laser'), angle_min=0., angle_increment=0., ranges=list(ranges))


def test_box_mode_filters_only_inside_the_robot_and_keeps_what_is_just_outside(adapter_module):
    inf = float('inf')
    node = make_node(adapter_module, carrying=True, box_mode=True)
    node._offset = lambda frame: (0., 0., 0.)
    node._scan_cb(_scan(.4))                 # 0.40 m ahead: inside the robot (0.45 + 0.05)
    assert node.scan_pub.publish.call_args.args[0].ranges == [inf]
    node._scan_cb(_scan(.56))                # 0.56 m ahead: an obstacle in front of the nose
    assert node.scan_pub.publish.call_args.args[0].ranges == [.56]
    cart = make_node(adapter_module, carrying=True, box_mode=False)
    cart._offset = lambda frame: (0., 0., 0.)
    cart._scan_cb(_scan(.56))                # the cart filter (0.67) is wider: unchanged behaviour
    assert cart.scan_pub.publish.call_args.args[0].ranges == [inf]


def test_unmeasured_cart_cannot_shrink_footprint(adapter_module):
    node = object.__new__(adapter_module.CartCarryAdapter)
    node.carrying, node._cart_measured = True, False
    node._straight_until = 0.
    node._publish_footprint = Mock()
    response = node._straight_footprint(NS(data=True), NS())
    assert not response.success
    assert node._straight_until == 0.
    node._publish_footprint.assert_not_called()
