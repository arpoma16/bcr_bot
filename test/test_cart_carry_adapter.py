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


def test_unmeasured_cart_cannot_shrink_footprint(adapter_module):
    node = object.__new__(adapter_module.CartCarryAdapter)
    node.carrying, node._cart_measured = True, False
    node._straight_until = 0.
    node._publish_footprint = Mock()
    response = node._straight_footprint(NS(data=True), NS())
    assert not response.success
    assert node._straight_until == 0.
    node._publish_footprint.assert_not_called()
