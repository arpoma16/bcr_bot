"""Planar cart geometry; all dimensions are metres and angles are radians."""
import math


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def staging_pose(cart, distance=1.8):
    x, y, yaw = cart
    return x - distance * math.cos(yaw), y - distance * math.sin(yaw), yaw


def local_pose(pose, frame):
    dx, dy = pose[0] - frame[0], pose[1] - frame[1]
    c, s = math.cos(frame[2]), math.sin(frame[2])
    return c * dx + s * dy, -s * dx + c * dy, wrap(pose[2] - frame[2])


def compose(frame, pose):
    c, s = math.cos(frame[2]), math.sin(frame[2])
    return (frame[0] + c * pose[0] - s * pose[1],
            frame[1] + s * pose[0] + c * pose[1], wrap(frame[2] + pose[2]))


def placement_robot_pose(cart, cart_in_base):
    yaw = wrap(cart[2] - cart_in_base[2])
    offset = compose((0., 0., yaw), cart_in_base)
    return cart[0] - offset[0], cart[1] - offset[1], yaw


def leading_strip(direction, half_x, half_y, lookahead):
    """New area beyond the leading edge, without an inflated lateral margin.

    Returns (xmin, xmax, ymin, ymax) in the cart frame (robot frame unloaded).
    The occupied cart rectangle itself is deliberately excluded.
    """
    if direction not in (-1, 1) or min(half_x, half_y, lookahead) <= 0:
        raise ValueError('invalid sweep geometry')
    if direction > 0:
        return half_x, half_x + lookahead, -half_y, half_y
    return -half_x - lookahead, -half_x, -half_y, half_y


def strip_blocked(points, cart_in_base, strip):
    xmin, xmax, ymin, ymax = strip
    return any(xmin <= x <= xmax and ymin <= y <= ymax
               for x, y, _ in (local_pose((x, y, 0.), cart_in_base) for x, y in points))


def motion_sweep_blocked(points, cart_in_base, half_x, half_y, direction, lookahead):
    """Exact extrusion along robot x, excluding the currently occupied rectangle.

    For an aligned cart this is precisely leading_strip. A rotated cart also
    sweeps a narrow wedge along one side; ignoring it would miss collisions.
    """
    vx, vy = direction * math.cos(cart_in_base[2]), -direction * math.sin(cart_in_base[2])
    for px, py in points:
        x, y, _ = local_pose((px, py, 0.), cart_in_base)
        if abs(x) <= half_x and abs(y) <= half_y:
            continue
        lo, hi = 0., lookahead
        for value, velocity, extent in ((x, vx, half_x), (y, vy, half_y)):
            if abs(velocity) < 1e-12:
                if abs(value) > extent:
                    hi = -1.
                    break
            else:
                a, b = (value - extent) / velocity, (value + extent) / velocity
                lo, hi = max(lo, min(a, b)), min(hi, max(a, b))
        if lo <= hi:
            return True
    return False


def deviation_ok(pose, axis, lateral_limit, yaw_limit, half_x, half_y, envelope):
    """Bound centre error and additional lateral corner excursion together."""
    _, lateral, yaw = local_pose(pose, axis)
    excursion = abs(lateral) + half_x * abs(math.sin(yaw)) + half_y * (math.cos(yaw) - 1.)
    return (all(math.isfinite(v) for v in (*pose, *axis))
            and abs(lateral) <= lateral_limit and abs(yaw) <= yaw_limit
            and excursion <= envelope)
