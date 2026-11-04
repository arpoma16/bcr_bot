"""Detect a lift cart from its four legs in a 2D laser scan (shared by cart_handler.py and
cart_carry_adapter.py; installed next to them).

The cart model is four legs at (+-leg_x, +-leg_y) in the cart frame (indra models/lift_cart:
0.58, 0.48). Given a rough cart pose in the robot base frame, each expected leg collects the
scan points within search_radius; the centroids found are fitted to the model with a 2D rigid
transform (least squares), which gives the cart pose in the base frame independently of the
robot localization.
"""

import math


def scan_points(scan, laser_offset):
    """Finite scan points in the base frame. laser_offset = (x, y, yaw) of the laser."""
    ox, oy, oyaw = laser_offset
    points = []
    for i, r in enumerate(scan.ranges):
        if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
            a = scan.angle_min + i * scan.angle_increment + oyaw
            points.append((ox + r * math.cos(a), oy + r * math.sin(a)))
    return points


def leg_model(leg_x, leg_y):
    return [(leg_x, leg_y), (leg_x, -leg_y), (-leg_x, -leg_y), (-leg_x, leg_y)]


def transform(pose, point):
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * point[0] - s * point[1], y + s * point[0] + c * point[1])


def fit_cart(points, guess, model, search_radius=0.25, min_legs=2):
    """Cart pose (x, y, yaw) in the base frame and number of legs used, or (None, n)."""
    matched_model, matched_scan = [], []
    for leg in model:
        ex, ey = transform(guess, leg)
        near = [p for p in points if (p[0] - ex) ** 2 + (p[1] - ey) ** 2 <= search_radius ** 2]
        if near:
            matched_model.append(leg)
            matched_scan.append((sum(p[0] for p in near) / len(near),
                                 sum(p[1] for p in near) / len(near)))
    n = len(matched_model)
    if n < min_legs:
        return None, n
    mx = sum(p[0] for p in matched_model) / n
    my = sum(p[1] for p in matched_model) / n
    sx = sum(p[0] for p in matched_scan) / n
    sy = sum(p[1] for p in matched_scan) / n
    dot = cross = 0.0
    for (ax, ay), (bx, by) in zip(matched_model, matched_scan):
        ax, ay, bx, by = ax - mx, ay - my, bx - sx, by - sy
        dot += ax * bx + ay * by
        cross += ax * by - ay * bx
    yaw = math.atan2(cross, dot)
    c, s = math.cos(yaw), math.sin(yaw)
    return (sx - (c * mx - s * my), sy - (s * mx + c * my), yaw), n


def laser_offset_from_tf(tf):
    t, q = tf.transform.translation, tf.transform.rotation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return (t.x, t.y, yaw)
