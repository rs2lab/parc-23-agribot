import numpy as np


def line2point_dist(line, dist) -> float:
    x1, y1, x2, y2 = line
    x0, y0 = point
    mod = np.abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
    ro = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return mod / ro


def point2point_dist(x0, y0, x1, y1) -> float:
    return np.sqrt((x1 - x0)**2 + (y1 - y0)**2)


def closest_point(point, other_points):
    x0, y0 = point
    result = None
    for other_point in other_points:
        x1, y1 = other_point
        d = point_distance(x0, y0, x1, y1)
        if result is None or d < result[-1]:
            result = (np.array((x1, y1)), d)
    return result


def weighted_line_dist_to_point(line, point):
    line_dist = line_dist_to_point(line, point)
    _, inverse_weight = closest_point(point, line.reshape(2, 2))
    return line_dist / inverse_weight


def mask_laser_scan(value, lower: int = None, upper: int = None):
    if value is not None:
        mask = np.ones_like(value)
        if lower is not None:
            mask[:lower] = float('inf')
        if upper is not None:
            mask[upper:] = float('inf')
        return mask * value
    return None


def laser_angles(laser_state):
    if laser_state is not None:
        min_angle = laser_state.angle_min
        max_angle = laser_state.angle_max
        step = (max_angle - min_angle) / len(laser_state.ranges)
        return np.arange(min_angle, max_angle, step)
    return None
