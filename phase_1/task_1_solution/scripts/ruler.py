import numpy as np


def line_dist_to_point(line, point):
    x1, y1, x2, y2 = line
    x0, y0 = point
    mod = np.abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
    ro = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return mod / ro


def point_distance(x0, y0, x1, y1):
    return np.sqrt((x1 - x0)**2 + (y1 - y0)**2)


def closest_point_distance(point, other_points):
    x0, y0 = point
    closest_dist = None
    for other_point in other_points:
        x1, y1 = other_point
        dist = point_distance(x0, y0, x1, y1)
        if closest_dist is None or dist < closest_dist:
            closest_dist = dist
    return closest_dist


def weighted_line_dist_to_point(line, point):
    line_dist = line_dist_to_point(line, point)
    inverse_weight = closest_point_distance(point, line.reshape(2, 2))
    return line_dist / inverse_weight


def define_line_reducer_on_point(point):
    def reducer(a, b):
        dist_a = closest_point_distance(point, a.reshape(2, 2))
        dist_b = closest_point_distance(point, b.reshape(2, 2))
        return a if dist_a < dist_b else b
    return reducer
