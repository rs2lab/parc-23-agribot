import numpy as np

from constants import (
    FRONT_VISION_LEFT_POINT,
    FRONT_VISION_RIGHT_POINT,
)


def line_dist_to_point(line, point):
    x1, y1, x2, y2 = line
    x0, y0 = point
    mod = np.abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
    ro = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return mod / ro


def point_distance(x0, y0, x1, y1):
    return np.sqrt((x1 - x0)**2 + (y1 - y0)**2)


def closest_point(point, other_points):
    x0, y0 = point
    result = None
    for other_point in other_points:
        x1, y1 = other_point
        d = point_distance(x0, y0, x1, y1)
        if result is None or d < result[-1]:
            result = [np.array((x1, y1)), d]
    return result


def weighted_line_dist_to_point(line, point):
    line_dist = line_dist_to_point(line, point)
    _, inverse_weight = closest_point(point, line.reshape(2, 2))
    return line_dist / inverse_weight


def define_line_reducer_on_point(point):
    def reducer(a, b):
        _, dist_a = closest_point(point, a.reshape(2, 2))
        _, dist_b = closest_point(point, b.reshape(2, 2))
        return a if dist_a < dist_b else b
    return reducer


def theta_front_transfer_function(closest_front_left_line, closest_front_right_line,
            left_ref_point=FRONT_VISION_LEFT_POINT, right_ref_point=FRONT_VISION_RIGHT_POINT):
        # -- skip this line --
        (xl, yl), dl = closest_point(left_ref_point, closest_front_left_line.reshape(2, 2))
        (xr, yr), dr = closest_point(right_ref_point, closest_front_right_line.reshape(2, 2))
        num = point_distance(xl, yl, xr, xr)
        num = np.log(num) if num > np.e else num
        if (denum := dl - dr) != 0:
            return (np.pi / 2) * np.tanh(num / (denum * 2))
        return 0


def lateral_shift_transfer_function(closest_left_line, closest_right_line):
    dl = None
    dr = None

    if closest_left_line is not None:
        _, dl = closest_point(LATERAL_LEFT_VISION_POINT, closest_left_line.reshape(2, 2))
    else:
        dl = 1.1 * point_distance(LATERAL_LEFT_VISION_POINT[0], 0, *LATERAL_LEFT_VISION_POINT)

    if closest_right_line is not None:
        _, dr = closest_point(LATERAL_RIGHT_VISION_POINT, closest_right_line.reshape(2, 2))
    else:
        dr = 1.1 * point_distance(LATERAL_RIGHT_VISION_POINT[0], 480, *LATERAL_RIGHT_VISION_POINT)

    if  (d := dl - dr) < -1 or d > 1:
        return  (np.pi / 2) * np.tanh(np.sign(d) * 0.1 * np.log10(np.abs(d)))
    return 0
