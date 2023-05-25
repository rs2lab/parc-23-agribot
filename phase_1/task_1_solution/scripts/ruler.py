import numpy as np


def line_dist_to_point(line, point):
    x1, y1, x2, y2 = line
    x0, y0 = point
    mod = np.abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
    ro = np.sqrt((x2 - x1)**2 + (y2 - y1)**2)
    return mod / ro


def define_lateral_line_reducer(highest_point, lowest_point):
    def reduce_function(a, b):
        lhw = line_dist_to_point(a, highest_point)
        llw = line_dist_to_point(a, lowest_point)
        rhw = line_dist_to_point(b, highest_point)
        rlh = line_dist_to_point(b, lowest_point)
        dist_a = np.sqrt(llw**2 + lhw**2)
        dist_b = np.sqrt(rhw**2 + rlh**2)
        return a if dist_a < dist_b else b
    return reduce_function


def define_frontal_line_reducer(point):
    def reduce_funtion(a, b):
        dist_a = line_dist_to_point(a, point)
        dist_b = line_dist_to_point(b, point)
        return a if dist_a < dist_b else b
    return reduce_funtion
