import cv2
import numpy as np

from . import vision as v


def line2point_dist(line, point) -> float:
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
        d = point2point_dist(x0, y0, x1, y1)
        if result is None or d < result[-1]:
            result = (np.array((x1, y1)), d)
    return result


def weighted_line2point_dist(line, point):
    line_dist = line2point_dist(line, point)
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

def define_line_reducer_on_point(point):
    def reducer(a, b):
        _, dist_a = closest_point(point, a.reshape(2, 2))
        _, dist_b = closest_point(point, b.reshape(2, 2))
        return a if dist_a < dist_b else b
    return reducer


def front_shift_transfer_function(closest_front_left_line, closest_front_right_line, hidden = 1.5):
    (xl, yl), dl = (None, None), None
    (xr, yr), dr = (None, None), None

    if closest_front_left_line is not None:
        (xl, yl), dl = closest_point(v.FRONT_CAM_LEFT_POINT_REF, closest_front_left_line.reshape(2, 2))
    else:
        xl, yl = v.FRONT_CAM_LEFT_POINT_REF - (160, 0)
        dl = hidden * point2point_dist(xl, yl, *v.FRONT_CAM_LEFT_POINT_REF)

    if closest_front_right_line is not None:
        (xr, yr), dr = closest_point(v.FRONT_CAM_RIGHT_POINT_REF, closest_front_right_line.reshape(2, 2))
    else:
        xr, yr = v.FRONT_CAM_RIGHT_POINT_REF + (160, 0)
        dr = hidden * point2point_dist(xr, yr, *v.FRONT_CAM_RIGHT_POINT_REF)

    denum = point2point_dist(xl, yl, xr, yr)
    denum = np.log(denum) if denum > np.e else denum

    if np.abs(num := dl - dr) > 1 and denum != 0:
        return (np.pi / 2) * np.tanh((num / denum ** 2))
    return 0


def calculate_front_theta(front_cam_state, **kwargs) -> float:
    front_cam_image = None if not 'front_cam_image' in kwargs else kwargs['front_cam_image']

    if front_cam_image is None and front_cam_state is not None:
        front_cam_image = v.imgmsg_to_cv2(front_cam_state)
        front_cam_image = v.adjust_image_brightness(front_cam_image)
        front_cam_image = v.mask_image(front_cam_image, v.FRONT_MASK)

    front_plant_theta = front_shift_transfer_function(
        closest_front_left_line=v.make_line_detection(
            front_cam_image,
            detect_fn=v.detect_front_cam_lanes,
            reduce_fn=define_line_reducer_on_point(
                point=v.FRONT_CAM_LEFT_POINT_REF,
            )
        ),
        closest_front_right_line=v.make_line_detection(
            front_cam_image,
            detect_fn=v.detect_front_cam_lanes,
            reduce_fn=define_line_reducer_on_point(
                point=v.FRONT_CAM_RIGHT_POINT_REF,
            ),
        ),
        hidden=1.5
    )

    return front_plant_theta


def theta_weighted_sum(*, front_theta, lateral_theta=0, lateral_weight = 0.65, front_weight = 0.35, last_theta=None):
    """Sums the different theta values according to a given weight for each theta"""
    theta = 0

    if lateral_theta != 0 and front_theta != 0:
        theta = lateral_theta * lateral_weight + front_theta * front_weight
    else:
        theta = lateral_theta + front_theta

    return theta


def alpha_theta(theta, last_theta=None):
    """Returns the angle in the oposite direction of theta that will be used
    to adjust the route after applying a theta angular rotation."""
    trace = 0
    if last_theta is not None:
        trace = last_theta / (12 * np.e)
    return -theta / 16 + trace
