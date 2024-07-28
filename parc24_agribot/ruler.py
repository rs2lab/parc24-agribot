import numpy as np
import math

from . import vision


def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    """
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def sgn(num):
    return 1 if num >= 0 else -1


def laser_range(laser_data):
    if laser_data is not None:
        min_angle = laser_data.angle_min
        max_angle = laser_data.angle_max
        step = (max_angle - min_angle) / len(laser_data.ranges)
        return np.arange(min_angle, max_angle, step)
    return None


def mask_laser_data(value: np.ndarray, lower: int = None, upper: int = None):
    if value is not None:
        mask = np.ones_like(value)
        if lower is not None:
            mask[:lower] = float("inf")
        if upper is not None:
            mask[upper:] = float("inf")
        return mask * value
    return None


def min_angle(theta):
    if theta > math.pi or theta < -math.pi:
        theta = -1 * np.sign(theta) * (math.pi - abs(theta))
    return theta


def line_dist_to_point(line, point):
    x1, y1, x2, y2 = line
    x0, y0 = point
    mod = np.abs((x2 - x1) * (y1 - y0) - (x1 - x0) * (y2 - y1))
    ro = np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
    return mod / ro


def point_distance(x0, y0, x1, y1):
    return np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)


def closest_point(point, other_points):
    result = None
    x0, y0 = point
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


def mask_laser_scan(value, lower: int = None, upper: int = None):
    if value is not None:
        mask = np.ones_like(value)
        if lower is not None:
            mask[:lower] = float("inf")
        if upper is not None:
            mask[upper:] = float("inf")
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


def front_shift_transfer_function(
    closest_front_left_line, closest_front_right_line, hidden=1.5
):
    # EXTEND = (160, 0)
    EXTEND = 0

    (xl, yl), dl = (None, None), None
    (xr, yr), dr = (None, None), None

    if closest_front_left_line is not None:
        (xl, yl), dl = closest_point(
            vision.FRONT_LEFT_CAM_REF, closest_front_left_line.reshape(2, 2)
        )
    else:
        xl, yl = vision.FRONT_LEFT_CAM_REF - EXTEND
        dl = hidden * point_distance(xl, yl, *vision.FRONT_LEFT_CAM_REF)

    if closest_front_right_line is not None:
        (xr, yr), dr = closest_point(
            vision.FRONT_RIGHT_CAM_REF, closest_front_right_line.reshape(2, 2)
        )
    else:
        xr, yr = vision.FRONT_RIGHT_CAM_REF + EXTEND
        dr = hidden * point_distance(xr, yr, *vision.FRONT_RIGHT_CAM_REF)

    denum = point_distance(xl, yl, xr, yr)
    denum = np.log(denum) if denum > np.e else denum

    if np.abs(num := dl - dr) > 1 and denum != 0:
        return (np.pi / 2) * np.tanh((num / denum**2))
    return 0


def calculate_front_theta(front_cam_image) -> float:
    front_cam_image = vision.mask_image(front_cam_image, vision.FRONT_MASK)

    plants_base_theta = front_shift_transfer_function(
        closest_front_left_line=vision.make_line_detection(
            front_cam_image,
            detect_fn=vision.detect_plant_base_obstacle,
            reduce_fn=define_line_reducer_on_point(
                point=vision.FRONT_LEFT_CAM_REF,
            ),
        ),
        closest_front_right_line=vision.make_line_detection(
            front_cam_image,
            detect_fn=vision.detect_plant_base_obstacle,
            reduce_fn=define_line_reducer_on_point(
                point=vision.FRONT_RIGHT_CAM_REF,
            ),
        ),
        hidden=2,
    )

    woden_fence_theta = front_shift_transfer_function(
        closest_front_left_line=vision.make_line_detection(
            front_cam_image,
            detect_fn=vision.detect_woden_fence_obstacle,
            reduce_fn=define_line_reducer_on_point(
                point=vision.FRONT_LEFT_CAM_REF,
            ),
        ),
        closest_front_right_line=vision.make_line_detection(
            front_cam_image,
            detect_fn=vision.detect_woden_fence_obstacle,
            reduce_fn=define_line_reducer_on_point(point=vision.FRONT_RIGHT_CAM_REF),
        ),
        hidden=2,
    )

    theta = woden_fence_theta + plants_base_theta

    if woden_fence_theta != 0 and plants_base_theta != 0:
        theta = woden_fence_theta * 0.6559 + plants_base_theta * 0.3441

    return theta


def theta_weighted_sum(
    *,
    front_theta,
    lateral_theta=0,
    lateral_weight=0.65,
    front_weight=0.35,
    last_theta=None,
):
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
