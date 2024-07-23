import numpy as np
import math


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
            mask[:lower] = float('inf')
        if upper is not None:
            mask[upper:] = float('inf')
        return mask * value
    return None


def min_angle(theta):
    if theta > math.pi or theta < -math.pi:
        theta = -1 * np.sign(theta) * (math.pi - abs(theta))
    return theta
