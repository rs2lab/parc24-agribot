import numpy as np
import open3d as o3d
import math
import sys
import struct
import ctypes

from sensor_msgs.msg import PointCloud2, PointField

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ("b", 1)
_DATATYPES[PointField.UINT8] = ("B", 1)
_DATATYPES[PointField.INT16] = ("h", 2)
_DATATYPES[PointField.UINT16] = ("H", 2)
_DATATYPES[PointField.INT32] = ("i", 4)
_DATATYPES[PointField.UINT32] = ("I", 4)
_DATATYPES[PointField.FLOAT32] = ("f", 4)
_DATATYPES[PointField.FLOAT64] = ("d", 8)


# imported from: https://gist.github.com/SebastianGrans/6ae5cab66e453a14a859b66cd9579239
def read_cloud_points(cloud: PointCloud2, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), "cloud is not a sensor_msgs.msg.PointCloud2"
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = (
        cloud.width,
        cloud.height,
        cloud.point_step,
        cloud.row_step,
        cloud.data,
        math.isnan,
    )
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = ">" if is_bigendian else "<"

    offset = 0
    for field in (
        f
        for f in sorted(fields, key=lambda f: f.offset)
        if field_names is None or f.name in field_names
    ):
        if offset < field.offset:
            fmt += "x" * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print(
                "Skipping unknown PointField datatype [%d]" % field.datatype,
                file=sys.stderr,
            )
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


# from https://answers.ros.org/question/255351/how-o-save-a-pointcloud2-data-in-python/
def destruct_point_cloud_to_xyz_rgb(cloud: PointCloud2) -> tuple[np.ndarray]:
    gen = read_cloud_points(cloud, skip_nans=True)
    int_data = list(gen)

    xyz = np.empty((len(int_data), 3))
    rgb = np.empty((len(int_data), 3))
    idx = 0
    for x in int_data:
        test = x[3]
        # cast float32 to int so that bitwise operations are possible
        s = struct.pack(">f", test)
        i = struct.unpack(">l", s)[0]
        # you can get back the float value by the inverse operations
        pack = ctypes.c_uint32(i).value
        r = (pack & 0x00FF0000) >> 16
        g = (pack & 0x0000FF00) >> 8
        b = pack & 0x000000FF
        # prints r,g,b values in the 0-255 range
        # x,y,z can be retrieved from the x[0],x[1],x[2]
        # xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
        # rgb = np.append(rgb,[[r,g,b]], axis = 0)
        xyz[idx] = [x[0], x[1], x[2]]
        rgb[idx] = [r, g, b]
        idx = idx + 1

    return xyz, rgb


def cloud_from_xyz_rgb(xyz: np.ndarray, rgb: np.ndarray) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    cloud.colors = o3d.utility.Vector3dVector(rgb / 255.0)
    return cloud


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
