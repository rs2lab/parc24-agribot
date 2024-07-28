import cv2
import numpy as np
import open3d as o3d
import math
import sys
import struct
import ctypes

from sensor_msgs.msg import PointCloud2, PointField
from functools import reduce
from cv_bridge import CvBridge

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ("b", 1)
_DATATYPES[PointField.UINT8] = ("B", 1)
_DATATYPES[PointField.INT16] = ("h", 2)
_DATATYPES[PointField.UINT16] = ("H", 2)
_DATATYPES[PointField.INT32] = ("i", 4)
_DATATYPES[PointField.UINT32] = ("I", 4)
_DATATYPES[PointField.FLOAT32] = ("f", 4)
_DATATYPES[PointField.FLOAT64] = ("d", 8)

DEFAULT_SEG_CRT = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)

FRONT_LEFT_CAM_REF = np.array([910, 1080])
FRONT_RIGHT_CAM_REF = np.array([370, 1080])

_BRIDGE_OBJ = CvBridge()


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


def od3_cloud_from_xyz_rgb(xyz: np.ndarray, rgb: np.ndarray) -> o3d.geometry.PointCloud:
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(xyz)
    cloud.colors = o3d.utility.Vector3dVector(rgb / 255.0)
    return cloud


def create_triangle_mask(width, height):
    """Creates a triangular mask of the specified size.

    Args:
      width: Width of the mask.
      height: Height of the mask.

    Returns:
      A numpy array representing the triangular mask.
    """

    mask = np.zeros((height, width), dtype=np.uint8)

    # Define the triangle vertices
    pt1 = (0, height)
    pt2 = (width, height)
    pt3 = (width // 2, 0)

    # Create a filled triangle
    cv2.fillConvexPoly(mask, np.array([pt1, pt2, pt3], dtype=np.int32), 255)

    return mask


def pad_image(
    img, top, bottom, left, right, borderType=cv2.BORDER_CONSTANT, value=[0, 0, 0]
):
    """Pads an image with zeros.

    Args:
      img: The input image.
      top, bottom, left, right: The amount of padding to add in each direction.
      borderType: The type of border.
      value: The value to fill the border with (default is black).

    Returns:
      The padded image.
    """

    return cv2.copyMakeBorder(img, top, bottom, left, right, borderType, value=value)


WIDTH = 1280
HEIGHT = 720

MASK_TRIANGLE_WIDTH = 1080
MASK_TRIANGLE_HEIGHT = 480

FRONT_MASK = pad_image(
    create_triangle_mask(
        MASK_TRIANGLE_WIDTH,
        MASK_TRIANGLE_HEIGHT,
    ),
    HEIGHT - MASK_TRIANGLE_HEIGHT,
    0,
    (WIDTH - MASK_TRIANGLE_WIDTH) // 2,
    (WIDTH - MASK_TRIANGLE_WIDTH) // 2,
)


def mask_image(image, mask):
    return cv2.bitwise_and(image, image, mask=mask)


def kmeans_segmentation(image, attempts=10, k=4, criteria=DEFAULT_SEG_CRT):
    td_img = np.float32(image.reshape((-1, 3)))
    _, label, center = cv2.kmeans(
        td_img, k, None, criteria, attempts, cv2.KMEANS_PP_CENTERS
    )

    center = np.uint8(center)
    res = center[label.flatten()]

    return res.reshape((image.shape))


def detect_color_profile(image, lower_bound, upper_bound, blur_image=False):
    if blur_image is True:
        image = cv2.GaussianBlur(image, (5, 5), 0)
    color_mask = cv2.inRange(image, lower_bound, upper_bound)
    return cv2.bitwise_and(image, image, mask=color_mask)


WODEN_FENCE_OBSTACLE_LOWER = np.array([5, 47, 99])
WODEN_FENCE_OBSTACLE_HIGHER = np.array([40, 255, 213])
PLANT_BASE_OBSTACLE_LOWER = np.array([34, 0, 95])
PLANT_BASE_OBSTACLE_HIGHER = np.array([85, 38, 195])


def detect_woden_fence_obstacle(image, lower_adjust=0, upper_adjust=0, apply_hsv=True):
    if apply_hsv:
        image = convert_image_to_hsv(image)
    return detect_color_profile(
        image=image,
        lower_bound=np.array(WODEN_FENCE_OBSTACLE_LOWER) + lower_adjust,
        upper_bound=np.array(WODEN_FENCE_OBSTACLE_HIGHER) + upper_adjust,
        blur_image=False,
    )


def detect_plant_base_obstacle(image, lower_adjust=0, upper_adjust=0, apply_hsv=True):
    if apply_hsv:
        image = convert_image_to_hsv(image)
    return detect_color_profile(
        image=image,
        lower_bound=np.array(PLANT_BASE_OBSTACLE_LOWER) + lower_adjust,
        upper_bound=np.array(PLANT_BASE_OBSTACLE_HIGHER) + upper_adjust,
        blur_image=False,
    )


def convert_image_to_hsv(image):
    return cv2.cvtColor(image, cv2.COLOR_RGB2HSV_FULL)


def canny(image, thresh1=50, thresh2=100, blur_image=False):
    gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    if blur_image is True:
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

    return cv2.Canny(gray, thresh1, thresh2)


def make_coordinates(image, line_params):
    slope, intercept = line_params
    y1 = image.shape[0]
    y2 = int(y1 * 0.8)
    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)
    return np.array([x1, y1, x2, y2])


def average_slope_intercep(image, lines):
    left_fit = []
    right_fit = []
    for x1, y1, x2, y2 in lines.reshape(-1, 4):
        params = np.polyfit((x1, x2), (y1, y2), 1)
        slope = params[0]
        intercept = params[1]
        if slope < 0:
            left_fit.append((slope, intercept))
        else:
            right_fit.append((slope, intercept))
    left_fit_average = np.average(left_fit, axis=0)
    right_fit_average = np.average(right_fit, axis=0)
    left_line = make_coordinates(image, left_fit_average)
    right_line = make_coordinates(image, right_fit_average)
    return np.array([left_line, right_line])


def hough_lines(image, min_line_len=20, max_line_gap=20, apply_canny=False):
    if apply_canny:
        image = canny(image, blur_image=True)
    return cv2.HoughLinesP(
        image, 2, np.pi / 180, 10, np.array([]), min_line_len, max_line_gap
    )


def draw_lines_on_image(image, lines, color_rgb=(255, 0, 0)):
    line_image = np.zeros_like(image)

    if lines is not None:
        for x1, y1, x2, y2 in lines.reshape(-1, 4):
            cv2.line(line_image, (x1, y1), (x2, y2), color_rgb, 10)

    return cv2.addWeighted(image, 0.8, line_image, 1, 1)


def make_line_detection(
    image, *, detect_fn, reduce_fn=None, crop_fn=None, mask=None, display_images=False
):
    lines = None
    if image is not None:
        original_image = image
        if crop_fn is not None:
            image = crop_fn(image)
        if mask is not None:
            image = mask_image(image, mask=mask)

        image = detect_fn(image, apply_hsv=True)
        lines = hough_lines(image, apply_canny=True)

        if display_images is True and image is not None and lines is not None:
            cv2.imshow("camera", draw_lines_on_image(original_image, lines))
            cv2.waitKey(1)
        if lines is not None and reduce_fn is not None:
            lines = reduce(reduce_fn, lines.reshape(-1, 4))
    return lines
