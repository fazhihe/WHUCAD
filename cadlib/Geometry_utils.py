import cadlib.CAD_Class
from cadlib.macro import *
import math
from plyfile import PlyData


def construct_curve_from_vector(vec, start_point, is_numerical=True):
    if vec.shape.__len__() == 1:
        type = vec[0]
    else:
        type = vec[0][0]
    if type == LINE_IDX:
        return cadlib.CAD_Class.Line.from_vector(vec, start_point, is_numerical=is_numerical)
    elif type == CIRCLE_IDX:
        return cadlib.CAD_Class.Circle.from_vector(vec, start_point, is_numerical=is_numerical)
    elif type == ARC_IDX:
        res = cadlib.CAD_Class.Arc.from_vector(vec, start_point, is_numerical=is_numerical)
        if res is None: # for visualization purpose, replace illed arc with line
            return cadlib.CAD_Class.Line.from_vector(vec, start_point, is_numerical=is_numerical)
        return res
    elif type == SPLINE_IDX:
        res = cadlib.CAD_Class.Spline.from_vector(vec, start_point, is_numerical=is_numerical)
        return res
    else:
        raise NotImplementedError("curve type not supported yet: command idx {}".format(vec[0]))

def angle_from_vector_to_x(vec):
    """computer the angle (0~2pi) between a unit vector and positive x axis"""
    angle = 0.0
    # 2 | 1
    # -------
    # 3 | 4
    if vec[0] >= 0:
        if vec[1] >= 0:
            # Qadrant 1
            angle = math.asin(vec[1])
        else:
            # Qadrant 4
            angle = 2.0 * math.pi - math.asin(-vec[1])
    else:
        if vec[1] >= 0:
            # Qadrant 2
            angle = math.pi - math.asin(vec[1])
        else:
            # Qadrant 3
            angle = math.pi + math.asin(-vec[1])
    return angle

def polar_parameterization_inverse(theta, phi, gamma):
    """build a coordinate system by the given rotation from the standard 3D coordinate system"""
    normal_3d = polar2cartesian([theta, phi])
    ref_x = rotate_by_z(rotate_by_y(np.array([1, 0, 0]), theta), phi)
    ref_y = np.cross(normal_3d, ref_x)
    x_axis_3d = ref_x * np.cos(gamma) + ref_y * np.sin(gamma)
    return normal_3d, x_axis_3d

def cartesian2polar(vec, with_radius=False):
    vec = vec.round(6)
    norm = np.linalg.norm(vec)
    theta = np.arccos(vec[2] / norm) # (0, pi)
    phi = np.arctan(vec[1] / (vec[0] + 1e-15)) # (-pi, pi) # FIXME: -0.0 cannot be identified here
    if not with_radius:
        return np.array([theta, phi])
    else:
        return np.array([theta, phi, norm])

def rotate_by_y(vec, theta):
    mat = np.array([[np.cos(theta), 0, np.sin(theta)],
                    [0, 1, 0],
                    [-np.sin(theta), 0, np.cos(theta)]])
    return np.dot(mat, vec)

def rotate_by_z(vec, phi):
    mat = np.array([[np.cos(phi), -np.sin(phi), 0],
                    [np.sin(phi), np.cos(phi), 0],
                    [0, 0, 1]])
    return np.dot(mat, vec)

def polar2cartesian(vec):
    """convert a vector in polar(spherical) coordinates to cartesian coordinates"""
    r = 1 if len(vec) == 2 else vec[2]
    theta, phi = vec[0], vec[1]
    x = r * np.sin(theta) * np.cos(phi)
    y = r * np.sin(theta) * np.sin(phi)
    z = r * np.cos(theta)
    return np.array([x, y, z])

def polar_parameterization(normal_3d, x_axis_3d):
    normal_polar = cartesian2polar(normal_3d)
    theta = normal_polar[0]
    phi = normal_polar[1]

    ref_x = rotate_by_z(rotate_by_y(np.array([1, 0, 0]), theta), phi)

    gamma = np.arccos(np.dot(x_axis_3d, ref_x).round(6))
    if np.dot(np.cross(ref_x, x_axis_3d), normal_3d) < 0:
        gamma = -gamma
    return theta, phi, gamma

def vec2arc(pos, center):
    angle = 0
    vec = pos - center
    vec_np = np.array([vec[0], vec[1]])
    # x轴基准
    x_np = np.array([1, 0])
    # y轴基准
    y_np = np.array([0, 1])
    # 向量绝对值
    L = np.sqrt(vec_np.dot(vec_np))
    Lx = np.sqrt(x_np.dot(x_np))
    Ly = np.sqrt(y_np.dot(y_np))
    cos_x = vec_np.dot(x_np) / (L * Lx)
    cos_y = vec_np.dot(y_np) / (L * Ly)

    # 开始位置判断象限
    # 象限一、二
    if cos_y > 0:
        angle = np.arccos(cos_x)
    # 象限三、四
    elif cos_y < 0:
        angle = 2 * np.pi - np.arccos(cos_x)
    # 在坐标轴上
    elif cos_y == 0:
        if cos_x > 0:
            return 0
        else:
            return np.pi
    return angle % (2 * np.pi)

def float_equal(a, b):
    if np.abs(a - b) < 0.001:
        return True
    else:
        return False

def read_ply(path):
    with open(path, 'rb') as f:
        plydata = PlyData.read(f)
        x = np.array(plydata['vertex']['x'])
        y = np.array(plydata['vertex']['y'])
        z = np.array(plydata['vertex']['z'])
        vertex = np.stack([x, y, z], axis=1)
    return vertex