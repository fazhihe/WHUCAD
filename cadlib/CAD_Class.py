from copy import deepcopy

from cadlib.macro import *
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from cadlib.Geometry_utils import construct_curve_from_vector, angle_from_vector_to_x, polar_parameterization_inverse, polar2cartesian, vec2arc

##########################   base  ###########################
class SketchBase(object):
    """Base class for sketch (a collection of curves). """
    def __init__(self, children, reorder=True):
        self.children = children

        if reorder:
            self.reorder()

    @staticmethod
    def from_dict(stat):
        """construct sketch from json data

        Args:
            stat (dict): dict from json data
        """
        raise NotImplementedError

    @staticmethod
    def from_vector(vec, start_point, is_numerical=True):
        """construct sketch from vector representation

        Args:
            vec (np.array): (seq_len, n_args)
            start_point (np.array): (2, ). If none, implicitly defined as the last end point.
        """
        raise NotImplementedError

    def reorder(self):
        """rearrange the curves to follow counter-clockwise direction"""
        raise NotImplementedError

    @property
    def start_point(self):
        return self.children[0].start_point

    @property
    def end_point(self):
        return self.children[-1].end_point

    @property
    def bbox(self):
        """compute bounding box (min/max points) of the sketch"""
        all_points = np.concatenate([child.bbox for child in self.children], axis=0)
        return np.stack([np.min(all_points, axis=0), np.max(all_points, axis=0)], axis=0)

    @property
    def bbox_size(self):
        """compute bounding box size (max of height and width)"""
        bbox_min, bbox_max = self.bbox[0], self.bbox[1]
        bbox_size = np.max(np.abs(np.concatenate([bbox_max - self.start_point, bbox_min - self.start_point])))
        return bbox_size

    @property
    def global_trans(self):
        """start point + sketch size (bbox_size)"""
        return np.concatenate([self.start_point, np.array([self.bbox_size])])

    def transform(self, translate, scale):
        """linear transformation"""
        for child in self.children:
            child.transform(translate, scale)

    def flip(self, axis):
        for child in self.children:
            child.flip(axis)
        self.reorder()

    def numericalize(self, n=256):
        """quantize curve parameters into integers"""
        for child in self.children:
            child.numericalize(n)

    def denumericalize(self, n=256):
        for child in self.children:
            child.denumericalize(n)

    def normalize(self, size=256):
        """normalize within the given size, with start_point in the middle center"""
        cur_size = self.bbox_size
        scale = (size / 2 * NORM_FACTOR - 1) / cur_size # prevent potential overflow if data augmentation applied
        self.transform(-self.start_point, scale)
        self.transform(np.array((size / 2, size / 2)), 1)

    def denormalize(self, bbox_size, size=256):
        """inverse procedure of normalize method"""
        scale = bbox_size / (size / 2 * NORM_FACTOR - 1)
        self.transform(-np.array((size / 2, size / 2)), scale)

    def to_vector(self):
        """convert to vector representation"""
        raise NotImplementedError

    def draw(self, ax):
        """draw sketch on matplotlib ax"""
        raise NotImplementedError

    def to_image(self):
        """convert to image"""
        fig, ax = plt.subplots()
        self.draw(ax)
        ax.axis('equal')
        fig.canvas.draw()
        X = np.array(fig.canvas.renderer.buffer_rgba())[:, :, :3]
        plt.close(fig)
        return X

    def sample_points(self, n=32):
        """uniformly sample points from the sketch"""
        raise NotImplementedError

####################### loop & profile #######################
class Loop(SketchBase):
    """Sketch loop, a sequence of connected curves."""
    def __str__(self):
        s = "Loop:"
        for curve in self.children:
            s += "\n      -" + str(curve)
        return s

    @staticmethod
    def from_vector(vec, start_point=None, is_numerical=True):
        all_curves = []
        if start_point is None:
            # FIXME: explicit for loop can be avoided here
            for i in range(vec.shape[0]):
                if vec[i][0] == EOS_IDX:
                    start_point = vec[i - 1][1:3]
                    break
        i = 0
        while i < vec.shape[0]:
            type = vec[i][0]
            if type == SOL_IDX:
                i = i + 1
                continue
            elif type == EOS_IDX:
                break
            # 若为Spline，则合并
            elif type == SPLINE_IDX:
                j = i + 1
                while j < vec.shape[0] and vec[j][0] == SCP_IDX:
                    j = j + 1
                curve = construct_curve_from_vector(vec[i:j], start_point, is_numerical=is_numerical)
                start_point = curve.end_point
                i = j - 1
            else:
                curve = construct_curve_from_vector(vec[i], start_point, is_numerical=is_numerical)
                start_point = vec[i][1:3]  # current curve's end_point serves as next curve's start_point
            all_curves.append(curve)
            i = i + 1
        return Loop(all_curves, reorder=False)

    def reorder(self):
        """reorder by starting left most and counter-clockwise"""
        if len(self.children) <= 1:
            return

        start_curve_idx = -1
        sx, sy = 10000, 10000

        # correct start-end point order
        if np.allclose(self.children[0].start_point, self.children[1].start_point) or \
            np.allclose(self.children[0].start_point, self.children[1].end_point):
            self.children[0].reverse()

        # correct start-end point order and find left-most point
        for i, curve in enumerate(self.children):
            if i < len(self.children) - 1 and np.allclose(curve.end_point, self.children[i + 1].end_point):
                self.children[i + 1].reverse()
            if round(curve.start_point[0], 6) < round(sx, 6) or \
                    (round(curve.start_point[0], 6) == round(sx, 6) and round(curve.start_point[1], 6) < round(sy, 6)):
                start_curve_idx = i
                sx, sy = curve.start_point

        self.children = self.children[start_curve_idx:] + self.children[:start_curve_idx]

        # ensure mostly counter-clock wise
        if isinstance(self.children[0], Circle) or isinstance(self.children[-1], Circle): # FIXME: hard-coded
            return
        start_vec = self.children[0].direction()
        end_vec = self.children[-1].direction(from_start=False)
        if np.cross(end_vec, start_vec) <= 0:
            for curve in self.children:
                curve.reverse()
            self.children.reverse()

    def to_vector(self, max_len=None, add_sol=True, add_eos=True):
        sta = []
        for curve in self.children:
            sta.append(curve.to_vector())
        sta = np.concatenate(sta, axis=0)
        loop_vec = np.stack(sta, axis=0)
        if add_sol:
            loop_vec = np.concatenate([SOL_VEC[np.newaxis], loop_vec], axis=0)
        if add_eos:
            loop_vec = np.concatenate([loop_vec, EOS_VEC[np.newaxis]], axis=0)
        if max_len is None:
            return loop_vec

        if loop_vec.shape[0] > max_len:
            return None
        elif loop_vec.shape[0] < max_len:
            pad_vec = np.tile(EOS_VEC, max_len - loop_vec.shape[0]).reshape((-1, len(EOS_VEC)))
            loop_vec = np.concatenate([loop_vec, pad_vec], axis=0) # (max_len, 1 + N_ARGS)
        return loop_vec

    def draw(self, ax):
        colors = ['red', 'blue', 'green', 'brown', 'pink', 'yellow', 'purple', 'black'] * 10
        for i, curve in enumerate(self.children):
            curve.draw(ax, colors[i])

    def sample_points(self, n=32):
        points = np.stack([curve.sample_points(n) for curve in self.children], axis=0) # (n_curves, n, 2)
        return points

class Profile(SketchBase):
    """Sketch profile，a closed region formed by one or more loops.
    The outer-most loop is placed at first."""
    @staticmethod
    def from_dict(stat):
        all_loops = [Loop.from_dict(item) for item in stat['loops']]
        return Profile(all_loops)

    def __str__(self):
        s = "Profile:"
        for loop in self.children:
            s += "\n    -" + str(loop)
        return s

    @staticmethod
    def from_vector(vec, start_point=None, is_numerical=True):
        all_loops = []
        command = vec[:, 0]
        end_idx = command.tolist().index(EOS_IDX)
        indices = np.where(command[:end_idx] == SOL_IDX)[0].tolist() + [end_idx]
        for i in range(len(indices) - 1):
            loop_vec = vec[indices[i]:indices[i + 1]]
            loop_vec = np.concatenate([loop_vec, EOS_VEC[np.newaxis]], axis=0)
            if loop_vec[0][0] == SOL_IDX and loop_vec[1][0] not in [SOL_IDX, EOS_IDX]:
                all_loops.append(Loop.from_vector(loop_vec, is_numerical=is_numerical))
        return Profile(all_loops, reorder=False)

    def reorder(self):
        if len(self.children) <= 1:
            return
        all_loops_bbox_min = np.stack([loop.bbox[0] for loop in self.children], axis=0).round(6)
        ind = np.lexsort(all_loops_bbox_min.transpose()[[1, 0]])
        self.children = [self.children[i] for i in ind]

    def draw(self, ax):
        for i, loop in enumerate(self.children):
            loop.draw(ax)
            ax.text(loop.start_point[0], loop.start_point[1], str(i))

    def to_vector(self, max_n_loops=None, max_len_loop=None, pad=True):
        loop_vecs = [loop.to_vector(None, add_eos=False) for loop in self.children]
        # if max_n_loops is not None and len(loop_vecs) > max_n_loops:
        #     return None
        # for vec in loop_vecs:
        #     if max_len_loop is not None and vec.shape[0] > max_len_loop:
        #         return None
        profile_vec = np.concatenate(loop_vecs, axis=0)
        profile_vec = np.concatenate([profile_vec, EOS_VEC[np.newaxis]], axis=0)
        if pad:
            pad_len = max_n_loops * max_len_loop - profile_vec.shape[0]
            profile_vec = np.concatenate([profile_vec, EOS_VEC[np.newaxis].repeat(pad_len, axis=0)], axis=0)
        return profile_vec

    def sample_points(self, n=32):
        points = np.concatenate([loop.sample_points(n) for loop in self.children], axis=0)
        return points

class Line():
    def __init__(self, start_point, end_point, reportName=0):
        self.start_point = start_point
        self.end_point = end_point
        self.reportName = reportName

    def reverse(self):
        self.start_point, self.end_point = self.end_point, self.start_point

    def direction(self, from_start=True):
        return self.end_point - self.start_point

    def transform(self, translation, scale):
        self.start_point = (self.start_point + translation) * scale
        self.end_point = (self.end_point + translation) * scale

    def numericalize(self, n=256):
        self.start_point, _ = Get_integer_and_fraction(self.start_point)
        self.end_point, _ = Get_integer_and_fraction(self.end_point)

    def denumericalize(self, n=256):
        pass

    @staticmethod
    def from_vector(vec, start_point, is_numerical=True):
        end_point = vec[1:3]
        return Line(start_point, end_point)

    def to_vector(self):
        vec = [LINE_IDX, self.end_point[0], self.end_point[1]]
        vec = np.array(vec + [PAD_VAL] * (1 + N_ARGS - len(vec)))
        vec = np.array([vec])
        return vec

    @property
    def bbox(self):
        points = np.stack([self.start_point, self.end_point], axis=0)
        return np.stack([np.min(points, axis=0), np.max(points, axis=0)], axis=0)

class Arc(object):
    def __init__(self, center, radius, start_arc, end_arc, mid_arc, reportName=0):
        self.center = center
        self.radius = radius
        self.start_arc = start_arc
        self.end_arc = end_arc
        self.reportName = reportName
        self.mid_arc = mid_arc
        self.mid_point = self.center + self.radius * np.array([np.cos(self.mid_arc), np.sin(self.mid_arc)])
        self.start_point = np.array([self.center[0] + np.cos(self.start_arc) * self.radius, self.center[1] + np.sin(self.start_arc) * self.radius])
        self.end_point = np.array([self.center[0] + np.cos(self.end_arc) * self.radius, self.center[1] + np.sin(self.end_arc) * self.radius])

    @property
    def ref_vec(self):
        if self.clock_sign == 0:
            tmp = self.end_point - self.center
        else:
            tmp = self.start_point - self.center
        return tmp / np.linalg.norm(tmp)

    @property
    def clock_sign(self):
        """get a boolean sign indicating whether the arc is on top of s->e """
        s2e = self.end_point - self.start_point
        s2m = self.mid_point - self.start_point
        sign = np.cross(s2m, s2e) >= 0 # counter-clockwise
        return sign

    def reverse(self):
        self.start_arc, self.end_arc = self.end_arc, self.start_arc
        self.start_point, self.end_point = self.end_point, self.start_point

    def numericalize(self, n=256):
        # 先将弧度规定在0~2pi
        self.start_arc = self.start_arc % (2 * np.pi)
        self.end_arc = self.end_arc % (2 * np.pi)
        tmp = np.array([self.start_arc, self.end_arc])
        self.start_arc, self.end_arc = tmp / (2 * np.pi) * n

        self.start_point, _ = Get_integer_and_fraction(self.start_point)
        self.end_point, _ = Get_integer_and_fraction(self.end_point)
        self.center, _ = Get_integer_and_fraction(self.center)
        self.mid_point, _ = Get_integer_and_fraction(self.mid_point)
        self.start_arc, _ = Get_integer_and_fraction(self.start_arc)
        self.end_arc, _ = Get_integer_and_fraction(self.end_arc)

    def denumericalize(self, n=256):
        pass

    def to_vector(self):
        if self.clock_sign:
            sweep_angle = (self.end_arc - self.start_arc + ARGS_N) % ARGS_N
        else:
            sweep_angle = (self.start_arc - self.end_arc + ARGS_N) % ARGS_N
        # sweep_angle = max(abs(self.start_arc - self.end_arc), 1)
        vec = np.array([ARC_IDX, self.end_point[0], self.end_point[1], sweep_angle, int(self.clock_sign), PAD_VAL,
                        *[PAD_VAL] * (N_ARGS - N_ARGS_SKETCH)])
        return np.array([vec])

    @staticmethod
    def from_vector(vec, start_point, is_numerical=True):
        end_point = vec[1:3]
        # 添加fraction
        sweep_angle = vec[3] / ARGS_N * 2 * np.pi if is_numerical else vec[3]
        clock_sign = vec[4]
        s2e_vec = end_point - start_point
        if np.linalg.norm(s2e_vec) == 0:
            return None
        radius = (np.linalg.norm(s2e_vec) / 2) / np.sin(sweep_angle / 2)
        s2e_mid = (start_point + end_point) / 2
        vertical = np.cross(s2e_vec, [0, 0, 1])[:2]
        vertical = vertical / np.linalg.norm(vertical)
        if clock_sign == 0:
            vertical = -vertical
        center_point = s2e_mid - vertical * (radius * np.cos(sweep_angle / 2))

        start_arc = vec2arc(start_point, center_point)
        end_arc = vec2arc(end_point, center_point)
        if clock_sign == 0:
            if end_arc < start_arc:
                mid_arc = (start_arc + end_arc) / 2
            else:
                mid_arc = (start_arc + end_arc + 2 * np.pi) / 2
        else:
            if start_arc < end_arc:
                mid_arc = (start_arc + end_arc) / 2
            else:
                mid_arc = (start_arc + end_arc + 2 * np.pi) / 2

        return Arc(center_point, radius, start_arc, end_arc, mid_arc)


    def transform(self, translation, scale):
        self.start_point = (self.start_point + translation) * scale
        self.mid_point = (self.mid_point + translation) * scale
        self.end_point = (self.end_point + translation) * scale
        self.center = (self.center + translation) * scale
        if isinstance(scale * 1.0, float):
            self.radius = abs(self.radius * scale)

    def get_angles_counterclockwise(self, eps=1e-8):
        c2s_vec = (self.start_point - self.center) / (np.linalg.norm(self.start_point - self.center) + eps)
        c2m_vec = (self.mid_point - self.center) / (np.linalg.norm(self.mid_point - self.center) + eps)
        c2e_vec = (self.end_point - self.center) / (np.linalg.norm(self.end_point - self.center) + eps)
        angle_s, angle_m, angle_e = angle_from_vector_to_x(c2s_vec), angle_from_vector_to_x(c2m_vec), \
                                    angle_from_vector_to_x(c2e_vec)
        angle_s, angle_e = min(angle_s, angle_e), max(angle_s, angle_e)
        if not angle_s < angle_m < angle_e:
            angle_s, angle_e = angle_e - np.pi * 2, angle_s
        return angle_s, angle_e

    @property
    def clock_sign(self):
        """get a boolean sign indicating whether the arc is on top of s->e """
        s2e = self.end_point - self.start_point
        s2m = self.mid_point - self.start_point
        sign = np.cross(s2m, s2e) >= 0 # counter-clockwise
        return sign

    @property
    def bbox(self):
        points = [self.start_point, self.end_point]
        angle_s, angle_e = self.get_angles_counterclockwise()
        if angle_s < 0 < angle_e:
            points.append(np.array([self.center[0] + self.radius, self.center[1]]))
        if angle_s < np.pi / 2 < angle_e or angle_s < -np.pi / 2 * 3 < angle_e:
            points.append(np.array([self.center[0], self.center[1] + self.radius]))
        if angle_s < np.pi < angle_e or angle_s < -np.pi < angle_e:
            points.append(np.array([self.center[0] - self.radius, self.center[1]]))
        if angle_s < np.pi / 2 * 3 < angle_e or angle_s < -np.pi/2 < angle_e:
            points.append(np.array([self.center[0], self.center[1] - self.radius]))
        points = np.stack(points, axis=0)
        return np.stack([np.min(points, axis=0), np.max(points, axis=0)], axis=0)

    def direction(self, from_start=True):
        if from_start:
            return self.mid_point - self.start_point
        else:
            return self.end_point - self.mid_point

class Circle(object):
    def __init__(self, center, radius, reportName=0):
        self.center = center
        self.radius = radius
        self.reportName = reportName

    def transform(self, translation, scale):
        self.center = (self.center + translation) * scale
        self.radius = self.radius * scale

    def numericalize(self, n=256):
        self.center, _ = Get_integer_and_fraction(self.center)
        self.radius, _ = Get_integer_and_fraction(self.radius)

    def denumericalize(self, n=256):
        pass

    @staticmethod
    def from_vector(vec, start_point=None, is_numerical=True):
        return Circle(vec[1:3], vec[5])

    def to_vector(self):
        vec = [CIRCLE_IDX, self.center[0], self.center[1], PAD_VAL, PAD_VAL, self.radius]
        vec = np.array(np.array(vec + [PAD_VAL] * (1 + N_ARGS - len(vec))))
        return np.array([vec])

    @property
    def bbox(self):
        return np.stack([self.center - self.radius, self.center + self.radius], axis=0)

    @property
    def start_point(self):
        return np.array([self.center[0] - self.radius, self.center[1]])

    @property
    def end_point(self):
        return np.array([self.center[0] + self.radius, self.center[1]])

    def direction(self, from_start=True):
        return self.center - self.start_point

class Spline(object):
    def __init__(self, point_list, reportName=0, no_in_loop=0):
        self.point_list = point_list
        self.reportName = reportName
        self.no_in_loop = no_in_loop

    def reverse(self):
        self.point_list = np.flipud(self.point_list)

    def direction(self, from_start=True):
        return self.end_point - self.start_point

    def transform(self, translation, scale):
        new_point_list = []
        for i in self.point_list:
            new_point_list.append((i + translation) * scale)
        self.point_list = np.array(new_point_list)

    def numericalize(self, n=256):
        new_point_list = []
        for i in self.point_list:
            i, _ = Get_integer_and_fraction(i)
            new_point_list.append(i)
        self.point_list = new_point_list

    def denumericalize(self, n=256):
        pass

    @staticmethod
    def from_vector(vec, start_point, is_numerical=True):
        point_list = [start_point]
        for i in vec[1:]:
            point_list.append(i[1:3])
        point_list = np.array(point_list)
        return Spline(point_list)

    def to_vector(self):
        vec = [SPLINE_VEC]
        for i in self.point_list[1:]:
            tmp = [SCP_IDX, i[0], i[1]]
            vec.append(tmp + [PAD_VAL] * (1 + N_ARGS - len(tmp)))
        vec = np.array(vec)
        return vec

    @property
    def bbox(self):
        # 挑出最左、最右、最上、最下的坐标，然后将左右距离和上下对比，看哪个更长
        min_x = 0x3f3f3f3f
        min_y = 0x3f3f3f3f
        max_x = -0x3f3f3f3f
        max_y = -0x3f3f3f3f
        min_x_point = -1
        min_y_point = -1
        max_x_point = -1
        max_y_point = -1
        for i in range(self.point_list.__len__()):
            if min_x > np.abs(self.point_list[i][0]):
                min_x = np.abs(self.point_list[i][0])
                min_x_point = i
            if min_y > np.abs(self.point_list[i][1]):
                min_y = np.abs(self.point_list[i][1])
                min_y_point = i
            if max_x < np.abs(self.point_list[i][0]):
                max_x = np.abs(self.point_list[i][0])
                max_x_point = i
            if max_y < np.abs(self.point_list[i][1]):
                max_y = np.abs(self.point_list[i][1])
                max_y_point = i
        x_dis = max_x - min_x
        y_dis = max_y - min_y
        if x_dis > y_dis:
            return np.stack([self.point_list[min_x_point], self.point_list[max_x_point]], axis=0)
        else:
            return np.stack([self.point_list[min_y_point], self.point_list[max_y_point]], axis=0)

    @property
    def start_point(self):
        return self.point_list[0]

    @property
    def end_point(self):
        return self.point_list[self.point_list.__len__() - 1]

class CoordSystem(object):
    def __init__(self, origin, theta, phi, gamma, y_axis=None, is_numerical=False):
        self.is_numerical = is_numerical
        # 直接在初始化中增加小数部分
        self.origin = origin
        self._theta = theta  # 0~pi
        self._phi = phi  # -pi~pi
        self._gamma = gamma  # -pi~pi
        self._y_axis = y_axis  # (theta, phi)

    def transform(self, translation, scale):
        self.origin = (self.origin + translation) * scale

    def numericalize(self, n=256):
        """NOTE: shall only be called after normalization"""
        self.origin = (self.origin + 1.0) / 2 * n
        tmp = np.array([self._theta, self._phi, self._gamma])
        self._theta, self._phi, self._gamma = (tmp / np.pi + 1.0) / 2 * n

        self.origin, _ = Get_integer_and_fraction(self.origin)
        self._theta, _ = Get_integer_and_fraction(self._theta)
        self._phi, _ = Get_integer_and_fraction(self._phi)
        self._gamma, _ = Get_integer_and_fraction(self._gamma)

    def denumericalize(self, n=256):
        self.origin = self.origin / n * 2 - 1.0
        tmp = np.array([self._theta, self._phi, self._gamma])
        self._theta, self._phi, self._gamma = (tmp / n * 2 - 1.0) * np.pi
        self.is_numerical = False

    def to_vector(self):
        return np.array([*self.origin, self._theta, self._phi, self._gamma])

    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        origin = vec[3:]
        theta, phi, gamma = vec[:3]
        system = CoordSystem(origin, theta, phi, gamma)
        if is_numerical:
            system.denumericalize(n)
        return system

    @property
    def normal(self):
        return polar2cartesian([self._theta, self._phi])

    @property
    def x_axis(self):
        normal_3d, x_axis_3d = polar_parameterization_inverse(self._theta, self._phi, self._gamma)
        return x_axis_3d

    @property
    def y_axis(self):
        if self._y_axis is None:
            return np.cross(self.normal, self.x_axis)
        return polar2cartesian(self._y_axis)

class Extrude(object):
    def __init__(self, extent_one, extent_two, isSymmetric, isInverse, operation, extent_type1, extent_type2, sketch_name, sketch_plane=None, sketch_pos=None, sketch_size=None, sketch_profile=None, select_list=None):
        self.extent_one = extent_one
        self.extent_two = extent_two
        self.isSymmetric = isSymmetric
        self.isInverse = isInverse
        self.operation = operation
        self.extent_type1 = extent_type1
        self.extent_type2 = extent_type2
        self.sketch_name = sketch_name
        self.sketch_plane = sketch_plane
        self.sketch_pos = sketch_pos
        self.sketch_size = sketch_size
        self.sketch_profile = sketch_profile
        self.select_list = select_list

    def transform(self, translation, scale):
        # 草图信息
        self.sketch_plane.transform(translation, scale)
        self.sketch_pos = (self.sketch_pos + translation) * scale
        self.sketch_size *= scale
        # 处理对称
        if self.isSymmetric and self.extent_type1 == "OffsetLimit":
            self.extent_two = self.extent_one
        # 处理Inverse
        if self.isInverse:
            self.extent_one, self.extent_two = self.extent_two, self.extent_one
            self.extent_type1, self.extent_type2 = self.extent_type2, self.extent_type1
            # 若两者都为直到面，则交换select_list中元素顺序
            if self.extent_type1 in ["UpToPlaneLimit", "UpToSurfaceLimit"] and self.extent_type2 in ["UpToPlaneLimit", "UpToSurfaceLimit"]:
                if self.select_list is not None:
                    self.select_list.reverse()

        self.extent_one = self.extent_one * scale
        self.extent_two = self.extent_two * scale

    def numericalize(self, n=256):
        self.sketch_profile.numericalize(n)
        # 确定拉伸类型
        self.extent_type1 = EXTENT_TYPE.index(self.extent_type1)
        self.extent_type2 = EXTENT_TYPE.index(self.extent_type2)
        self.operation = BOOLEAN_OPERATIONS.index(self.operation)
        if self.select_list is not None:
            for i in range(len(self.select_list)):
                self.select_list[i].numericalize()
        self.extent_one = (self.extent_one + 1.0) / 2 * n
        self.extent_two = (self.extent_two + 1.0) / 2 * n

        self.extent_one, _ = Get_integer_and_fraction(self.extent_one)
        self.extent_two, _ = Get_integer_and_fraction(self.extent_two)

        self.sketch_pos = (self.sketch_pos + 1.0) / 2 * n
        self.sketch_size = self.sketch_size / 2 * n
        self.sketch_pos, _ = Get_integer_and_fraction(self.sketch_pos)
        self.sketch_size, _ = Get_integer_and_fraction(self.sketch_size)

        copy_plane = deepcopy(self.sketch_plane)
        copy_plane.numericalize(n)
        self.sketch_plane = copy_plane

    def denumericalize(self, n=256):
        """de-quantize the representation."""
        self.extent_one = self.extent_one / n * 2 - 1.0
        self.extent_two = self.extent_two / n * 2 - 1.0
        self.sketch_plane.denumericalize(n)
        self.sketch_profile.denumericalize(n)
        self.sketch_pos = self.sketch_pos / n * 2 - 1.0
        self.sketch_size = self.sketch_size / n * 2
        self.extent_type1 = EXTENT_TYPE[int(self.extent_type1)]
        self.extent_type2 = EXTENT_TYPE[int(self.extent_type2)]
        self.operation = BOOLEAN_OPERATIONS[int(self.operation)]
        if self.select_list is not None:
            for i in self.select_list:
                i.denumericalize()

    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        """vector representation: commands [SOL, ..., SOL, ..., EXT]"""
        assert vec[-1][0] == EXT_IDX and vec[0][0] == SOL_IDX
        # 寻找第一个Topo
        topo_point = 0
        while topo_point < len(vec):
            if vec[topo_point][0] == TOPO_IDX or vec[topo_point][0] == EXT_IDX:
                break
            else:
                topo_point = topo_point + 1

        profile_vec = np.concatenate([vec[:topo_point], EOS_VEC[np.newaxis]])

        profile = Profile.from_vector(profile_vec, is_numerical=is_numerical)
        ext_vec = vec[-1][1 + N_ARGS_SKETCH:1 + N_ARGS_SKETCH + N_ARGS_EXT]

        sket_pos = ext_vec[N_ARGS_PLANE:N_ARGS_PLANE + 3]
        sket_size = ext_vec[N_ARGS_PLANE + N_ARGS_TRANS - 1]

        sket_plane = CoordSystem.from_vector(ext_vec[:N_ARGS_PLANE + N_ARGS_TRANS - 1])
        ext_param = ext_vec[N_ARGS_PLANE + N_ARGS_TRANS:]

        select_list = None
        if topo_point < len(vec) - 1:
            select_vec = vec[topo_point:-1]
            select_commands = select_vec[:, 0]
            select_indices = np.where(select_commands == TOPO_IDX)[0].tolist()
            select_list = []
            for i in range(select_indices.__len__() - 1):
                select_list.append(Select.to_select(select_vec[select_indices[i]:select_indices[i + 1]]))
            select_list.append(Select.to_select(select_vec[select_indices[-1]:]))


        res = Extrude(ext_param[0], ext_param[1], False, False, ext_param[6], ext_param[2], ext_param[3], '', sket_plane, sket_pos, sket_size, profile, select_list=select_list)
        if is_numerical:
            res.denumericalize(n)
        return res

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True):
        vec = self.sketch_profile.to_vector(max_n_loops, max_len_loop, pad=False)[:-1]
        sket_plane_orientation = self.sketch_plane.to_vector()
        sket_plane_orientation = sket_plane_orientation[3:]
        ext_param = list(sket_plane_orientation) + list(self.sketch_pos) + [self.sketch_size] + [self.extent_one, self.extent_two, self.extent_type1, self.extent_type2, -1, -1, self.operation]
        ext_vec = np.array([EXT_IDX, *[PAD_VAL] * N_ARGS_SKETCH, *ext_param, *[PAD_VAL] * (N_ARGS_FINISH_PARAM + N_ARGS_SELECT_PARAM)])
        ext_vec = ext_vec[np.newaxis, :]
        if pad:
            pad_len = max_n_loops * max_len_loop - ext_vec.shape[0]
            ext_vec = np.concatenate([ext_vec, EOS_VEC[np.newaxis].repeat(pad_len, axis=0)], axis=0)
        if self.select_list is not None:
            all_vec = None
            for select in self.select_list:
                select_vec = select.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True)
                # 转np.array
                select_vec_np = np.array(select_vec)
                if all_vec is None:
                    all_vec = select_vec_np
                else:
                    all_vec = np.concatenate([all_vec, select_vec_np], axis=0)
            vec = np.concatenate([vec, all_vec, ext_vec], axis=0)
        else:
            vec = np.concatenate([vec, ext_vec], axis=0)
        return vec

class Groove(object):
    def __init__(self, select_list, angle_one, angle_two, isInverse, sketch_name, sketch_plane=None, sketch_pos=None, sketch_size=None, sketch_profile=None):
        self.select_list = select_list
        self.angle_one = angle_one
        self.angle_two = angle_two
        self.isInverse = isInverse
        self.sketch_name = sketch_name
        self.sketch_plane = sketch_plane
        self.sketch_pos = sketch_pos
        self.sketch_size = sketch_size
        self.sketch_profile = sketch_profile

    def transform(self, translation, scale):
        if self.isInverse:
            self.angle_one, self.angle_two = self.angle_two, self.angle_one
        # 草图信息
        self.sketch_plane.transform(translation, scale)
        self.sketch_pos = (self.sketch_pos + translation) * scale
        self.sketch_size *= scale

    def numericalize(self, n=256):
        # catia的角度参数过于自由且不太符合逻辑，因此将旋转角度angle_one、angle_two转换成onshape的形式
        # 将在denumericalize中转换回catia的形式
        if self.angle_one != 360:
            self.angle_one = (self.angle_one + 360) % 360
        self.angle_two = (360 - self.angle_two) % 360

        for i in range(self.select_list.__len__()):
            self.select_list[i].numericalize(n)
        self.sketch_profile.numericalize(n)
        self.angle_one = self.angle_one / 360 * (ARGS_N - 1)
        self.angle_two = self.angle_two / 360 * (ARGS_N - 1)
        self.angle_one, _ = Get_integer_and_fraction(self.angle_one)
        self.angle_two, _ = Get_integer_and_fraction(self.angle_two)

        self.sketch_pos = (self.sketch_pos + 1.0) / 2 * n
        self.sketch_size = self.sketch_size / 2 * n
        self.sketch_pos, _ = Get_integer_and_fraction(self.sketch_pos)
        self.sketch_size, _ = Get_integer_and_fraction(self.sketch_size)
        copy_plane = deepcopy(self.sketch_plane)
        copy_plane.numericalize(n)
        self.sketch_plane = copy_plane

    def denumericalize(self, n=256):
        """de-quantize the representation."""
        self.angle_one = self.angle_one / (ARGS_N - 1) * 360
        self.angle_two = self.angle_two / (ARGS_N - 1) * 360
        self.sketch_plane.denumericalize(n)
        self.sketch_profile.denumericalize(n)
        self.sketch_pos = self.sketch_pos / n * 2 - 1.0
        self.sketch_size = self.sketch_size / n * 2
        for i in self.select_list:
            i.denumericalize()


    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        """vector representation: commands [SOL, ..., SOL, ..., EXT]"""
        assert vec[-1][0] == GROOVE_IDX and vec[0][0] == SOL_IDX
        # 这里应考虑选取不一定只有两条
        profile_vec = []
        for i in vec:
            if i[0] != TOPO_IDX:
                profile_vec.append(i)
            else:
                break
        profile_vec = np.concatenate([profile_vec, EOS_VEC[np.newaxis]])
        profile = Profile.from_vector(np.array(profile_vec), is_numerical=is_numerical)
        rev_vec = vec[-1][1 + N_ARGS_SKETCH:1 + N_ARGS_SKETCH + N_ARGS_EXT]

        sket_pos = rev_vec[N_ARGS_PLANE:N_ARGS_PLANE + N_ARGS_TRANS - 1]
        sket_size = rev_vec[N_ARGS_PLANE + N_ARGS_TRANS - 1]

        sket_plane = CoordSystem.from_vector(rev_vec[:N_ARGS_PLANE + N_ARGS_TRANS - 1])
        rev_param = rev_vec[N_ARGS_PLANE + N_ARGS_TRANS:]

        select_vec = np.array([vec[-2]])
        select = Select.to_select(select_vec)

        res = Groove([select], rev_param[4], rev_param[5], False, '', sket_plane, sket_pos, sket_size, profile)
        if is_numerical:
            res.denumericalize(n)
        return res

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True):
        all_vec = []
        for select in self.select_list:
            select_vec = np.array(select.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True))
            vec = self.sketch_profile.to_vector(max_n_loops, max_len_loop, pad=False)[:-1]
            sket_plane_orientation = self.sketch_plane.to_vector()
            sket_plane_orientation = sket_plane_orientation[3:]
            rev_param = list(sket_plane_orientation) + list(self.sketch_pos) + [self.sketch_size] + [-1, -1, -1, -1, self.angle_one, self.angle_two, -1]
            rev_vec = np.array([GROOVE_IDX, *[PAD_VAL] * N_ARGS_SKETCH, *rev_param, *[PAD_VAL] * (N_ARGS_FINISH_PARAM + N_ARGS_SELECT_PARAM)])
            rev_vec = rev_vec[np.newaxis, :]
            if pad:
                pad_len = max_n_loops * max_len_loop - rev_vec.shape[0]
                rev_vec = np.concatenate([rev_vec, EOS_VEC[np.newaxis].repeat(pad_len, axis=0)], axis=0)
            vec = np.concatenate([vec, select_vec, rev_vec], axis=0)
            all_vec.append(vec)
        all_vec = np.array(all_vec)
        all_vec = np.concatenate(all_vec, axis=0)
        return all_vec

class Revolve(object):
    def __init__(self, select_list, angle_one, angle_two, isInverse, operation, sketch_name, sketch_plane=None, sketch_pos=None, sketch_size=None, sketch_profile=None):
        self.select_list = select_list
        self.angle_one = angle_one
        self.angle_two = angle_two
        self.isInverse = isInverse
        self.operation = operation
        self.sketch_name = sketch_name
        self.sketch_plane = sketch_plane
        self.sketch_pos = sketch_pos
        self.sketch_size = sketch_size
        self.sketch_profile = sketch_profile

    def transform(self, translation, scale):
        if self.isInverse:
            self.angle_one, self.angle_two = self.angle_two, self.angle_one
        # 草图信息
        self.sketch_plane.transform(translation, scale)
        self.sketch_pos = (self.sketch_pos + translation) * scale
        self.sketch_size *= scale

    def numericalize(self, n=256):
        # catia的角度参数过于自由且不太符合逻辑，因此将旋转角度angle_one、angle_two转换成onshape的形式
        # 将在denumericalize中转换回catia的形式
        if self.angle_one != 360:
            self.angle_one = (self.angle_one + 360) % 360
        self.angle_two = (360 - self.angle_two) % 360

        self.operation = BOOLEAN_OPERATIONS.index(self.operation)

        for i in range(self.select_list.__len__()):
            self.select_list[i].numericalize(n)
        self.sketch_profile.numericalize(n)
        self.angle_one = self.angle_one / 360 * (ARGS_N - 1)
        self.angle_two = self.angle_two / 360 * (ARGS_N - 1)
        self.angle_one, _ = Get_integer_and_fraction(self.angle_one)
        self.angle_two, _ = Get_integer_and_fraction(self.angle_two)

        self.sketch_pos = (self.sketch_pos + 1.0) / 2 * n
        self.sketch_size = self.sketch_size / 2 * n
        self.sketch_pos, _ = Get_integer_and_fraction(self.sketch_pos)
        self.sketch_size, _ = Get_integer_and_fraction(self.sketch_size)
        copy_plane = deepcopy(self.sketch_plane)
        copy_plane.numericalize(n)
        self.sketch_plane = copy_plane

    def denumericalize(self, n=256):
        """de-quantize the representation."""
        self.angle_one = self.angle_one / (ARGS_N - 1) * 360
        self.angle_two = self.angle_two / (ARGS_N - 1) * 360
        self.sketch_plane.denumericalize(n)
        self.sketch_profile.denumericalize(n)
        self.sketch_pos = self.sketch_pos / n * 2 - 1.0
        self.sketch_size = self.sketch_size / n * 2
        self.operation = BOOLEAN_OPERATIONS[int(self.operation)]
        for i in self.select_list:
            i.denumericalize()


    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        """vector representation: commands [SOL, ..., SOL, ..., EXT]"""
        assert vec[-1][0] == REV_IDX and vec[0][0] == SOL_IDX
        # 这里应考虑选取不一定只有两条
        profile_vec = []
        for i in vec:
            if i[0] != TOPO_IDX:
                profile_vec.append(i)
            else:
                break
        profile_vec = np.concatenate([profile_vec, EOS_VEC[np.newaxis]])
        profile = Profile.from_vector(np.array(profile_vec), is_numerical=is_numerical)
        rev_vec = vec[-1][1 + N_ARGS_SKETCH:1 + N_ARGS_SKETCH + N_ARGS_EXT]

        sket_pos = rev_vec[N_ARGS_PLANE:N_ARGS_PLANE + N_ARGS_TRANS - 1]
        sket_size = rev_vec[N_ARGS_PLANE + N_ARGS_TRANS - 1]

        sket_plane = CoordSystem.from_vector(rev_vec[:N_ARGS_PLANE + N_ARGS_TRANS - 1])
        rev_param = rev_vec[N_ARGS_PLANE + N_ARGS_TRANS:]

        select_vec = np.array([vec[-2]])
        select = Select.to_select(select_vec)

        res = Revolve([select], rev_param[4], rev_param[5], False, rev_param[6], '', sket_plane, sket_pos, sket_size, profile)
        if is_numerical:
            res.denumericalize(n)
        return res

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True):
        all_vec = []
        for select in self.select_list:
            select_vec = np.array(select.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True))
            vec = self.sketch_profile.to_vector(max_n_loops, max_len_loop, pad=False)[:-1]
            sket_plane_orientation = self.sketch_plane.to_vector()
            sket_plane_orientation = sket_plane_orientation[3:]
            rev_param = list(sket_plane_orientation) + list(self.sketch_pos) + [self.sketch_size] + [-1, -1, -1, -1, self.angle_one, self.angle_two, self.operation]
            rev_vec = np.array([REV_IDX, *[PAD_VAL] * N_ARGS_SKETCH, *rev_param, *[PAD_VAL] * (N_ARGS_FINISH_PARAM + N_ARGS_SELECT_PARAM)])
            rev_vec = rev_vec[np.newaxis, :]
            if pad:
                pad_len = max_n_loops * max_len_loop - rev_vec.shape[0]
                rev_vec = np.concatenate([rev_vec, EOS_VEC[np.newaxis].repeat(pad_len, axis=0)], axis=0)
            vec = np.concatenate([vec, select_vec, rev_vec], axis=0)
            all_vec.append(vec)
        all_vec = np.array(all_vec)
        all_vec = np.concatenate(all_vec, axis=0)
        return all_vec

class Pocket(object):
    def __init__(self, extent_one, extent_two, isSymmetric, isInverse, extent_type1, extent_type2, sketch_name, sketch_plane=None, sketch_pos=None, sketch_size=None, sketch_profile=None, limit_mode=0, select_list=None):
        self.extent_one = extent_one
        self.extent_two = extent_two
        self.isSymmetric = isSymmetric
        self.isInverse = isInverse
        self.extent_type1 = extent_type1
        self.extent_type2 = extent_type2
        self.sketch_name = sketch_name
        self.sketch_plane = sketch_plane
        self.sketch_pos = sketch_pos
        self.sketch_size = sketch_size
        self.sketch_profile = sketch_profile
        self.limit_mode = limit_mode
        self.select_list = select_list

    def transform(self, translation, scale):
        # 草图信息
        self.sketch_plane.transform(translation, scale)
        self.sketch_pos = (self.sketch_pos + translation) * scale
        self.sketch_size *= scale
        # 处理对称
        if self.isSymmetric and self.extent_type1 == "OffsetLimit":
            self.extent_two = self.extent_one
        # 处理Inverse
        if self.isInverse:
            self.extent_one, self.extent_two = self.extent_two, self.extent_one
            self.extent_type1, self.extent_type2 = self.extent_type2, self.extent_type1
            # 若两者都为直到面，则交换select_list中元素顺序
            if self.extent_type1 in ["UpToPlaneLimit", "UpToSurfaceLimit"] and self.extent_type2 in ["UpToPlaneLimit", "UpToSurfaceLimit"]:
                if self.select_list is not None:
                    self.select_list.reverse()

        self.extent_one = self.extent_one * scale
        self.extent_two = self.extent_two * scale

    def numericalize(self, n=256):
        self.sketch_profile.numericalize(n)
        # 确定拉伸类型
        self.extent_type1 = EXTENT_TYPE.index(self.extent_type1)
        self.extent_type2 = EXTENT_TYPE.index(self.extent_type2)

        if self.select_list is not None:
            for i in range(len(self.select_list)):
                self.select_list[i].numericalize()

        self.extent_one = (self.extent_one + 1.0) / 2 * n
        self.extent_two = (self.extent_two + 1.0) / 2 * n

        self.extent_one, _ = Get_integer_and_fraction(self.extent_one)
        self.extent_two, _ = Get_integer_and_fraction(self.extent_two)

        self.sketch_pos = (self.sketch_pos + 1.0) / 2 * n
        self.sketch_size = self.sketch_size / 2 * n
        self.sketch_pos, _ = Get_integer_and_fraction(self.sketch_pos)
        self.sketch_size, _ = Get_integer_and_fraction(self.sketch_size)
        copy_plane = deepcopy(self.sketch_plane)
        copy_plane.numericalize(n)
        self.sketch_plane = copy_plane

    def denumericalize(self, n=256):
        """de-quantize the representation."""
        self.extent_type1 = EXTENT_TYPE[int(self.extent_type1)]
        self.extent_type2 = EXTENT_TYPE[int(self.extent_type2)]
        self.extent_one = self.extent_one / n * 2 - 1.0
        self.extent_two = self.extent_two / n * 2 - 1.0
        self.sketch_plane.denumericalize(n)
        self.sketch_profile.denumericalize(n)
        self.sketch_pos = self.sketch_pos / n * 2 - 1.0
        self.sketch_size = self.sketch_size / n * 2
        if self.select_list is not None:
            for i in self.select_list:
                i.denumericalize()

    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        """vector representation: commands [SOL, ..., SOL, ..., EXT]"""
        assert vec[-1][0] == POCKET_IDX and vec[0][0] == SOL_IDX

        # 寻找第一个Topo
        topo_point = 0
        while topo_point < len(vec):
            if vec[topo_point][0] == TOPO_IDX or vec[topo_point][0] == POCKET_IDX:
                break
            else:
                topo_point = topo_point + 1

        profile_vec = np.concatenate([vec[:topo_point], EOS_VEC[np.newaxis]])

        profile = Profile.from_vector(profile_vec, is_numerical=is_numerical)
        ext_vec = vec[-1][1 + N_ARGS_SKETCH:1 + N_ARGS_SKETCH + N_ARGS_EXT]

        sket_pos = ext_vec[N_ARGS_PLANE:N_ARGS_PLANE + 3]
        sket_size = ext_vec[N_ARGS_PLANE + N_ARGS_TRANS - 1]

        sket_plane = CoordSystem.from_vector(ext_vec[:N_ARGS_PLANE + N_ARGS_TRANS - 1])
        ext_param = ext_vec[N_ARGS_PLANE + N_ARGS_TRANS:]

        select_list = None
        if topo_point < len(vec) - 1:
            select_vec = vec[topo_point:-1]
            select_commands = select_vec[:, 0]
            select_indices = np.where(select_commands == TOPO_IDX)[0].tolist()
            select_list = []
            for i in range(select_indices.__len__() - 1):
                select_list.append(Select.to_select(select_vec[select_indices[i]:select_indices[i + 1]]))
            select_list.append(Select.to_select(select_vec[select_indices[-1]:]))

        res = Pocket(ext_param[0], ext_param[1], False, False, ext_param[2], ext_param[3], '',
                      sket_plane, sket_pos, sket_size, profile, select_list=select_list)
        if is_numerical:
            res.denumericalize(n)
        return res

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True):
        vec = self.sketch_profile.to_vector(max_n_loops, max_len_loop, pad=False)[:-1]
        sket_plane_orientation = self.sketch_plane.to_vector()
        sket_plane_orientation = sket_plane_orientation[3:]
        ext_param = list(sket_plane_orientation) + list(self.sketch_pos) + [self.sketch_size] + [self.extent_one,
                                                                                                 self.extent_two,
                                                                                                 self.extent_type1,
                                                                                                 self.extent_type2, -1,
                                                                                                 -1, -1]
        ext_vec = np.array(
            [POCKET_IDX, *[PAD_VAL] * N_ARGS_SKETCH, *ext_param, *[PAD_VAL] * (N_ARGS_FINISH_PARAM + N_ARGS_SELECT_PARAM)])
        ext_vec = ext_vec[np.newaxis, :]
        if pad:
            pad_len = max_n_loops * max_len_loop - ext_vec.shape[0]
            ext_vec = np.concatenate([ext_vec, EOS_VEC[np.newaxis].repeat(pad_len, axis=0)], axis=0)
        if self.select_list is not None:
            all_vec = None
            for select in self.select_list:
                select_vec = select.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True)
                # 转np.array
                select_vec_np = np.array(select_vec)
                if all_vec is None:
                    all_vec = select_vec_np
                else:
                    all_vec = np.concatenate([all_vec, select_vec_np], axis=0)
            vec = np.concatenate([vec, all_vec, ext_vec], axis=0)
        else:
            vec = np.concatenate([vec, ext_vec], axis=0)
        return vec

class Select(object):
    def __init__(self, select_type, body_type="None", body_no=0, no=0, operation_list=[], no_shared_included=[], all_oriented_included={}, all_partially_included=None):
        self.select_type = select_type
        self.body_type = body_type
        self.body_no = body_no
        self.no = no
        self.operation_list = operation_list
        self.no_shared_included = no_shared_included
        self.all_oriented_included = all_oriented_included
        self.all_partially_included = all_partially_included


    def numericalize(self, n=256):
        self.select_type = SELECT_TYPE.index(self.select_type)
        self.body_type = BODY_TYPE.index(self.body_type)
        self.body_no = int(self.body_no)
        self.no = int(self.no)
        for i in self.operation_list:
            i.numericalize(n)
        for i in self.no_shared_included:
            i.numericalize(n)
        for i in self.all_oriented_included.values():
            for j in i:
                j.numericalize(n)
        if self.all_partially_included != None:
            for i in self.all_partially_included:
                i.numericalize(n)

    def denumericalize(self, n=256):
        self.select_type = SELECT_TYPE[int(self.select_type)]
        self.body_type = BODY_TYPE[int(self.body_type)]
        self.body_no = int(self.body_no)
        self.no = int(self.no)
        for i in self.operation_list:
            i.denumericalize(n)
        for i in self.no_shared_included:
            i.denumericalize(n)
        for i in self.all_oriented_included.values():
            for j in i:
                j.denumericalize(n)
        if self.all_partially_included != None:
            for i in self.all_partially_included:
                i.denumericalize(n)

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True, is_last=False, no_shared=False, all_oriented_type=0, partially_shared=False):
        vec = []
        # 记录是否处理了混淆情况，若是则为True
        no_shared_flag = False
        all_oriented_limits1_flag = False
        all_oriented_limits2_flag = False
        partially_shared_flag = False
        # 若自身为MirrorFace，则在子面定义开始时声明，以免和其他子面混淆
        if self.body_type == BODY_TYPE.index('Mirror'):
            vec.append(MIRROR_START_VEC)
        for i in self.operation_list:
            if vec.__len__() == 0:
                vec = i.to_vector(max_n_loops, max_len_loop, pad=False)
            else:
                vec = vec + i.to_vector(max_n_loops, max_len_loop, pad=False)
        for i in self.no_shared_included:
            vec = vec + i.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True, no_shared=True)
            no_shared_flag = True
        # 若处理了混淆情况，最后加上结束符标志混淆结束
        if no_shared_flag:
            vec.append(NO_SHARED_INCLUDED_END_VEC)
        if len(self.all_oriented_included) != 0:
            for i in self.all_oriented_included['Limits1']:
                vec = vec + i.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True, no_shared=False, all_oriented_type=1)
                all_oriented_limits1_flag = True
            if all_oriented_limits1_flag:
                vec.append(ALL_ORIENTED_INCLUDED_END_VEC)
            if 'Limits2' in self.all_oriented_included.keys():
                for i in self.all_oriented_included['Limits2']:
                    vec = vec + i.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True, no_shared=False, all_oriented_type=2)
                    all_oriented_limits2_flag = True
                if all_oriented_limits2_flag:
                    vec.append(ALL_ORIENTED_INCLUDED_END_VEC)
        if self.all_partially_included != None:
            for i in self.all_partially_included:
                vec = vec + i.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True, no_shared=False, partially_shared=True)
                partially_shared_flag = True
        # 若处理了混淆情况，最后加上结束符标志混淆结束
            if partially_shared_flag:
                vec.append(ALL_PARTIALLY_INCLUDED_END_VEC)
        vec.append([SELECT_IDX, *[PAD_VAL]*(N_ARGS - N_ARGS_SELECT_PARAM), self.select_type, self.body_type, self.body_no, self.no])
        if is_last:
            if no_shared:
                vec = [NO_SHARED_INCLUDED_VEC] + vec
            elif partially_shared:
                vec = [ALL_PARTIALLY_INCLUDED_VEC] + vec
            elif all_oriented_type == 1:
                vec = [ALL_ORIENTED_INCLUDED_1_VEC] + vec
            elif all_oriented_type == 2:
                vec = [ALL_ORIENTED_INCLUDED_2_VEC] + vec
            else:
                vec = [TOPO_VEC] + vec
        return vec

    def to_select(vec):
        # 当碰到TOPO时代表为select，设置为True，放入select_list，当碰到NO_SHARED_INCLUDED时代表混淆，设置为False，放入no_shared_included
        select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 = 0
        select_list = []
        no_shared_included = []
        partially_shared_included = []
        all_oriented_included_limits1 = []
        all_oriented_included_limits2 = []
        mirror_list = []
        # 记录是否应当把no_shared_included赋值给当前select，赋值后置False
        time_to_no_shared = False
        time_to_partially_shared = False
        # 同理记录是否应当把all_oriented_included赋值给当前select，赋值后置False
        time_to_all_oriented1 = False
        time_to_all_oriented2 = False
        # 该标志优先于select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2
        start_mirror = 0
        # 不需要Mirror的END VEC，出现Mirror的body_type即为结束
        for i in range(vec.shape[0]):
            if vec[i][0] == TOPO_IDX:
                select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 = 0
            elif vec[i][0] == NO_SHARED_INCLUDED_IDX:
                select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 = 1
            elif vec[i][0] == ALL_ORIENTED_INCLUDED_1_IDX:
                select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 = 2
            elif vec[i][0] == ALL_ORIENTED_INCLUDED_2_IDX:
                select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 = 3
            elif vec[i][0] == ALL_PARTIALLY_INCLUDED_IDX:
                select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 = 4
            elif vec[i][0] == MIRROR_START_IDX:
                start_mirror = start_mirror + 1
            elif vec[i][0] == NO_SHARED_INCLUDED_END_IDX:
                time_to_no_shared = True
                select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 = 0
            elif vec[i][0] == ALL_PARTIALLY_INCLUDED_END_IDX:
                time_to_partially_shared = True
                select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 = 0
            elif vec[i][0] == ALL_ORIENTED_INCLUDED_END_IDX:
                if select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                    time_to_all_oriented1 = True
                elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                    time_to_all_oriented2 = True
                select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 = 0
            else:
                select_vec = vec[i][-4:]
                if select_vec[0] == SELECT_TYPE.index("Wire"):
                    if start_mirror > 0:
                        mirror_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                        select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                        no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                        all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                        all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                        partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                elif select_vec[0] == SELECT_TYPE.index("Face"):
                    if int(select_vec[1]) == BODY_TYPE.index("Shell"):
                        tmp_list = []
                        if start_mirror > 0:
                            # Shell正常不应该只有一个子面？
                            # while mirror_list.__len__() > 0 and mirror_list[-1].select_type == 'Sub_Face':
                            #     tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                            # tmp_list.reverse()
                            tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                            mirror_list.append(
                                Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            # while select_list.__len__() > 0 and select_list[-1].select_type == 'Sub_Face':
                            #     tmp_list.append(select_list.pop(select_list.__len__() - 1))
                            # tmp_list.reverse()
                            tmp_list.append(select_list.pop(select_list.__len__() - 1))
                            select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                            # 插入混淆标识面
                            if time_to_no_shared:
                                time_to_no_shared = False
                                select_list[-1].no_shared_included = deepcopy(no_shared_included)
                                no_shared_included = []
                            if time_to_partially_shared:
                                time_to_partially_shared = False
                                select_list[-1].all_partially_included = deepcopy(partially_shared_included)
                                partially_shared_included = []
                            if time_to_all_oriented1:
                                time_to_all_oriented1 = False
                                select_list[-1].all_oriented_included['Limits1'] = deepcopy(all_oriented_included_limits1)
                                all_oriented_included_limits1 = []
                            if time_to_all_oriented2:
                                time_to_all_oriented2 = False
                                select_list[-1].all_oriented_included['Limits2'] = deepcopy(all_oriented_included_limits2)
                                all_oriented_included_limits2 = []
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 1))
                            no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            # while all_oriented_included_limits1.__len__() > 0 and all_oriented_included_limits1[-1].select_type == 'Sub_Face':
                            #     tmp_list.append(all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 1))
                            # tmp_list.reverse()
                            tmp_list.append(
                                all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 1))
                            all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            # while all_oriented_included_limits2.__len__() > 0 and all_oriented_included_limits2[-1].select_type == 'Sub_Face':
                            #     tmp_list.append(all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 1))
                            # tmp_list.reverse()
                            tmp_list.append(
                                all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 1))
                            all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 1))
                            partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                    elif int(select_vec[1]) == BODY_TYPE.index("Chamfer") or int(select_vec[1]) == BODY_TYPE.index("EdgeFillet"):
                        tmp_list = []
                        # 注意顺序
                        if start_mirror > 0:
                            tmp_list.append(mirror_list.pop(mirror_list.__len__() - 2))
                            tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                            mirror_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]),
                                                      operation_list=tmp_list, no_shared_included=[],
                                                      all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            tmp_list.append(select_list.pop(select_list.__len__() - 2))
                            tmp_list.append(select_list.pop(select_list.__len__() - 1))
                            select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                            # 插入混淆标识面
                            if time_to_no_shared:
                                time_to_no_shared = False
                                select_list[-1].no_shared_included = deepcopy(no_shared_included)
                                no_shared_included = []
                            if time_to_partially_shared:
                                time_to_partially_shared = False
                                select_list[-1].all_partially_included = deepcopy(partially_shared_included)
                                partially_shared_included = []
                            if time_to_all_oriented1:
                                time_to_all_oriented1 = False
                                select_list[-1].all_oriented_included['Limits1'] = deepcopy(all_oriented_included_limits1)
                                all_oriented_included_limits1 = []
                            if time_to_all_oriented2:
                                time_to_all_oriented2 = False
                                select_list[-1].all_oriented_included['Limits2'] = deepcopy(all_oriented_included_limits2)
                                all_oriented_included_limits2 = []
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 2))
                            tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 1))
                            no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            tmp_list.append(
                                all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 2))
                            tmp_list.append(
                                all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 1))
                            all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            tmp_list.append(
                                all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 2))
                            tmp_list.append(
                                all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 1))
                            all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 2))
                            tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 1))
                            partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                    elif int(select_vec[1]) == BODY_TYPE.index("Mirror"):
                        start_mirror = start_mirror - 1
                        tmp_list = []
                        tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                        if start_mirror > 0:
                            mirror_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                    elif int(select_vec[1]) == BODY_TYPE.index("Hole"):
                        tmp_list = []
                        if start_mirror > 0:
                            tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                            mirror_list.append(
                                Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list,
                                       [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            select_list.append(
                                Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [],
                                       [], {}))
                            # 插入混淆标识面
                            if time_to_no_shared:
                                time_to_no_shared = False
                                select_list[-1].no_shared_included = deepcopy(no_shared_included)
                                no_shared_included = []
                            if time_to_partially_shared:
                                time_to_partially_shared = False
                                select_list[-1].all_partially_included = deepcopy(partially_shared_included)
                                partially_shared_included = []
                            if time_to_all_oriented1:
                                time_to_all_oriented1 = False
                                select_list[-1].all_oriented_included['Limits1'] = deepcopy(
                                    all_oriented_included_limits1)
                                all_oriented_included_limits1 = []
                            if time_to_all_oriented2:
                                time_to_all_oriented2 = False
                                select_list[-1].all_oriented_included['Limits2'] = deepcopy(
                                    all_oriented_included_limits2)
                                all_oriented_included_limits2 = []
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            no_shared_included.append(
                                Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [],
                                       [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            all_oriented_included_limits1.append(
                                Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [],
                                       [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            all_oriented_included_limits2.append(
                                Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [],
                                       [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            partially_shared_included.append(
                                Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [],
                                       [], {}))
                    elif int(select_vec[3]) == 0:
                        tmp_list = []
                        if start_mirror > 0:
                            tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                            mirror_list.append(
                                Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]),
                                       tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            tmp_list.append(select_list.pop(select_list.__len__() - 1))
                            select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                            # 插入混淆标识面
                            if time_to_no_shared:
                                time_to_no_shared = False
                                select_list[-1].no_shared_included = deepcopy(no_shared_included)
                                no_shared_included = []
                            if time_to_partially_shared:
                                time_to_partially_shared = False
                                select_list[-1].all_partially_included = deepcopy(partially_shared_included)
                                partially_shared_included = []
                            if time_to_all_oriented1:
                                time_to_all_oriented1 = False
                                select_list[-1].all_oriented_included['Limits1'] = deepcopy(all_oriented_included_limits1)
                                all_oriented_included_limits1 = []
                            if time_to_all_oriented2:
                                time_to_all_oriented2 = False
                                select_list[-1].all_oriented_included['Limits2'] = deepcopy(all_oriented_included_limits2)
                                all_oriented_included_limits2 = []
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 1))
                            no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            tmp_list.append(
                                all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 1))
                            all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            tmp_list.append(
                                all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 1))
                            all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 1))
                            partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                    else:
                        if start_mirror > 0:
                            mirror_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [] , [], {}))
                            # 插入混淆标识面
                            if time_to_no_shared:
                                time_to_no_shared = False
                                select_list[-1].no_shared_included = deepcopy(no_shared_included)
                                no_shared_included = []
                            if time_to_partially_shared:
                                time_to_partially_shared = False
                                select_list[-1].all_partially_included = deepcopy(partially_shared_included)
                                partially_shared_included = []
                            if time_to_all_oriented1:
                                time_to_all_oriented1 = False
                                select_list[-1].all_oriented_included['Limits1'] = deepcopy(all_oriented_included_limits1)
                                all_oriented_included_limits1 = []
                            if time_to_all_oriented2:
                                time_to_all_oriented2 = False
                                select_list[-1].all_oriented_included['Limits2'] = deepcopy(all_oriented_included_limits2)
                                all_oriented_included_limits2 = []
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                elif select_vec[0] == SELECT_TYPE.index("Sub_Face"):
                    if int(select_vec[1]) == BODY_TYPE.index("Chamfer") or int(select_vec[1]) == BODY_TYPE.index("EdgeFillet"):
                        tmp_list = []
                        # 注意顺序
                        if start_mirror > 0:
                            tmp_list.append(mirror_list.pop(mirror_list.__len__() - 2))
                            tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                            mirror_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                            # 插入混淆标识面
                            if time_to_no_shared:
                                time_to_no_shared = False
                                select_list[-1].no_shared_included = deepcopy(no_shared_included)
                                no_shared_included = []
                            if time_to_partially_shared:
                                time_to_partially_shared = False
                                select_list[-1].all_partially_included = deepcopy(partially_shared_included)
                                partially_shared_included = []
                            if time_to_all_oriented1:
                                time_to_all_oriented1 = False
                                select_list[-1].all_oriented_included['Limits1'] = deepcopy(all_oriented_included_limits1)
                                all_oriented_included_limits1 = []
                            if time_to_all_oriented2:
                                time_to_all_oriented2 = False
                                select_list[-1].all_oriented_included['Limits2'] = deepcopy(all_oriented_included_limits2)
                                all_oriented_included_limits2 = []
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            tmp_list.append(select_list.pop(select_list.__len__() - 2))
                            tmp_list.append(select_list.pop(select_list.__len__() - 1))
                            select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 2))
                            tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 1))
                            no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            tmp_list.append(
                                all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 2))
                            tmp_list.append(
                                all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 1))
                            all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            tmp_list.append(
                                all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 2))
                            tmp_list.append(
                                all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 1))
                            all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 2))
                            tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 1))
                            partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                    elif int(select_vec[1]) == BODY_TYPE.index("Shell"):
                        tmp_list = []
                        # 注意顺序
                        if start_mirror > 0:
                            tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                            mirror_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                            # 插入混淆标识面
                            if time_to_no_shared:
                                time_to_no_shared = False
                                select_list[-1].no_shared_included = deepcopy(no_shared_included)
                                no_shared_included = []
                            if time_to_partially_shared:
                                time_to_partially_shared = False
                                select_list[-1].all_partially_included = deepcopy(partially_shared_included)
                                partially_shared_included = []
                            if time_to_all_oriented1:
                                time_to_all_oriented1 = False
                                select_list[-1].all_oriented_included['Limits1'] = deepcopy(all_oriented_included_limits1)
                                all_oriented_included_limits1 = []
                            if time_to_all_oriented2:
                                time_to_all_oriented2 = False
                                select_list[-1].all_oriented_included['Limits2'] = deepcopy(all_oriented_included_limits2)
                                all_oriented_included_limits2 = []
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            tmp_list.append(select_list.pop(select_list.__len__() - 1))
                            select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 1))
                            no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            tmp_list.append(
                                all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 1))
                            all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            tmp_list.append(
                                all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 1))
                            all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 1))
                            partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                    elif int(select_vec[1]) == BODY_TYPE.index("Mirror"):
                        start_mirror = start_mirror - 1
                        tmp_list = []
                        tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                        if start_mirror > 0:
                            mirror_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), tmp_list, [], {}))
                    elif int(select_vec[3]) == 0:
                        tmp_list = []
                        if start_mirror > 0:
                            tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                            mirror_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]),
                                                      int(select_vec[3]),
                                                      tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            tmp_list.append(select_list.pop(select_list.__len__() - 1))
                            select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]),
                                       tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 1))
                            no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]),
                                       tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            tmp_list.append(
                                all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 1))
                            all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]),
                                       tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            tmp_list.append(
                                all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 1))
                            all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]),
                                       tmp_list, [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 1))
                            partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]),
                                       tmp_list, [], {}))
                    else:
                        if start_mirror > 0:
                            mirror_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]),
                                                      int(select_vec[3]), [], [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                            select_list.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                            no_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                            all_oriented_included_limits1.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                            all_oriented_included_limits2.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                        elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                            partially_shared_included.append(Select(select_vec[0], select_vec[1], int(select_vec[2]), int(select_vec[3]), [], [], {}))
                elif select_vec[0] == SELECT_TYPE.index("Multiply_Face"):
                    tmp_list = []
                    if start_mirror > 0:
                        while mirror_list.__len__() > 0 and mirror_list[-1].select_type == SELECT_TYPE.index('Sub_Face'):
                            tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                        tmp_list.reverse()
                        mirror_list.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                        while select_list.__len__() > 0 and select_list[-1].select_type == SELECT_TYPE.index('Sub_Face'):
                            tmp_list.append(select_list.pop(select_list.__len__() - 1))
                        tmp_list.reverse()
                        select_list.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        # 插入混淆标识面
                        if time_to_no_shared:
                            time_to_no_shared = False
                            select_list[-1].no_shared_included = deepcopy(no_shared_included)
                            no_shared_included = []
                        if time_to_partially_shared:
                            time_to_partially_shared = False
                            select_list[-1].all_partially_included = deepcopy(partially_shared_included)
                            partially_shared_included = []
                        if time_to_all_oriented1:
                            time_to_all_oriented1 = False
                            select_list[-1].all_oriented_included['Limits1'] = deepcopy(all_oriented_included_limits1)
                            all_oriented_included_limits1 = []
                        if time_to_all_oriented2:
                            time_to_all_oriented2 = False
                            select_list[-1].all_oriented_included['Limits2'] = deepcopy(all_oriented_included_limits2)
                            all_oriented_included_limits2 = []
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                        while no_shared_included.__len__() > 0 and no_shared_included[-1].select_type == SELECT_TYPE.index('Sub_Face'):
                            tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 1))
                        tmp_list.reverse()
                        no_shared_included.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                        while all_oriented_included_limits1.__len__() > 0 and all_oriented_included_limits1[-1].select_type == SELECT_TYPE.index('Sub_Face'):
                            tmp_list.append(all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 1))
                        tmp_list.reverse()
                        all_oriented_included_limits1.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                        while all_oriented_included_limits2.__len__() > 0 and all_oriented_included_limits2[-1].select_type == SELECT_TYPE.index('Sub_Face'):
                            tmp_list.append(all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 1))
                        tmp_list.reverse()
                        all_oriented_included_limits2.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                        while partially_shared_included.__len__() > 0 and partially_shared_included[-1].select_type == SELECT_TYPE.index('Sub_Face'):
                            tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 1))
                        tmp_list.reverse()
                        partially_shared_included.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                elif select_vec[0] == SELECT_TYPE.index("Edge"):
                    tmp_list = []
                    # 注意顺序
                    if start_mirror > 0:
                        tmp_list.append(mirror_list.pop(mirror_list.__len__() - 2))
                        tmp_list.append(mirror_list.pop(mirror_list.__len__() - 1))
                        mirror_list.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 0:
                        tmp_list.append(select_list.pop(select_list.__len__() - 2))
                        tmp_list.append(select_list.pop(select_list.__len__() - 1))
                        select_list.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        # 插入混淆标识面
                        if time_to_no_shared:
                            time_to_no_shared = False
                            select_list[-1].no_shared_included = deepcopy(no_shared_included)
                            no_shared_included = []
                        if time_to_partially_shared:
                            time_to_partially_shared = False
                            select_list[-1].all_partially_included = deepcopy(partially_shared_included)
                            partially_shared_included = []
                        if time_to_all_oriented1:
                            time_to_all_oriented1 = False
                            select_list[-1].all_oriented_included['Limits1'] = deepcopy(all_oriented_included_limits1)
                            all_oriented_included_limits1 = []
                        if time_to_all_oriented2:
                            time_to_all_oriented2 = False
                            select_list[-1].all_oriented_included['Limits2'] = deepcopy(all_oriented_included_limits2)
                            all_oriented_included_limits2 = []
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 1:
                        tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 2))
                        tmp_list.append(no_shared_included.pop(no_shared_included.__len__() - 1))
                        no_shared_included.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 2:
                        all_oriented_included_limits1.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                        tmp_list.append(all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 2))
                        tmp_list.append(all_oriented_included_limits1.pop(all_oriented_included_limits1.__len__() - 1))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 3:
                        tmp_list.append(all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 2))
                        tmp_list.append(all_oriented_included_limits2.pop(all_oriented_included_limits2.__len__() - 1))
                        all_oriented_included_limits2.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
                    elif select_or_noShared_or_allOrientedLimits1_or_allOrientedLimits2 == 4:
                        tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 2))
                        tmp_list.append(partially_shared_included.pop(partially_shared_included.__len__() - 1))
                        partially_shared_included.append(Select(select_vec[0], body_type=BODY_TYPE.index('None'), operation_list=tmp_list, no_shared_included=[], all_oriented_included={}))
        return select_list[0]

class Hole(object):
    def __init__(self, point_pos, plane_ref, radius, depth, bottom_mode, sketch_plane=None, select_list=None):
        self.point_pos = point_pos
        self.plane_ref = plane_ref
        self.radius = radius
        self.depth = depth
        self.bottom_mode = bottom_mode
        self.sketch_plane = sketch_plane
        self.select_list = select_list

    def transform(self, translation, scale):
        self.sketch_plane.transform(translation, scale)
        self.depth = self.depth * scale
        self.radius = self.radius * scale
        self.point_pos = np.array(self.point_pos) * scale

    def numericalize(self, n=256):
        self.sketch_plane.numericalize(n)
        self.bottom_mode = EXTENT_TYPE.index(self.bottom_mode)
        if self.select_list is not None:
            for i in range(self.select_list.__len__()):
                self.select_list[i].numericalize(n)
        self.plane_ref.numericalize(n)
        self.depth = (self.depth + 1.0) / 2 * n
        self.depth, _ = Get_integer_and_fraction(self.depth)
        self.radius = (self.radius + 1.0) / 2 * n
        self.radius, _ = Get_integer_and_fraction(self.radius)
        self.point_pos = (np.array(self.point_pos) + 1.0) / 2 * n
        self.point_pos, _ = Get_integer_and_fraction(self.point_pos)

    def denumericalize(self, n=256):
        """de-quantize the representation."""
        self.bottom_mode = EXTENT_TYPE[int(self.bottom_mode)]
        self.depth = self.depth / n * 2 - 1.0
        self.radius = self.radius / n * 2 - 1.0
        self.point_pos = np.array(self.point_pos) / n * 2 - 1.0
        self.sketch_plane.denumericalize(n)
        if self.select_list is not None:
            for i in self.select_list:
                i.denumericalize()
        self.plane_ref.denumericalize(n)

    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        """vector representation: commands [SOL, ..., SOL, ..., EXT]"""
        assert vec[-1][0] == HOLE_IDX and vec[0][0] == TOPO_IDX
        select_vec = np.concatenate([vec[:-1], EOS_VEC[np.newaxis]])[:-1]
        hole_vec = vec[-1][1 + N_ARGS_SKETCH + N_ARGS_EXT + 6:1 + N_ARGS_SKETCH + N_ARGS_EXT + 9]
        point_pos = vec[-1][1:1 + 2]
        sketch_plane = CoordSystem.from_vector(vec[-1][1 + N_ARGS_SKETCH: 1 + N_ARGS_SKETCH + 6])
        select_commands = select_vec[:, 0]
        select_indices = np.where(select_commands == TOPO_IDX)[0].tolist()
        select_list = []
        for i in range(select_indices.__len__() - 1):
            select_list.append(Select.to_select(select_vec[select_indices[i]:select_indices[i+1]]))
        select_list.append(Select.to_select(select_vec[select_indices[-1]:]))

        plane_ref = select_list[0]
        if len(select_list) > 1:
            select_list = select_list[1:]
        else:
            select_list = None
        hole = Hole(point_pos, deepcopy(plane_ref), hole_vec[0], hole_vec[1], hole_vec[2], sketch_plane, deepcopy(select_list))

        if is_numerical:
            hole.denumericalize(n)
        return hole

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True):
        all_vec = self.plane_ref.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True)
        if self.select_list is not None:
            for select in self.select_list:
                select_vec = select.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True)
                # 转np.array
                select_vec_np = np.array(select_vec)
                all_vec = np.concatenate([all_vec, select_vec_np], axis=0)
        plane_vec = self.sketch_plane.to_vector()
        vec = np.array([np.array([HOLE_IDX, self.point_pos[0], self.point_pos[1], -1, -1, -1, plane_vec[3], plane_vec[4], plane_vec[5], plane_vec[0], plane_vec[1], plane_vec[2], *[PAD_VAL]*(N_ARGS_EXT - 6), -1, -1, -1, -1, -1, -1, self.radius, self.depth, self.bottom_mode, *[PAD_VAL] * N_ARGS_SELECT_PARAM])])
        return np.concatenate([all_vec, vec], axis=0)


class Chamfer(object):
    def __init__(self, select_list, length1, angle_or_length2, mode='catTwoLengthChamfer', propagation='catTangencyChamfer', orientation='catNoReverseChamfer'):
        self.select_list = select_list
        self.length1 = length1
        self.angle_or_length2 = angle_or_length2
        self.propagation = propagation
        self.mode = mode
        self.orientation = orientation

    def transform(self, translation, scale):
        # 将angle_or_length2转化为length2
        if self.mode == 'catLengthAngleChamfer':
            self.angle_or_length2 = np.tan(self.angle_or_length2 / 180 * np.pi) * self.length1
            self.mode = 'catTwoLengthChamfer'
        self.length1 = self.length1 * scale
        self.angle_or_length2 = self.angle_or_length2 * scale

    def numericalize(self, n=256):
        for i in range(self.select_list.__len__()):
            self.select_list[i].numericalize(n)
        self.length1 = (self.length1 + 1.0) / 2 * n
        self.length1, _ = Get_integer_and_fraction(self.length1)
        self.angle_or_length2 = (self.angle_or_length2 + 1.0) / 2 * n
        self.angle_or_length2, _ = Get_integer_and_fraction(self.angle_or_length2)

    def denumericalize(self, n=256):
        """de-quantize the representation."""
        self.length1 = self.length1 / n * 2 - 1.0
        self.angle_or_length2 = self.angle_or_length2 / n * 2 - 1.0
        for i in self.select_list:
            i.denumericalize()

    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        """vector representation: commands [SOL, ..., SOL, ..., EXT]"""
        assert vec[-1][0] == CHAMFER_IDX and vec[0][0] == TOPO_IDX
        select_vec = np.concatenate([vec[:-1], EOS_VEC[np.newaxis]])[:-1]
        chamfer_vec = vec[-1][1 + N_ARGS_SKETCH + N_ARGS_EXT + 2:1 + N_ARGS_SKETCH + N_ARGS_EXT + 4]

        select_commands = select_vec[:, 0]
        select_indices = np.where(select_commands == TOPO_IDX)[0].tolist()
        select_list = []
        for i in range(select_indices.__len__() - 1):
            select_list.append(Select.to_select(select_vec[select_indices[i]:select_indices[i+1]]))
        select_list.append(Select.to_select(select_vec[select_indices[-1]:]))

        chamfer = Chamfer(select_list, chamfer_vec[0], chamfer_vec[1])

        if is_numerical:
            chamfer.denumericalize(n)
        return chamfer

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True):
        all_vec = None
        for select in self.select_list:
            select_vec = select.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True)
            # 转np.array
            select_vec_np = np.array(select_vec)
            if all_vec is None:
                all_vec = select_vec_np
            else:
                all_vec = np.concatenate([all_vec, select_vec_np], axis=0)
        vec = np.array([np.array([CHAMFER_IDX, *[PAD_VAL] * (N_ARGS_SKETCH + N_ARGS_EXT),
                        -1, -1, max(129, self.length1), max(129, self.angle_or_length2), -1, -1, -1, -1, -1, *[PAD_VAL] * N_ARGS_SELECT_PARAM])])
        return np.concatenate([all_vec, vec], axis=0)

class Fillet(object):
    def __init__(self, select_list, radius, edgePropagation=None):
        self.select_list = select_list
        self.radius = radius
        self.edgePropagation = edgePropagation

    def transform(self, translation, scale):
        self.radius = self.radius * scale

    def numericalize(self, n=256):
        for i in self.select_list:
            i.numericalize(n)
        self.radius = (self.radius + 1.0) / 2 * n
        self.radius, _ = Get_integer_and_fraction(self.radius)

    def denumericalize(self, n=256):
        """de-quantize the representation."""
        self.radius = self.radius / n * 2 - 1.0
        for i in self.select_list:
            i.denumericalize()

    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        """vector representation: commands [SOL, ..., SOL, ..., EXT]"""
        assert vec[-1][0] == FILLET_IDX and vec[0][0] == TOPO_IDX
        select_vec = np.concatenate([vec[:-1], EOS_VEC[np.newaxis]])[:-1]
        fillet_radius = vec[-1][1 + N_ARGS_SKETCH + N_ARGS_EXT + 4]

        select_commands = select_vec[:, 0]
        select_indices = np.where(select_commands == TOPO_IDX)[0].tolist()
        select_list = []
        for i in range(select_indices.__len__() - 1):
            select_list.append(Select.to_select(select_vec[select_indices[i]:select_indices[i + 1]]))
        select_list.append(Select.to_select(select_vec[select_indices[-1]:]))

        fillet = Fillet(select_list, fillet_radius)

        if is_numerical:
            fillet.denumericalize(n)
        return fillet

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True):
        all_vec = None
        for select in self.select_list:
            select_vec = select.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True)
            # 转np.array
            select_vec_np = np.array(select_vec)
            if all_vec is None:
                all_vec = select_vec_np
            else:
                all_vec = np.concatenate([all_vec, select_vec_np], axis=0)
        vec = np.array([np.array([FILLET_IDX, *[PAD_VAL]*(N_ARGS_SKETCH + N_ARGS_EXT), -1, -1, -1, -1, max(129, self.radius), -1, -1, -1, -1, *[PAD_VAL]*N_ARGS_SELECT_PARAM])])
        return np.concatenate([all_vec, vec], axis=0)

class Shell(object):
    def __init__(self, select_list, thickness, second_thickness):
        self.select_list = select_list
        self.thickness = thickness
        self.second_thickness = second_thickness

    def transform(self, translation, scale):
        self.thickness = self.thickness * scale
        self.second_thickness = self.second_thickness * scale

    def numericalize(self, n=256):
        for i in range(self.select_list.__len__()):
            self.select_list[i].numericalize(n)
        self.thickness = (self.thickness + 1.0) / 2 * n
        self.second_thickness = (self.second_thickness + 1.0) / 2 * n
        self.thickness, _ = Get_integer_and_fraction(self.thickness)
        self.second_thickness, _ = Get_integer_and_fraction(self.second_thickness)

    def denumericalize(self, n=256):
        self.thickness = self.thickness / n * 2 - 1.0
        self.second_thickness = self.second_thickness / n * 2 - 1.0
        for i in self.select_list:
            i.denumericalize()

    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        """vector representation: commands [SOL, ..., SOL, ..., EXT]"""
        assert vec[-1][0] == SHELL_IDX and vec[0][0] == TOPO_IDX
        select_vec = np.concatenate([vec[:-1], EOS_VEC[np.newaxis]])[:-1]
        shell_vec = vec[-1][1 + N_ARGS_SKETCH + N_ARGS_EXT:1 + N_ARGS_SKETCH + N_ARGS_EXT + 2]

        select_commands = select_vec[:, 0]
        select_indices = np.where(select_commands == TOPO_IDX)[0].tolist()
        select_list = []
        for i in range(select_indices.__len__() - 1):
            select_list.append(Select.to_select(select_vec[select_indices[i]:select_indices[i + 1]]))
        select_list.append(Select.to_select(select_vec[select_indices[-1]:]))

        shell = Shell(select_list, shell_vec[0], shell_vec[1])

        if is_numerical:
            shell.denumericalize(n)
        return shell

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True):
        all_vec = None
        for select in self.select_list:
            select_vec = select.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True)
            # 转np.array
            select_vec_np = np.array(select_vec)
            if all_vec is None:
                all_vec = select_vec_np
            else:
                all_vec = np.concatenate([all_vec, select_vec_np], axis=0)
        vec = np.array([np.array([SHELL_IDX, *[PAD_VAL]*(N_ARGS_SKETCH + N_ARGS_EXT), max(129, self.thickness), self.second_thickness,
                                  -1, -1, -1, -1, -1, -1, -1, *[PAD_VAL]*N_ARGS_SELECT_PARAM])])
        return np.concatenate([all_vec, vec], axis=0)

class Macro_Seq(object):
    def __init__(self, extrude_operation, bounding_size=0):
        self.bounding_size = bounding_size
        self.extrude_operation = extrude_operation

    def normalize(self, size=1.0):
        """(1)normalize the shape into unit cube (-1~1). """
        scale = size * NORM_FACTOR / np.abs(self.bounding_size)
        self.transform(0.0, scale)

    def transform(self, translation, scale):
        """linear transformation"""
        # 拉伸/旋转体
        for item in self.extrude_operation:
            item.transform(translation, scale)

    def numericalize(self, n=256):
        for item in self.extrude_operation:
            item.numericalize(n)

    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        commands = vec[:, 0]
        operation_indices = [-1] + np.where((commands == EXT_IDX) | (commands == REV_IDX) |
                                            (commands == SHELL_IDX) | (commands == CHAMFER_IDX) |
                                            (commands == FILLET_IDX) | (commands == POCKET_IDX) |
                                            (commands == DRAFT_IDX) | (commands == MIRROR_IDX) |
                                            (commands == GROOVE_IDX) | (commands == HOLE_IDX))[0].tolist()
        operation_seq = []
        for i in range(len(operation_indices) - 1):
            start, end = operation_indices[i], operation_indices[i + 1]
            if commands[end] == EXT_IDX:
                operation_seq.append(Extrude.from_vector(vec[start+1:end+1], is_numerical, n))
            elif commands[end] == POCKET_IDX:
                operation_seq.append(Pocket.from_vector(vec[start+1:end+1], is_numerical, n))
            elif commands[end] == REV_IDX:
                operation_seq.append(Revolve.from_vector(vec[start+1:end+1], is_numerical, n))
            elif commands[end] == GROOVE_IDX:
                operation_seq.append(Groove.from_vector(vec[start + 1:end + 1], is_numerical, n))
            elif commands[end] == SHELL_IDX:
                operation_seq.append(Shell.from_vector(vec[start+1:end+1], is_numerical, n))
            elif commands[end] == CHAMFER_IDX:
                operation_seq.append(Chamfer.from_vector(vec[start + 1:end + 1], is_numerical, n))
            elif commands[end] == FILLET_IDX:
                operation_seq.append(Fillet.from_vector(vec[start + 1:end + 1], is_numerical, n))
            elif commands[end] == DRAFT_IDX:
                operation_seq.append(Draft.from_vector(vec[start + 1:end + 1], is_numerical, n))
            elif commands[end] == MIRROR_IDX:
                operation_seq.append(Mirror.from_vector(vec[start + 1:end + 1], is_numerical, n))
            elif commands[end] == HOLE_IDX:
                operation_seq.append(Hole.from_vector(vec[start + 1:end + 1], is_numerical, n))
        cad_seq = Macro_Seq(operation_seq)
        return cad_seq

    def to_vector(self, max_n_ext=10, max_n_loops=6, max_len_loop=15, max_total_len=60, pad=False):
        vec_sequence = []

        for item in self.extrude_operation:
            vec = item.to_vector(max_n_loops, max_len_loop, pad=False)
            if vec is None:
                return None
            vec_sequence.append(vec)

        vec_sequence = np.concatenate(vec_sequence, axis=0)
        vec_sequence = np.concatenate([vec_sequence, EOS_VEC[np.newaxis]], axis=0)

        # add EOS padding
        if pad and vec_sequence.shape[0] < max_total_len:
            pad_len = max_total_len - vec_sequence.shape[0]
            vec_sequence = np.concatenate([vec_sequence, EOS_VEC[np.newaxis].repeat(pad_len, axis=0)], axis=0)
        return vec_sequence

class Draft(object):
    def __init__(self, FaceToDraft, Neutral, Parting, dir, Angle, NeutralMode=0, Mode=0, MultiselectionMode=0):
        self.select_list = FaceToDraft
        self.Neutral = Neutral
        self.NeutralMode = NeutralMode
        self.Parting = Parting
        self.dir = dir
        self.Mode = Mode
        self.DraftAngle = Angle
        self.MultiselectionMode = MultiselectionMode

    def transform(self, translation, scale):
        return

    def numericalize(self, n=256):
        for i in range(self.select_list.__len__()):
            self.select_list[i].numericalize(n)
        self.Neutral.numericalize(n)

        self.Mode = CatDraftMode.index(self.Mode)
        self.MultiselectionMode = CatDraftMultiselectionMode.index(self.MultiselectionMode)
        self.NeutralMode = CatDraftNeutralPropagationMode.index(self.NeutralMode)
        self.DraftAngle = ((self.DraftAngle / 90.0) + 1) / 2 * n
        # dir的长度固定为1，因此不需要归一化
        self.dir = (np.array(self.dir) + 1) / 2 * n
        self.DraftAngle, _ = Get_integer_and_fraction(self.DraftAngle)
        self.dir, _ = Get_integer_and_fraction(self.dir)

    def denumericalize(self, n=256):
        self.dir = self.dir * 2.0 / n - 1.0
        self.DraftAngle = (self.DraftAngle / n * 2 - 1) * 90
        self.Mode = CatDraftMode[int(self.Mode)]
        self.MultiselectionMode = CatDraftMultiselectionMode[int(self.MultiselectionMode)]
        self.NeutralMode = CatDraftNeutralPropagationMode[int(self.NeutralMode)]
        for i in self.select_list:
            i.denumericalize()
        self.Neutral.denumericalize()

    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        """vector representation: commands [SOL, ..., SOL, ..., EXT]"""
        assert vec[-1][0] == DRAFT_IDX and vec[0][0] == TOPO_IDX
        select_vec = np.concatenate([vec[:-1], EOS_VEC[np.newaxis]])[:-1]
        draft_angle = vec[-1][1 + N_ARGS_SKETCH + N_ARGS_EXT + 5]
        dir = vec[-1][1 + N_ARGS_SKETCH:1 + N_ARGS_SKETCH + 3]

        select_commands = select_vec[:, 0]
        select_indices = np.where(select_commands == TOPO_IDX)[0].tolist()
        select_list = []
        for i in range(select_indices.__len__() - 1):
            select_list.append(Select.to_select(select_vec[select_indices[i]:select_indices[i + 1]]))
        select_list.append(Select.to_select(select_vec[select_indices[-1]:]))
        neutral_face = deepcopy(select_list[0])
        select_list = select_list[1:]
        draft = Draft(select_list, neutral_face, neutral_face, dir, draft_angle)

        if is_numerical:
            draft.denumericalize(n)
        return draft

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True):
        neutral_vec = self.Neutral.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True)
        all_vec = np.array(neutral_vec)
        for select in self.select_list:
            select_vec = select.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True)
            # 转np.array
            select_vec_np = np.array(select_vec)
            all_vec = np.concatenate([all_vec, select_vec_np], axis=0)
        vec = np.array([np.array([DRAFT_IDX, *[PAD_VAL] * N_ARGS_SKETCH, self.dir[0], self.dir[1], self.dir[2], *[PAD_VAL] * (N_ARGS_EXT - 3), -1, -1, -1, -1, -1, self.DraftAngle, -1, -1, -1, *[PAD_VAL] * N_ARGS_SELECT_PARAM])])
        return np.concatenate([all_vec, vec], axis=0)

class Mirror(object):
    def __init__(self, select_list):
        self.select_list = select_list

    def transform(self, translation, scale):
        return

    def numericalize(self, n=256):
        for i in self.select_list:
            i.numericalize(n)

    def denumericalize(self, n=256):
        for i in self.select_list:
            i.denumericalize()
        return

    @staticmethod
    def from_vector(vec, is_numerical=False, n=256):
        """vector representation: commands [SOL, ..., SOL, ..., EXT]"""
        assert vec[-1][0] == MIRROR_IDX and vec[0][0] == TOPO_IDX
        select_vec = np.concatenate([vec[:-1], EOS_VEC[np.newaxis]])[:-1]
        select_commands = select_vec[:, 0]
        select_indices = np.where(select_commands == TOPO_IDX)[0].tolist()
        select_list = []
        for i in range(select_indices.__len__() - 1):
            select_list.append(Select.to_select(select_vec[select_indices[i]:select_indices[i + 1]]))
        select_list.append(Select.to_select(select_vec[select_indices[-1]:]))

        mirror = Mirror(select_list)

        if is_numerical:
            mirror.denumericalize(n)
        return mirror

    def to_vector(self, max_n_loops=6, max_len_loop=15, pad=True):
        all_vec = None
        for select in self.select_list:
            select_vec = select.to_vector(max_n_loops, max_len_loop, pad=False, is_last=True)
            # 转np.array
            select_vec_np = np.array(select_vec)
            if all_vec is None:
                all_vec = select_vec_np
            else:
                all_vec = np.concatenate([all_vec, select_vec_np], axis=0)
        vec = np.array([np.array([MIRROR_IDX, *[PAD_VAL]*N_ARGS])])
        return np.concatenate([all_vec, vec], axis=0)

# 记录哪一body对应什么属性设置为lengthx，在后面修改的时候可以找到并修改
class Parameter(object):
    def __init__(self, body_point, op_point, para_type, curve_point=0, sketch_name=''):
        self.body_point = body_point
        self.op_point = op_point
        self.para_type = para_type
        self.curve_point = curve_point
        self.sketch_name = sketch_name

# 囊括草图的一系列信息，包括sketch_plane, sketch_pos, sketch_size, sketch_profile
class Sketch(object):
    def __init__(self, sketch_curves, sketch_plane, sketch_no=0, sketch_pos=None, sketch_size=None, sketch_profile=None):
        self.sketch_curves = sketch_curves
        self.sketch_plane = sketch_plane
        self.sketch_no = sketch_no
        self.sketch_pos = sketch_pos
        self.sketch_size = sketch_size
        self.sketch_profile = sketch_profile