import os
from copy import copy
from copy import deepcopy
import numpy as np
import glob
import json
import h5py

from cadlib.CAD_Class import *
from cadlib.Geometry_utils import *
import trimesh
from trimesh.sample import sample_surface
NORM_FACTOR = 0.75

def get_plane(measurable, catia):

    vba_function_name = 'get_plane'
    vba_function = 'GetPlane'
    vba_code = f'''        
    Public Function {vba_function_name}(measurable)
        Dim Components (8)
        measurable.{vba_function} Components
        {vba_function_name} = Components
    End Function
    '''

    system_service = catia.SystemService
    return system_service.evaluate(vba_code, 0, vba_function_name, [measurable])


def create_pocket_CATIA(part, extrude_op, cur_body_ref=None, sketch_offset=0, hole_sketch_map=None):
    profile = deepcopy(extrude_op.sketch_profile)  # use copy to prevent changing extrude_op internally
    profile.denormalize(extrude_op.sketch_size, size=ARGS_N)

    sketch_plane = deepcopy(extrude_op.sketch_plane)
    sketch_plane.origin = extrude_op.sketch_pos
    body = part.bodies.Item(1)
    # 获得草图平面
    hybridShapeFactory = part.HybridShapeFactory
    # 两点，前者为原点，后者用于构成法线
    hybridShapePointCoord1 = hybridShapeFactory.AddNewPointCoord(sketch_plane.origin[0], sketch_plane.origin[1], sketch_plane.origin[2])
    hybridShapePointCoord2 = hybridShapeFactory.AddNewPointCoord(sketch_plane.origin[0] + sketch_plane.normal[0], sketch_plane.origin[1] + sketch_plane.normal[1], sketch_plane.origin[2] + sketch_plane.normal[2])
    reference1 = part.CreateReferenceFromObject(hybridShapePointCoord1)
    reference2 = part.CreateReferenceFromObject(hybridShapePointCoord2)
    # 法线
    hybridShapeLinePtPt = hybridShapeFactory.AddNewLinePtPt(reference1, reference2)
    reference3 = part.CreateReferenceFromObject(hybridShapeLinePtPt)
    reference4 = part.CreateReferenceFromObject(hybridShapePointCoord1)
    hybridShapePlaneNormal = hybridShapeFactory.AddNewPlaneNormal(reference3, reference4)

    # 添加定义平面上的草图
    body.InsertHybridShape(hybridShapePlaneNormal)
    reference5 = body.HybridShapes.Item(body.HybridShapes.Count)
    sketch = body.sketches.add(reference5)
    sketch.SetAbsoluteAxisData(tuple([sketch_plane.origin[0], sketch_plane.origin[1], sketch_plane.origin[2],
                                      sketch_plane.x_axis[0], sketch_plane.x_axis[1], sketch_plane.x_axis[2],
                                      sketch_plane.y_axis[0], sketch_plane.y_axis[1], sketch_plane.y_axis[2]]))
    part.InWorkObject = sketch
    arrayOfObject = []
    shapefactory = sketch.openedition()
    report_count = 0
    for loop in profile.children:
        start_point = None
        end_point = None
        start_point_coordinate = []
        for curve in loop.children:
            """create a 3D edge"""
            if isinstance(curve, Line):
                cur_line = shapefactory.createline(curve.start_point[0], curve.start_point[1],
                                               curve.end_point[0], curve.end_point[1])
                report_count = report_count + 1
                cur_line.ReportName = report_count
                # 判断是否为开始
                if start_point == None:
                    start_point = shapefactory.CreatePoint(curve.start_point[0], curve.start_point[1])
                    end_point = start_point
                    start_point_coordinate = [curve.start_point[0], curve.start_point[1]]
                # 判断是否为最后一条线：
                elif np.allclose(start_point_coordinate, curve.end_point):
                    cur_line.StartPoint = end_point
                    cur_line.EndPoint = start_point
                    continue
                cur_point = shapefactory.CreatePoint(curve.end_point[0], curve.end_point[1])
                cur_line.StartPoint = end_point
                cur_line.EndPoint = cur_point
                end_point = cur_point
                part.update()
            elif isinstance(curve, Circle):
                cur_circle = shapefactory.createclosedcircle(curve.center[0], curve.center[1], curve.radius)
                report_count = report_count + 1
                cur_circle.ReportName = report_count
                part.update()
            elif isinstance(curve, Arc):
                # 记录开始点和结束点是否颠倒
                flag = False
                angle_start = vec2arc(curve.start_point, curve.center)
                angle_end = vec2arc(curve.end_point, curve.center)
                angle_mid = vec2arc(curve.mid_point, curve.center)
                # 比较两端弧度，重新将小的作为start
                if angle_start > angle_end:
                    angle_start, angle_end = angle_end, angle_start
                    flag = not flag
                if angle_mid < angle_start or angle_mid > angle_end:
                    angle_start, angle_end = angle_end, angle_start
                    flag = not flag
                cur_arc = shapefactory.createcircle(curve.center[0], curve.center[1], curve.radius, angle_start, angle_end)
                report_count = report_count + 1
                cur_arc.ReportName = report_count

                start_point_temp = [curve.start_point[0], curve.start_point[1]]
                end_point_temp = [curve.end_point[0], curve.end_point[1]]

                # 判断是否为开始
                if start_point == None:
                    start_point = shapefactory.CreatePoint(start_point_temp[0], start_point_temp[1])
                    end_point = start_point
                    start_point_coordinate = [start_point_temp[0], start_point_temp[1]]
                # 判断是否为最后一条线：
                elif np.allclose(start_point_coordinate, end_point_temp):
                    if flag:
                        cur_arc.StartPoint = start_point
                        cur_arc.EndPoint = end_point
                    else:
                        cur_arc.StartPoint = end_point
                        cur_arc.EndPoint = start_point
                    continue
                cur_point = shapefactory.CreatePoint(end_point_temp[0], end_point_temp[1])
                if flag:
                    cur_arc.StartPoint = cur_point
                    cur_arc.EndPoint = end_point
                else:
                    cur_arc.StartPoint = end_point
                    cur_arc.EndPoint = cur_point
                end_point = cur_point

                part.update()
            elif isinstance(curve, Spline):
                for i in curve.point_list:
                    arrayOfObject.append(shapefactory.CreateControlPoint(i[0], i[1]))
                cur_spline = shapefactory.CreateSpline(arrayOfObject)
                report_count = report_count + 1
                cur_spline.ReportName = report_count
                if not np.allclose(curve.point_list[0], curve.point_list[curve.point_list.__len__() - 1]):
                    # 判断是否为开始
                    if start_point == None:
                        start_point = arrayOfObject[0]
                        end_point = start_point
                        start_point_coordinate = curve.start_point
                    # 判断是否为最后一条线：
                    elif np.allclose(start_point_coordinate, curve.end_point):
                        cur_spline.StartPoint = end_point
                        cur_spline.EndPoint = start_point
                        arrayOfObject = []
                        continue
                    cur_point = arrayOfObject[arrayOfObject.__len__() - 1]
                    cur_spline.StartPoint = end_point
                    cur_spline.EndPoint = cur_point
                    end_point = cur_point
                part.update()
                arrayOfObject = []
    sketch.closeedition()

    if isinstance(extrude_op, Pocket):
        # 对称拉伸
        if extrude_op.extent_type1 == "OffsetLimit":
            pad = part.shapefactory.AddNewPocket(sketch, extrude_op.extent_one)
        if extrude_op.extent_type1 == "UpToNextLimit":
            pad = part.shapefactory.AddNewPocket(sketch, extrude_op.extent_one)
            first_limit = pad.FirstLimit
            first_limit.LimitMode = 1
        if extrude_op.extent_type1 == "UpToLastLimit":
            pad = part.shapefactory.AddNewPocket(sketch, extrude_op.extent_one)
            first_limit = pad.FirstLimit
            first_limit.LimitMode = 2
        # UpToPlaneLimit
        if extrude_op.extent_type1 == "UpToPlaneLimit":
            pad = part.shapefactory.AddNewPocket(sketch, extrude_op.extent_one)
            first_limit = pad.FirstLimit
            first_limit.LimitMode = 3
            if extrude_op.select_list[0].body_type == 'OriginElements':
                originElements1 = part.OriginElements
                if extrude_op.select_list[0].no == 1:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneXY)
                elif extrude_op.select_list[0].no == 2:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneYZ)
                else:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneZX)
            else:
                plane_name = parse_BrepName(extrude_op.select_list[0], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
                plane_name = 'FSur:(' + plane_name + ';WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)'
                plane_ref = part.CreateReferenceFromBRepName(plane_name, cur_body_ref)
            first_limit.LimitingElement = plane_ref
        # UpToSurfaceLimit
        if extrude_op.extent_type1 == "UpToSurfaceLimit":
            pad = part.shapefactory.AddNewPocket(sketch, extrude_op.extent_one)
            first_limit = pad.FirstLimit
            first_limit.LimitMode = 4
            if extrude_op.select_list[0].body_type == 'OriginElements':
                originElements1 = part.OriginElements
                if extrude_op.select_list[0].no == 1:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneXY)
                elif extrude_op.select_list[0].no == 2:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneYZ)
                else:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneZX)
            else:
                plane_name = parse_BrepName(extrude_op.select_list[0], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
                plane_name = 'RSur:(' + plane_name + ';WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)'
                plane_ref = part.CreateReferenceFromBRepName(plane_name, cur_body_ref)
            first_limit.LimitingElement = plane_ref
        # UpThruNextLimit
        if extrude_op.extent_type1 == "UpThruNextLimit":
            pad = part.shapefactory.AddNewPocket(sketch, extrude_op.extent_one)
            first_limit = pad.FirstLimit
            first_limit.LimitMode = 5

        limit = pad.SecondLimit
        length = limit.Dimension
        length.Value = extrude_op.extent_two

        if extrude_op.isInverse:
            pad.DirectionOrientation = 0
        cur_body_ref = pad
    else:
        angle = extrude_op.angle_one
        second_angle = extrude_op.angle_two
        if angle > second_angle:
            second_angle = -second_angle
        else:
            second_angle = 360 - second_angle
        if extrude_op.select_list[0].body_type == 'OriginElements':
            geometricElements1 = sketch.GeometricElements
            axis2D1 = geometricElements1.Item("AbsoluteAxis")
            if extrude_op.select_list[0].no == 1:
                revolve_axis_ref = axis2D1.GetItem("HDirection")
            else:
                revolve_axis_ref = axis2D1.GetItem("VDirection")
        else:
            revolve_axis_name = parse_BrepName(extrude_op.select_list[0], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            revolve_axis_name = 'WireREdge:(' + revolve_axis_name + ';WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)'
            revolve_axis_ref = part.CreateReferenceFromBRepName(revolve_axis_name, sketch)

        groove = part.shapefactory.AddNewGroove(sketch)
        groove.FirstAngle.Value = angle
        groove.SecondAngle.Value = second_angle
        groove.RevoluteAxis = revolve_axis_ref
        cur_body_ref = groove

    part.update()
    return cur_body_ref

def create_Body_CATIA(part, extrude_op, cur_body_ref=None, is_first=False, sketch_offset=0, hole_sketch_map=None):

    profile = deepcopy(extrude_op.sketch_profile)  # use copy to prevent changing extrude_op internally
    profile.denormalize(extrude_op.sketch_size, size=ARGS_N)

    sketch_plane = deepcopy(extrude_op.sketch_plane)
    sketch_plane.origin = extrude_op.sketch_pos

    if is_first:
        body = part.bodies.Item(1)
    else:
        body = part.bodies.add()

    # 获得草图平面
    hybridShapeFactory = part.HybridShapeFactory
    # 两点，前者为原点，后者用于构成法线
    hybridShapePointCoord1 = hybridShapeFactory.AddNewPointCoord(sketch_plane.origin[0], sketch_plane.origin[1], sketch_plane.origin[2])
    hybridShapePointCoord2 = hybridShapeFactory.AddNewPointCoord(sketch_plane.origin[0] + sketch_plane.normal[0], sketch_plane.origin[1] + sketch_plane.normal[1], sketch_plane.origin[2] + sketch_plane.normal[2])
    reference1 = part.CreateReferenceFromObject(hybridShapePointCoord1)
    reference2 = part.CreateReferenceFromObject(hybridShapePointCoord2)
    # 法线
    hybridShapeLinePtPt = hybridShapeFactory.AddNewLinePtPt(reference1, reference2)
    reference3 = part.CreateReferenceFromObject(hybridShapeLinePtPt)
    reference4 = part.CreateReferenceFromObject(hybridShapePointCoord1)
    hybridShapePlaneNormal = hybridShapeFactory.AddNewPlaneNormal(reference3, reference4)
    # hybridShapePlaneNormal.name = plane_name

    # 添加定义平面上的草图
    body.InsertHybridShape(hybridShapePlaneNormal)
    # reference5 = body.HybridShapes.Item(plane_name)
    reference5 = body.HybridShapes.Item(body.HybridShapes.Count)
    sketch = body.sketches.add(reference5)

    sketch.SetAbsoluteAxisData(tuple([sketch_plane.origin[0], sketch_plane.origin[1], sketch_plane.origin[2],
                                      sketch_plane.x_axis[0], sketch_plane.x_axis[1], sketch_plane.x_axis[2],
                                      sketch_plane.y_axis[0], sketch_plane.y_axis[1], sketch_plane.y_axis[2]]))

    part.InWorkObject = sketch
    arrayOfObject = []
    shapefactory = sketch.openedition()
    report_count = 0
    for loop in profile.children:
        start_point = None
        end_point = None
        start_point_coordinate = []
        for curve in loop.children:
            """create a 3D edge"""
            if isinstance(curve, Line):
                cur_line = shapefactory.createline(curve.start_point[0], curve.start_point[1],
                                               curve.end_point[0], curve.end_point[1])
                report_count = report_count + 1
                cur_line.ReportName = report_count
                # 判断是否为开始
                if start_point == None:
                    start_point = shapefactory.CreatePoint(curve.start_point[0], curve.start_point[1])
                    end_point = start_point
                    start_point_coordinate = [curve.start_point[0], curve.start_point[1]]
                # 判断是否为最后一条线：
                elif np.allclose(start_point_coordinate, curve.end_point):
                    cur_line.StartPoint = end_point
                    cur_line.EndPoint = start_point
                    continue
                cur_point = shapefactory.CreatePoint(curve.end_point[0], curve.end_point[1])
                cur_line.StartPoint = end_point
                cur_line.EndPoint = cur_point
                end_point = cur_point
                part.update()
            elif isinstance(curve, Circle):
                cur_circle = shapefactory.createclosedcircle(curve.center[0], curve.center[1], curve.radius)
                report_count = report_count + 1
                cur_circle.ReportName = report_count
                part.update()
            elif isinstance(curve, Arc):
                # 记录开始点和结束点是否颠倒
                flag = False
                angle_start = vec2arc(curve.start_point, curve.center)
                angle_end = vec2arc(curve.end_point, curve.center)
                angle_mid = curve.mid_arc % (2 * np.pi)
                # 比较两端弧度，重新将小的作为start
                if angle_start > angle_end:
                    angle_start, angle_end = angle_end, angle_start
                    flag = not flag
                if angle_mid < angle_start or angle_mid > angle_end:
                    angle_start, angle_end = angle_end, angle_start
                    flag = not flag
                cur_arc = shapefactory.createcircle(curve.center[0], curve.center[1], curve.radius, angle_start, angle_end)
                report_count = report_count + 1
                cur_arc.ReportName = report_count

                start_point_temp = [curve.start_point[0], curve.start_point[1]]
                end_point_temp = [curve.end_point[0], curve.end_point[1]]

                # 判断是否为开始
                if start_point == None:
                    start_point = shapefactory.CreatePoint(start_point_temp[0], start_point_temp[1])
                    end_point = start_point
                    start_point_coordinate = [start_point_temp[0], start_point_temp[1]]
                # 判断是否为最后一条线：
                elif np.allclose(start_point_coordinate, end_point_temp):
                    if flag:
                        cur_arc.StartPoint = start_point
                        cur_arc.EndPoint = end_point
                    else:
                        cur_arc.StartPoint = end_point
                        cur_arc.EndPoint = start_point
                    continue
                cur_point = shapefactory.CreatePoint(end_point_temp[0], end_point_temp[1])
                if flag:
                    cur_arc.StartPoint = cur_point
                    cur_arc.EndPoint = end_point
                else:
                    cur_arc.StartPoint = end_point
                    cur_arc.EndPoint = cur_point
                end_point = cur_point

                part.update()
            elif isinstance(curve, Spline):
                for i in curve.point_list:
                    arrayOfObject.append(shapefactory.CreateControlPoint(i[0], i[1]))
                cur_spline = shapefactory.CreateSpline(arrayOfObject)
                report_count = report_count + 1
                cur_spline.ReportName = report_count
                if not np.allclose(curve.point_list[0], curve.point_list[curve.point_list.__len__() - 1]):
                    # 判断是否为开始
                    if start_point == None:
                        start_point = arrayOfObject[0]
                        end_point = start_point
                        start_point_coordinate = curve.start_point
                    # 判断是否为最后一条线：
                    elif np.allclose(start_point_coordinate, curve.end_point):
                        cur_spline.StartPoint = end_point
                        cur_spline.EndPoint = start_point
                        arrayOfObject = []
                        continue
                    cur_point = arrayOfObject[arrayOfObject.__len__() - 1]
                    cur_spline.StartPoint = end_point
                    cur_spline.EndPoint = cur_point
                    end_point = cur_point
                part.update()
                arrayOfObject = []
    sketch.closeedition()

    if isinstance(extrude_op, Extrude):
        # 对称拉伸
        if extrude_op.extent_type1 == "OffsetLimit":
            pad = part.shapefactory.AddNewPad(sketch, extrude_op.extent_one)

            # reference_extrude_dir = part.CreateReferenceFromObject(hybridShapeLinePtPt)
            # pad.SetDirection(reference_extrude_dir)
        # UpToNextLimit
        if extrude_op.extent_type1 == "UpToNextLimit":
            pad = part.shapefactory.AddNewPad(sketch, extrude_op.extent_one)

            first_limit = pad.FirstLimit
            first_limit.LimitMode = 1
        # UpToLastLimit
        if extrude_op.extent_type1 == "UpToLastLimit":
            pad = part.shapefactory.AddNewPad(sketch, extrude_op.extent_one)

            first_limit = pad.FirstLimit
            first_limit.LimitMode = 2
        # UpToPlaneLimit
        if extrude_op.extent_type1 == "UpToPlaneLimit":
            pad = part.shapefactory.AddNewPad(sketch, extrude_op.extent_one)

            first_limit = pad.FirstLimit
            first_limit.LimitMode = 3
            if extrude_op.select_list[0].body_type == 'OriginElements':
                originElements1 = part.OriginElements
                if extrude_op.select_list[0].no == 1:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneXY)
                elif extrude_op.select_list[0].no == 2:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneYZ)
                else:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneZX)
            else:
                plane_name = parse_BrepName(extrude_op.select_list[0], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
                plane_name = 'FSur:(' + plane_name + ';WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)'
                plane_ref = part.CreateReferenceFromBRepName(plane_name, cur_body_ref)
            first_limit.LimitingElement = plane_ref
        # UpToSurfaceLimit
        if extrude_op.extent_type1 == "UpToSurfaceLimit":
            pad = part.shapefactory.AddNewPad(sketch, extrude_op.extent_one)

            first_limit = pad.FirstLimit
            first_limit.LimitMode = 4
            if extrude_op.select_list[0].body_type == 'OriginElements':
                originElements1 = part.OriginElements
                if extrude_op.select_list[0].no == 1:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneXY)
                elif extrude_op.select_list[0].no == 2:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneYZ)
                else:
                    plane_ref = part.CreateReferenceFromObject(originElements1.PlaneZX)
            else:
                plane_name = parse_BrepName(extrude_op.select_list[0], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
                plane_name = 'RSur:(' + plane_name + ';WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)'
                plane_ref = part.CreateReferenceFromBRepName(plane_name, cur_body_ref)
            first_limit.LimitingElement = plane_ref
        # UpThruNextLimit
        if extrude_op.extent_type1 == "UpThruNextLimit":
            pad = part.shapefactory.AddNewPad(sketch, extrude_op.extent_one)

            first_limit = pad.FirstLimit
            first_limit.LimitMode = 5

        limit = pad.SecondLimit
        length = limit.Dimension
        length.Value = extrude_op.extent_two

        if extrude_op.isInverse:
            pad.DirectionOrientation = 1
        cur_body_ref = pad
    elif isinstance(extrude_op, Revolve):
        angle = extrude_op.angle_one
        second_angle = extrude_op.angle_two
        if angle > second_angle:
            second_angle = -second_angle
        else:
            second_angle = 360 - second_angle
        if extrude_op.select_list[0].body_type == 'OriginElements':
            geometricElements1 = sketch.GeometricElements
            axis2D1 = geometricElements1.Item("AbsoluteAxis")
            if extrude_op.select_list[0].no == 1:
                revolve_axis_ref = axis2D1.GetItem("HDirection")
            else:
                revolve_axis_ref = axis2D1.GetItem("VDirection")
        else:
            revolve_axis_name = parse_BrepName(extrude_op.select_list[0], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            revolve_axis_name = 'WireREdge:(' + revolve_axis_name + ';WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)'
            revolve_axis_ref = part.CreateReferenceFromBRepName(revolve_axis_name, sketch)

        shaft = part.shapefactory.AddNewShaft(sketch)
        shaft.FirstAngle.Value = angle
        shaft.SecondAngle.Value = second_angle
        shaft.RevoluteAxis = revolve_axis_ref

        cur_body_ref = shaft
    part.update()
    return cur_body_ref

def parse_BrepName(target: Select, is_last:bool, sketch_offset=0, hole_sketch_map=None):
    if target.select_type == "Wire":
        # 添加sketch_offset以处理
        bodyName = target.body_type + "." + repr(target.body_no + sketch_offset)
        BrepName = "Brp:(" + bodyName + ";" + repr(target.no) + ")"
        if not is_last:
            return BrepName
        else:
            return "Wire:(" + BrepName + ";None:(Limits1:();Limits2:());Cf11:())"
    elif target.select_type == "Face" or target.select_type == "Sub_Face":
        if target.body_type == "Shell":
            sub_name = ""
            for i in target.operation_list:
                sub_name = sub_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
            # 只有Shell不用加，还是都不用加？
            # BrepName = "Brp:((" + sub_name[:-1] + "))"
            BrepName = "Brp:(" + target.body_type + '.' + repr(target.body_no) + '_ResultOUT;' + repr(target.no) + ':(' + sub_name[:-1] + '))'
            if not is_last:
                return BrepName
            else:
                # 考虑混淆情况
                if len(target.no_shared_included) != 0 and len(target.all_oriented_included) != 0:
                    print("同时出现了！")
                if len(target.no_shared_included) == 0 and len(target.all_oriented_included) == 0 and target.all_partially_included ==None:
                    return "Face:(" + BrepName + ";None:();Cf11:())"
                # 暂认为no_shared_included和all_oriented_included不会同时出现
                elif len(target.no_shared_included) != 0:
                    no_shared_name = ""
                    for i in target.no_shared_included:
                        no_shared_name = no_shared_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
                    no_shared_name = "AtLeastOneNoSharedIncluded:(" + no_shared_name[:-1] + ");"
                    return "Face:(" + BrepName + ";" + no_shared_name + "Cf11:())"
                elif target.all_partially_included != None:
                    all_partially_name = ""
                    for i in target.all_partially_included:
                        all_partially_name = all_partially_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
                    all_partially_name = "AllPartiallySharedIncluded:(" + all_partially_name[:-1] + ");"
                    return "Face:(" + BrepName + ";" + all_partially_name + "Cf11:())"
                elif len(target.all_oriented_included) != 0:
                    limits1_name = ''
                    for i in target.all_oriented_included['Limits1']:
                        limits1_name = limits1_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
                    all_oriented_name = "AllOrientedIncluded:(" + limits1_name[:-1] + ")"
                    return "Face:(" + BrepName + ";" + all_oriented_name + ";Cf11:())"
        if target.body_type == "Chamfer" or target.body_type == "EdgeFillet":
            sub_BrepName1 = parse_BrepName(target.operation_list[0], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            sub_BrepName2 = parse_BrepName(target.operation_list[1], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            sub_BrepName1 = sub_BrepName1[sub_BrepName1.find('(') + 1:sub_BrepName1.find(';None')]
            sub_BrepName2 = sub_BrepName2[sub_BrepName2.find('(') + 1:sub_BrepName2.find(';None')]
            BrepName = "Brp:(" + target.body_type + '.' + repr(target.body_no) + '_ResultOUT;' + '(' + sub_BrepName1 + ';' + sub_BrepName2 + '))'
            if not is_last:
                return BrepName
            else:
                return "Face:(" + BrepName + ";None:();Cf11:())"
        if target.body_type == "Mirror":
            sub_BrepName = parse_BrepName(target.operation_list[0], False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            BrepName = "Brp:(" + target.body_type + '.' + repr(target.body_no) + ';(' + sub_BrepName + '))'
            if not is_last:
                return BrepName
            else:
                return "Face:(" + BrepName + ";None:();Cf11:())"
        if target.body_type == 'Hole':
            # 通过记录hole和sketch对应关系的map，找到sketch的序号，并根据no转换为1或3
            if target.no == 1:
                true_no = 1
            else:
                true_no = 3
            return "Face:(Brp:(Hole." + repr(target.body_no) + ";0:(Brp:(Sketch."+ repr(hole_sketch_map["Hole." + repr(target.body_no)]) + ";" + repr(true_no) + ")));None:();Cf11:())"
        if target.no == 0:
            sub_BrepName = parse_BrepName(target.operation_list[0], False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            BrepName = "Brp:(" + target.body_type + "." + repr(target.body_no) + ";" + repr(target.no) + ":(" + sub_BrepName + "))"
        else:
            BrepName = "Brp:(" + target.body_type + "." + repr(target.body_no) + ";" + repr(target.no) + ")"
        if not is_last:
            return BrepName
        else:
            # 考虑混淆情况
            if len(target.no_shared_included) != 0 and len(target.all_oriented_included) != 0:
                print("同时出现了！")
            if len(target.no_shared_included) == 0 and len(target.all_oriented_included) == 0 and target.all_partially_included == None:
                return "Face:(" + BrepName + ";None:();Cf11:())"
            # 暂认为no_shared_included和all_oriented_included不会同时出现
            elif len(target.no_shared_included) != 0:
                no_shared_name = ""
                for i in target.no_shared_included:
                    no_shared_name = no_shared_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
                no_shared_name = "AtLeastOneNoSharedIncluded:(" + no_shared_name[:-1] + ");"
                return "Face:(" + BrepName + ";" + no_shared_name + "Cf11:())"
            elif target.all_partially_included != None:
                all_partially_name = ""
                for i in target.all_partially_included:
                    all_partially_name = all_partially_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
                all_partially_name = "AllPartiallySharedIncluded:(" + all_partially_name[:-1] + ");"
                return "Face:(" + BrepName + ";" + all_partially_name + "Cf11:())"
            elif len(target.all_oriented_included) != 0:
                limits1_name = ''
                for i in target.all_oriented_included['Limits1']:
                    limits1_name = limits1_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
                all_oriented_name = "AllOrientedIncluded:(" + limits1_name[:-1] + ")"
                return "Face:(" + BrepName + ";" + all_oriented_name + ";Cf11:())"
    elif target.select_type == "Multiply_Face":
        sub_name = ""
        for i in target.operation_list:
            sub_name = sub_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"

        BrepName = "Brp:((" + sub_name[:-1] + "))"
        if not is_last:
            return BrepName
        else:
            # 考虑混淆情况
            if len(target.no_shared_included) != 0 and len(target.all_oriented_included) != 0:
                print("同时出现了！")
            if len(target.no_shared_included) == 0 and len(target.all_oriented_included) == 0 and target.all_partially_included == None:
                return "Face:(" + BrepName + ";None:();Cf11:())"
            elif len(target.no_shared_included) != 0:
                no_shared_name = ""
                for i in target.no_shared_included:
                    no_shared_name = no_shared_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
                no_shared_name = "AtLeastOneNoSharedIncluded:(" + no_shared_name[:-1] + ");"
                return "Face:(" + BrepName + ";" + no_shared_name + "Cf11:())"
            elif target.all_partially_included != None:
                all_partially_name = ""
                for i in target.all_partially_included:
                    all_partially_name = all_partially_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
                all_partially_name = "AllPartiallySharedIncluded:(" + all_partially_name[:-1] + ");"
                return "Face:(" + BrepName + ";" + all_partially_name + "Cf11:())"
            elif len(target.all_oriented_included) != 0:
                limits1_name = ''
                limits2_name = ''
                for i in target.all_oriented_included['Limits1']:
                    limits1_name = limits1_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
                limits1_name = "Limits1:(" + limits1_name[:-1] + ")"
                for i in target.all_oriented_included['Limits2']:
                    limits2_name = limits2_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
                limits2_name = "Limits2:(" + limits2_name[:-1] + ")"
                all_oriented_name = "AllOrientedIncluded:(" + limits1_name + ";" + limits2_name + ")"
                return "Face:(" + BrepName + ";" + all_oriented_name + ";Cf11:())"
    elif target.select_type == "Edge":
        sub_BrepName1 = parse_BrepName(target.operation_list[0], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
        sub_BrepName2 = parse_BrepName(target.operation_list[1], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
        # 考虑混淆情况
        if len(target.no_shared_included) != 0 and len(target.all_oriented_included) != 0:
            print("同时出现了！")
        if len(target.no_shared_included) == 0 and len(target.all_oriented_included) == 0 and target.all_partially_included == None:
            return "Edge:(" + sub_BrepName1 + ";" + sub_BrepName2 + ";None:(Limits1:();Limits2:());Cf11:())"
        elif len(target.no_shared_included) != 0:
            no_shared_name = ""
            for i in target.no_shared_included:
                no_shared_name = no_shared_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
            no_shared_name = "AtLeastOneNoSharedIncluded:(Limits1:(" + no_shared_name[:-1] + ");Limits2:());"
            return "Edge:(" + sub_BrepName1 + ";" + sub_BrepName2 + ";" + no_shared_name + "Cf11:())"
        elif target.all_partially_included != None:
            all_partially_name = ""
            for i in target.all_partially_included:
                all_partially_name = all_partially_name + parse_BrepName(i, False, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map) + ";"
            all_partially_name = "AllPartiallySharedIncluded:(Limits1:(" + all_partially_name[:-1] + ");Limits2:());"
            return "Edge:(" + sub_BrepName1 + ";" + sub_BrepName2 + ";" + all_partially_name + "Cf11:())"
        elif len(target.all_oriented_included) != 0:
            limits1_name = ''
            limits2_name = ''
            for i in target.all_oriented_included['Limits1']:
                limits1_name = limits1_name + parse_BrepName(i, False, sketch_offset=sketch_offset) + ";"
            limits1_name = "Limits1:(" + limits1_name[:-1] + ")"
            for i in target.all_oriented_included['Limits2']:
                limits2_name = limits2_name + parse_BrepName(i, False, sketch_offset=sketch_offset) + ";"
            limits2_name = "Limits2:(" + limits2_name[:-1] + ")"
            all_oriented_name = "AllOrientedIncluded:(" + limits1_name + ";" + limits2_name + ")"
            return "Edge:(" + sub_BrepName1 + ";" + sub_BrepName2 + ";" + all_oriented_name + ";Cf11:())"

def create_Feature_on_select(doc, part, op, cur_body_ref, catia, sketch_offset, body, hole_sketch_map):
    shapeFactory = part.ShapeFactory
    temp_reference = part.CreateReferenceFromName("")
    if isinstance(op, Hole):
        sketch_offset = sketch_offset + 3
        plane_ref = parse_BrepName(op.plane_ref, True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
        last_name = "FSur:(" + plane_ref + ";WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)"
        # reference_plane = part.CreateReferenceFromBrepName(last_name, cur_body_ref)
        reference_plane = part.CreateReferenceFromName(last_name)
        # 获得草图平面
        hybridShapeFactory = part.HybridShapeFactory
        # 两点，前者为原点，后者用于构成法线
        hybridShapePointCoord1 = hybridShapeFactory.AddNewPointCoord(op.sketch_plane.origin[0], op.sketch_plane.origin[1],
                                                                     op.sketch_plane.origin[2])
        hybridShapePointCoord2 = hybridShapeFactory.AddNewPointCoord(op.sketch_plane.origin[0] + op.sketch_plane.normal[0],
                                                                     op.sketch_plane.origin[1] + op.sketch_plane.normal[1],
                                                                     op.sketch_plane.origin[2] + op.sketch_plane.normal[2])
        reference1 = part.CreateReferenceFromObject(hybridShapePointCoord1)
        reference2 = part.CreateReferenceFromObject(hybridShapePointCoord2)
        # 法线
        hybridShapeLinePtPt = hybridShapeFactory.AddNewLinePtPt(reference1, reference2)
        reference3 = part.CreateReferenceFromObject(hybridShapeLinePtPt)
        reference4 = part.CreateReferenceFromObject(hybridShapePointCoord1)
        hybridShapePlaneNormal = hybridShapeFactory.AddNewPlaneNormal(reference3, reference4)

        # 添加定义平面上的草图
        body.InsertHybridShape(hybridShapePlaneNormal)

        reference_hole = body.HybridShapes.Item(body.HybridShapes.Count)
        sketch_hole = body.sketches.add(reference_hole)

        sketch_hole.SetAbsoluteAxisData(tuple([op.sketch_plane.origin[0], op.sketch_plane.origin[1], op.sketch_plane.origin[2],
                                          op.sketch_plane.x_axis[0], op.sketch_plane.x_axis[1], op.sketch_plane.x_axis[2],
                                          op.sketch_plane.y_axis[0], op.sketch_plane.y_axis[1], op.sketch_plane.y_axis[2]]))

        part.InWorkObject = sketch_hole

        factory2D_hole = sketch_hole.OpenEdition()
        point2D_hole = factory2D_hole.CreatePoint(op.point_pos[0], op.point_pos[1])
        point2D_hole.ReportName = 1
        point2D_hole.Construction = False
        sketch_hole.CloseEdition()
        part.InWorkObject = sketch_hole
        part.Update()
        cur_sketch_name = part.CreateReferenceFromObject(body.sketches.item(body.sketches.Count)).displayname
        no = int(cur_sketch_name[cur_sketch_name.find('.') + 1:])

        reference_hole = part.CreateReferenceFromBRepName("BorderFVertex:(BEdge:(Brp:(Sketch." + repr(no) + ";1);None:(Limits1:();Limits2:();+1);Cf11:());WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)", sketch_hole)
        hole1 = part.ShapeFactory.AddNewHoleFromRefPoint(reference_hole, reference_plane, op.depth)

        hole1.Type = 0 # catSimpleHole
        hole1.AnchorMode = 0 # catExtremPointHoleAnchor
        hole1.BottomType = 0 # catFlatHoleBottom
        limit2 = hole1.BottomLimit
        limit2.LimitMode = 0 # catOffsetLimit
        length = hole1.Diameter
        length.Value = op.radius * 2
        hole1.ThreadingMode = 1 # catSmoothHoleThreading
        hole1.ThreadSide = 0 # catRightThreadSide

        part.update()
        hole_sketch_map[hole1.name] = body.sketches.Count
        cur_body_ref = hole1

    if isinstance(op, Draft):
        draft1 = shapeFactory.AddNewDraft(temp_reference, temp_reference, 0, temp_reference, 0, 0, 1, 0, 5.0, 0)
        draftDomains1 = draft1.DraftDomains
        draftDomain1 = draftDomains1.Item(1)

        draftDomain1.SetPullingDirection(0.000000, 0.000000, 1.000000)
        for i in op.select_list:
            BrepName = parse_BrepName(i, True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            last_name = "RSur:(" + BrepName + ";WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)"
            reference_tar = part.CreateReferenceFromBrepName(last_name, cur_body_ref)
            draftDomain1.AddFaceToDraft(reference_tar)

        draftDomain1.SetPullingDirection(op.dir[0], op.dir[1], op.dir[2])
        neutral_name = parse_BrepName(op.Neutral, True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
        last_name = "FSur:(" + neutral_name + ";WithTemporaryBody;WithoutBuildError;WithInitialFeatureSupport;MFBRepVersion_CXR15)"
        reference_tar = part.CreateReferenceFromBrepName(last_name, cur_body_ref)
        draftDomain1.PullingDirectionElement = reference_tar

        last_name = "RSur:(" + neutral_name + ";WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)"
        reference_tar = part.CreateReferenceFromBrepName(last_name, cur_body_ref)
        draftDomain1.NeutralElement = reference_tar

        try:
            # 求取中性面的垂直向量
            spa_workbench = doc.GetWorkbench("SPAWorkbench")
            measurable = spa_workbench.GetMeasurable(draftDomain1.NeutralElement)
            neutral_vec = get_plane(measurable, catia)
            z_vec = np.cross(neutral_vec[3:6], neutral_vec[6:9])
            draftDomain1.SetPullingDirection(z_vec[0], z_vec[1], z_vec[2])
        except:
            print('get_plane failed!')

        angle1 = draftDomain1.DraftAngle
        angle1.Value = op.DraftAngle
        part.update()
        cur_body_ref = draft1

    if isinstance(op, Mirror):
        if op.select_list[0].body_type == 'OriginElements':
            originElements1 = part.OriginElements
            if op.select_list[0].no == 1:
                hybridShapePlaneExplicit1 = originElements1.PlaneXY
            if op.select_list[0].no == 2:
                hybridShapePlaneExplicit1 = originElements1.PlaneYZ
            if op.select_list[0].no == 3:
                hybridShapePlaneExplicit1 = originElements1.PlaneZX
            reference_tar = part.CreateReferenceFromObject(hybridShapePlaneExplicit1)
        else:
            BrepName = parse_BrepName(op.select_list[0], True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            last_name = "RSur:(" + BrepName + ";WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)"
            reference_tar = part.CreateReferenceFromBrepName(last_name, cur_body_ref)
        mirror1 = shapeFactory.AddNewMirror(reference_tar)
        part.update()
        cur_body_ref = mirror1

    if isinstance(op, Shell):
        shell1 = shapeFactory.AddNewShell(temp_reference, 1.000000, 0.000000)
        length1 = shell1.InternalThickness
        length1.Value = op.thickness
        length2 = shell1.ExternalThickness
        length2.Value = op.second_thickness

        for i in op.select_list:
            BrepName = parse_BrepName(i, True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            last_name = "RSur:(" + BrepName + ";WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)"
            reference_tar = part.CreateReferenceFromBrepName(last_name, cur_body_ref)
            shell1.AddFaceToRemove(reference_tar)
        part.update()
        cur_body_ref = shell1

    if isinstance(op, Chamfer):
        chamfer1 = shapeFactory.AddNewChamfer(temp_reference, 0, 0, 0, op.length1, op.angle_or_length2)
        for i in op.select_list:
            BrepName = parse_BrepName(i, True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            if i.select_type == "Face" or i.select_type == "Multiply_Face":
                last_name = "RSur:(" + BrepName + ";WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)"
            else:
                last_name = "REdge:(" + BrepName + ";WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)"
            reference_tar = part.CreateReferenceFromBrepName(last_name, cur_body_ref)
            chamfer1.AddElementToChamfer(reference_tar)
            chamfer1.Mode = CatChamferMode.index("catTwoLengthChamfer")
            chamfer1.Propagation = CatChamferPropagation.index("catTangencyChamfer")
            chamfer1.Orientation = CatChamferOrientation.index("catNoReverseChamfer")
        part.update()
        cur_body_ref = chamfer1

    if isinstance(op, Fillet):
        shapeFactory = part.ShapeFactory
        temp_reference = part.CreateReferenceFromName("")
        constRadEdgeFillet1 = shapeFactory.AddNewSolidEdgeFilletWithConstantRadius(temp_reference, 1, op.radius)
        for i in op.select_list:
            BrepName = parse_BrepName(i, True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            if i.select_type == "Face" or i.select_type == "Multiply_Face":
                last_name = "RSur:(" + BrepName + ";WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)"
            else:
                last_name = "REdge:(" + BrepName + ";WithTemporaryBody;WithoutBuildError;WithSelectingFeatureSupport;MFBRepVersion_CXR15)"

            reference_tar = part.CreateReferenceFromBrepName(last_name, cur_body_ref)
            constRadEdgeFillet1.AddObjectToFillet(reference_tar)
            constRadEdgeFillet1.EdgePropagation = 1
        part.Update()
        cur_body_ref = constRadEdgeFillet1
    return cur_body_ref, sketch_offset

def create_CAD_CATIA(cad_seq: Macro_Seq, catia, doc, part, remove_bug=False):
    if not remove_bug:
        #记录hole操作对应草图序号
        hole_sketch_map = {}
        # 用于记录因hole操作产生的多余草图的数量，以在命名时进行修正
        sketch_offset = 0
        # 记录当前body对象，用于CreateReferenceFromBrepName
        # 创建第一个body
        cur_body_ref = create_Body_CATIA(part, cad_seq.extrude_operation[0], is_first=True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
        #将剩余body加入
        for extrude_op in cad_seq.extrude_operation[1:]:
            if isinstance(extrude_op, Pocket) or isinstance(extrude_op, Groove):
                cur_body_ref = create_pocket_CATIA(part, extrude_op, cur_body_ref=cur_body_ref, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)

            if isinstance(extrude_op, Extrude) or isinstance(extrude_op, Revolve):
                cur_body_ref = create_Body_CATIA(part, extrude_op, cur_body_ref=cur_body_ref, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
                # 布尔
                body_cur = part.bodies.Item(part.bodies.Count)
                body_first = part.bodies.Item(1)
                part.InWorkObject = body_first

                if extrude_op.operation == "CutFeatureOperation":
                    cur_body_ref = part.shapefactory.AddNewRemove(body_cur)
                elif extrude_op.operation == "IntersectFeatureOperation":
                        cur_body_ref = part.shapefactory.AddNewIntersect(body_cur)
                elif extrude_op.operation == "AddFeatureOperation":
                    cur_body_ref = part.shapefactory.AddNewAdd(body_cur)
                part.update()
            if isinstance(extrude_op, Shell) or isinstance(extrude_op, Chamfer) or isinstance(extrude_op, Fillet) or \
                    isinstance(extrude_op, Draft) or isinstance(extrude_op, Mirror) or isinstance(extrude_op, Hole):
                cur_body_ref, sketch_offset = create_Feature_on_select(doc, part, extrude_op, cur_body_ref, catia, sketch_offset, part.bodies.Item(1), hole_sketch_map)
        # 放大以便观察
        specsAndGeomWindow1 = catia.ActiveWindow
        viewer3D1 = specsAndGeomWindow1.ActiveViewer
        viewer3D1.Reframe()
    else:
        try:
            # 记录hole操作对应草图序号
            hole_sketch_map = {}
            # 用于记录因hole操作产生的多余草图的数量，以在命名时进行修正
            sketch_offset = 0
            cur_point = 0
            cur_body_ref = create_Body_CATIA(part, cad_seq.extrude_operation[0], is_first=True, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
            cur_point = cur_point + 1
            # 将剩余body加入
            for extrude_op in cad_seq.extrude_operation[1:]:
                if isinstance(extrude_op, Pocket):
                    cur_body_ref = create_pocket_CATIA(part, extrude_op, cur_body_ref=cur_body_ref, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
                    cur_point = cur_point + 1

                if isinstance(extrude_op, Extrude) or isinstance(extrude_op, Revolve):
                    cur_body_ref = create_Body_CATIA(part, extrude_op, cur_body_ref=cur_body_ref, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
                    # 布尔
                    body_cur = part.bodies.Item(part.bodies.Count)

                    body_first = part.bodies.Item(1)
                    part.InWorkObject = body_first

                    if extrude_op.operation == "CutFeatureOperation":
                        cur_body_ref = part.shapefactory.AddNewRemove(body_cur)
                    elif extrude_op.operation == "IntersectFeatureOperation":
                        cur_body_ref = part.shapefactory.AddNewIntersect(body_cur)
                    elif extrude_op.operation == "AddFeatureOperation":
                        cur_body_ref = part.shapefactory.AddNewAdd(body_cur)
                    part.update()
                    cur_point = cur_point + 1
                if isinstance(extrude_op, Shell) or isinstance(extrude_op, Chamfer) or isinstance(extrude_op, Fillet) or \
                        isinstance(extrude_op, Draft) or isinstance(extrude_op, Mirror):
                    cur_body_ref = create_Feature_on_select(doc, part, extrude_op, cur_body_ref, catia, sketch_offset=sketch_offset, hole_sketch_map=hole_sketch_map)
                    cur_point = cur_point + 1
            return -1
        except:
            return cur_point

def macro2pc(catia, vec, n_points):
    catia.visible = 1

    cad = Macro_Seq.from_vector(vec, is_numerical=True, n=256)
    doc = catia.documents.add('Part')
    part = doc.part
    try:
        create_CAD_CATIA(cad, catia,doc, part)
    except:
        doc.close()
        print('macro2pc failed')
        return
    partDocument1 = catia.ActiveDocument
    if os.path.exists("C:/Users/45088/Desktop/WHU_dataset/test.stl"):
        os.remove("C:/Users/45088/Desktop/WHU_dataset/test.stl")
    partDocument1.ExportData("C:/Users/45088/Desktop/WHU_dataset/test.stl", "stl")
    doc.close()
    out_mesh = trimesh.load("C:/Users/45088/Desktop/WHU_dataset/test.stl")
    os.system("rm " + "C:/Users/45088/Desktop/WHU_dataset/test.stl")
    # 采样点原来为8096
    out_pc, _ = sample_surface(out_mesh, n_points)
    # write_ply(out_pc, save_path + ".ply")
    os.remove("C:/Users/45088/Desktop/WHU_dataset/test.stl")
    print("macro2pc OK")
    return out_pc

# input: select_name, output: select
def parse_select(select_name, no_map, sketch_offset_map, body_sketch_map, shaft_count=0, groove_count=0, shaft_or_groove=True):
    select = None
    if isinstance(select_name, Select):
        return select_name
    # 若既不是Edge也不是Face，则为空引用，用来预定义操作的
    if 'WireREdge' in select_name:
        # 一般都是旋转采用WireREdge，因此使用body_sketch_map进行矫正
        name_str = select_name[select_name.find('WireREdge'):]
        name_str = name_str[name_str.find('(') + 1:]
        if shaft_or_groove:
            select = parse_select_name(name_str, 'WireREdge', no_map, body_sketch_map, sketch_offset_map,
                                       'Shaft.' + repr(shaft_count))
        else:
            select = parse_select_name(name_str, 'WireREdge', no_map, body_sketch_map, sketch_offset_map,
                                       'Groove.' + repr(groove_count))
    elif 'REdge' in select_name:
        name_str = select_name[select_name.find('REdge'):]
        name_str = name_str[name_str.find('(') + 1:]
        select = parse_select_name(name_str, 'Edge', no_map, body_sketch_map, sketch_offset_map)
    elif 'FEdge' in select_name:
        name_str = select_name[select_name.find('FEdge'):]
        name_str = name_str[name_str.find('(') + 1:]
        select = parse_select_name(name_str, 'Edge', no_map, body_sketch_map, sketch_offset_map)
    elif 'RFace' in select_name:
        name_str = select_name[select_name.find('RFace'):]
        name_str = name_str[name_str.find('(') + 1:]
        select = parse_select_name(name_str, 'Face', no_map, body_sketch_map, sketch_offset_map)
    elif 'RSur' in select_name:
        name_str = select_name[select_name.find('RSur'):]
        name_str = name_str[name_str.find('(') + 1:]
        select = parse_select_name(name_str, 'Face', no_map, body_sketch_map, sketch_offset_map)
    elif 'FSur' in select_name:
        name_str = select_name[select_name.find('FSur'):]
        name_str = name_str[name_str.find('(') + 1:]
        select = parse_select_name(name_str, 'Face', no_map, body_sketch_map, sketch_offset_map)
    return deepcopy(select)

def parse_select_name(select_name, type, no_map, body_sketch_map, offset_map={}, higher_str=''):
    if type == 'Wire':
        select_name = select_name[select_name.find('(') + 1:-1]
        body_name = select_name[:select_name.find('.')]
        # 根据offset_map对草图序号调整
        body_no = int(select_name[select_name.find('.') + 1:select_name.find(';')])
        tmp = 0
        for i in offset_map:
            if body_no >= i:
                tmp = tmp + offset_map[i]
        body_no = body_no + tmp

        # 使用body_sketch_map对body_no纠正
        if higher_str in body_sketch_map.keys():
            if body_sketch_map[higher_str] != body_no:
                body_no = body_sketch_map[higher_str]
        # curve的序号不一定是按序，需进行转化
        topo_no = no_map[select_name]
        select = Select('Wire', body_name, body_no, topo_no, [], [], {})
        return select
    if type == 'WireREdge':
        select_name = select_name[select_name.find('(') + 1:]
        name_str = select_name[select_name.find('(') + 1: select_name.find(')')]
        body_name = name_str[:name_str.find('.')]
        # 根据offset_map对草图序号调整
        body_no = int(name_str[name_str.find('.') + 1:name_str.find(';')])

        for i in offset_map:
            if body_no >= i:
                body_no = body_no + offset_map[i]

        # 使用body_sketch_map对body_no纠正
        if higher_str in body_sketch_map.keys():
            if body_sketch_map[higher_str] != body_no:
                body_no = body_sketch_map[higher_str]

        topo_no = no_map[name_str]
        select = Select('Wire', body_name, body_no, topo_no, [], [], {})
        return select
    if type == 'Edge':
        select_name = select_name[select_name.find('(') + 1:]
        select = Select('Edge', 'None', 0, 0, [], [], {})
        # 将各个面的命名分开
        while select_name[:4] == 'Face':
            start_point = 0
            end_point = select_name.find('(') + 1
            bracket_count = 1
            while bracket_count > 0:
                if select_name[end_point] == '(':
                    bracket_count = bracket_count + 1
                elif select_name[end_point] == ')':
                    bracket_count = bracket_count - 1
                end_point = end_point + 1
            select.operation_list.append(parse_select_name(select_name[start_point: end_point], 'Face', no_map, body_sketch_map, offset_map))
            select_name = select_name[end_point + 1:]

        # 查看是否有NoSharedIncluded
        if 'AtLeastOneNoSharedIncluded' in select_name:
            no_shared_str = select_name[select_name.find('AtLeastOneNoSharedIncluded') + len('AtLeastOneNoSharedIncluded') + 1:]
            end_point = no_shared_str.index('(') + 1
            bracket_count = 1
            while bracket_count > 0 and end_point < no_shared_str.__len__():
                if no_shared_str[end_point] == '(':
                    bracket_count = bracket_count + 1
                elif no_shared_str[end_point] == ')':
                    bracket_count = bracket_count - 1
                end_point = end_point + 1
            no_shared_str = no_shared_str[1:end_point - 1]
            # 去掉Limits1
            no_shared_str = no_shared_str[no_shared_str.find('Limits1:(') + len('Limits1:('):]
            while no_shared_str[:3] == 'Brp':
                start_point = 0
                end_point = no_shared_str.find('(') + 1
                bracket_count = 1
                while bracket_count > 0:
                    if no_shared_str[end_point] == '(':
                        bracket_count = bracket_count + 1
                    elif no_shared_str[end_point] == ')':
                        bracket_count = bracket_count - 1
                    end_point = end_point + 1
                select.no_shared_included.append(
                    parse_select_name('Face:(' + no_shared_str[start_point: end_point] + ')', 'Face', no_map, body_sketch_map,
                                      offset_map))
                no_shared_str = no_shared_str[end_point + 1:]

        # 查看是否有AllOrientedIncluded
        if 'AllOrientedIncluded' in select_name:
            all_oriented_str = select_name[select_name.find('AllOrientedIncluded') + len('AllOrientedIncluded') + 1:]
            end_point = all_oriented_str.index('(') + 1
            bracket_count = 1
            while bracket_count > 0 and end_point < all_oriented_str.__len__():
                if all_oriented_str[end_point] == '(':
                    bracket_count = bracket_count + 1
                elif all_oriented_str[end_point] == ')':
                    bracket_count = bracket_count - 1
                end_point = end_point + 1
            all_oriented_str = all_oriented_str[1:end_point - 1]

            # 将Limits1和Limits2分开
            Limits1_str = all_oriented_str[all_oriented_str.find('Limits1'):all_oriented_str.find('Limits2')]
            Limits2_str = all_oriented_str[all_oriented_str.find('Limits2'):]
            Limits1_str = Limits1_str[Limits1_str.find('Limits1:(') + len('Limits1:('):]
            Limits2_str = Limits2_str[Limits2_str.find('Limits2:(') + len('Limits2:('):]

            Limits1_select = []
            Limits2_select = []
            while Limits1_str[:3] == 'Brp':
                start_point = 0
                end_point = Limits1_str.find('(') + 1
                bracket_count = 1
                while bracket_count > 0:
                    if Limits1_str[end_point] == '(':
                        bracket_count = bracket_count + 1
                    elif Limits1_str[end_point] == ')':
                        bracket_count = bracket_count - 1
                    end_point = end_point + 1
                Limits1_select.append(
                    parse_select_name('Face:(' + Limits1_str[start_point: end_point] + ')', 'Face', no_map, body_sketch_map, offset_map))
                Limits1_str = Limits1_str[end_point + 1:]

            while Limits2_str[:3] == 'Brp':
                start_point = 0
                end_point = Limits2_str.find('(') + 1
                bracket_count = 1
                while bracket_count > 0:
                    if Limits2_str[end_point] == '(':
                        bracket_count = bracket_count + 1
                    elif Limits2_str[end_point] == ')':
                        bracket_count = bracket_count - 1
                    end_point = end_point + 1
                Limits2_select.append(
                    parse_select_name('Face:(' + Limits2_str[start_point: end_point] + ')', 'Face', no_map, body_sketch_map,
                                      offset_map))
                Limits2_str = Limits2_str[end_point + 1:]

            select.all_oriented_included['Limits1'] = deepcopy(Limits1_select)
            select.all_oriented_included['Limits2'] = deepcopy(Limits2_select)
        return select
    if type == 'Face':
        select_name = select_name[select_name.find('(') + 1:]
        end_point = select_name.index('(') + 1
        strat_point = end_point
        bracket_count = 1
        while bracket_count > 0 and end_point < select_name.__len__():
            if select_name[end_point] == '(':
                bracket_count = bracket_count + 1
            elif select_name[end_point] == ')':
                bracket_count = bracket_count - 1
            end_point = end_point + 1
        name_str = select_name[strat_point:end_point - 1]


        # 此时判断是否有多个Brp，若有，则说明此面由多个面聚合
        if name_str[0] == '(':
            name_str = name_str[1:-1]
            select = Select('Multiply_Face', 'None', 0, 0, [], [], {})
            # 将各个面的命名分开
            while name_str[:3] == 'Brp':
                start_point = 0
                end_point = name_str.find('(') + 1
                bracket_count = 1
                while bracket_count > 0:
                    if name_str[end_point] == '(':
                        bracket_count = bracket_count + 1
                    elif name_str[end_point] == ')':
                        bracket_count = bracket_count - 1
                    end_point = end_point + 1
                select.operation_list.append(
                    parse_select_name('Face:(' + name_str[start_point: end_point] + ')', 'Face', no_map, body_sketch_map, offset_map))
                select.operation_list[-1].select_type = 'Sub_Face'
                name_str = name_str[end_point + 1:]
        # 若为Shell，则视为Multiply_Face
        elif name_str[:5] == 'Shell':
            body_no = int(name_str[name_str.find('.') + 1:name_str.find('_')])
            no = int(name_str[name_str.find(';') + 1: name_str.find(':')])
            select = Select('Face', 'Shell', body_no, no, [], [], {})
            name_str = name_str[name_str.find('Brp:(') + 5:-2]
            # 若是，说明为多个面
            if name_str[0] == '(':
                name_str = name_str[1:-1]
                # 将各个面的命名分开
                while name_str[:3] == 'Brp':
                    start_point = 0
                    end_point = name_str.find('(') + 1
                    bracket_count = 1
                    while bracket_count > 0:
                        if name_str[end_point] == '(':
                            bracket_count = bracket_count + 1
                        elif name_str[end_point] == ')':
                            bracket_count = bracket_count - 1
                        end_point = end_point + 1
                    select.operation_list.append(
                        parse_select_name('Face:(' + name_str[start_point: end_point] + ')', 'Face', no_map, body_sketch_map, offset_map))
                    # select.operation_list[-1].select_type = 'Sub_Face'
                    name_str = name_str[end_point + 1:]
            else:
                select.operation_list.append(
                    parse_select_name('Face:(Brp:(' + name_str + '))', 'Face', no_map, body_sketch_map, offset_map))
                # select.operation_list[-1].select_type = 'Sub_Face'
        else:
            body_name = name_str[:name_str.find('.')]
            if body_name == 'Chamfer' or body_name == 'EdgeFillet':
                body_no = int(name_str[name_str.find('.') + 1:name_str.find('_')])
                select_name_tmp = select_name[select_name.find('(') + 1:]
                select_name_tmp = select_name_tmp[select_name_tmp.find('(') + 1:]
                select = Select('Face', body_name, body_no, 0, [], [], {})
                # 将各个面的命名分开
                while select_name_tmp[:3] == 'Brp':
                    start_point = 0
                    end_point = select_name_tmp.find('(') + 1
                    bracket_count = 1
                    while bracket_count > 0:
                        if select_name_tmp[end_point] == '(':
                            bracket_count = bracket_count + 1
                        elif select_name_tmp[end_point] == ')':
                            bracket_count = bracket_count - 1
                        end_point = end_point + 1
                    select.operation_list.append(parse_select_name('Face:(' + select_name_tmp[start_point: end_point] + ')', 'Face', no_map, body_sketch_map, offset_map))
                    select_name_tmp = select_name_tmp[end_point + 1:]
            elif body_name == 'Mirror':
                body_no = int(name_str[name_str.find('.') + 1:name_str.find(';')])
                topo_no = 0
                select = Select('Face', body_name, body_no, topo_no, [], [], {})
                # 若是，说明为多个面
                name_str = name_str[name_str.find('(') + 1:-1]
                select.operation_list.append(parse_select_name('Face:(' + name_str + ')', 'Face', no_map, body_sketch_map, offset_map))
                # 考虑到Mirror的子面也可能是Multi面，暂时先不设置为sub面
                # select.operation_list[-1].select_type = 'Sub_Face'
            elif body_name == 'Hole':
                # hole需要特殊对待，将其草图等命名更改为result_out,直接使用序号而非sketch.no;no
                body_no = int(name_str[name_str.find('.') + 1:name_str.find(';')])
                sketch_name = name_str[name_str.find('Sketch'):name_str.find(')')]
                topo_no = int(sketch_name[sketch_name.find(';') + 1:] == '1')
                select = Select('Face', body_name, body_no, topo_no, [], [], {})
            else:
                body_no = int(name_str[name_str.find('.') + 1:name_str.find(';')])
                topo_no = int(name_str[name_str.find(';') + 1])
                select = Select('Face', body_name, body_no, topo_no, [], [], {})
                if topo_no == 0:
                    sub_str = name_str[name_str.find(':(') + 2:-1]
                    sub_select = parse_select_name(sub_str, 'Wire', no_map, body_sketch_map, offset_map, body_name + '.' + repr(body_no))
                    select.operation_list.append(sub_select)

        # 查看是否有NoSharedIncluded
        if 'AtLeastOneNoSharedIncluded' in select_name:
            no_shared_str = select_name[select_name.find('AtLeastOneNoSharedIncluded') + len('AtLeastOneNoSharedIncluded') + 1:]
            end_point = no_shared_str.index('(') + 1
            bracket_count = 1
            while bracket_count > 0 and end_point < no_shared_str.__len__():
                if no_shared_str[end_point] == '(':
                    bracket_count = bracket_count + 1
                elif no_shared_str[end_point] == ')':
                    bracket_count = bracket_count - 1
                end_point = end_point + 1
            no_shared_str = no_shared_str[1:end_point - 1]
            while no_shared_str[:3] == 'Brp':
                start_point = 0
                end_point = no_shared_str.find('(') + 1
                bracket_count = 1
                while bracket_count > 0:
                    if no_shared_str[end_point] == '(':
                        bracket_count = bracket_count + 1
                    elif no_shared_str[end_point] == ')':
                        bracket_count = bracket_count - 1
                    end_point = end_point + 1
                select.no_shared_included.append(
                    parse_select_name('Face:(' + no_shared_str[start_point: end_point] + ')', 'Face', no_map, body_sketch_map, offset_map))
                no_shared_str = no_shared_str[end_point + 1:]
        # 查看是否有AllOrientedIncluded
        if 'AllOrientedIncluded' in select_name:
            all_oriented_str = select_name[
                               select_name.find('AllOrientedIncluded') + len('AllOrientedIncluded') + 1:]
            end_point = all_oriented_str.index('(') + 1
            bracket_count = 1
            while bracket_count > 0 and end_point < all_oriented_str.__len__():
                if all_oriented_str[end_point] == '(':
                    bracket_count = bracket_count + 1
                elif all_oriented_str[end_point] == ')':
                    bracket_count = bracket_count - 1
                end_point = end_point + 1
            all_oriented_str = all_oriented_str[1:end_point - 1]
            limits1_list = []
            limits2_list = []
            while all_oriented_str[:3] == 'Brp':
                start_point = 0
                end_point = all_oriented_str.find('(') + 1
                bracket_count = 1
                while bracket_count > 0:
                    if all_oriented_str[end_point] == '(':
                        bracket_count = bracket_count + 1
                    elif all_oriented_str[end_point] == ')':
                        bracket_count = bracket_count - 1
                    end_point = end_point + 1
                limits1_list.append(
                    parse_select_name('Face:(' + all_oriented_str[start_point: end_point] + ')', 'Face', no_map, body_sketch_map, offset_map))
                all_oriented_str = all_oriented_str[end_point + 1:]
            select.all_oriented_included['Limits1'] = deepcopy(limits1_list)
            select.all_oriented_included['Limits2'] = deepcopy(limits2_list)
        # 查看是否有AllPartiallySharedIncluded
        if 'AllPartiallySharedIncluded' in select_name:
            partially_included = []
            no_shared_str = select_name[select_name.find('AllPartiallySharedIncluded') + len('AllPartiallySharedIncluded') + 1:]
            end_point = no_shared_str.index('(') + 1
            bracket_count = 1
            while bracket_count > 0 and end_point < no_shared_str.__len__():
                if no_shared_str[end_point] == '(':
                    bracket_count = bracket_count + 1
                elif no_shared_str[end_point] == ')':
                    bracket_count = bracket_count - 1
                end_point = end_point + 1
            no_shared_str = no_shared_str[1:end_point - 1]
            while no_shared_str[:3] == 'Brp':
                start_point = 0
                end_point = no_shared_str.find('(') + 1
                bracket_count = 1
                while bracket_count > 0:
                    if no_shared_str[end_point] == '(':
                        bracket_count = bracket_count + 1
                    elif no_shared_str[end_point] == ')':
                        bracket_count = bracket_count - 1
                    end_point = end_point + 1
                partially_included.append(
                    parse_select_name('Face:(' + no_shared_str[start_point: end_point] + ')', 'Face', no_map, body_sketch_map, offset_map))
                no_shared_str = no_shared_str[end_point + 1:]
            select.all_partially_included = deepcopy(partially_included)
        return deepcopy(select)

def process_sketch(op, curve_cache, no_map, sketch_cache):
    # 首先将curves划分为不同的loop， 假设都按逆时针绘制草图
    loop_start_point = []
    loop_cur_point = []
    loops = []
    begin_point = 0

    # 首先将所有圆剔除出来，因为圆不需要组成loop
    # 且将闭合的spline剔除，同样不组成loop
    sketch_curve_copy = deepcopy(curve_cache[op.sketch_name])
    sketch_curve = []
    for i in range(0, sketch_curve_copy.__len__()):
        if isinstance(sketch_curve_copy[i], Circle):
            loops.append([sketch_curve_copy[i]])
        elif isinstance(sketch_curve_copy[i], Spline) and np.allclose(sketch_curve_copy[i].start_point, sketch_curve_copy[i].end_point):
            loops.append([sketch_curve_copy[i]])
        else:
            sketch_curve.append(sketch_curve_copy[i])
    if sketch_curve.__len__() > 0:
        # 矫正其他点的顺逆关系
        for i in range(0, sketch_curve.__len__()):
            if len(loop_start_point) == 0:
                # 矫正开始点
                if np.allclose(sketch_curve[i].start_point, sketch_curve[i + 1].start_point,
                               atol=0.0001) or np.allclose(
                        sketch_curve[i].start_point, sketch_curve[i + 1].end_point, atol=0.0001):
                    sketch_curve[i].reverse()
                # 否则，若开始curve不与下一curve有任何关系，则说明loop不连续，向下寻找到连续的点
                elif not np.allclose(sketch_curve[i].end_point, sketch_curve[i + 1].start_point,
                                     atol=0.0001) or np.allclose(
                        sketch_curve[i].end_point, sketch_curve[i + 1].end_point, atol=0.0001):
                    j = i + 1
                    while j < sketch_curve.__len__():
                        if np.allclose(sketch_curve[i].end_point, sketch_curve[j].start_point,
                                       atol=0.0001) or np.allclose(
                                sketch_curve[i].end_point, sketch_curve[j].end_point, atol=0.0001):
                            sketch_curve[i + 1], sketch_curve[j] = sketch_curve[j], sketch_curve[i + 1]
                            break
                        j = j + 1
                # 开始点处理
                loop_start_point = sketch_curve[i].start_point
                loop_cur_point = sketch_curve[i].end_point
                continue
            # 若当前curve的结束点与上一curve的结束点重叠，reverse当前curve
            if np.allclose(loop_cur_point, sketch_curve[i].end_point, atol=0.0001):
                sketch_curve[i].reverse()
            # 若当前curve的结束点与上一curve的开始点重叠，继续向下
            if np.allclose(loop_cur_point, sketch_curve[i].start_point, atol=0.0001):
                loop_cur_point = sketch_curve[i].end_point
            # 若当前curve不与下一条curve相交，则可能是1.与开始点重合，形成了loop; 2.loop定义不连续
            if i + 1 < sketch_curve.__len__() and not (
                    np.allclose(sketch_curve[i].end_point, sketch_curve[i + 1].start_point, atol=0.0001) or np.allclose(
                    sketch_curve[i].end_point, sketch_curve[i + 1].end_point, atol=0.0001)):
                # 若当前curve的结束点与最开始点重合，将之前的curves分为一个loop
                if np.allclose(loop_start_point, sketch_curve[i].end_point, atol=0.0001):
                    loops.append(sketch_curve[begin_point:i + 1])
                    begin_point = i + 1
                    loop_start_point = []
                    loop_cur_point = []
                # 否则当前curve既不与开始curve相交，又不与下一个curve相交，这种情况可能是出现了loop不连续的情况
                # 此时进行遍历，找到下一个相交的点并与下一个curve位置交换，以达成连续
                else:
                    j = i + 1
                    while j < sketch_curve.__len__():
                        if np.allclose(sketch_curve[i].end_point, sketch_curve[j].start_point,
                                       atol=0.0001) or np.allclose(sketch_curve[i].end_point, sketch_curve[j].end_point,
                                                                   atol=0.0001):
                            sketch_curve[i + 1], sketch_curve[j] = sketch_curve[j], sketch_curve[i + 1]
                            break
                        j = j + 1
        if begin_point < sketch_curve.__len__():
            loops.append(sketch_curve[begin_point:])
    # 遍历loops，对同一loop中的spline进行标序以便区分开
    for i in range(loops.__len__()):
        no_count = 0
        for j in range(loops[i].__len__()):
            if isinstance(loops[i][j], Spline):
                no_count = no_count + 1
                loops[i][j].no_in_loop = no_count
    all_loop = []
    for i in loops:
        all_loop.append(Loop(i))

    sketch_profile = Profile(all_loop)
    # 矫正草图的顺逆时针，且将开始点矫正到原点，因为会打乱顺序，所以要修改no_map
    count = 1
    for this_loop in sketch_profile.children:
        for i in this_loop.children:
            no_map['Sketch.' + repr(sketch_cache[op.sketch_name].sketch_no) + ';' + repr(i.reportName)] = count
            count = count + 1

    point = sketch_profile.start_point
    sketch_plane = sketch_cache[op.sketch_name].sketch_plane
    sketch_pos = point[0] * sketch_plane.x_axis + point[1] * sketch_plane.y_axis + sketch_plane.origin
    sketch_size = sketch_profile.bbox_size
    sketch_profile.normalize(size=ARGS_N)

    op.sketch_plane = sketch_plane
    op.sketch_pos = sketch_pos
    op.sketch_size = sketch_size
    op.sketch_profile = sketch_profile
    return deepcopy(op)


# 读取当前路径中的宏文件、bounding box并解析，remove_bug表示移除vec中的错误指令使之合法，just_test表示仅用于测试宏文件是否通过解析并重现
def process_on(input_path, catia, doc, remove_bug=False, just_test=False):
    input_macro = sorted(glob.glob(os.path.join(input_path, "*.catvbs")))[0]
    input_json = sorted(glob.glob(os.path.join(input_path, "*.json")))[0]

    with open(input_macro, "r", encoding='UTF-8') as f:
        str = f.read()

    # 通过换行将命令之间分开
    command_list = str.split('\n')
    for i in command_list:
        if i == '':
            command_list.remove(i)

    # 记录拉伸、旋转或其他操作
    extrude_operation = []
    # 直接寻找草图参数
    sketch_para = []
    # 记录实际应该的序号
    trueNo = 0
    # 记录curve真实序号和命名序号的map
    no_map = {}
    sketch_plane = None
    sketch_profile = None
    sketch_pos = None
    sketch_size = None

    # 用于记录旋转的轴是否出现两次
    not_first_flag = False
    first_ref = ''

    all_operation = []
    # 用于记录上一个创建的Spline控制点，与下一个连成线
    spline_point_list = []

    # 标记当前的Extrude、Revolve、Pocket、Groove
    extrude_pocket_point = 0
    # 标记当前操作在所有操作中的序号
    point_in_all = 0

    # 记录多个选取对象
    select_list = []

    # 记录草图数量
    sketch_count = 0

    # 用于记录例如lengthx对应的是哪个body的哪个参数
    parameter_map = {}

    # 负责记录目前出现的倒角、圆角的数量，使得在parameter_map中记录名称
    # 并且辅助body_sketch_map记录body和草图的对应关系
    # 在例如Set length2 = parameters2.Item("Part6\Body.1\Chamfer.1\ChamferRibbon.1\Length1")的时候可以找到对应参数
    operation_count = {'Chamfer': 0, 'Fillet': 0, 'Pad': 0, 'Pocket': 0, 'Shaft': 0, 'Sketch': 0, 'Hole': 0,
                       'Groove': 0}

    # 正常是一个body对应一个草图，但会出现一个草图对应多个body的情况，或者出现多余草图的情况，需要记录
    # 格式：A(int): B(int), 意为当草图的命名序号大于A时，其序号需自加B
    # 例如，若map = {3: 1, 5: -1}, 代表当命名中出现草图sketch.3、sketch.4时，需自加1变为sketch.4, sketch.5， 当出现sketch.5及以后序号时，需先加1再减1
    sketch_offset_map = {}
    # 标记是否一个草图对应一个body，若是则为True
    offset_flag = False

    # 增加一个缓存select的容器，用于暂时保存未被Add的对象，且保存所有点命名和坐标的对应关系
    select_cache = {}
    # 增加一个缓存sketch的容器，用于暂时保存可能被选取的sketch对象
    sketch_cache = {}
    # 增加一个缓存curve的容器，用于暂时保存仍可能被修改的草图curves
    curve_cache = {}
    # 增加一个缓存point的容器，用于保存所有curve的start、endpoint信息
    point_cache = {}
    # 记录当前变量中的sketch序号
    cur_sketch = 0
    # 记录草图变量名和草图命名的一对一关系，因误操作、hole等原因可能导致二者序号不一致
    sketch_name_no_map = {}
    # 记录草图变量对应平面信息, 与point_sketch_map配合使用得到hole对应的平面信息
    sketch_plane_map = {}

    # 存储hybridShapePlaneExplicit和坐标面的对应关系
    hybridShapePlaneExplicit_map = {}
    # 记录当前操作的草图名称
    global_sketch_name = ''

    bounding_box = None
    bounding_size = 0

    # 用于判断openedition是否有对应的closeedition，若没有，则判断为无效草图，直接忽略
    edition_flag = True

    # 每一个body都应该对应一个草图，以此矫正草图序号错乱的情况
    body_sketch_map = {}

    with open(input_json, "r", encoding='UTF-8') as js:
        js = json.load(js)
        bounding_box = js
    for key in bounding_box.keys():
        if bounding_box[key] == "" or bounding_box[key] == "-":
            bounding_box[key] = 0
        elif repr(bounding_box[key])[-2] == 'm' or repr(bounding_box[key])[-2] == 'n':
            bounding_box[key] = bounding_box[key][:-2]
    x_size = float(bounding_box['X_max']) - float(bounding_box['X_min'])
    y_size = float(bounding_box['Y_max']) - float(bounding_box['Y_min'])
    z_size = float(bounding_box['Z_max']) - float(bounding_box['Z_min'])

    bounding_size = max(max(x_size, y_size), z_size)

    # for i in bounding_box:
    #     if i == '' or i == '-':
    #         i = 0
    #     else:
    #         size = float(i)
    #     if bounding_size < abs(size):
    #         bounding_size = abs(size)

    # 指令指针
    command_no = 0
    while command_no < command_list.__len__():
        # 若定义数组，则为定义草图位置信息
        # 出现这句，可能是第一个body的定义，也可能是上一个body的结束
        if 'Dim arrayOfVariantOfDouble' in command_list[command_no]:
            # 若edition_flag为false，说明上一个草图没有closeedition，按无效处理
            if not edition_flag:
                sketch_offset_map[sketch_count] = -1
            edition_flag = False

            # 初始化
            if extrude_operation != []:
                all_operation.append(extrude_operation)
            extrude_operation = []
            sketch_para = []
            trueNo = 0
            sketch_plane = None
            sketch_profile = None
            sketch_pos = None
            sketch_size = None
            not_first_flag = False
            first_ref = ''
            spline_point_list = []
            extrude_pocket_point = 0
            select_list = []

            for i in range(9):
                command_no = command_no + 1
                para_str = command_list[command_no][command_list[command_no].find('=') + 2:]
                sketch_para.append(float(para_str))
            command_no = command_no + 1
            origin = np.array(sketch_para[:3])
            x_axis = np.array(sketch_para[3:6])
            y_axis = np.array(sketch_para[6:])
            z_axis = np.cross(x_axis, y_axis)
            theta, phi, gamma = polar_parameterization(z_axis, x_axis)
            sketch_plane = CoordSystem(origin, theta, phi, gamma, y_axis=cartesian2polar(y_axis))
            sketch_count = sketch_count + 1
            offset_flag = True

            sketch_name = int(command_list[command_no][command_list[command_no].find('sketch') + 6:command_list[command_no].find('.')])
            sketch_plane_map[sketch_name] = deepcopy(sketch_plane)
            sketch_name_no_map['Sketch.' + repr(sketch_count)] = sketch_name
        elif '.InWorkObject = sketch' in command_list[command_no]:
            cur_sketch = int(command_list[command_no][command_list[command_no].find('sketch') + 6:])
        elif '.CreatePoint' in command_list[command_no]:
            para = (command_list[command_no][command_list[command_no].find('(') + 1:-1]).split(',')
            point_name = command_list[command_no][
                         command_list[command_no].find('Set ') + 4:command_list[command_no].find(' =')]
            point_cache[point_name] = [float(para[0]), float(para[1])]
            # 且记录其reportname以及当前草图序号，以便通过命名找到点，例如：sketch.2;4
            tmp_point = command_no + 1
            while tmp_point < len(command_list):
                if point_name + '.ReportName' in command_list[tmp_point]:
                    # 这里有问题，cur_sketch为宏程序里的变量名，而非选取时的命名，选取命名可能因hole的产生而后移
                    for key in sketch_name_no_map:
                        if sketch_name_no_map[key] == cur_sketch:
                            select_cache[key + ';' + command_list[tmp_point][command_list[tmp_point].find('= ') + 2:]] = [float(para[0]), float(para[1])]
                            break
                    break
                else:
                    tmp_point = tmp_point + 1

        # 选取原点
        elif '.GetItem("原点")' in command_list[command_no] or '.GetItem("Origin")' in command_list[command_no]:
            point_name = command_list[command_no][
                         command_list[command_no].find('Set ') + 4:command_list[command_no].find(' =')]
            point_cache[point_name] = [0, 0]
        elif '.StartPoint' in command_list[command_no]:
            curve_name = command_list[command_no][:command_list[command_no].find('.')]
            point_name = command_list[command_no][command_list[command_no].find('= ') + 2:]
            if not point_name in point_cache.keys():
                for tmp_i in command_list:
                    if point_name in tmp_i:
                        if '.GetItem("原点")' in tmp_i or '.GetItem("Origin")' in tmp_i:
                            point_cache[point_name] = [0, 0]
                        elif '.CreatePoint' in tmp_i:
                            para = (command_list[command_no][command_list[command_no].find('(') + 1:-1]).split(',')
                            point_cache[point_name] = [float(para[0]), float(para[1])]
            if command_list[command_no][:command_list[command_no].find('.')] in parameter_map.keys():
                parameter = parameter_map[command_list[command_no][:command_list[command_no].find('.')]]
                curve_cache[parameter.sketch_name][parameter.curve_point].start_point = np.array(
                    [float(point_cache[point_name][0]), float(point_cache[point_name][1])])
        elif '.EndPoint' in command_list[command_no]:
            curve_name = command_list[command_no][:command_list[command_no].find('.')]
            point_name = command_list[command_no][command_list[command_no].find('= ') + 2:]
            if not point_name in point_cache.keys():
                for tmp_i in command_list:
                    if point_name in tmp_i:
                        if '.GetItem("原点")' in tmp_i or '.GetItem("Origin")' in tmp_i:
                            point_cache[point_name] = [0, 0]
                        elif '.CreatePoint' in tmp_i:
                            para = (tmp_i[tmp_i.find('(') + 1:-1]).split(',')
                            point_cache[point_name] = [float(para[0]), float(para[1])]
            if command_list[command_no][:command_list[command_no].find('.')] in parameter_map.keys():
                parameter = parameter_map[command_list[command_no][:command_list[command_no].find('.')]]
                curve_cache[parameter.sketch_name][parameter.curve_point].end_point = np.array(
                    [float(point_cache[point_name][0]), float(point_cache[point_name][1])])
        elif '.OpenEdition()' in command_list[command_no]:
            global_sketch_name = command_list[command_no][
                                 command_list[command_no].find('= ') + 2:command_list[command_no].find('.')]
            if global_sketch_name not in curve_cache.keys():
                curve_cache[global_sketch_name] = []
        elif 'CreateLine' in command_list[command_no]:
            trueNo = trueNo + 1
            para = (command_list[command_no][command_list[command_no].find('(') + 1:-1]).split(',')
            para_name = command_list[command_no][
                        command_list[command_no].find('Set ') + 4:command_list[command_no].find(' =')]
            command_no = command_no + 1
            while 'ReportName' not in command_list[command_no]:
                command_no = command_no + 1
            reportNo = int(command_list[command_no][command_list[command_no].find('=') + 2:])
            start_point = np.array([float(para[0]), float(para[1])])
            end_point = np.array([float(para[2]), float(para[3])])
            curve_cache[global_sketch_name].append(Line(start_point, end_point, reportNo))
            parameter_map[para_name] = Parameter(all_operation.__len__(), extrude_operation.__len__(), 'line',
                                                 curve_cache[global_sketch_name].__len__() - 1,
                                                 sketch_name=global_sketch_name)
        elif 'CreateCircle' in command_list[command_no]:
            trueNo = trueNo + 1
            para = (command_list[command_no][command_list[command_no].find('(') + 1:-1]).split(',')
            para_name = command_list[command_no][
                        command_list[command_no].find('Set ') + 4:command_list[command_no].find(' =')]
            command_no = command_no + 1
            while 'ReportName' not in command_list[command_no]:
                command_no = command_no + 1
            reportNo = int(command_list[command_no][command_list[command_no].find('=') + 2:])
            center = np.array([float(para[0]), float(para[1])])
            start_arc = float(para[3])
            end_arc = float(para[4])
            if start_arc < end_arc:
                mid_arc = (start_arc + end_arc) / 2
            else:
                mid_arc = (start_arc + end_arc + 2 * np.pi) / 2
            curve_cache[global_sketch_name].append(Arc(center, float(para[2]), start_arc, end_arc, mid_arc, reportNo))
            parameter_map[para_name] = Parameter(all_operation.__len__(), extrude_operation.__len__(), 'arc',
                                                 curve_cache[global_sketch_name].__len__() - 1,
                                                 sketch_name=global_sketch_name)
        elif 'CreateClosedCircle' in command_list[command_no]:
            trueNo = trueNo + 1
            para = (command_list[command_no][command_list[command_no].find('(') + 1:-1]).split(',')
            para_name = command_list[command_no][
                        command_list[command_no].find(' ') + 1:command_list[command_no].find('=') - 1]
            command_no = command_no + 1
            while 'ReportName' not in command_list[command_no]:
                command_no = command_no + 1
            reportNo = int(command_list[command_no][command_list[command_no].find('=') + 2:])
            center = np.array([float(para[0]), float(para[1])])
            curve_cache[global_sketch_name].append(Circle(center, float(para[2]), reportNo))
            parameter_map[para_name] = Parameter(all_operation.__len__(), extrude_operation.__len__(), 'circle',
                                                 curve_cache[global_sketch_name].__len__() - 1,
                                                 sketch_name=global_sketch_name)
        # Spline_point
        elif 'CreateControlPoint' in command_list[command_no]:
            point_x = float(
                command_list[command_no][command_list[command_no].find('(') + 1:command_list[command_no].find(',')])
            point_y = float(
                command_list[command_no][command_list[command_no].find(',') + 1:command_list[command_no].find(')')])
            spline_point_list.append(np.array([point_x, point_y]))
        # Spline
        elif 'CreateSpline' in command_list[command_no]:
            while 'ReportName' not in command_list[command_no]:
                command_no = command_no + 1
            reportNo = int(command_list[command_no][command_list[command_no].find('=') + 2:])
            spline_point_list.reverse()
            curve_cache[global_sketch_name].append(Spline(spline_point_list, reportNo))
            spline_point_list = []
        # 若.Construction出现，去掉当前curve的最后一条
        elif '.Construction' in command_list[command_no]:
            if 'line' in command_list[command_no][:command_list[command_no].find('.')] or 'circle' in command_list[
                                                                                                          command_no][
                                                                                                      :command_list[
                                                                                                          command_no].find(
                                                                                                              '.')]:
                curve_cache[global_sketch_name].remove(curve_cache[global_sketch_name][-1])
                parameter_map.pop(command_list[command_no][:command_list[command_no].find('.')])
        # 若CloseEdition，则矫正草图的顺逆顺序
        elif 'CloseEdition' in command_list[command_no]:
            # 修改后不需要再检测草图是否编辑结束了
            if len(curve_cache[global_sketch_name]) > 0:
                sketch_cache[global_sketch_name] = Sketch(deepcopy(curve_cache[global_sketch_name]), sketch_plane,
                                                          sketch_no=sketch_count)
        # 若为AddNewPad, 则为拉伸
        elif 'AddNewPad' in command_list[command_no]:
            # 因为默认定义时参数为20，所以要注意后面是否定义新的修改
            extrude_one = float(
                command_list[command_no][command_list[command_no].find(',') + 1:command_list[command_no].find(')')])
            extrude_two = 0
            sketch_name = command_list[command_no][
                          command_list[command_no].find('(') + 1:command_list[command_no].find(',')]
            isSymmetric, isInverse = False, False
            operation = 'AddFeatureOperation'
            extent_type = 'OffsetLimit'

            # 此处只记录使用的草图名称，因此也不再存在草图和Pad的先后定义问题
            extrude_operation.append(
                Extrude(extrude_one, extrude_two, isSymmetric, isInverse, operation, extent_type, extent_type,
                        sketch_name))

            extrude_pocket_point = extrude_operation.__len__() - 1
            parameter_map[command_list[command_no][
                          command_list[command_no].find(' ') + 1:command_list[command_no].find(' =')]] = Parameter(
                all_operation.__len__(), extrude_operation.__len__() - 1, 'pad')
            if not offset_flag:
                sketch_offset_map[sketch_count + 1] = 1
            offset_flag = False

            # 每出现一个Body，代表对应一个草图
            operation_count['Pad'] = operation_count['Pad'] + 1
            operation_count['Sketch'] = operation_count['Sketch'] + 1
            body_sketch_map['Pad.' + repr(operation_count['Pad'])] = operation_count['Sketch']

        elif 'AddNewPocket' in command_list[command_no]:
            extrude_one = float(
                command_list[command_no][command_list[command_no].find(',') + 1:command_list[command_no].find(')')])
            extrude_two = 0
            sketch_name = command_list[command_no][
                          command_list[command_no].find('(') + 1:command_list[command_no].find(',')]
            isSymmetric, isInverse = False, False
            extent_type = 'OffsetLimit'

            extrude_operation.append(
                Pocket(extrude_one, extrude_two, isSymmetric, isInverse, extent_type, extent_type, sketch_name))

            extrude_pocket_point = extrude_operation.__len__() - 1
            parameter_map[command_list[command_no][
                          command_list[command_no].find(' ') + 1:command_list[command_no].find(' =')]] = Parameter(
                all_operation.__len__(), extrude_operation.__len__() - 1, 'pocket')
            if not offset_flag:
                sketch_offset_map[sketch_count + 1] = 1
            offset_flag = False

            # 每出现一个Body，代表对应一个草图
            operation_count['Pocket'] = operation_count['Pocket'] + 1
            operation_count['Sketch'] = operation_count['Sketch'] + 1
            body_sketch_map['Pocket.' + repr(operation_count['Pocket'])] = operation_count['Sketch']
        elif 'AddNewShaft' in command_list[command_no]:
            select = {}
            angle_one = 360
            angle_two = 0
            sketch_name = command_list[command_no][
                          command_list[command_no].find('(') + 1:command_list[command_no].find(')')]
            isInverse = False
            operation = 'AddFeatureOperation'

            extrude_operation.append(Revolve(select, angle_one, angle_two, isInverse, operation, sketch_name))

            extrude_pocket_point = extrude_operation.__len__() - 1
            parameter_map[command_list[command_no][
                          command_list[command_no].find(' ') + 1:command_list[command_no].find(' =')]] = Parameter(
                all_operation.__len__(), extrude_operation.__len__() - 1, 'shaft')
            if not offset_flag:
                # 因为旋转一定用的是自身新建草图的线作为轴，因此offset从自身开始而非下一个
                sketch_offset_map[sketch_count] = 1
            offset_flag = False
            # 每出现一个Body，代表对应一个草图
            operation_count['Shaft'] = operation_count['Shaft'] + 1
            operation_count['Sketch'] = operation_count['Sketch'] + 1
            body_sketch_map['Shaft.' + repr(operation_count['Shaft'])] = operation_count['Sketch']
        elif 'AddNewGroove' in command_list[command_no]:
            select = {}
            angle_one = 360
            angle_two = 0
            sketch_name = command_list[command_no][
                          command_list[command_no].find('(') + 1:command_list[command_no].find(')')]
            isInverse = False

            extrude_operation.append(Groove(select, angle_one, angle_two, isInverse, sketch_name))

            extrude_pocket_point = extrude_operation.__len__() - 1
            parameter_map[command_list[command_no][
                          command_list[command_no].find(' ') + 1:command_list[command_no].find(' =')]] = Parameter(
                all_operation.__len__(), extrude_operation.__len__() - 1, 'groove')
            if not offset_flag:
                # 因为旋转一定用的是自身新建草图的线作为轴，因此offset从自身开始而非下一个
                sketch_offset_map[sketch_count] = 1
            offset_flag = False
            # 每出现一个Body，代表对应一个草图
            operation_count['Groove'] = operation_count['Groove'] + 1
            operation_count['Sketch'] = operation_count['Sketch'] + 1
            body_sketch_map['Groove.' + repr(operation_count['Groove'])] = operation_count['Sketch']
        elif '.SetProfileElement' in command_list[command_no]:
            ref_name = command_list[command_no][command_list[command_no].find(' ') + 1:]
            # 寻找定义该ref的语句
            tmp_point = command_no - 1
            while tmp_point >= 0:
                if 'Set ' + ref_name in command_list[tmp_point]:
                    sketch_name = command_list[tmp_point][command_list[tmp_point].find('(') + 1:-1]
                    if command_list[command_no][:command_list[command_no].find('.')] in parameter_map.keys():
                        parameter = parameter_map[command_list[command_no][:command_list[command_no].find('.')]]
                        body_point = parameter.body_point
                        op_point = parameter.op_point
                        para_type = parameter.para_type
                        if all_operation.__len__() <= body_point:
                            extrude_operation[op_point].sketch_name = sketch_name
                        else:
                            all_operation[body_point][op_point].sketch_name = sketch_name
                    break
                else:
                    tmp_point = tmp_point - 1
        elif 'AddNewAdd' in command_list[command_no]:
            extrude_operation[extrude_pocket_point].operation = BOOLEAN_OPERATIONS.index('AddFeatureOperation')
        elif 'AddNewIntersect' in command_list[command_no]:
            extrude_operation[extrude_pocket_point].operation = BOOLEAN_OPERATIONS.index('IntersectFeatureOperation')
        elif 'AddNewRemove' in command_list[command_no]:
            extrude_operation[extrude_pocket_point].operation = BOOLEAN_OPERATIONS.index('CutFeatureOperation')
        # 添加upto...等新类型
        elif '.FirstLimit' in command_list[command_no]:
            body_name = command_list[command_no][
                        command_list[command_no].find('= ') + 2:command_list[command_no].find('.FirstLimit')]
            limit_name = command_list[command_no][
                         command_list[command_no].find('Set ') + 4:command_list[command_no].find(' =')]
            while '.Value' not in command_list[command_no] and '.LimitMode' not in command_list[command_no]:
                command_no = command_no + 1
            if '.Value' in command_list[command_no]:
                extrude_one = float(command_list[command_no][command_list[command_no].find('=') + 2:])
                if not body_name in parameter_map.keys():
                    extrude_operation[extrude_pocket_point].extent_one = extrude_one
                else:
                    parameter = parameter_map[body_name]
                    body_point = parameter.body_point
                    op_point = parameter.op_point
                    para_type = parameter.para_type
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].extent_one = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                    else:
                        all_operation[body_point][op_point].extent_one = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                parameter_map[command_list[command_no][:command_list[command_no].find('.')]] = Parameter(
                    all_operation.__len__(), extrude_operation.__len__() - 1, 'extent_one')
            elif '.LimitMode' in command_list[command_no]:
                extent_type = command_list[command_no][command_list[command_no].find('cat') + 3:]
                extrude_operation[extrude_pocket_point].extent_type1 = extent_type
                parameter_map[limit_name] = Parameter(all_operation.__len__(), extrude_operation.__len__() - 1,
                                                      'extent_type1')
                # 若为UpToPlaneLimit或UpToSurfaceLimit，还需记录选取面
                if extent_type == 'UpToLastLimit' or extent_type == 'UpToSurfaceLimit':
                    while '.LimitingElement' not in command_list[command_no]:
                        command_no = command_no + 1
                    limit_ref = command_list[command_no][command_list[command_no].find('= ') + 2:]
                    if extrude_operation[extrude_pocket_point].select_list is None:
                        extrude_operation[extrude_pocket_point].select_list = {'length1': select_cache[limit_ref]}
                    else:
                        extrude_operation[extrude_pocket_point].select_list['length1'] = select_cache[limit_ref]

        elif '.SecondLimit' in command_list[command_no]:
            body_name = command_list[command_no][
                        command_list[command_no].find('= ') + 2:command_list[command_no].find('.FirstLimit')]
            limit_name = command_list[command_no][
                         command_list[command_no].find('Set ') + 4:command_list[command_no].find(' =')]
            while '.Value' not in command_list[command_no] and '.LimitMode' not in command_list[command_no]:
                command_no = command_no + 1
            if '.Value' in command_list[command_no]:
                extrude_two = float(command_list[command_no][command_list[command_no].find('=') + 2:])
                extrude_operation[extrude_pocket_point].extent_two = extrude_two
                parameter_map[command_list[command_no][:command_list[command_no].find('.')]] = Parameter(
                    all_operation.__len__(), extrude_operation.__len__() - 1, 'extent_two')
            elif '.LimitMode' in command_list[command_no]:
                extent_type = command_list[command_no][command_list[command_no].find('cat') + 3:]
                extrude_operation[extrude_pocket_point].extent_type2 = extent_type
                parameter_map[limit_name] = Parameter(all_operation.__len__(), extrude_operation.__len__() - 1,
                                                      'extent_type2')
                print('extent_type2 changes!')
                # 若为UpToPlaneLimit或UpToSurfaceLimit，还需记录选取面
                if extent_type == 'UpToLastLimit' or extent_type == 'UpToSurfaceLimit':
                    while '.LimitingElement' not in command_list[command_no]:
                        command_no = command_no + 1
                    limit_ref = command_list[command_no][command_list[command_no].find('= ') + 2:]
                    if extrude_operation[extrude_pocket_point].select_list is None:
                        extrude_operation[extrude_pocket_point].select_list = {'length2': select_cache[limit_ref]}
                    else:
                        extrude_operation[extrude_pocket_point].select_list['length2'] = select_cache[limit_ref]
        elif '.FirstAngle' in command_list[command_no]:
            while '.Value' not in command_list[command_no]:
                command_no = command_no + 1
            angle_one = float(command_list[command_no][command_list[command_no].find('=') + 2:])
            extrude_operation[extrude_pocket_point].angle_one = angle_one
            parameter_map[command_list[command_no][:command_list[command_no].find('.')]] = Parameter(
                all_operation.__len__(), extrude_operation.__len__() - 1, 'angle_one')
        elif '.SecondAngle' in command_list[command_no]:
            while '.Value' not in command_list[command_no]:
                command_no = command_no + 1
            angle_two = float(command_list[command_no][command_list[command_no].find('=') + 2:])
            extrude_operation[extrude_pocket_point].angle_two = angle_two
            parameter_map[command_list[command_no][:command_list[command_no].find('.')]] = Parameter(
                all_operation.__len__(), extrude_operation.__len__() - 1, 'angle_two')
        elif '.IsSymmetric' in command_list[command_no]:
            if command_list[command_no][:command_list[command_no].find('.')] in parameter_map.keys():
                parameter = parameter_map[command_list[command_no][:command_list[command_no].find('.')]]
                body_point = parameter.body_point
                op_point = parameter.op_point
                para_type = parameter.para_type
                if all_operation.__len__() <= body_point:
                    extrude_operation[op_point].isSymmetric = command_list[command_no][
                                                              command_list[command_no].find('=') + 2:] == 'True'
                else:
                    all_operation[body_point][op_point].isSymmetric = command_list[command_no][
                                                                      command_list[command_no].find('=') + 2:] == 'True'
        elif 'catInverseOrientation' in command_list[command_no]:
            if command_list[command_no][:command_list[command_no].find('.')] in parameter_map.keys():
                parameter = parameter_map[command_list[command_no][:command_list[command_no].find('.')]]
                body_point = parameter.body_point
                op_point = parameter.op_point
                para_type = parameter.para_type
                if all_operation.__len__() <= body_point:
                    extrude_operation[op_point].isInverse = True
                else:
                    all_operation[body_point][op_point].isInverse = True
        elif 'catRegularOrientation' in command_list[command_no]:
            if command_list[command_no][:command_list[command_no].find('.')] in parameter_map.keys():
                parameter = parameter_map[command_list[command_no][:command_list[command_no].find('.')]]
                body_point = parameter.body_point
                op_point = parameter.op_point
                para_type = parameter.para_type
                if all_operation.__len__() <= body_point:
                    extrude_operation[op_point].isInverse = False
                else:
                    all_operation[body_point][op_point].isInverse = False
        # 选取坐标轴
        elif 'Set reference' in command_list[command_no] and (
                '.GetItem("横向")' in command_list[command_no] or '.GetItem("HDirection")' in command_list[command_no]):
            ref_name = command_list[command_no][
                       command_list[command_no].find(' ') + 1:command_list[command_no].find('=') - 1]
            select_cache[ref_name] = Select('Wire', 'OriginElements', 0, 1)
        elif 'Set reference' in command_list[command_no] and (
                '.GetItem("纵向")' in command_list[command_no] or '.GetItem("VDirection")' in command_list[command_no]):
            ref_name = command_list[command_no][
                       command_list[command_no].find(' ') + 1:command_list[command_no].find('=') - 1]
            select_cache[ref_name] = Select('Wire', 'OriginElements', 0, 2)
        # 选取坐标面
        elif '.PlaneXY' in command_list[command_no]:
            hybridShapePlaneExplicit_name = command_list[command_no][
                                            command_list[command_no].find('hybridShapePlaneExplicit'):command_list[
                                                command_no].find(' = ')]
            if hybridShapePlaneExplicit_name != '':
                hybridShapePlaneExplicit_map[hybridShapePlaneExplicit_name] = Select('Face', 'OriginElements', 0, 1)
        elif '.PlaneYZ' in command_list[command_no]:
            hybridShapePlaneExplicit_name = command_list[command_no][
                                            command_list[command_no].find('hybridShapePlaneExplicit'):command_list[
                                                command_no].find(' = ')]
            if hybridShapePlaneExplicit_name != '':
                hybridShapePlaneExplicit_map[hybridShapePlaneExplicit_name] = Select('Face', 'OriginElements', 0, 2)
        elif '.PlaneZX' in command_list[command_no]:
            hybridShapePlaneExplicit_name = command_list[command_no][
                                            command_list[command_no].find('hybridShapePlaneExplicit'):command_list[
                                                command_no].find(' = ')]
            if hybridShapePlaneExplicit_name != '':
                hybridShapePlaneExplicit_map[hybridShapePlaneExplicit_name] = Select('Face', 'OriginElements', 0, 3)
        # 旋转的Inverse很奇怪，每重新选择一次相同的轴引用则为一次颠倒
        elif '.RevoluteAxis' in command_list[command_no]:
            ref = command_list[command_no][command_list[command_no].find('reference'):]
            extrude_operation[extrude_pocket_point].select_list[ref] = select_cache[ref]
            if not_first_flag:
                if ref == first_ref:
                    extrude_operation[extrude_pocket_point].isInverse = not extrude_operation[
                        extrude_pocket_point].isInverse
                else:
                    first_ref = ref
            else:
                not_first_flag = True
                first_ref = ref

        # 圆角，后面可能会修改，所以先保存
        elif 'AddNewSolidEdgeFilletWithConstantRadius' in command_list[command_no]:
            para_list = command_list[command_no][command_list[command_no].find('(') + 1:-1].split(', ')
            radius = float(para_list[2])
            select_mode = para_list[1]
            select_ref = {}
            extrude_operation.append(Fillet(select_ref, radius, select_mode))
            operation_count['Fillet'] = operation_count['Fillet'] + 1
            parameter_map['Fillet.' + repr(operation_count['Fillet'])] = Parameter(all_operation.__len__(),
                                                                                   extrude_operation.__len__() - 1,
                                                                                   'Fillet')
        elif '\\Radius' in command_list[command_no] and 'EdgeFillet' in command_list[command_no]:
            para_name = command_list[command_no][
                        command_list[command_no].find(' ') + 1:command_list[command_no].find('=') - 1]
            operation_name = command_list[command_no][command_list[command_no].find('EdgeFillet'):]
            operation_name = operation_name[:operation_name.find('\\')]
            operation_no = operation_name[operation_name.find('.') + 1:]
            operation_name = 'Fillet.' + operation_no
            if operation_name in parameter_map.keys():
                parameter = parameter_map[operation_name]
                body_point = parameter.body_point
                op_point = parameter.op_point
                para_type = parameter.para_type
                parameter_map[para_name] = Parameter(body_point, op_point, 'Radius')
        elif '\\半径' in command_list[command_no] and '倒圆角' in command_list[command_no]:
            para_name = command_list[command_no][
                        command_list[command_no].find(' ') + 1:command_list[command_no].find('=') - 1]
            operation_name = command_list[command_no][command_list[command_no].find('倒圆角'):]
            operation_name = operation_name[:operation_name.find('\\')]
            operation_no = operation_name[operation_name.find('.') + 1:]
            operation_name = 'Fillet.' + operation_no
            if operation_name in parameter_map.keys():
                parameter = parameter_map[operation_name]
                body_point = parameter.body_point
                op_point = parameter.op_point
                para_type = parameter.para_type
                parameter_map[para_name] = Parameter(body_point, op_point, 'Radius')
        elif '.EdgePropagation' in command_list[command_no]:
            extrude_operation[extrude_operation.__len__() - 1].edgePropagation = command_list[command_no][
                                                                                 command_list[command_no].find(
                                                                                     '=') + 2:]
        # 若出现选取，则为extrude_operation中最后一条指令的对象
        # 利用栈将命名取出
        elif 'CreateReferenceFromBRepName' in command_list[command_no]:
            # 先查看字符串内是否有" & "，若有，去掉
            while '" & "' in command_list[command_no]:
                command_list[command_no] = command_list[command_no][:command_list[command_no].find('" & "')] + \
                                           command_list[command_no][
                                           command_list[command_no].find('" & "') + len('" & "'):]
            # 若为点，则立即进行处理

            # 由于将草图解析放在最后，选取受到影响，也需要放到最后解析
            select_name = command_list[command_no][
                          command_list[command_no].find(' ') + 1:command_list[command_no].find(' =')]
            select_cache[select_name] = command_list[command_no]

        elif 'CreateReferenceFromObject' in command_list[command_no]:
            hybridShapePlaneExplicit_name = command_list[command_no][
                                            command_list[command_no].find('hybridShapePlaneExplicit'):-1]
            ref_name = command_list[command_no][
                       command_list[command_no].find('reference'):command_list[command_no].find(' = ')]
            if hybridShapePlaneExplicit_name in hybridShapePlaneExplicit_map.keys():
                select_cache[ref_name] = hybridShapePlaneExplicit_map[hybridShapePlaneExplicit_name]
        elif 'NeutralElement' in command_list[command_no]:
            select_name = command_list[command_no][command_list[command_no].find('reference'):]
            extrude_operation[extrude_operation.__len__() - 1].Neutral = select_cache[select_name]
            extrude_operation[extrude_operation.__len__() - 1].Parting = select_cache[select_name]
        # elif 'PullingDirectionElement' in command_list[command_no]:
        #     select_name = command_list[command_no][command_list[command_no].find('reference'):]
        #     extrude_operation[extrude_operation.__len__() - 1].Parting = select_cache[select_name]
        elif 'AddFaceToDraft' in command_list[command_no] or 'AddFaceToRemove' in command_list[command_no] or \
                'AddObjectToFillet' in command_list[command_no] or 'AddElementToChamfer' in command_list[command_no]:
            select_name = command_list[command_no][command_list[command_no].find('reference'):]
            extrude_operation[extrude_operation.__len__() - 1].select_list[select_name] = select_cache[select_name]
        # 若出现remove、withdraw等，说明取消了某选取
        elif 'WithdrawElementToChamfer' in command_list[command_no] or 'RemoveFaceToDraft' in command_list[
            command_no] or \
                'WithdrawFaceToRemove' in command_list[command_no] or 'WithdrawObjectToFillet' in command_list[
            command_no]:
            select_name = command_list[command_no][command_list[command_no].find(' ') + 1:]
            extrude_operation[extrude_operation.__len__() - 1].select_list.pop(select_name)
        elif '.BottomLimit' in command_list[command_no]:
            hole_name = command_list[command_no][command_list[command_no].find('= ') + 2:command_list[command_no].find('.BottomLimit')]
            if hole_name in parameter_map.keys():
                limit_name = command_list[command_no][command_list[command_no].find(' ') + 1:command_list[command_no].find(' = ')]
                parameter_map[limit_name] = Parameter(parameter_map[hole_name].body_point, parameter_map[hole_name].op_point, 'bottom_mode')
        elif '.Diameter' in command_list[command_no]:
            hole_name = command_list[command_no][command_list[command_no].find('= ') + 2:command_list[command_no].find('.Diameter')]
            if hole_name in parameter_map.keys():
                limit_name = command_list[command_no][command_list[command_no].find(' ') + 1:command_list[command_no].find(' = ')]
                parameter_map[limit_name] = Parameter(parameter_map[hole_name].body_point, parameter_map[hole_name].op_point, 'hole_diameter')
        elif 'AddNewHoleFromRefPoint' in command_list[command_no]:
            para_list = command_list[command_no][command_list[command_no].find('(') + 1:-1].split(', ')
            point_ref = para_list[0]
            plane_ref = para_list[1]
            depth = float(para_list[2])
            diameter = 10
            radius = diameter / 2.0
            extrude_operation.append(Hole(select_cache[point_ref], select_cache[plane_ref], radius, depth, 'OffsetLimit'))
            operation_count['Hole'] = operation_count['Hole'] + 1
            hole_name = command_list[command_no][command_list[command_no].find(' ') + 1:command_list[command_no].find(' = ')]
            parameter_map[hole_name] = Parameter(all_operation.__len__(), extrude_operation.__len__() - 1, 'Hole')

            # 因为hole的添加会奇怪地多添加两个草图，因此草图数量+2
            sketch_count = sketch_count + 2
        elif 'AddNewHoleFromPoint' in command_list[command_no]:
            # 虽然不处理此种情况，但草图数量仍会增加，需处理
            sketch_count = sketch_count + 2
        elif 'AddNewChamfer' in command_list[command_no]:
            para_list = command_list[command_no][command_list[command_no].find('(') + 1:-1].split(', ')
            length1 = float(para_list[-2])
            angle_or_length2 = float(para_list[-1])
            mode = para_list[2]
            orientation = para_list[3]
            propagation = para_list[1]
            select_ref = {}
            extrude_operation.append(Chamfer(select_ref, length1, angle_or_length2, mode, propagation, orientation))
            operation_count['Chamfer'] = operation_count['Chamfer'] + 1
            parameter_map['Chamfer.' + repr(operation_count['Chamfer'])] = Parameter(all_operation.__len__(),
                                                                                     extrude_operation.__len__() - 1,
                                                                                     'Chamfer')
        elif '.Mode' in command_list[command_no] and 'chamfer' in command_list[command_no]:
            extrude_operation[extrude_operation.__len__() - 1].mode = command_list[command_no][
                                                                      command_list[command_no].find('=') + 2:]
            if extrude_operation[extrude_operation.__len__() - 1].mode == 'catTwoLengthChamfer':
                extrude_operation[extrude_operation.__len__() - 1].angle_or_length2 = 1
        elif '.Propagation' in command_list[command_no] and 'chamfer' in command_list[command_no]:
            extrude_operation[extrude_operation.__len__() - 1].propagation = command_list[command_no][
                                                                             command_list[command_no].find('=') + 2:]
        elif '.Orientation' in command_list[command_no] and 'chamfer' in command_list[command_no]:
            extrude_operation[extrude_operation.__len__() - 1].orientation = command_list[command_no][
                                                                             command_list[command_no].find('=') + 2:]
        elif '\\长度 1' in command_list[command_no] and '\\倒角' in command_list[command_no]:
            para_name = command_list[command_no][
                        command_list[command_no].find(' ') + 1:command_list[command_no].find('=') - 1]
            operation_name = command_list[command_no][command_list[command_no].find('倒角'):]
            operation_name = operation_name[:operation_name.find('\\')]
            operation_no = operation_name[operation_name.find('.') + 1:]
            operation_name = 'Chamfer.' + operation_no
            if operation_name in parameter_map.keys():
                parameter = parameter_map[operation_name]
                body_point = parameter.body_point
                op_point = parameter.op_point
                para_type = parameter.para_type
                parameter_map[para_name] = Parameter(body_point, op_point, 'Length1')
        elif '\\Length1' in command_list[command_no] and '\\Chamfer' in command_list[command_no]:
            para_name = command_list[command_no][
                        command_list[command_no].find(' ') + 1:command_list[command_no].find('=') - 1]
            operation_name = command_list[command_no][command_list[command_no].find('Chamfer'):]
            operation_name = operation_name[:operation_name.find('\\')]
            operation_no = operation_name[operation_name.find('.') + 1:]
            operation_name = 'Chamfer.' + operation_no
            if operation_name in parameter_map.keys():
                parameter = parameter_map[operation_name]
                body_point = parameter.body_point
                op_point = parameter.op_point
                para_type = parameter.para_type
                parameter_map[para_name] = Parameter(body_point, op_point, 'Length1')
        elif '\\角度' in command_list[command_no] or '\\长度 2' in command_list[command_no]:
            para_name = command_list[command_no][
                        command_list[command_no].find(' ') + 1:command_list[command_no].find('=') - 1]
            operation_name = command_list[command_no][command_list[command_no].find('倒角'):]
            operation_name = operation_name[:operation_name.find('\\')]
            operation_no = operation_name[operation_name.find('.') + 1:]
            operation_name = 'Chamfer.' + operation_no
            if operation_name in parameter_map.keys():
                parameter = parameter_map[operation_name]
                body_point = parameter.body_point
                op_point = parameter.op_point
                para_type = parameter.para_type
                parameter_map[para_name] = Parameter(body_point, op_point, 'Angle_or_length2')
        elif '\\Angle' in command_list[command_no] or '\\Length2' in command_list[command_no]:
            para_name = command_list[command_no][
                        command_list[command_no].find(' ') + 1:command_list[command_no].find('=') - 1]
            operation_name = command_list[command_no][command_list[command_no].find('Chamfer'):]
            operation_name = operation_name[:operation_name.find('\\')]
            operation_no = operation_name[operation_name.find('.') + 1:]
            operation_name = 'Chamfer.' + operation_no
            if operation_name in parameter_map.keys():
                parameter = parameter_map[operation_name]
                body_point = parameter.body_point
                op_point = parameter.op_point
                para_type = parameter.para_type
                parameter_map[para_name] = Parameter(body_point, op_point, 'Angle_or_length2')
        elif 'AddNewShell' in command_list[command_no]:
            para_list = command_list[command_no][command_list[command_no].find('(') + 1:-1].split(', ')
            select_ref = {}
            internalThickness = float(para_list[1])
            externalThickness = float(para_list[2])
            shell = Shell(select_ref, internalThickness, externalThickness)
            extrude_operation.append(shell)
        elif 'InternalThickness' in command_list[command_no]:
            command_no = command_no + 1
            extrude_operation[extrude_operation.__len__() - 1].thickness = float(
                command_list[command_no][command_list[command_no].find('=') + 2:])
        elif 'ExternalThickness' in command_list[command_no]:
            command_no = command_no + 1
            extrude_operation[extrude_operation.__len__() - 1].second_thickness = float(
                command_list[command_no][command_list[command_no].find('=') + 2:])
        # 若.Value单独出现，则可能为单独修改，也可能为不需要考虑的参数定义
        elif '.Value' in command_list[command_no] or '.MirroringPlane' in command_list[command_no] or \
                '.SetData' in command_list[command_no] or '.LimitMode' in command_list[command_no] or \
                '.LimitingElement' in command_list[command_no]:
            if command_list[command_no][:command_list[command_no].find('.')] in parameter_map.keys():
                parameter = parameter_map[command_list[command_no][:command_list[command_no].find('.')]]
                body_point = parameter.body_point
                op_point = parameter.op_point
                para_type = parameter.para_type
                if para_type == 'extent_one':
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].extent_one = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                    else:
                        all_operation[body_point][op_point].extent_one = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                elif para_type == 'extent_two':
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].extent_two = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                    else:
                        all_operation[body_point][op_point].extent_two = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                elif para_type == 'hole_diameter':
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].radius = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:]) / 2
                    else:
                        all_operation[body_point][op_point].radius = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:]) / 2
                elif para_type == 'depth':
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].depth = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                    else:
                        all_operation[body_point][op_point].depth = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                elif para_type == 'bottom_mode' and '.LimitMode' in command_list[command_no]:
                    bottom_mode = command_list[command_no][command_list[command_no].find('cat') + 3:]
                    if bottom_mode == 'OffsetLimit':
                        # 向下搜索该limit的Dimension
                        limit_name = command_list[command_no][:command_list[command_no].find('.')]
                        tmp_p = command_no + 1
                        while tmp_p < len(command_list):
                            if not (limit_name in command_list[tmp_p] and '.Dimension' in command_list[tmp_p]):
                                tmp_p = tmp_p + 1
                            else:
                                break
                        if tmp_p < len(command_list):
                            length_name = command_list[tmp_p][command_list[tmp_p].find(' ') + 1:command_list[tmp_p].find(' = ')]
                            parameter_map[length_name] = Parameter(body_point, op_point, 'depth')
                elif para_type == 'bottom_mode' and '.LimitingElement' in command_list[command_no]:
                    limit_ref = command_list[command_no][command_list[command_no].find('= ') + 2:]
                    if all_operation.__len__() <= body_point:
                        if extrude_operation[op_point].select_list is None:
                            extrude_operation[op_point].select_list = {limit_ref: select_cache[limit_ref]}
                        else:
                            extrude_operation[op_point].select_list[limit_ref] = select_cache[limit_ref]

                    else:
                        if all_operation[body_point][op_point].select_list is None:
                            all_operation[body_point][op_point].select_list = {limit_ref: select_cache[limit_ref]}
                        else:
                            all_operation[body_point][op_point].select_list[limit_ref] = select_cache[limit_ref]
                elif para_type == 'extent_type1' and '.LimitMode' in command_list[command_no]:
                    extent_type1 = command_list[command_no][command_list[command_no].find('cat') + 3:]
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].extent_type1 = extent_type1
                        # 正常情况下第一次设置不会是OffsetLimit，而在修改时可能改回OffsetLimit
                        if extent_type1 == 'OffsetLimit':
                            while '.Value' not in command_list[command_no]:
                                command_no = command_no + 1
                            if '.Value' in command_list[command_no]:
                                extrude_one = float(command_list[command_no][command_list[command_no].find('=') + 2:])
                                extrude_operation[op_point].extent_one = extrude_one
                                parameter_map[
                                    command_list[command_no][:command_list[command_no].find('.')]] = Parameter(
                                    body_point, op_point, 'extent_one')
                    else:
                        all_operation[body_point][op_point].extent_type1 = extent_type1
                        if extent_type1 == 'OffsetLimit':
                            while '.Value' not in command_list[command_no]:
                                command_no = command_no + 1
                            if '.Value' in command_list[command_no]:
                                extrude_one = float(command_list[command_no][command_list[command_no].find('=') + 2:])
                                all_operation[body_point][op_point].extent_one = extrude_one
                                parameter_map[
                                    command_list[command_no][:command_list[command_no].find('.')]] = Parameter(
                                    body_point, op_point, 'extent_one')
                elif para_type == 'extent_type2' and '.LimitMode' in command_list[command_no]:
                    print('extent_type2 changes!')
                    extent_type2 = command_list[command_no][command_list[command_no].find('cat') + 3:]
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].extent_type2 = extent_type2
                        # 正常情况下第一次设置不会是OffsetLimit，而在修改时可能改回OffsetLimit
                        if extent_type2 == 'OffsetLimit':
                            while '.Value' not in command_list[command_no]:
                                command_no = command_no + 1
                            if '.Value' in command_list[command_no]:
                                extrude_two = float(command_list[command_no][command_list[command_no].find('=') + 2:])
                                extrude_operation[op_point].extent_two = extrude_two
                                parameter_map[
                                    command_list[command_no][:command_list[command_no].find('.')]] = Parameter(
                                    body_point, op_point, 'extent_two')
                    else:
                        all_operation[body_point][op_point].extent_type2 = extent_type2
                        if extent_type2 == 'OffsetLimit':
                            while '.Value' not in command_list[command_no]:
                                command_no = command_no + 1
                            if '.Value' in command_list[command_no]:
                                extrude_two = float(command_list[command_no][command_list[command_no].find('=') + 2:])
                                all_operation[body_point][op_point].extent_two = extrude_two
                                parameter_map[
                                    command_list[command_no][:command_list[command_no].find('.')]] = Parameter(
                                    body_point, op_point, 'extent_two')
                elif para_type == 'extent_type1' and '.LimitingElement' in command_list[command_no]:
                    limit_ref = command_list[command_no][command_list[command_no].find('= ') + 2:]
                    if all_operation.__len__() <= body_point:
                        if extrude_operation[op_point].select_list is None:
                            extrude_operation[op_point].select_list = {'length1': select_cache[limit_ref]}
                        else:
                            extrude_operation[op_point].select_list['length1'] = select_cache[limit_ref]

                    else:
                        if all_operation[body_point][op_point].select_list is None:
                            all_operation[body_point][op_point].select_list = {'length1': select_cache[limit_ref]}
                        else:
                            all_operation[body_point][op_point].select_list['length1'] = select_cache[limit_ref]
                elif para_type == 'extent_type2' and '.LimitingElement' in command_list[command_no]:
                    limit_ref = command_list[command_no][command_list[command_no].find('= ') + 2:]
                    if all_operation.__len__() <= body_point:
                        if extrude_operation[op_point].select_list is None:
                            extrude_operation[op_point].select_list = {'length2': select_cache[limit_ref]}
                        else:
                            extrude_operation[op_point].select_list['length2'] = select_cache[limit_ref]

                    else:
                        if all_operation[body_point][op_point].select_list is None:
                            all_operation[body_point][op_point].select_list = {'length2': select_cache[limit_ref]}
                        else:
                            all_operation[body_point][op_point].select_list['length2'] = select_cache[limit_ref]
                elif para_type == 'angle_one':
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].angle_one = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                    else:
                        all_operation[body_point][op_point].angle_one = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                elif para_type == 'angle_two':
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].extent_two = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                    else:
                        all_operation[body_point][op_point].angle_two = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                elif para_type == 'Length1':
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].length1 = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                    else:
                        all_operation[body_point][op_point].length1 = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                elif para_type == 'Angle_or_length2':
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].angle_or_length2 = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                    else:
                        all_operation[body_point][op_point].angle_or_length2 = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                elif para_type == 'Radius':
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].radius = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                    else:
                        all_operation[body_point][op_point].radius = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                elif para_type == 'DraftAngle':
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].DraftAngle = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                    else:
                        all_operation[body_point][op_point].DraftAngle = float(
                            command_list[command_no][command_list[command_no].find('=') + 2:])
                elif para_type == 'MirrorPlane':
                    select_name = command_list[command_no][command_list[command_no].find('=') + 2:]
                    if all_operation.__len__() <= body_point:
                        extrude_operation[op_point].select_list = [select_cache[select_name]]
                    else:
                        all_operation[body_point][op_point].select_list = [select_cache[select_name]]
                elif para_type == 'circle':
                    para_list = command_list[command_no].split(', ')
                    curve_cache[parameter.sketch_name][parameter.curve_point].center = np.array(
                        [float(para_list[0][para_list[0].find(' ') + 1:]), float(para_list[1])])
                    curve_cache[parameter.sketch_name][parameter.curve_point].radius = float(para_list[2])
                # elif para_type == 'line':
                #     para_list = command_list[command_no].split(', ')
                #     curve_cache[parameter.sketch_name][parameter.curve_point].start_point = np.array([float(para_list[0][para_list[0].find(' ') + 1:]), float(para_list[1])])
                #     curve_cache[parameter.sketch_name][parameter.curve_point].end_point = np.array([float(para_list[2]), float(para_list[3])])
                elif para_type == 'arc':
                    print('arc appear!')
        # 若.DraftValue单独出现，则可能为单独修改，也可能为不需要考虑的参数定义
        elif '.DraftAngle' in command_list[command_no]:
            while '.Value' not in command_list[command_no]:
                command_no = command_no + 1
            draft_angle = float(command_list[command_no][command_list[command_no].find('=') + 2:])
            extrude_operation[extrude_operation.__len__() - 1].DraftAngle = draft_angle
            parameter_map[command_list[command_no][:command_list[command_no].find('.')]] = Parameter(
                all_operation.__len__(), extrude_operation.__len__() - 1, 'DraftAngle')
        elif 'AddNewDraft' in command_list[command_no]:
            para_list = command_list[command_no][command_list[command_no].find('(') + 1:-1].split(', ')
            draft = Draft({}, None, None, [float(para_list[4]), float(para_list[5]), float(para_list[6])],
                          float(para_list[8]), para_list[2], para_list[7], para_list[9])
            extrude_operation.append(draft)
        elif 'SetPullingDirection' in command_list[command_no]:
            command_str = command_list[command_no][command_list[command_no].find(' ') + 1:]
            para_list = command_str.split(', ')
            extrude_operation[extrude_operation.__len__() - 1].dir = [float(para_list[0]), float(para_list[1]),
                                                                      float(para_list[2])]
        elif 'AddNewMirror' in command_list[command_no]:
            # 镜面操作是现选取再操作，而非先使用空引用操作
            select_name = command_list[command_no][command_list[command_no].find('(') + 1:-1]
            select_list = {select_name: select_cache[select_name]}
            mirror = Mirror(select_list)
            extrude_operation.append(mirror)
            parameter_map[command_list[command_no][:command_list[command_no].find('.')]] = Parameter(
                all_operation.__len__(), extrude_operation.__len__() - 1, 'MirroringPlane')
        command_no = command_no + 1

    if extrude_operation != []:
        all_operation.append(extrude_operation)
    all_operation = np.array(all_operation, dtype=object)
    all_operation = np.concatenate(all_operation, axis=0)

    # 对于所有草图集中进行归一化
    for i in range(all_operation.shape[0]):
        if isinstance(all_operation[i], Extrude) or isinstance(all_operation[i], Pocket) or \
                isinstance(all_operation[i], Revolve) or isinstance(all_operation[i], Groove):
            all_operation[i] = process_sketch(all_operation[i], curve_cache, no_map, sketch_cache)

    # 草图规范化后解析选取
    tmp_shaft_count = 0
    tmp_groove_count = 0
    for i in range(all_operation.shape[0]):
        if isinstance(all_operation[i], Chamfer) or isinstance(all_operation[i], Fillet) or \
                isinstance(all_operation[i], Shell) or isinstance(all_operation[i], Mirror) or \
                isinstance(all_operation[i], Draft) or isinstance(all_operation[i], Revolve) or \
                isinstance(all_operation[i], Groove):
            if isinstance(all_operation[i], Revolve):
                tmp_shaft_count = tmp_shaft_count + 1
            elif isinstance(all_operation[i], Groove):
                tmp_groove_count = tmp_groove_count + 1
            elif isinstance(all_operation[i], Draft):
                neutral_select = parse_select(all_operation[i].Neutral, no_map, sketch_offset_map, body_sketch_map,
                                              tmp_shaft_count, tmp_groove_count)
                parting_select = parse_select(all_operation[i].Parting, no_map, sketch_offset_map, body_sketch_map,
                                              tmp_shaft_count, tmp_groove_count)
                all_operation[i].Neutral = neutral_select
                all_operation[i].Parting = parting_select
            for key in all_operation[i].select_list.keys():
                select = parse_select(all_operation[i].select_list[key], no_map, sketch_offset_map, body_sketch_map,
                                      tmp_shaft_count, tmp_groove_count, isinstance(all_operation[i], Revolve))
                all_operation[i].select_list[key] = select
        elif isinstance(all_operation[i], Extrude) or isinstance(all_operation[i], Pocket):
            if all_operation[i].extent_type1 == 'UpToPlaneLimit' or all_operation[i].extent_type1 == 'UpToSurfaceLimit':
                select = parse_select(all_operation[i].select_list['length1'], no_map, sketch_offset_map,
                                      body_sketch_map)
                all_operation[i].select_list['length1'] = select
            if all_operation[i].extent_type2 == 'UpToPlaneLimit' or all_operation[i].extent_type2 == 'UpToSurfaceLimit':
                select = parse_select(all_operation[i].select_list['length2'], no_map, sketch_offset_map,
                                      body_sketch_map)
                all_operation[i].select_list['length2'] = select
        elif isinstance(all_operation[i], Hole):
            plane_select = parse_select(all_operation[i].plane_ref, no_map, sketch_offset_map, body_sketch_map,
                                          tmp_shaft_count, tmp_groove_count)
            all_operation[i].plane_ref = plane_select
            if all_operation[i].bottom_mode == 'UpToPlaneLimit' or all_operation[i].bottom_mode == 'UpToSurfaceLimit':
                for key in all_operation[i].select_list.keys():
                    select = parse_select(all_operation[i].select_list[key], no_map, sketch_offset_map,
                                          body_sketch_map)
                    all_operation[i].select_list[key] = select
            # 根据命名获取平面信息
            sketch_name_hole = all_operation[i].point_pos[all_operation[i].point_pos.find('Sketch'):all_operation[i].point_pos.find(')')]
            sketch_name_hole = sketch_name_hole[:sketch_name_hole.find(';')]
            all_operation[i].sketch_plane = deepcopy(sketch_plane_map[sketch_name_no_map[sketch_name_hole]])
            # 根据命名获取点的坐标
            all_operation[i].point_pos = all_operation[i].point_pos[all_operation[i].point_pos.find('Sketch'):all_operation[i].point_pos.find(')')]
            all_operation[i].point_pos = select_cache[all_operation[i].point_pos]


    # 将所有select_list从字典转为list
    for i in range(all_operation.__len__()):
        if isinstance(all_operation[i], Chamfer) or isinstance(all_operation[i], Fillet) or \
                isinstance(all_operation[i], Shell) or isinstance(all_operation[i], Draft) or \
                isinstance(all_operation[i], Mirror) or isinstance(all_operation[i], Revolve) or \
                isinstance(all_operation[i], Groove) or isinstance(all_operation[i], Hole):
            if all_operation[i].select_list is not None:
                all_operation[i].select_list = list(all_operation[i].select_list.values())
        if isinstance(all_operation[i], Extrude) or isinstance(all_operation[i], Pocket):
            if all_operation[i].select_list is not None:
                tmp_list = []
                if 'length1' in all_operation[i].select_list.keys():
                    tmp_list.append(all_operation[i].select_list['length1'])
                if 'length2' in all_operation[i].select_list.keys():
                    tmp_list.append(all_operation[i].select_list['length2'])
                all_operation[i].select_list = deepcopy(tmp_list)

    all_operation = list(all_operation)
    for i in all_operation:
        if isinstance(i, Chamfer) or isinstance(i, Fillet) or \
                isinstance(i, Shell) or isinstance(i, Draft) or \
                isinstance(i, Mirror):
            if len(i.select_list) == 0:
                all_operation.remove(i)

    macro_seq = Macro_Seq(all_operation, bounding_size)
    macro_seq.normalize()
    macro_seq.numericalize(n=ARGS_N)
    macro_vec = macro_seq.to_vector(MAX_N_EXT, MAX_N_LOOPS, MAX_N_CURVES, MAX_TOTAL_LEN, pad=False)
    for i in macro_vec:
        print(i)
    ##################################################################################################
    cad = Macro_Seq.from_vector(macro_vec, is_numerical=True, n=ARGS_N)
    part = doc.part
    if not remove_bug:
        create_CAD_CATIA(cad, catia, doc, part, remove_bug=False)
        with h5py.File('C:\\Users\\45088\\Desktop\\SWModel\\macro1' + '.h5') as f:
            f['vec'] = macro_vec
    else:
        bug_point = create_CAD_CATIA(cad, catia, doc, part, remove_bug=True)
        while bug_point != -1 and len(cad.extrude_operation) > 0:
            doc.close()
            doc = catia.documents.add('Part')
            part = doc.part
            cad.extrude_operation.remove(cad.extrude_operation[bug_point])
            if len(cad.extrude_operation) <= 0:
                return None
            bug_point = create_CAD_CATIA(cad, catia, doc, part, remove_bug=True)
    if not just_test:
        doc.close()
        if remove_bug:
            cad.numericalize(n=ARGS_N)
            macro_vec = cad.to_vector(MAX_N_EXT, MAX_N_LOOPS, MAX_N_CURVES, MAX_TOTAL_LEN, pad=False)
        return macro_vec