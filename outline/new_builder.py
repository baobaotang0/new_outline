import os, numpy, math
from matplotlib import pyplot
from shapely.geometry import MultiPoint, LineString
from build_outline import load_outline
from build_tools import find_horline_circle_intersection, interpolate, \
    build_circle, calculate_dis, find_horline_outline_intersection, find_2verline_outline_intersection
from vtktool.vtktool import vtk_show, point_actor, line_actor
from scipy.spatial.distance import pdist

motor_range = {2: [1, 1.78], 4: [-125, 125]}
line_speed = 0.3
set_angle_to_normal = 10
shoot_range_xy = .24
dis_l1l1 = 0.1
command_interval = 0.1
l1_length = 0.6471

motor_velmax = {1: 0.3, 2: 0.2, 3: 0.1, 4: 20, 6: 20}


def takeFirst(elem):
    return elem[0]


def takeSecondFirst(elem):
    return elem[1][0]


if __name__ == '__main__':
    # motor_range[4] = [motor_range[4][0]+90, motor_range[4][1]+90]
    r4_y_min = motor_range[2][0] + l1_length * math.cos(math.radians(motor_range[4][0]))
    print(r4_y_min)
    for outline, car in load_outline(type="short", dis=shoot_range_xy, highest=motor_range[2][1] + l1_length):
        outline = interpolate(outline, 0.01)

        # 寻找轮廓线和origin最低范围的交点
        idx_origin_min = find_horline_outline_intersection(outline=outline, line_y=motor_range[2][0])
        # 寻找轮廓线和r4最低范围的交点
        idx_r4_min = find_horline_outline_intersection(outline=outline, line_y=r4_y_min)
        # 寻找轮廓线向下平移r4长度与origin最低范围的交点
        origin_cur_displace = []
        idx_cur_displace = find_horline_outline_intersection(start_idx=idx_origin_min[0][0],
                                                             end_idx=idx_origin_min[1][0],
                                                             outline=outline,
                                                             line_y=motor_range[2][0] + l1_length)
        print(idx_cur_displace)
        # 寻找轮廓线向内offset r4长度与origin最低范围的交点
        lines = LineString(outline)
        patch = lines.parallel_offset(distance=l1_length, side="right")
        origin_cur_offset = list(patch.coords)
        origin_cur_offset = interpolate(origin_cur_offset, 0.01)
        idx_origin_cur_offset = find_horline_outline_intersection(outline=origin_cur_offset, line_y=motor_range[2][0])
        idx_cur_offset = []
        cur_outline_cur_offset = []
        cur_origin_cur_offset = []
        for i in range(len(idx_origin_cur_offset) // 2):
            cur_origin_cur_offset = origin_cur_offset[
                                    idx_origin_cur_offset[2 * i][0]:idx_origin_cur_offset[2 * i + 1][0]]
            if len(cur_origin_cur_offset) <= 1:
                continue
            lines = LineString(cur_origin_cur_offset)
            patch = lines.parallel_offset(distance=l1_length, side="right")
            try:
                cur_outline_cur_offset = list(patch.coords)
            except:
                print(type(patch))
                for i in patch:
                    cur_outline_cur_offset += list(i.coords)
            idx_cur_offset += find_2verline_outline_intersection(start_idx=idx_origin_min[0][0],
                                                                 end_idx=idx_origin_min[1][0],
                                                                 outline=outline,
                                                                 x_range=[
                                                                     min([i[0] for i in cur_outline_cur_offset]),
                                                                     max([i[0] for i in cur_outline_cur_offset])])
        idx_cur_offset.sort(key=takeFirst)
        print("idx_cur_offset", idx_cur_offset)
        # 比较轮廓线向内offset和轮廓线向下平移分别与origin最低范围的交点，取其中范围较大的交点
        idx_cur = []
        local_list = []
        if len(idx_cur_displace) == len(idx_cur_offset):
            for i in range(len(idx_cur_displace) // 2):
                local_list = [idx_cur_displace[2 * i], idx_cur_displace[2 * i + 1],
                              idx_cur_offset[2 * i], idx_cur_offset[2 * i + 1]]
                local_list.sort(key=takeFirst)
                idx_cur.append(local_list[0])
                idx_cur.append(local_list[-1])

        outline_boundary = idx_origin_min + idx_r4_min + idx_cur
        outline_boundary.sort(key=takeFirst)

        if idx_cur:
            out_circle = [[], []]
            origin_straight = [[], []]
            r4_origin_straight = [[], []]
            circle_center = [[idx_origin_min[0][1][0] + l1_length, idx_origin_min[0][1][1]],
                             [idx_origin_min[1][1][0] - l1_length, idx_origin_min[1][1][1]]]
            for i in range(idx_origin_min[0][0], idx_cur[0][0]):
                if outline[i][0] > circle_center[0][0]:
                    break
                if calculate_dis(circle_center[0], outline[i]) > l1_length:
                    out_circle[0].append(outline[i])
            if not out_circle[0]:
                for i in range(idx_r4_min[0][0] + 1, idx_cur[0][0] + 1):
                    origin_straight[0].append(find_horline_circle_intersection(center=outline[i], radius=l1_length,
                                                                               line_y=motor_range[2][0])[0])
                    r4_origin_straight[0].append(180 - math.degrees(math.asin(
                        (outline[i][1] - motor_range[2][0]) / calculate_dis(
                            point1=[origin_straight[0][-1], motor_range[2][0]],
                            point2=outline[i]))))

            for i in range(idx_origin_min[1][0], idx_cur[-1][0], -1):
                if outline[i][0] > circle_center[1][0]:
                    break
                if calculate_dis(circle_center[1], outline[i]) > l1_length:
                    out_circle[1].append(outline[i])
            if not out_circle[1]:
                for i in range(idx_cur[-1][0], idx_r4_min[1][0]):
                    origin_straight[1].append(find_horline_circle_intersection(center=outline[i], radius=l1_length,
                                                                               line_y=motor_range[2][0])[1])
                    r4_origin_straight[1].append(math.degrees(math.asin(
                        (outline[i][1] - motor_range[2][0]) / calculate_dis(
                            point1=[origin_straight[1][-1], motor_range[2][0]],
                            point2=outline[i]))))
            if r4_origin_straight[0]:
                r4_cur = [r4_origin_straight[0][-1]]
            else:
                r4_cur = [180]
            origin_cur = []
            last_dir = None

                # if curvity >= 10:
                #     turning_points[i] = cur_dir[1]
                #     continue
                # if cur_dir:
                #     if last_dir:
                #         if last_dir[1] != cur_dir[1]:
                #             round_boundary.append(last_dir[0])
                #     last_dir = cur_dir
            curvity = []
            for i in range(idx_cur[0][0], idx_cur[1][0]):
                vector1 = [(outline[i - 1][0] - outline[i][0]) / calculate_dis(outline[i - 1], outline[i]),
                           (outline[i - 1][1] - outline[i][1]) / calculate_dis(outline[i - 1], outline[i])]
                vector2 = [(outline[i + 1][0] - outline[i][0]) / calculate_dis(outline[i + 1], outline[i]),
                           (outline[i + 1][1] - outline[i][1]) / calculate_dis(outline[i + 1], outline[i])]
                if vector2[1] + vector1[1] < -0.0001:
                    curvity.append(math.degrees(math.acos(abs(1 - pdist([vector1, vector2], 'cosine')[0]))))
                else:
                    curvity.append(0)

                direction = [vector1[0] + vector2[0], vector1[1] + vector2[1]]
                local_length = math.sqrt(direction[0] ** 2 + direction[1] ** 2)
                if local_length == 0:
                    r4_cur.append(math.degrees(math.asin(vector2[1]))+90)
                    # r4_cur.append(r4_cur[-1])

                else:
                    direction = [direction[0] / local_length, direction[1] / local_length]
                    if direction[1]>0:
                        r4_cur.append(math.degrees(math.acos(direction[0])))
                    else:
                        r4_cur.append(180 - math.degrees(math.acos(direction[0])))
                origin_cur.append([outline[i][0]-math.cos(math.radians(r4_cur[-1]))*l1_length,
                                   outline[i][1]-math.sin(math.radians(r4_cur[-1]))*l1_length])
            pyplot.plot(curvity)
            pyplot.show()
            pyplot.plot(origin_straight[0]+[i[0] for i in origin_cur]+origin_straight[1])
            pyplot.show()
            pyplot.plot(r4_origin_straight[0] + r4_cur + r4_origin_straight[1])
            pyplot.show()
            pyplot.plot([i[0] for i in origin_cur], [i[1] for i in origin_cur])
            pyplot.show()
        pyplot.figure(figsize=(7, 3))
        pyplot.plot([p[0] for p in outline], [p[1] for p in outline], "go")
        pyplot.plot([p[0] for p in outline], [p[1] - l1_length for p in outline])
        pyplot.plot([p[0] for p in car], [p[1] for p in car])
        # pyplot.plot([p[0] for p in cur_origin_cur_offset], [p[1] for p in cur_origin_cur_offset], "*")
        # pyplot.plot([p[0] for p in cur_outline_cur_offset], [p[1] for p in cur_outline_cur_offset], "y*")
        pyplot.plot([p[0] for p in origin_cur_offset], [p[1] for p in origin_cur_offset])
        pyplot.plot([p[1][0] for p in outline_boundary], [p[1][1] for p in outline_boundary], "bo")
        pyplot.plot([outline[0][0], outline[-1][0]], [motor_range[2][0], motor_range[2][0]])
        pyplot.xlim(0, 7)
        pyplot.ylim(0, 3)
        pyplot.show()
