import math
import os
import pickle

import numpy
from matplotlib import pyplot
from shapely.geometry import LineString

from build_outline import load_outline, cut_xy_spline
from build_tools import interpolate, unify_list, \
    find_horline_outline_intersection, is_clockwise, calculate_angle, cut_outline_with_horline,rebuild_outline

motor_range = {2: [1, 1.78], 4: [-125, 125], 1: [0.75, 6.35]}
line_speed = 0.3
set_angle_to_normal = 10
shoot_range_xy = .24
dis_l1l1 = 0.1
command_interval = 0.1
l1_length = 0.6471

motor_velmax = {1: 0.3, 2: 0.2, 3: 0.1, 4: 20, 6: 20}


def takeFirst(elem):
    return elem[0]


def car_loader():
    root_path = "./data_for_chemicals/"
    for filename in os.listdir(root_path):
        file_path = root_path + filename
        if file_path[-4:] == ".npy":
            # if "21" not in file_path:
            #     continue
            with open(file_path, 'rb') as f_pos:
                print(file_path)
                o, command, car = pickle.load(f_pos)
                yield o, command, car


if __name__ == '__main__':
    # motor_range[4] = [motor_range[4][0]+90, motor_range[4][1]+90]
    r4_y_min = motor_range[2][0] + l1_length * math.cos(math.radians(motor_range[4][0]))
    print(r4_y_min)
    for outline, car in load_outline(type="short", dis=shoot_range_xy, highest=motor_range[2][1] + l1_length):
        outline = interpolate(outline, 0.01)

        # 寻找轮廓线和origin最低范围的交点
        idx_origin_min = find_horline_outline_intersection(outline=outline, line_y=motor_range[2][0])
        # 寻找车向下平移r4-shoot_range长度与origin最低范围的交点
        new_car = [[p[0], p[1] - l1_length + shoot_range_xy] for p in car]
        new_car = cut_xy_spline(new_car, motor_range[2][0])
        down_car_offset = []
        if len(new_car) >= 2:
            lines = LineString(new_car)
            patch = lines.parallel_offset(distance=l1_length, side="left")
            try:
                down_car_offset = list(patch.coords)
            except:
                print(type(patch))
                for i in patch:
                    down_car_offset += list(i.coords)

        # 寻找最多能射到的地方
        idx_car_origin_min = find_horline_outline_intersection(outline=car, line_y=motor_range[2][0])
        car = rebuild_outline(line=car, line_y=motor_range[2][0])
        up_car, down_car = cut_outline_with_horline(outline=car, line_y=motor_range[2][0])
        up_car, down_car = unify_list(up_car), unify_list(down_car)

        # r4_angle = [[180], [0]]
        # if is_clockwise(car):
        #     for i in range(idx_car_origin_min[0][0], idx_car_origin_min[1][0] + 1):
        #         cur_r4 = calculate_angle(point1=car[i], point2=[motor_range[1][1], motor_range[2][0]])
        #         if cur_r4 >= r4_angle[1][-1]:
        #             r4_angle[1].append(cur_r4)
        #         else:
        #             break
        #     for i in range(idx_car_origin_min[1][0], idx_car_origin_min[0][0] + 1, -1):
        #         cur_r4 = calculate_angle(point1=car[i], point2=[motor_range[1][0], motor_range[2][0]])
        #         if cur_r4 <= r4_angle[0][-1]:
        #             r4_angle[1].append(cur_r4)
        #         else:
        #             break
        # else:
        #     for i in range(idx_car_origin_min[0][0], idx_car_origin_min[1][0] + 1):
        #         cur_r4 = calculate_angle(point1=car[i], point2=[motor_range[1][1], motor_range[2][0]])
        #         if cur_r4 >= r4_angle[1][-1]:
        #             r4_angle[1].append(cur_r4)
        #         else:
        #             break
        #     for i in range(idx_car_origin_min[1][0], idx_car_origin_min[0][0] + 1, -1):
        #         cur_r4 = calculate_angle(point1=car[i], point2=[motor_range[1][0], motor_range[2][0]])
        #         if cur_r4 <= r4_angle[0][-1]:
        #             r4_angle[1].append(cur_r4)
        #         else:
        #             break
        pyplot.figure(figsize=(7, 3))
        pyplot.plot([p[0] for p in outline], [p[1] for p in outline], "g--")
        pyplot.plot([p[0] for p in new_car], [p[1] for p in new_car])
        pyplot.plot([p[0] for p in car], [p[1] for p in car])
        pyplot.plot([p[0] for p in up_car], [p[1] for p in up_car],"y")
        pyplot.plot([p[0] for p in down_car], [p[1] for p in down_car],"r")
        # n = numpy.arange(len(up_car))
        # for i, txt in enumerate(n):
        #     pyplot.annotate(txt, (up_car[i][0], up_car[i][1]))
        # n = numpy.arange(len(down_car))
        # for i, txt in enumerate(n):
        #     pyplot.annotate(txt, (down_car[i][0], down_car[i][1]))
        pyplot.plot([p[1][0] for p in idx_origin_min], [p[1][1] for p in idx_origin_min], "bo")
        pyplot.plot([p[1][0] for p in idx_car_origin_min], [p[1][1] for p in idx_car_origin_min], "bo")
        pyplot.plot(motor_range[1], [motor_range[2][0], motor_range[2][0]])
        pyplot.plot([p[0] for p in down_car_offset], [p[1] for p in down_car_offset])
        # for i in up_car:
        #     pyplot.plot([p[0] for p in i], [p[1] for p in i], "r-")
        # for i in down_car:
        #     pyplot.plot([p[0] for p in i], [p[1] for p in i], "c-")
        # pyplot.plot([order[0][i] + math.cos(math.radians(order[3][i]+90)) * l1_length for i in range(len(order[0]))],
        #             [order[1][i] + math.sin(math.radians(order[3][i]+90)) * l1_length for i in range(len(order[0]))], "y*")
        pyplot.xlim(0, 7)
        pyplot.ylim(0, 3)
        pyplot.show()
