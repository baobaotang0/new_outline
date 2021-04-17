import math,numpy
import os
import pickle

from matplotlib import pyplot
from shapely.geometry import LineString

from build_outline import load_outline
from build_tools import interpolate_by_stepLen, unify_list, \
    calculate_angle, cut_outline_with_horline, rebuild_outline, make_ends, round_connect

motor_range = {2: [1, 1.78], 4: [-125, 125], 1: [0.75, 6.35]}
line_speed = 0.3
set_angle_to_normal = 10
shoot_range_xy = .24
dis_l1l1 = 0.1
command_interval = 0.1
l1_length = 0.35
motor_velmax = {1: 0.3, 2: 0.2, 3: 0.1, 4: 20, 6: 20}
time_step_min = .1


def takeFirst(elem):
    return elem[0]


def get_straightline_outline(x_bds:list, r4_bds:list):
    step_num = math.ceil(max(abs(x_bds[1]-x_bds[0]) / (time_step_min * motor_velmax[1]),
                             abs(r4_bds[1]-r4_bds[0]) / (time_step_min * motor_velmax[4])))
    x = list(numpy.linspace(x_bds[1], x_bds[0], step_num))
    r4 = list(numpy.linspace(r4_bds[1], r4_bds[0], step_num))
    outline = [[x[i] + math.cos(math.radians(r4[i])) * l1_length,
                     motor_range[2][0] + math.sin(math.radians(r4[i])) * l1_length]
                    for i in range(len(r4))]
    origin = [[x[i], motor_range[2][0]] for i in range(len(x))]
    if outline[0][0]> outline[-1][0]:
        outline.reverse()
        origin.reverse()
    return outline, origin


if __name__ == '__main__':
    # motor_range[4] = [motor_range[4][0]+90, motor_range[4][1]+90]
    r4_y_min = motor_range[2][0] + l1_length * math.cos(math.radians(motor_range[4][0]))
    for outline, car in load_outline(type="short", dis=shoot_range_xy, highest=motor_range[2][1] + l1_length):
        outline = interpolate_by_stepLen(outline, 0.01)
        car = rebuild_outline(line=car, line_y=motor_range[2][0])
        # 寻找最低能射到的地方
        up_car, down_car = cut_outline_with_horline(outline=car, line_y=motor_range[2][0])
        up_car, down_car = unify_list(up_car), unify_list(down_car)
        idx_down_car = [[0, down_car[0], 180], [0, down_car[0], 0]]
        r4_down_car = [[], []]
        for i in range(len(down_car)):
            cur_r4 = calculate_angle(point1=down_car[i], point2=[motor_range[1][1], motor_range[2][0]])
            r4_down_car[1].append(cur_r4)
            if cur_r4 > idx_down_car[1][2]:
                idx_down_car[1] = [i, down_car[i], cur_r4]
            cur_r4 = calculate_angle(point1=down_car[i], point2=[motor_range[1][0], motor_range[2][0]])
            r4_down_car[0].append(cur_r4)
            if cur_r4 < idx_down_car[0][2]:
                idx_down_car[0] = [i, down_car[i], cur_r4]
        boundary = [[[motor_range[1][0], motor_range[2][0]], idx_down_car[0][2]],
                    [[motor_range[1][1], motor_range[2][0]], idx_down_car[1][2]]]

        # 寻找车向下平移r4-shoot_range长度与origin最低范围的交点
        new_car = [[p[0], p[1] - l1_length + shoot_range_xy] for p in car]
        new_car = cut_outline_with_horline(outline=new_car, line_y=motor_range[2][0])[0]
        new_car = unify_list(new_car)
        if len(new_car) >= 2:
            new_car = make_ends(new_car, motor_range[2][0])
            new_car = interpolate_by_stepLen(new_car, time_step_min * motor_velmax[1])
            down_car_offset = []
            lines = LineString(new_car)
            patch = lines.parallel_offset(distance=l1_length, side="left")
            try:
                down_car_offset = list(patch.coords)
            except:
                print(type(patch))
                for i in patch:
                    down_car_offset += list(i.coords)
            if new_car[0][0]> new_car[-1][0]:
                new_car.reverse()
            if down_car_offset[0][0]> down_car_offset[-1][0]:
                down_car_offset.reverse()
            down_car_offset = interpolate_by_stepLen(down_car_offset, time_step_min * motor_velmax[1])
            outline_left, origin_left = get_straightline_outline(x_bds=[boundary[0][0][0], new_car[0][0]],
                                            r4_bds=[boundary[0][1], calculate_angle(new_car[0], down_car_offset[0])])
            outline_right, origin_right = get_straightline_outline(x_bds=[new_car[-1][0], boundary[-1][0][0]],
                                            r4_bds=[calculate_angle(new_car[-1], down_car_offset[-1]), boundary[-1][1]])
            chemicial_outline = round_connect([outline_left, down_car_offset, outline_right])
            # chemicial_outline = interpolate_by_stepLen(chemicial_outline, time_step_min * motor_velmax[1])
            chemicial_origin = round_connect([origin_left, new_car, origin_right])
            # chemicial_origin = interpolate_by_stepLen(chemicial_origin, time_step_min * motor_velmax[1])

        else:
            chemicial_outline, chemicial_origin = get_straightline_outline(x_bds=[boundary[0][0][0], boundary[1][0][0]],
                                            r4_bds=[boundary[0][1], boundary[1][1]])

        # step_num, r4, x, y = [], [], [], []
        # for i in range(len(boundary) - 1):
        #     step_num.append(
        #         math.ceil(max((boundary[i + 1][0][0] - boundary[i][0][0]) / (time_step_min * motor_velmax[1]),
        #                       (boundary[i + 1][0][1] - boundary[i][0][1]) / (time_step_min * motor_velmax[2]),
        #                       (boundary[i + 1][1] - boundary[i][1]) / (time_step_min * motor_velmax[4]))))
        #     x.append(list(numpy.linspace(boundary[i + 1][0][0], boundary[i][0][0], step_num[-1])))
        #     y.append(list(numpy.linspace(boundary[i + 1][0][1], boundary[i][0][1], step_num[-1])))
        #     r4.append(list(numpy.linspace(boundary[i + 1][1], boundary[i][1], step_num[-1])))
        # r4, x, y = unify_list(r4), unify_list(x), unify_list(y)

        # pyplot.subplot(2, 1, 2)
        pyplot.figure(figsize=(7, 3))
        pyplot.plot([p[0] for p in chemicial_origin], [p[1] for p in chemicial_origin], "b-*")
        pyplot.plot([p[0] for p in chemicial_outline], [p[1] for p in chemicial_outline], "g-*")
        pyplot.plot([p[0] for p in outline], [p[1] for p in outline], "g--")
        pyplot.plot([p[0] for p in new_car], [p[1] for p in new_car])
        pyplot.plot([p[0] for p in car], [p[1] for p in car])

        # for i in range(len(r4)):
        #     pyplot.plot([x[i] + math.cos(math.radians(r4[i])) * l1_length, x[i]],
        #                 [y[i] + math.sin(math.radians(r4[i])) * l1_length, y[i]], "c")
        # pyplot.plot([x[i] + math.cos(math.radians(r4[i])) * l1_length for i in range(len(r4))],
        #             [y[i] + math.sin(math.radians(r4[i])) * l1_length for i in range(len(r4))])
        # n = numpy.arange(len(chemicial_origin))
        # for i, txt in enumerate(n):
        #     pyplot.annotate(txt, (chemicial_origin[i][0], chemicial_origin[i][1]))
        # n = numpy.arange(len(chemicial_outline))
        # for i, txt in enumerate(n):
        #     pyplot.annotate(txt, (chemicial_outline[i][0], chemicial_outline[i][1]))

        pyplot.plot([p[1][0] for p in idx_down_car], [p[1][1] for p in idx_down_car], "bo")
        pyplot.plot(motor_range[1], [motor_range[2][0], motor_range[2][0]])
        pyplot.plot([p[0] for p in down_car_offset], [p[1] for p in down_car_offset],"-y")
        # for i in up_car:
        #     pyplot.plot([p[0] for p in i], [p[1] for p in i], "r-")
        # for i in down_car:
        #     pyplot.plot([p[0] for p in i], [p[1] for p in i], "c-")
        # pyplot.plot([order[0][i] + math.cos(math.radians(order[3][i]+90)) * l1_length for i in range(len(order[0]))],
        #             [order[1][i] + math.sin(math.radians(order[3][i]+90)) * l1_length for i in range(len(order[0]))], "y*")
        pyplot.xlim(0, 7)
        pyplot.ylim(0, 3)
        pyplot.show()
