import math

import numpy
from matplotlib import pyplot

from build_outline import load_outline
from build_tools import interpolate_by_stepLen, unify_list, offset, \
    calculate_angle, cut_outline_with_horline, rebuild_outline, make_ends, round_connect, new_plot, \
    interpolate_by_stepLen_plus, is_in_circle, build_cicle, find_horline_outline_intersection, get_unit_vector, \
    is_positive

motor_range = {2: [1, 1.78], 4: [-125, 125], 1: [0.75, 6.35]}
line_speed = 0.3
set_angle_to_normal = 10
shoot_range_xy = .24
dis_l1l1 = 0.1
command_interval = 0.1
l1_length = 0.35
motor_velmax = {1: 0.3, 2: 0.2, 3: 0.1, 4: 20, 6: 20}
time_step_min = .02


def takeFirst(elem):
    return elem[0]


def get_straightline_outline(origin_bds: list, r4_bds: list):
    step_num = math.ceil(max(abs(origin_bds[1][0] - origin_bds[0][0]) / loose_step_len[1],
                             abs(origin_bds[1][1] - origin_bds[0][1]) / loose_step_len[2],
                             abs(r4_bds[1] - r4_bds[0]) / loose_step_len[4]))
    x = list(numpy.linspace(origin_bds[0][0], origin_bds[1][0], step_num))
    y = list(numpy.linspace(origin_bds[0][1], origin_bds[1][1], step_num))
    r4 = list(numpy.linspace(r4_bds[0], r4_bds[1], step_num))
    outline = [[x[i] + math.cos(math.radians(r4[i])) * l1_length,
                y[i] + math.sin(math.radians(r4[i])) * l1_length]
               for i in range(len(r4))]
    origin = [[x[i], y[i]] for i in range(len(x))]
    return outline, origin


def exam_boundary(boundary: list, offset_outline: list, origin_line: list, left: bool):
    cnt = len(origin_line)
    def generate_range(front:bool, turnning_point:int,length):
        if front:
            return [0,turnning_point]
        else:
            return [turnning_point,length]
    if left:
        idx = 0
        research_para = [1, len(offset_outline), 1]
        compare_sign = 1
        range1 = lambda x:generate_range(front=True,turnning_point=x, length=cnt)
        range2 = lambda x:generate_range(front=False,turnning_point=x, length=cnt)
    else:
        idx = -1
        research_para = [len(offset_outline) - 2, -1, -1]
        compare_sign = -1
        range1 = lambda x:generate_range(front=False,turnning_point=x, length=cnt)
        range2 = lambda x:generate_range(front=True,turnning_point=x, length=cnt)
    if abs(boundary[idx][0][0] - origin_line[idx][0]) < 2 * l1_length:
        turnning_point = None
        last_circle_flag = is_in_circle(boundary[idx][0], l1_length, offset_outline[idx])
        for i in range(research_para[0], research_para[1], research_para[2]):
            current_circle_flag = is_in_circle(boundary[idx][0], l1_length, offset_outline[i])
            if last_circle_flag in [1, 0] and current_circle_flag == -1:
                turnning_point = i
                print("circle_intersection", i)
                break
            last_circle_flag = current_circle_flag
        if turnning_point:
            bds_angle = calculate_angle(origin_line[turnning_point], offset_outline[turnning_point])
            print(bds_angle, boundary[idx][1])
            if is_positive(bds_angle - boundary[idx][1]) in [compare_sign,0]:
                vector = get_unit_vector(origin_line[turnning_point], offset_outline[turnning_point])
                vector = [vector[0] * l1_length, vector[1] * l1_length]
                offset_outline[idx] = [origin_line[idx][0] + vector[0], origin_line[idx][1] + vector[1]]
                outline_point = find_horline_outline_intersection(outline=outline, line_y=offset_outline[idx][1])
                if is_positive(offset_outline[idx][0]-outline_point[idx][0]) in [compare_sign, 0]:
                    offset_outline[range1(turnning_point)[0]:range1(turnning_point)[1]] = \
                        [[i[0] + vector[0], i[1] + vector[1]] for i in
                         origin_line[range1(turnning_point)[0]:range1(turnning_point)[1]]]
                    boundary[idx][1] = bds_angle
                else:
                    origin_line = origin_line[range2(turnning_point)[0]:range2(turnning_point)[1]]
                    offset_outline = offset_outline[range2(turnning_point)[0]:range2(turnning_point)[1]]
                    boundary[idx][1] = bds_angle
    return boundary, offset_outline, origin_line


def vas_filter(time_step, source, limit_acc):
    result = source.copy()
    length1 = len(source)
    vel_list = []
    for i in range(length1 - 1):
        vel = (source[i + 1] - source[i]) / time_step[i]
        vel_list.append(vel)
    acc_list = []
    for i in range(length1 - 2):
        acc = (vel_list[i + 1] - vel_list[i]) / (time_step[i + 1] + time_step[i]) * 2
        acc_list.append(acc)
    for i in range(1, length1 - 1):
        v_cur = (vel_list[i] + vel_list[i - 1]) / 2
        result[i] += v_cur * abs(v_cur) / limit_acc / 2
    return result


def line_filter(time_step, source, limit_acc):
    length1 = len(source)
    vel_list = []
    for i in range(length1 - 1):
        vel = (source[i + 1] - source[i]) / time_step[i]
        vel_list.append(vel)

    acc_list = []
    for i in range(length1 - 2):
        acc = (vel_list[i + 1] - vel_list[i]) / (time_step[i + 1] + time_step[i]) * 2
        acc_list.append(acc)
    # 开始操作
    window = 2
    acc_adv_list = [0] * (length1 - 2)
    acc_adv_list[:window] = acc_list[:window]
    acc_adv_list[-window:] = acc_list[-window:]
    '''超级劣化版'''
    change_list = set()
    for i in range(window, length1 - 2 - window):
        acc_raw = acc_list[i]
        if abs(acc_raw) > limit_acc:
            for j in range(i - window, i + window + 1):
                d = window + 1 - abs(i - j)
                acc_new = acc_raw * d / (window + 1) / (window + 1)
                acc_adv_list[j] += acc_new
                change_list.add(j + 1)
        else:
            acc_adv_list[i] += acc_raw

    check_dict = {}
    for i in range(length1 - 2):
        acc = acc_list[i]
        if abs(acc) > limit_acc:
            check_dict[i] = math.sqrt(abs(acc) / limit_acc)

    new_tar = source.copy()
    last_normal_idx = 0
    for i in range(1, length1 - 1):
        if i in change_list:
            v0 = (new_tar[i] - new_tar[i - 1]) / time_step[i - 1]
            v1 = v0 + acc_adv_list[i - 1] * (time_step[i] + time_step[i - 1]) / 2
            s = v1 * time_step[i]
            new_tar[i + 1] = new_tar[i] + s
        else:
            if last_normal_idx < i - 1:
                d_tar = source[i] - new_tar[i]
                for j in range(last_normal_idx + 1, i + 1):
                    new_tar[j] += d_tar * (j - last_normal_idx + 1) / (i - last_normal_idx + 1)  # TODO:XXX

            last_normal_idx = i

    new_ts = time_step.copy()
    for idx, scale in check_dict.items():
        new_ts[idx] *= scale

    # 未来预测再放送
    new_tar = vas_filter(new_ts, new_tar, limit_acc)
    return new_ts, new_tar


def compute_one(cmt_t, cmd1, vmax, acc):
    from motor import build_math_path, get_math_spd_by_ts, get_pos_by_ts
    pos = [cmd1[0]]
    v = [0]
    offset = [0]
    for i in range(1, len(cmt_t) + 1):
        time_list, spd_list, run_type = build_math_path(v[i - 1], vmax, pos[i - 1],
                                                        cmd1[i], acc, acc)
        pos_step = get_pos_by_ts(cmt_t[i - 1], time_list, spd_list)[0]
        pos.append(pos[i - 1] + pos_step)
        offset.append(pos[i] - cmd1[i])
        v.append(get_math_spd_by_ts(cmt_t[i - 1], time_list, spd_list)[0])
    return pos


if __name__ == '__main__':
    r4_y_min = motor_range[2][0] + l1_length * math.cos(math.radians(motor_range[4][0]))
    outline_step_min = time_step_min * motor_velmax[1] + math.tan(
        math.radians(time_step_min * motor_velmax[4])) * l1_length
    loose_step_len = {}
    for num, vel in motor_velmax.items():
        loose_step_len[num] = time_step_min * vel * 5

    for outline, car in load_outline(type="short", dis=shoot_range_xy, highest=motor_range[2][1] + l1_length):
        # outline = interpolate_by_stepLen(outline, 0.01)
        car = rebuild_outline(line=car, line_y=motor_range[2][0])

        # 寻找最低能射到的地方
        up_car, down_car = cut_outline_with_horline(outline=car, line_y=motor_range[2][0])
        up_car, down_car = unify_list(up_car), unify_list(down_car)
        if down_car[0][0] + shoot_range_xy > motor_range[1][1] + l1_length or \
                down_car[-1][0] - shoot_range_xy < motor_range[1][0] - l1_length:  # 检车是否过界
            print("车越界")
            continue
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
        displaced_car = [[p[0], p[1] - l1_length + shoot_range_xy] for p in up_car]
        displaced_car = cut_outline_with_horline(outline=displaced_car, line_y=motor_range[2][0])[0]
        displaced_car = unify_list(displaced_car)
        if len(displaced_car) >= 2:
            displaced_car = make_ends(displaced_car, motor_range[2][0])
            displaced_car = interpolate_by_stepLen(displaced_car, loose_step_len[1])
            hard_offset_car = offset(displaced_car, l1_length, True)
            boundary, hard_offset_car, displaced_car = exam_boundary(boundary=boundary,offset_outline=hard_offset_car,
                                                                     origin_line=displaced_car,left=True)
            boundary, hard_offset_car, displaced_car = exam_boundary(boundary=boundary, offset_outline=hard_offset_car,
                                                                     origin_line=displaced_car, left=False)
            outline_left, origin_left = get_straightline_outline(origin_bds=[boundary[0][0], displaced_car[0]],
                                                                 r4_bds=[boundary[0][1],
                                                                         calculate_angle(displaced_car[0],
                                                                                         hard_offset_car[0],
                                                                                         False)])
            outline_right, origin_right = get_straightline_outline(origin_bds=[displaced_car[-1], boundary[-1][0]],
                                                                   r4_bds=[calculate_angle(displaced_car[-1],
                                                                                           hard_offset_car[-1]),
                                                                           boundary[-1][1]])
            chemical_outline = round_connect([outline_left, hard_offset_car, outline_right])
            chemical_origin = round_connect([origin_left, displaced_car, origin_right])
            r4 = [calculate_angle(chemical_origin[i], chemical_outline[i]) for i in range(len(chemical_origin))]


            new_chemical_outline, new_chemical_origin = interpolate_by_stepLen_plus(chemical_outline,
                                                                                    outline_step_min, chemical_origin)
            new_r4 = [calculate_angle(new_chemical_origin[i], new_chemical_outline[i]) for i in
                      range(len(new_chemical_origin))]


        else:
            chemicial_outline, chemicial_origin = get_straightline_outline(x_bds=[boundary[0][0][0], boundary[1][0][0]],
                                                                           r4_bds=[boundary[0][1], boundary[1][1]])

        pyplot.figure(figsize=(7, 3))
        new_plot(chemical_origin, "o-")
        new_plot(chemical_outline, "o-")
        # new_plot(new_chemical_outline,"g-*")
        # new_plot(new_chemical_origin, "b-*")
        new_plot(hard_offset_car)
        # new_plot(outline, "g--")
        # # new_plot(down_car)
        #
        # new_plot(car)
        #
        # # for i in chemicial_origin:
        # #     new_plot(build_cicle(i, l1_length))
        # for i in range(len(new_r4)):
        #     pyplot.plot([new_chemical_origin[i][0] + math.cos(math.radians(new_r4[i])) * l1_length, new_chemical_origin[i][0]],
        #                 [new_chemical_origin[i][1] + math.sin(math.radians(new_r4[i])) * l1_length, new_chemical_origin[i][1]], "c")
        # new_plot([[new_chemical_origin[i][0] + math.cos(math.radians(new_r4[i])) * l1_length,
        #              new_chemical_origin[i][1] + math.sin(math.radians(new_r4[i])) * l1_length] for i in range(len(new_r4))],"r+")
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
        # pyplot.plot([order[0][i] + math.cos(math.radians(order[3][i]+90)) * l1_length for i in range(len(order[0]))],
        #             [order[1][i] + math.sin(math.radians(order[3][i]+90)) * l1_length for i in range(len(order[0]))], "y*")
        pyplot.xlim(0, 7)
        pyplot.ylim(0, 3)
        pyplot.show()
        acc_list = [.5, .3, 60]
        vel_list = [.3, .2, 20]
        ts = []
        cnt = [0, 0, 0]
        for i in range(1, len(new_chemical_origin)):
            ts.append(max([abs(new_chemical_origin[i][0] - new_chemical_origin[i - 1][0]) / vel_list[0],
                           abs(new_chemical_origin[i][1] - new_chemical_origin[i - 1][1]) / vel_list[1],
                           abs(new_r4[i] - new_r4[i - 1]) / vel_list[2],
                           time_step_min]))
            # if ts[-1] ==(chemical_origin[i][0]-chemical_origin[i-1][0])/vel_list[0]:
            #     cnt[0] += 1
            # elif  (chemical_origin[i][1] - chemical_origin[i-1][1]) / vel_list[1]:
            #     cnt[1] += 1
            # elif (r4[i] - r4[i-1]) / vel_list[2]

        pyplot.plot(ts)
        pyplot.show()

        x = [p[0] for p in new_chemical_origin]
        y = [p[1] for p in new_chemical_origin]
        new_t1, new_x = line_filter(ts, x, acc_list[0])
        new_t2, new_y = line_filter(ts, y, acc_list[1])
        new_t3, new_r = line_filter(ts, new_r4, acc_list[2])
        new_t = [max(a, b, c, time_step_min) for a, b, c in zip(new_t1, new_t2, new_t3)]

        new_t_sum = [0]
        new_t_cached = 0
        for t in new_t:
            new_t_cached += t
            new_t_sum.append(new_t_cached)
        print("t", sum(new_t), sum(ts))

        pyplot.figure(figsize=(20, 10))
        real_x = compute_one(new_t, new_x, vel_list[0], acc_list[0])
        real_y = compute_one(new_t, new_y, vel_list[1], acc_list[1])
        real_r = compute_one(new_t, new_r, vel_list[2], acc_list[2])
        original_x = compute_one(ts, x, vel_list[0], acc_list[0])
        original_y = compute_one(ts, y, vel_list[1], acc_list[1])
        original_r = compute_one(ts, new_r4, vel_list[2], acc_list[2])
        pyplot.subplot(3, 1, 1)
        pyplot.plot(x, "b")
        pyplot.plot(original_x, "r")
        pyplot.plot(real_x, "y")
        pyplot.subplot(3, 1, 2)
        pyplot.plot(y, "bo")
        pyplot.plot(original_y, "r*")
        pyplot.plot(real_y, "y*")
        pyplot.subplot(3, 1, 3)
        pyplot.plot(new_r4, "bo")
        pyplot.plot(original_r, "r*")
        pyplot.plot(real_r, "y*")
        pyplot.show()

        # new_plot([[new_x[i]+l1_length*math.cos(math.radians(new_r[i])),
        #         new_y[i]+l1_length*math.sin(math.radians(new_r[i]))] for i in range(len(new_y))],"*-g")
        new_plot([[original_x[i] + l1_length * math.cos(math.radians(original_r[i])),
                   original_y[i] + l1_length * math.sin(math.radians(original_r[i]))] for i in range(len(original_y))],
                 "*-r")
        new_plot([[real_x[i] + l1_length * math.cos(math.radians(real_r[i])),
                   real_y[i] + l1_length * math.sin(math.radians(real_r[i]))] for i in range(len(real_y))], "*-y")
        # new_plot([[x[i] + l1_length * math.cos(math.radians(new_r4[i])),
        #            y[i] + l1_length * math.sin(math.radians(new_r4[i]))] for i in range(len(y))], "*-b")
        new_plot(outline)
        new_plot(car)
        new_plot(new_chemical_origin)
        new_plot(chemical_outline)

        pyplot.show()

        # origin_left = interpolate_by_stepLen([boundary[0][0], down_car[0]], time_step_min * motor_velmax[1])
        # origin_right = interpolate_by_stepLen([down_car[-1], boundary[-1][0]], time_step_min * motor_velmax[1])
        # chemicial_origin, round_list = round_connect([origin_left, down_car, origin_right])
        # left, right = round_list[0][0], round_list[1][1]
        # print(round_list)
        # print(left, right)
        # chemical_outline = offset(chemicial_origin, l1_length, True)
        # r4 = [calculate_angle(chemicial_origin[i], chemical_outline[i]) for i in range(len(chemical_outline))]
        # pos1 = find_horline_outline_intersection([[i, r4[i]] for i in range(len(r4))], boundary[0][1])
        # pos2 = find_horline_outline_intersection([[i, r4[i]] for i in range(len(r4))], boundary[1][1])
        # print("*********", pos1, pos2)
        # if pos1:
        #     r4[:pos1[0][0]] = [boundary[0][1]] * pos1[0][0]
        # else:
        #     start = int((round_list[0][0] + round_list[0][1]) / 2)
        #     r4[:start + 1] = list(numpy.linspace(boundary[0][1], r4[start], start))
        # if pos2:
        #     r4[pos2[-1][0]:] = [boundary[1][1]] * (len(chemical_outline) - pos2[-1][0])
        # else:
        #     start = int((round_list[1][0] + round_list[1][1]) / 2)
        #     print(len(r4) - start, start)
        #     r4[start:] = list(numpy.linspace(r4[start], boundary[1][1], len(r4) - start))
        # pyplot.subplot(2, 1, 1)
        # pyplot.plot(r4)
        # new_plot([i[1] for i in pos1])
        # new_plot([i[1] for i in pos2])

        # pyplot.subplot(2, 1, 2)
