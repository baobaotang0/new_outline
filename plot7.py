import os
from math import radians, sqrt,cos, sin
from pickle import load

import numpy
from matplotlib import pyplot

from motor import build_math_path, get_math_spd_by_ts, get_pos_by_ts
from offset import get_motor_max_para, is_postive

motor = [0, 1, 3]
time_step_min = 0.1
l1_length = 0.6471
# 读最大加速度，最大速度
motor_max = get_motor_max_para("bot1.json")
vmax_value = [motor_max[num]["vel"] for num in range(1, 7)]
a_value = [motor_max[num]["acc"] for num in range(1, 7)]
vmax_value[3] = radians(vmax_value[3])
a_value[3] = radians(a_value[3])


def takeFirst(elem):
    return elem[0]


def is_bigger(x, xmax) -> int:
    if x > xmax:
        return 1
    else:
        return 0


def car_loader():
    root_path = "./data/"
    for filename in os.listdir(root_path):
        file_path = root_path + filename
        if file_path[-4:] == ".npy":
            # if "21" not in file_path:
            #     continue
            with open(file_path, 'rb') as f_pos:
                print(file_path)
                time_step, r4, xy, outline = load(f_pos)
                yield time_step, r4, xy, outline


def calculate_offset(v0: float, amax: float, pos0: float, destination: float, t: float) -> float:
    s = destination - pos0
    a = is_postive(s) * amax
    if is_postive(s) == 0:
        new_s = (v0 ** 2 / 2 / amax) * is_postive(v0)
        print("位移0",new_s)
    elif is_postive(v0) == is_postive(s) or is_postive(v0) == 0:
        t2 = (a * t + v0) / 2 / a
        t1 = t - t2
        if v0 / a < t and abs(s) < abs(v0 ** 2 / 2 / a):
            new_s = v0 ** 2 / 2 / a  # TODO:如果下一步反向，则可继续往前走
            print("直接减速至0，超过s", v0, t, s,new_s)
        elif v0 / a < t and abs(s) <= abs(v0 * t1 + a / 2 * t1 ** 2 + a / 2 * t2 ** 2):
            new_s = s
            print("加速减速至0，s不变", v0, t, s,new_s)
        elif v0 / a > t and abs(s) < abs(v0 * t - a / 2 * t ** 2):
            new_s = v0 ** 2 / 2 / a
            print("直接减速不到0，超过s", v0, t, s,new_s)
        elif abs(s) > abs(v0 * t + a / 2 * t ** 2):
            new_s = v0 * t + a / 2 * t ** 2 + (v0 + a * t) ** 2 / 2 / a
            print("加速不到s", v0, t, s,new_s)
        else:
            t1 = t - sqrt((1 / 2 * a * t ** 2 + v0 * t - s) / a)
            new_s = v0 * t1 + a / 2 * t1 ** 2 + (v0 + a * t1) ** 2 / 2 / a
            print("加速减速，s不变", v0, t, s,new_s)
    else:
        t1 = -v0 / a
        t2 = (t - t1) / 2
        if abs(2 * v0 / a) > t:
            new_s = s
            print("反向，s来不及反向", v0, t, s,new_s)
        elif abs(s) > abs(v0 * t + a / 2 * t ** 2):
            new_s = v0 * t + a / 2 * t ** 2 + (v0 + a * t) ** 2 / 2 / a
            print("反向，不到s", v0, t, s,new_s)
        elif abs(s) < abs(a * t2 ** 2 - a / 2 * t1 ** 2):
            new_s = s
            print("反向，加速减速至0，s不变", v0, t, s,new_s)
        else:
            t1 = t - sqrt((1 / 2 * a * t ** 2 + v0 * t - s) / a)
            new_s = v0 * t1 + a / 2 * t1 ** 2 + (v0 + a * t1) ** 2 / 2 / a
            print("反向，加速减速，s不变", v0, t, s,new_s)

    return new_s + pos0


def real_compute(cmt_t, cmd_all):
    pos = [[cmd_all[0][0]], [cmd_all[1][0]], [], [cmd_all[3][0]], [], []]
    v = [[0] for i in range(6)]
    offset = [[0] for i in range(6)]
    tl = [[] for i in range(6)]
    spdl = [[] for i in range(6)]
    a = [[] for i in range(6)]
    for num in motor:
        for i in range(1, len(cmt_t) + 1):
            res = build_math_path(v[num][i - 1], vmax_value[num], pos[num][i - 1],
                                  cmd_all[num][i], a_value[num], a_value[num])
            tl[num].append(res[0])
            spdl[num].append(res[1])
            pos_step = get_pos_by_ts(cmt_t[i - 1], tl[num][-1], spdl[num][-1])[0]
            pos[num].append(pos[num][i - 1] + pos_step)
            offset[num].append(pos[num][i] - cmd_all[num][i])
            v[num].append(get_math_spd_by_ts(cmt_t[i - 1], tl[num][-1], spdl[num][-1])[0])
            a[num].append((v[num][-1] - v[num][-2]) / cmt_t[i - 1])
    return pos, tl, spdl, v, a


def new_compute(cmt_t, cmd_all):
    pos = [[cmd_all[0][0]], [cmd_all[1][0]], [], [cmd_all[3][0]], [], []]
    v = [[0] for i in range(6)]
    tl = [[] for i in range(6)]
    spdl = [[] for i in range(6)]
    for i in range(1, len(cmt_t) + 1):
        for num in motor:
            print(num, i , pos[num][i - 1],cmd_all[num][i])
            new_command = calculate_offset(v0=v[num][-1], amax=a_value[num], pos0=pos[num][i - 1],
                                           destination=cmd_all[num][i], t=cmt_t[i - 1])
            # print(v[num][i - 1])
            res = build_math_path(v[num][i - 1], vmax_value[num], pos[num][i - 1],
                                  new_command, a_value[num], a_value[num])
            tl[num].append(res[0])
            spdl[num].append(res[1])
            pos_step = get_pos_by_ts(cmt_t[i - 1], tl[num][-1], spdl[num][-1])[0]
            pos[num].append(pos[num][i - 1] + pos_step)
            v[num].append(get_math_spd_by_ts(cmt_t[i - 1], tl[num][-1], spdl[num][-1])[0])

        print("should be", cmd_all[0][i] + cos(cmd_all[3][i]) * l1_length, cmd_all[1][i] + sin(cmd_all[3][i]) * l1_length)
        print("actually", pos[0][i] + cos(pos[3][i]) * l1_length, pos[1][i] + sin(pos[3][i]) * l1_length)
    return pos, tl, spdl, v


def real_vt(t: list, time_list: list, spd_list: list, moment: list):
    j = 0
    res = []
    for i in range(len(moment)):
        while moment[i] > t[j + 1]:
            j += 1
        res.append(get_math_spd_by_ts(moment[i] - [t[j]], time_list[j], spd_list[j])[0])
    return res


if __name__ == '__main__':
    for time_step, r4, xy, outline in car_loader():
        order = [[] for i in range(6)]
        order[0] = [x[0] for x in xy]
        order[1] = [x[1] for x in xy]
        order[3] = r4
        # 计算实际位置
        pos, tl, spdl, v, a = real_compute(time_step, order)
        new_pos, tl, spdl, v = new_compute(time_step, order)

        # 计算绝对时间
        time = [0]
        for i in range(len(time_step)):
            time.append(time[i] + time_step[i])



        pyplot.plot([pos[0][i] + cos(pos[3][i]) * l1_length for i in range(len(pos[0]))],
                    [pos[1][i] + sin(pos[3][i]) * l1_length for i in range(len(pos[0]))], "b*")

        pyplot.plot([new_pos[0][i] + cos(new_pos[3][i]) * l1_length for i in range(len(new_pos[0]))],
                    [new_pos[1][i] + sin(new_pos[3][i]) * l1_length for i in range(len(new_pos[0]))], "ys")


        pyplot.plot([order[0][i] + cos(order[3][i]) * l1_length for i in range(len(order[0]))],
                    [order[1][i] + sin(order[3][i]) * l1_length for i in range(len(order[0]))], "r*")
        print(new_pos[0][32] + cos(new_pos[3][32]) * l1_length,new_pos[1][32] + sin(new_pos[3][32])* l1_length)
        print(order[0][32] + cos(order[3][32]) * l1_length, order[1][32] + sin(order[3][32])* l1_length)
        n = numpy.arange(len(new_pos[0]))
        for i, txt in enumerate(n):
            pyplot.annotate(txt, (new_pos[0][i] + cos(new_pos[3][i]) * l1_length,
                    new_pos[1][i] + sin(new_pos[3][i]) * l1_length))
        n = numpy.arange(len(pos[0]))
        for i, txt in enumerate(n):
            pyplot.annotate(txt, (pos[0][i] + cos(pos[3][i]) * l1_length,
                    pos[1][i] + sin(pos[3][i]) * l1_length))
        n = numpy.arange(len(order[0]))
        for i, txt in enumerate(n):
            pyplot.annotate(txt, (order[0][i] + cos(order[3][i]) * l1_length,
                    order[1][i] + sin(order[3][i]) * l1_length))
        # pyplot.plot([i[0] for i in outline], [i[1] for i in outline], "c*")
        pyplot.show()


        print(len(time))
        moment = numpy.linspace(0, time[-1], 5000)

        pyplot.plot(moment, real_vt(time, tl[0], spdl[0], moment))
        pyplot.plot([(time[i] + time[i + 1]) / 2 for i in range(len(time) - 1)],
                    [(pos[0][i + 1] - pos[0][i]) / time_step[i] for i in range(len(pos[0]) - 1)], "rs-")
        pyplot.plot([(time[i] + time[i + 1]) / 2 for i in range(len(time) - 1)],
                    [(order[0][i + 1] - order[0][i]) / time_step[i] for i in range(len(pos[0]) - 1)], "g*-")
        pyplot.plot(time, v[0], "o-")
        pyplot.show()
        pyplot.plot(moment, real_vt(time, tl[1], spdl[1], moment))
        pyplot.plot([(time[i] + time[i + 1]) / 2 for i in range(len(time) - 1)],
                    [(pos[1][i + 1] - pos[1][i]) / time_step[i] for i in range(len(pos[0]) - 1)], "s-")
        pyplot.plot([(time[i] + time[i + 1]) / 2 for i in range(len(time) - 1)],
                    [(order[1][i + 1] - order[1][i]) / time_step[i] for i in range(len(pos[0]) - 1)], "*-")

        pyplot.plot(time, v[1], "o-")
        pyplot.show()
        pyplot.plot(moment, real_vt(time, tl[3], spdl[3], moment))
        pyplot.plot([(time[i] + time[i + 1]) / 2 for i in range(len(time) - 1)],
                    [(pos[3][i + 1] - pos[3][i]) / time_step[i] for i in range(len(pos[0]) - 1)], "s-")
        pyplot.plot([(time[i] + time[i + 1]) / 2 for i in range(len(time) - 1)],
                    [(order[3][i + 1] - order[3][i]) / time_step[i] for i in range(len(pos[0]) - 1)], "*-")

        pyplot.plot(time, v[3], "o-")
        pyplot.show()
