import numpy
import os
from math import radians
from pickle import load

from matplotlib import pyplot
from scipy.optimize import minimize

from motor import build_math_path, get_math_spd_by_ts, get_pos_by_ts
from offset import get_motor_max_para, is_postive

motor = [0, 1, 3]
# 读最大加速度，最大速度
motor_max = get_motor_max_para("bot1.json")
vmax_value = [motor_max[num]["vel"] for num in range(1, 7)]
a_value = [motor_max[num]["acc"] for num in range(1, 7)]
vmax_value[3] = radians(vmax_value[3])
a_value[3] = radians(a_value[3])


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


def compute_one(cmt_t, cmd1, vmax, acc, v0, pos0):
    time_list, spd_list, run_type = build_math_path(v0, vmax, pos0, cmd1, acc, acc)
    pos_step = get_pos_by_ts(cmt_t, time_list, spd_list)[0]
    pos = pos0 + pos_step
    v = get_math_spd_by_ts(cmt_t, time_list, spd_list)[0]
    return pos, time_list, spd_list, v


# def calculate_t1(v0, a, s, t):
#     a = is_postive(s)*a
#     if abs(s) < v0*t - 1/2 * a * t ** 2:




def trangle_s(t1, v0, t, a):
    return v0 * t1 + 1 / 2 * a * t1 ** 2 + (v0 + a * (2 * t1 - t)) * (t - t1) + 1 / 2 * a * (t - t1) ** 2


def error(t1, v0, t, s, a_value, pos0):
    res = 0
    v_start = [v0]
    pos = [pos0]
    for mid_t, time_step, destination in zip(t1, t, s):
        a = is_postive(destination-pos0)*a_value
        pos.append(pos[-1] + trangle_s(t1=mid_t, v0=v_start[-1], t=time_step, a=a))
        res += (pos[-1] - destination) ** 2
        v_start.append(v_start[-1] + a * (2 * mid_t - time_step))
    return res


def new_compute(cmt_t, cmd_all):
    pos = [[cmd_all[0][0]], [cmd_all[1][0]], [], [cmd_all[3][0]], [], []]
    v = [[0] for i in range(6)]
    tl = [[] for i in range(6)]
    spdl = [[] for i in range(6)]
    new_command = [[] for i in range(6)]
    predict_len = 2
    for num in motor:
        for i in range(1, len(cmt_t) + 1):
            if abs(cmd_all[num][i]-pos[num][i - 1])> 1/4*a_value[num]*cmt_t[i - 1]**2 and i < len(cmt_t) - predict_len:
                bds = [(0, cmt_t[j]) for j in range(i - 1, i - 1 + predict_len)]
                min1 = minimize(fun=lambda t1: abs(error(t1=t1, v0=v[num][-1], t=cmt_t[i - 1:i - 1 + predict_len],
                                                     s=cmd_all[num][i:i + predict_len], pos0=pos[num][i - 1],
                                                     a_value=a_value[num])),method='Powell',
                                x0=numpy.array([0 for i in cmt_t[i - 1:i - 1 + predict_len]]), bounds=bds)

                # print(min1)
                t1 = min1.x[0]
                this_time_step = sum(cmt_t[:i - 1])
                # print(cmt_t[i - 1], t1)
                v0 = v[num][-1]
                p0 = pos[num][i - 1]
                s0 =cmd_all[num][i:i + predict_len]
                up = v[num][-1] * t1 + 1 / 2 * a_value[num] * t1 ** 2
                down = (v[num][-1] + a_value[num] * t1) ** 2 / 2 / a_value[num]
                trangle = trangle_s(t1=t1, v0=v[num][-1], t=cmt_t[i - 1], a=a_value[num])
                new_command[num].append(p0 + up + down)


            else:
                new_command[num].append(cmd_all[num][i])
            # print(v[num][i - 1])
            res = build_math_path(v[num][i - 1], vmax_value[num], pos[num][i - 1],
                                  new_command[num][-1], a_value[num], a_value[num])
            tl[num].append(res[0])
            spdl[num].append(res[1])
            pos_step = get_pos_by_ts(cmt_t[i - 1], tl[num][-1], spdl[num][-1])[0]
            pos[num].append(pos[num][i - 1] + pos_step)
            v[num].append(get_math_spd_by_ts(cmt_t[i - 1], tl[num][-1], spdl[num][-1])[0])
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
        # pos, tl, spdl, v = new_compute(time_step, order)
        # 计算绝对时间
        time = [0]
        for i in range(len(time_step)):
            time.append(time[i] + time_step[i])

        check_points = [[] for i in range(6)]
        for num in motor:
            for i in range(len(a[num])):
                if abs(a[num][i]) > a_value[num]:
                    print(a[num][i], a_value[num])
                    check_points[num].append(i)

        print(check_points)

        # pyplot.plot(time[1:], a[0],"r")
        # pyplot.plot([time[i]for i in check_points[0]], [a[0][i] for i in check_points[0]], "o")
        # pyplot.plot(time[1:], a[1],"g")
        # pyplot.plot([time[i]for i in check_points[1]], [a[1][i] for i in check_points[1]], "o")
        # pyplot.plot(time[1:], a[3],"b")
        # pyplot.plot([time[i]for i in check_points[3]], [a[3][i] for i in check_points[3]], "o")
        # pyplot.show()

        moment = numpy.linspace(0, time[-1], 5000)
        pyplot.plot(moment, real_vt(time, tl[0], spdl[0], moment))
        pyplot.plot([(time[i] + time[i + 1]) / 2 for i in range(len(time) - 1)],
                    [(pos[0][i + 1] - pos[0][i]) / time_step[i] for i in range(len(pos[0]) - 1)], "s-")
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
