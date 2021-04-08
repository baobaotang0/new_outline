from matplotlib import pyplot
from math import sqrt, e, degrees, atan, pi, cos, tan, sin
from pickle import load
from offset import get_motor_max_para, calculate_motor_send_interval, is_postive
from motor import build_math_path, get_math_spd_by_ts, get_pos_by_ts
import copy, os,numpy

motor = [0, 1, 3]
# 读最大加速度，最大速度
motor_max = get_motor_max_para("bot1.json")
vmax_value = [motor_max[num]["vel"] for num in range(1, 7)]
a_value = [motor_max[num]["acc"] for num in range(1, 7)]

def car_loader():
    root_path = "./data/"
    for filename in os.listdir(root_path):
        file_path = root_path + filename
        if file_path[-4:] == ".npy":
            with open(file_path, 'rb') as f_pos:
                print(file_path)
                time_step, r4, xy = load(f_pos)
                yield time_step, r4, xy


def compute_one(cmt_t, cmd1, vmax, acc):
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


def real_compute(cmt_t, cmd_all):
    pos = [[cmd_all[0][0]], [cmd_all[1][0]], [], [cmd_all[3][0]], [], []]
    v = [[0] for i in range(6)]
    offset = [[0] for i in range(6)]
    for num in motor:
        for i in range(1, len(cmt_t) + 1):
            time_list, spd_list, run_type = build_math_path(v[num][i - 1], vmax_value[num], pos[num][i - 1],
                                                            cmd_all[num][i], a_value[num], a_value[num])
            pos_step = get_pos_by_ts(cmt_t[i - 1], time_list, spd_list)[0]
            pos[num].append(pos[num][i - 1] + pos_step)
            offset[num].append(pos[num][i] - cmd_all[num][i])
            v[num].append(get_math_spd_by_ts(cmt_t[i - 1], time_list, spd_list)[0])
    return pos


def base_creator():
    # 计算理想轨迹
    order = [[] for i in range(6)]
    order[0] = [x[0] for x in xy]
    order[1] = [x[1] for x in xy]
    order[3] = r4

    # 计算实际位置
    pos = real_compute(time_step, order)
    # 计算绝对时间
    time = [0]
    for i in range(len(time_step)):
        time.append(time[i] + time_step[i])
    return time, order, pos





def point_check():
    xlist = [[] for i in range(6)]
    ylist = [[] for i in range(6)]
    for o1, p1, v1, rx, ry in zip(o, p, vmax_value, xlist, ylist):
        if not o1:
            continue
        low = min(o1)
        high = max(o1)
        length = len(o1)
        for i in range(length - 1):
            dis = abs(o1[i] - p1[i])
            percent = + dis / (high - low)
            o_spd = (o1[i + 1] - o1[i]) / (t[i + 1] - t[i])
            if percent > 0.005:
                rx.append(t[i])
                ry.append(o1[i])
    return xlist, ylist


def f1(cmd, t, acc):
    """
    处理单条曲线指令!
    """
    # 补偿减速部分
    cmd1 = cmd.copy()
    length = len(cmd1)

    # 圆滑

    if not length:
        return []

    change_list = set()
    acc_list = [0] * length
    for i in range(length - 2):
        p0 = cmd[i]
        p1 = cmd[i + 1]
        p2 = cmd[i + 2]
        v1 = (p1 - p0) / (t[i + 1] - t[i])
        v2 = (p2 - p1) / (t[i + 2] - t[i + 1])
        acc_cur = (v2 - v1) / (t[i + 2] - t[i]) * 2

        if abs(acc_cur) > acc * 2:
            c = int(abs(acc_cur) // acc)
            # print(i, c)
            for j in range(i - c + 1, i + c):  # TODO 检测边界
                d = c - abs(j - i)
                acc_list[j] += acc_cur * d / c / c
                change_list.add(j)
        else:
            acc_list[i] += acc_cur

    for i in change_list:
        p0 = cmd1[i-1]
        p1 = cmd1[i]
        v1 = (p1 - p0) / (t[i] - t[i - 1])
        v2 = (t[i + 2] - t[i]) / 2 * acc_list[i] + v1
        cmd1[i + 1] = p1 + v2 * (t[i + 1] - t[i])

    low = min(cmd1)
    high = max(cmd1)

    for i in range(length - 1):
        dv = (cmd1[i + 1] - cmd1[i]) / (t[i + 1] - t[i])
        ds = dv * abs(dv) / acc / 2
        cmd1[i] += ds

    return cmd1


def f2():
    a_list_all = []
    for o1, a in zip(o, a_value):
        length = len(o1)
        a_list = [0] * length
        for i in range(length - 2):
            p0 = o1[i]
            p1 = o1[i + 1]
            p2 = o1[i + 2]
            v1 = (p1 - p0) / (t[i + 1] - t[i])
            v2 = (p2 - p1) / (t[i + 2] - t[i + 1])
            acc = (v2 - v1) / (t[i + 2] - t[i]) * 2

            if abs(acc) > a * 4:
                c = int(abs(acc) // a)
                for j in range(i - c + 1, i + c):
                    d = c - abs(j - i)
                    a_list[j] += acc * d / c / c
            else:
                a_list[i] += acc

        a_list_all.append(a_list)
    return a_list_all


def new_cmd_pos():
    length = len(t)
    res_o = []
    for o1, a in zip(o, a_value):
        o2 = f1(o1, t, a)
        res_o.append(o2)
    return res_o

def func(delta_t, s0, a, t):
    return -0.5 * a * (t - delta_t) ** 2 + s0

if __name__ == '__main__':
    for time_step, r4, xy in car_loader():
    # with open("./data/car_03.npy", 'rb') as f_pos:
    #     time_step, r4, xy = load(f_pos)

        t, o, p = base_creator()
        pyplot.plot(o[0],o[1])
        pyplot.xlim((1,6))
        pyplot.ylim((0, 5))
        pyplot.show()

        for n in motor:
            pyplot.plot(t, o[n], 'b')
            pyplot.plot(t, p[n], 'r')

            a_all = f2()
            # pyplot.plot(t, a_all[n], 'g')

            new_o = new_cmd_pos()
            new_p = real_compute(time_step, new_o)
            # pyplot.plot(t, new_o[n], 'g')
            pyplot.plot(t, new_p[n], 'y')
            pyplot.show()