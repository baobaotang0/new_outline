import os
from math import radians, cos, sin, sqrt
from pickle import load

import numpy
from matplotlib import pyplot

from offset import get_motor_max_para
from motor import build_math_path, get_math_spd_by_ts, get_pos_by_ts

motor = [0, 1, 3]
time_step_min = 0.1
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


def check_acc(ts: list, s: list, v: list, amax: list, check_list: list):
    duration = sum(ts[check_list[0]:check_list[-1] + 1])
    s_sum = sum(s[check_list[0]:check_list[-1] + 1])
    new_v = [v[check_list[0] - 1]]

    for i in range(check_list[0], check_list[-1] + 1):
        ts.pop(check_list[0])
        for num in motor:
            s[num].pop(check_list[0])
            v[num].pop(check_list[0])
    step_num = int(duration / time_step_min) + 1
    print(step_num * time_step_min, duration)
    recheck_list = [-1]
    while recheck_list:
        recheck_list = []
        increase = (s_sum - step_num * s[check_list[0] - 1]) / ((1 + step_num) * step_num / 2)
        for i in range(step_num):
            ts.insert(check_list[0], time_step_min)
            for num in motor:
                s[num].insert(check_list[0] + i, s_sum + increase * (i + 1))
                v[num].insert(s[num][check_list[0] + i] / time_step_min)
        for i in range(check_list[0], check_list[0] + step_num):
            if abs((v[num][i + 1] - v[num][i]) / (ts[i] + ts[i + 1]) * 2) > amax[num]:
                recheck_list.append(i)


def insert_one(ts: list, s: list, check_list: list):
    for i in range(len(check_list)):
        ts.insert(check_list[i] + i + 1, ts[check_list[i] + i + 1])
        for num in motor:
            local_s = s[num][check_list[i] + i + 1]
            s[num].pop(check_list[i] + i + 1)
            s[num].insert(check_list[i] + i + 1, local_s / 2)
            s[num].insert(check_list[i] + i + 1, local_s / 2)


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
    window = 1
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
            acc_adv_list[i] = acc_raw

    check_dict = {}
    for i in range(length1 - 2):
        for acc in acc_adv_list:
            if abs(acc) > limit_acc:
                check_dict[i] = sqrt(abs(acc) / limit_acc)

    new_tar = source.copy()
    for i in range(1, length1 - 1):
        if i in change_list:
            v0 = (new_tar[i] - new_tar[i - 1]) / time_step[i - 1]
            v1 = v0 + acc_adv_list[i - 1] * (time_step[i] + time_step[i - 1]) / 2
            s = v1 * time_step[i]
            new_tar[i+1] = new_tar[i] + s

    new_ts = time_step.copy()
    for idx, r in check_dict.items():
        new_ts[idx] *= r

    return new_ts, new_tar

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




if __name__ == '__main__':
    for time_step, r4, xy, outline in car_loader():

        order = [[] for i in range(6)]
        order[0] = [x[0] for x in xy]
        order[1] = [x[1] for x in xy]
        order[3] = r4
        # 计算绝对时间
        t = [0]
        for i in range(len(time_step)):
            t.append(t[i] + time_step[i])

        s = [[] for i in range(6)]
        v_assume = [[] for i in range(6)]
        a_assume = [[] for i in range(6)]
        check_list = []
        for num in motor:
            s[num] = [order[num][i + 1] - order[num][i] for i in range(len(order[num]) - 1)]
            v_assume[num] = [(s[num][i]) / time_step[i] for i in range(len(time_step))]
            a_assume[num] = [(v_assume[num][i + 1] - v_assume[num][i]) / (time_step[i] + time_step[i + 1]) * 2 for i in
                             range(len(time_step) - 1)]
            for i in range(len(a_assume[num])):
                if abs(a_assume[num][i]) > a_value[num]:
                    print(num, i, )
                    print(s[num][i], time_step[i], s[num][i + 1], time_step[i + 1])
                    # check_list.append([num, i])
                    check_list.append(i)
        print(check_list)
        new_order = [[]for i in range(6)]
        for num in motor:
            new_time_step, new_order[num] = line_filter(time_step, order[num], a_value[num])
        new_pos, tl, spdl, v, a = real_compute(new_time_step, new_order)
        pos, tl, spdl, v, a = real_compute(time_step, order)
        l1_length = 0.6471
        pyplot.plot([new_order[0][i] + cos(new_order[3][i]) * l1_length for i in range(len(new_order[0]))],
                    [new_order[1][i] + sin(new_order[3][i]) * l1_length for i in range(len(new_order[0]))], "go")
        pyplot.plot([pos[0][i] + cos(pos[3][i]) * l1_length for i in range(len(pos[0]))],
                    [pos[1][i] + sin(pos[3][i]) * l1_length for i in range(len(pos[0]))], "b*")
        pyplot.plot([new_pos[0][i] + cos(new_pos[3][i]) * l1_length for i in range(len(new_pos[0]))],
                    [new_pos[1][i] + sin(new_pos[3][i]) * l1_length for i in range(len(new_pos[0]))], "y*")
        pyplot.plot([i[0] for i in outline], [i[1] for i in outline], "c*")
        pyplot.plot([outline[i][0] for i in check_list], [outline[i][1] for i in check_list], "r*")
        pyplot.show()




            # result =[]
        # for _, i in groupby(enumerate(check_list), lambda x: x[1] - x[0]):
        #     result.append([j for _, j in i])
        # check_list= set(check_list)
        # check_list= list(check_list)
        # print(check_list)
        # check_list.sort()
        # l = check_acc(time_step, s, a_value)
        # print(l)
        recheck_list = []

        new_v_assume = [[] for i in range(6)]
        new_a_assume = [[] for i in range(6)]
        for num in motor:
            new_v_assume[num] = [(new_order[num][i+1]-new_order[num][i]) / new_time_step[i] for i in range(len(new_time_step))]
            new_a_assume[num] = [
                (new_v_assume[num][i + 1] - new_v_assume[num][i]) / (new_time_step[i] + new_time_step[i + 1]) * 2 for i in
                range(len(new_time_step) - 1)]
            for i in range(len(new_a_assume[num])):
                if abs(new_a_assume[num][i]) > a_value[num]:
                    print(num, i, new_a_assume[num][i])
                    print(new_v_assume[num][i], new_v_assume[num][i + 1])
                    print(s[num][i], new_time_step[i], s[num][i + 1], new_time_step[i + 1])
        # for num in motor:
        #     for i in s[num]:
        #         new_order[num].append(new_order[num][-1] + i)
                    recheck_list.append(i)
        print(recheck_list)
        # new_order = [[order[0][0]], [order[1][0]], [], [order[3][0]], [], []]
        #

        # l1_length = 0.6471
        # pyplot.plot([new_order[0][i] + cos(new_order[3][i]) * l1_length for i in range(len(new_order[0]))],
        #             [new_order[1][i] + sin(new_order[3][i]) * l1_length for i in range(len(new_order[0]))], "go")
        # pyplot.plot([i[0] for i in outline], [i[1] for i in outline], "c*")
        # pyplot.plot([outline[i][0] for i in check_list], [outline[i][1] for i in check_list], "r*")
        # pyplot.show()

        for num in motor:
            if num == 3:
                pyplot.subplot(2, 1, 2)
            else:
                pyplot.subplot(2, 2, num + 1)
            pyplot.plot(v_assume[num], "go-")
            pyplot.plot(new_v_assume[num], "r*-")
            pyplot.plot([i for i in recheck_list], [v_assume[num][i] for i in recheck_list], "D")
        pyplot.show()


