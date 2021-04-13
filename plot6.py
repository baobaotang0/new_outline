import copy
import os
from math import radians, cos, sin
from pickle import load

import numpy
from matplotlib import pyplot

from offset import get_motor_max_para, is_postive

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



def make_even(ts: list, s: list, v: list, amax: list, check_point: list):
    num, i = check_point[0], check_point[1]
    s_sum = sum(s[num][i - 1:i + 2])
    a = amax[num]
    vl, vr = v[num][i - 2], v[num][i + 2]
    tl, t1, t2, t3, tr = ts[i - 2], ts[i - 1], ts[i], ts[i + 1], ts[i + 2]
    line1_a, line1_b = 1 / t1 + 1 / t2, 1 / t2
    line1_c = [s_sum / t2 - a * (t1 + t2) / 2, s_sum / t2 + a * (t1 + t2) / 2]
    line2_a, line2_b = 1 / t2, 1 / t2 + 1 / t3
    line2_c = [s_sum / t2 - a * (t3 + t2) / 2, s_sum / t2 + a * (t3 + t2) / 2]
    boundary_s1 = [(vl - a * (tl + t1) / 2) * t1, (vl + a * (tl + t1) / 2) * t1]
    boundary_s3 = [(vr - a * (tr + t3) / 2) * t1, (vr + a * (tr + t3) / 2) * t3]
    x = numpy.linspace(-s_sum, s_sum, 1000)
    pyplot.plot(x, [(line1_c[0] - line1_a * j) / line1_b for j in x])
    pyplot.plot(x, [(line1_c[1] - line1_a * j) / line1_b for j in x])
    pyplot.plot(x, [(line2_c[0] - line2_a * j) / line2_b for j in x])
    pyplot.plot(x, [(line2_c[1] - line2_a * j) / line2_b for j in x])
    pyplot.plot(x, [(line2_c[1] - line2_a * j) / line2_b for j in x])
    pyplot.plot([boundary_s1[0] for j in x], x)
    pyplot.plot([boundary_s1[1] for j in x], x)
    pyplot.plot(x, [boundary_s3[0] for j in x])
    pyplot.plot(x, [boundary_s3[1] for j in x])
    pyplot.show()


if __name__ == '__main__':
    for time_step, r4, xy, outline in car_loader():

        order = [[] for i in range(6)]
        order[0] = [x[0] for x in xy]
        order[1] = [x[1] for x in xy]
        order[3] = r4
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

        for i in check_list:
            time_step[i] = sum(time_step[i-1:i+2])/3
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
        # new_v_assume = copy.deepcopy(v_assume)
        # new_a_assume = [[] for i in range(6)]

        # for num in motor:
        #     new_a_assume[num] = [
        #         (new_v_assume[num][i + 1] - new_v_assume[num][i]) / (time_step[i] + time_step[i + 1]) * 2
        #         for i in range(len(time_step) - 1)]
        #     for i in range(len(new_a_assume[num])):
        #         if abs(new_a_assume[num][i]) > a_value[num]:
        #             print(num, i, new_a_assume[num][i])
        #             print(new_v_assume[num][i - 1], new_v_assume[num][i], new_v_assume[num][i + 1])
        #             recheck_list.append(i)

        # recheck_list = []
        new_v_assume = [[] for i in range(6)]
        new_a_assume = [[] for i in range(6)]
        for num in motor:
            new_v_assume[num] = [(s[num][i]) / time_step[i] for i in range(len(time_step))]
            new_a_assume[num] = [(new_v_assume[num][i + 1] - new_v_assume[num][i]) / (time_step[i] + time_step[i + 1]) * 2 for i in
                             range(len(time_step) - 1)]
            for i in range(len(new_a_assume[num])):
                if abs(new_a_assume[num][i]) > a_value[num]:
                    print(num, i,new_a_assume[num][i])
                    print(new_v_assume[num][i],new_v_assume[num][i+1])
                    print(s[num][i], time_step[i], s[num][i + 1], time_step[i + 1])
                    recheck_list.append(i)
        print(recheck_list)
        new_order = [[order[0][0]], [order[1][0]], [], [order[3][0]], [], []]

        for num in motor:
            for i in s[num]:
                new_order[num].append(new_order[num][-1] + i)

        l1_length = 0.6471
        pyplot.plot([new_order[0][i] + cos(new_order[3][i]) * l1_length for i in range(len(new_order[0]))],
                    [new_order[1][i] + sin(new_order[3][i]) * l1_length for i in range(len(new_order[0]))], "go")
        pyplot.plot([i[0] for i in outline], [i[1] for i in outline], "c*")
        pyplot.plot([outline[i][0] for i in check_list], [outline[i][1] for i in check_list], "r*")
        pyplot.show()

        for num in motor:
            if num == 3:
                pyplot.subplot(2, 1, 2)
            else:
                pyplot.subplot(2, 2, num + 1)
            pyplot.plot(v_assume[num], "go-")
            pyplot.plot(new_v_assume[num], "r*-")
            pyplot.plot([i for i in recheck_list], [v_assume[num][i] for i in recheck_list], "D")
        pyplot.show()

        # 计算绝对时间
        t = [0]
        for i in range(len(time_step)):
            t.append(t[i] + time_step[i])
