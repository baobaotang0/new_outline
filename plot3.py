from matplotlib import pyplot
from math import sqrt, e, radians, sin, asin
from pickle import load
from scipy.spatial.distance import pdist
from scipy.optimize import fmin
from itertools import groupby
from offset import get_motor_max_para, calculate_motor_send_interval, is_postive
from motor import build_math_path, get_math_spd_by_ts, get_pos_by_ts
import os

motor = [0, 1, 3]
# 读最大加速度，最大速度
motor_max = get_motor_max_para("bot1.json")
vmax_value = [motor_max[num]["vel"] for num in range(1, 7)]
a_value = [motor_max[num]["acc"] for num in range(1, 7)]

def car_loader():
    root_path = "./data/"
    for filename in os.listdir(root_path):
        file_path = root_path + filename
        with open(file_path, 'rb') as f_pos:
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

# 计算曲率比较大的波动点
def compute_fluctuation(o,t, round_len_1side=5):
    fluctuation_points =[[] for i in range(6)]
    curvity = [[] for i in range(6)]
    for num in motor:
        for i in range(1,len(o[num])-1):
            vector1 = (t[i]-t[i-1], o[num][i]-o[num][i-1])
            vector2 = (t[i+1]-t[i], o[num][i+1]-o[num][i])
            curvity[num].append(asin(abs(pdist([vector1, vector2], 'cosine')[0])))
    curvity_avg = [sum(curvity[0]) / len(curvity[0]), sum(curvity[1]) / len(curvity[1]), None,
                       sum(curvity[3]) / len(curvity[3]), None, None]
    for num in motor:
        for i in range(len(curvity[num])):
            if curvity[num][i] > curvity_avg[num]: # TODO: hahaha
                fluctuation_points[num].append(i+1)
    # 计算波动点的密度，平均密度
    fluctuation_points_density = [None for i in range(6)]
    fluc_points_density_avg = [None for i in range(6)]
    for num in motor:
        fluctuation_points_density[num] = calculate_density(fluctuation_points[num], round_len_1side, len(o[num]))
        fluc_points_density_avg[num] = sum(fluctuation_points_density[num]) / len(fluctuation_points_density[num])
    # 计算波动点周围需要特殊处理的范围
    rough_range = [[] for i in range(6)]
    for num in motor:
        for i in range(len(o[num])):
            if fluctuation_points_density[num][i] > fluc_points_density_avg[num]:
                rough_range[num].append(i)
    # 对特殊处理的范围进行分段
    rough_range_boundary = [[]for i in range(6)]
    for num in motor:
        for _, i in groupby(enumerate(rough_range[num]), lambda x: x[1]-x[0]):
            rough_range_boundary[num].append([j for _, j in i])
    return fluctuation_points,rough_range_boundary


# 一维点的密度函数
def calculate_density(data: list, window_len_1side: int, total_len:int):
    if total_len <= window_len_1side:
        return f"窗口太长，单侧范围最大为{len(data)-1}"
    y = [1 if i in data else 0 for i in range(total_len)]
    new_data = [0] * window_len_1side + y + [0] * window_len_1side
    density = []
    for i in range(window_len_1side, len(new_data) - window_len_1side):
        density.append(sum([new_data[j] / sqrt(1+(j-i)**2)for j in range(i-window_len_1side, i+window_len_1side)]))
    return density


def base_creator(time_step, xy, r4):
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

def f1(cmd, t, acc):
    """
    处理单条曲线指令!
    """
    # 补偿减速部分
    cmd1 = cmd.copy()

    length = len(cmd1)
    if not length:
        return []

    low = min(cmd1)
    high = max(cmd1)

    # _t = time_step.copy()
    # _t.reverse()
    # cmd1.reverse()
    # cmd1 = compute_one(_t, cmd1, vol * 1.4, acc * 2)
    # cmd1.reverse()

    for i in range(length - 1):
        dv = (cmd1[i + 1] - cmd1[i]) / (t[i + 1] - t[i])
        ds = dv * abs(dv) / acc / 2
        cmd1[i] += ds

    '''测试函数，模拟边界'''
    # for i in range(length):
    #     if cmd1[i] < 1:
    #         cmd1[i] = 1

    return cmd1



def new_cmd_pos(order,t):
    res_o = []
    for o1, a in zip(order, a_value):
        o2 = f1(o1, t, a)
        res_o.append(o2)
    return res_o


if __name__ == '__main__':
    for time_step, r4, xy in car_loader():
        t, o, p = base_creator(time_step, xy, r4)
        fluc_p, rough_rg = compute_fluctuation(o=o, t=t)
        print(rough_rg)
        for n in motor:

            pyplot.plot(o[n], 'b')
            pyplot.plot(p[n], 'r')
            new_o = new_cmd_pos(o,t)
            new_p = accelerate(new_order=new_o, order=o, rough_range=rough_rg, time_step=time_step)
            # pyplot.plot([t[i] for i in fluc_p[n]], [o[n][i] for i in fluc_p[n]], "*")
            pyplot.plot(fluc_p[n], [o[n][i] for i in fluc_p[n]], "*")
            # pyplot.plot(t, new_o[n], 'g')
            pyplot.plot(new_p[n], 'y')
            pyplot.show()