from matplotlib import pyplot
from math import sqrt, e
import numpy
from itertools import groupby
from scipy.optimize import fmin, leastsq
from scipy.stats import gaussian_kde
from scipy.spatial.distance import pdist
from pickle import load
from offset import get_motor_max_para, calculate_motor_send_interval, is_postive
from motor import build_math_path, get_math_spd_by_ts, get_pos_by_ts
import copy
motor = [0, 1, 3]
# 读最大加速度，最大速度
motor_max = get_motor_max_para("bot1.json")
vmax_value = [motor_max[num]["vel"] for num in range(1, 7)]
a_value = [motor_max[num]["acc"] for num in range(1, 7)]

# 读位置信息
with open("./data/car_01.npy", 'rb') as f_pos:
    time_step, r4, xy = load(f_pos)

# 计算理想轨迹
order = [[] for i in range(6)]
order[0] = [x[0] for x in xy]
order[1] = [x[1] for x in xy]
order[3] = r4

# 计算位移
s = [[] for i in range(6)]
for i in range(len(order[0]) - 1):
    for num in motor:
        s[num].append(order[num][i + 1] - order[num][i])

# 计算每两点间理想平均速度,加速度，
v_assume = [[] for i in range(6)]
a_assume = [[] for i in range(6)]
for num in [0, 1, 3]:
    v_assume[num] = [(order[num][i+1] - order[num][i]) / time_step[i] for i in range(len(time_step))]
    a_assume[num] = [(v_assume[num][i+1] - v_assume[num][i]) / time_step[i+1] for i in range(len(time_step)-1)]

# 计算实际位置
pos = [[order[0][0]], [order[1][0]], [], [order[3][0]], [], []]
v = [[0] for i in range(6)]
offset = [[0] for i in range(6)]
for num in motor:
    for i in range(1, len(time_step)+1):
        time_list, spd_list, run_type = build_math_path(v[num][i-1], vmax_value[num], pos[num][i-1],
                                                        order[num][i], a_value[num], a_value[num])
        pos_step = get_pos_by_ts(time_step[i-1], time_list, spd_list)[0]
        pos[num].append(pos[num][i-1]+pos_step)
        offset[num].append(pos[num][i] - order[num][i])
        v[num].append(get_math_spd_by_ts(time_step[i-1], time_list, spd_list)[0])

# 计算绝对时间
t = [0]
for i in range(len(time_step)):
    t.append(t[i]+time_step[i])

# 计算曲率比较大的波动点
fluctuation_points =[[] for i in range(6)]
curvity = [[] for i in range(6)]
for num in motor:
    for i in range(1,len(order[num])-1):
        vector1 = (t[i]-t[i-1], order[num][i]-order[num][i-1])
        vector2 = (t[i+1]-t[i], order[num][i+1]-order[num][i])
        curvity[num].append(abs(pdist([vector1, vector2], 'cosine')[0]))
curvity_avg = [sum(curvity[0])/len(curvity[0]), sum(curvity[1])/len(curvity[1]), None,
               sum(curvity[3])/len(curvity[3]), None, None]
for num in motor:
    for i in range(len(curvity[num])):
        if curvity[num][i] > curvity_avg[num]:
            fluctuation_points[num].append(i+1)

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

# 计算波动点的密度，平均密度
fluctuation_points_density = [None for i in range(6)]
fluc_points_density_avg = [None for i in range(6)]
for num in motor:
    fluctuation_points_density[num] = calculate_density(fluctuation_points[num], 5, len(order[num]))
    fluc_points_density_avg[num] = sum(fluctuation_points_density[num]) / len(fluctuation_points_density[num])

# 计算波动点周围需要特殊处理的范围
rough_range = [[] for i in range(6)]
for num in motor:
    for i in range(len(order[num])):
        if fluctuation_points_density[num][i] > fluc_points_density_avg[num]:
            rough_range[num].append(i)

# 对特殊处理的范围进行分段
rough_range_boundary = [[]for i in range(6)]
for num in motor:
    for _, i in groupby(enumerate(rough_range[num]), lambda x: x[1]-x[0]):
        rough_range_boundary[num].append([j for _, j in i])
print(rough_range_boundary)





# 计算平均速度，平均加速度，方差
a_avg = [sum(a_assume[0])/len(a_assume[0]), sum(a_assume[1])/len(a_assume[1]), None,
         sum(a_assume[3])/len(a_assume[3]), None, None]
v_avg = [sum(v_assume[0])/len(v_assume[0]), sum(v_assume[1])/len(v_assume[1]), None,
         sum(v_assume[3])/len(v_assume[3]), None, None]
print(a_avg)
a_deviation = [None] * 6
v_deviation = [None] * 6
for num in [0, 1, 3]:
    a_deviation[num] = sqrt(sum([(a - a_avg[num]) ** 2 for a in a_assume[num]]) / len(a_assume[num]))
    v_deviation[num] = sqrt(sum([(v - v_avg[num]) ** 2 for v in v_assume[num]]) / len(v_assume[num]))


# 寻找启动，反向，停止点
switch = [{} for i in range(6)]
for num in motor:
    for i in range(len(s[num])-1):
        if is_postive(s[num][i]) != is_postive(s[num][i - 1]):  # and is_postive(s[num][i-1]) != 0
            left, right = is_postive(s[num][i-1]), is_postive(s[num][i])
            switch[num][i] = [left, right]
print(switch)

# 对启动，反向，停止点进行排序
turning_points = [[] for i in range(6)]
for num in [0, 1, 3]:
    l = list(switch[num].keys())
    l.sort()
    turning_points[num] = l + [len(order[0])-1]

# 求每一段的平均速度，与平均偏差
v_avg = [{} for i in range(6)]
offset_avg = [{} for i in range(6)]
for num in [0, 1, 3]:
    start = 0
    for i in range(len(turning_points[num])):
        end = turning_points[num][i]
        if len(v[num][start:end]) != 0:
            v_avg[num][(start, end)] = (order[num][end]-order[num][start])/(t[end]-t[start])
            offset_avg[num][start, end] = sum(offset[num][start:end]) / len(offset[num][start:end])
        start = turning_points[num][i]


rough_range = [{} for i in range(6)]
for num in motor:
    start = 0
    for i in range(len(turning_points[num])-1):
        key = turning_points[num][i]
        next = turning_points[num][i+1]
        t1 = abs(v_avg[num][(start, key)] / a_value[num])
        t2 = abs(v_avg[num][(key, next)] / a_value[num])
        mid_t = t[key]
        rough_range[num][key] = [[], [], []]
        for j in range(int(-t1/0.02)-1, int(t2/0.02)+1):
            if mid_t-t1 <= t[key + j] <= mid_t+t2:
                rough_range[num][key][0].append(key + j)
                rough_range[num][key][1].append(t[key + j])
                rough_range[num][key][2].append(order[num][key + j])
        if rough_range[num][key][0][0] == key:
            switch[num][key][0] = 0
            # print(v_assume[num][rough_range[num][key][0][-1]])
        if rough_range[num][key][0][-1] == key:
            switch[num][key][1] = 0
            # print(v_assume[num][rough_range[num][key][0][0]])
        # while abs(v_assume[num][rough_range[num][key][0][0]] - v_avg[num][(start, key)]) > abs(v_avg[num][(start, key)])*0.3:
        #     # print(rough_range[num][key][0][0] -1)
        #     rough_range[num][key][0].insert(0, rough_range[num][key][0][0] -1)
        #     rough_range[num][key][1].insert(0, t[rough_range[num][key][0][0] -1])
        #     rough_range[num][key][2].insert(0, order[num][rough_range[num][key][0][0] -1])
        # while abs(v_assume[num][rough_range[num][key][0][-1]] - v_avg[num][(key, next)]) > abs(
        #         v_avg[num][(key, next)]) * 0.3:
        #     # print(rough_range[num][key][0][-1] +1)
        #     rough_range[num][key][0].append(rough_range[num][key][0][-1] + 1)
        #     rough_range[num][key][1].append(t[rough_range[num][key][0][-1] + 1])
        #     rough_range[num][key][2].append(order[num][rough_range[num][key][0][-1] + 1])
        # if switch[num][key] == [0, 0]:
        #     v_avg[num].pop(start, key)
        #     switch[num].pop(key)
        #     turning_points[num].remove(key)
        start = key
print(switch)
print(rough_range[0])


# 根据平均偏差进行平移
new_order = copy.deepcopy(order)
for num in [0, 1, 3]:
    for rg in offset_avg[num].keys():
        for i in range(rg[0], rg[1]):
            new_order[num][i] = new_order[num][i] - offset_avg[num][rg]

# def Fun(order, amax, vmax, time_step, v0, s0):
#     new_pos = [s0]
#     new_v = [v0]
#     for i in range(len(time_step)):
#         time_list, spd_list, run_type = build_math_path(new_v[i], vmax, new_pos[i], order[i], amax, amax)
#         pos_step = get_pos_by_ts(time_step[i], time_list, spd_list)[0]
#         new_pos.append(new_pos[i] + pos_step)
#         new_v.append(get_math_spd_by_ts(time_step[i], time_list, spd_list)[0])
#     return new_pos, new_v
#
#
# def error(new_order, order, amax, vmax, time_step, v0, s0):
#     real_pos, real_v = Fun(order=new_order , amax=amax, vmax=vmax, time_step=time_step, v0=v0, s0=s0)
#     return sum([(real_pos[i] - order[i])**2 for i in range(len(order))])
#
#
# new_pos = [[order[0][0]], [order[1][0]], [], [order[3][0]], [], []]
# new_v = [[0] for i in range(6)]
# for num in motor:
#     start = 0
#     for special_points in rough_range[num].values():
#         print(special_points[0][0])
#         real_pos, real_v = Fun(order=new_order[num][start+1:special_points[0][0]], amax=a_value[num],
#                                vmax=vmax_value[num], time_step=time_step[start:special_points[0][0]-1],
#                                v0=new_v[num][-1], s0=new_pos[num][-1])
#         new_pos[num] = new_pos[num] + real_pos[1:]
#         new_v[num] = new_v[num] + real_v[1:]
#
#         min1 = fmin(lambda o: error(new_order=o, order=order[num][special_points[0][0]-1:special_points[0][-1]],
#                                     amax=a_value[num], vmax=vmax_value[num],
#                                     time_step=time_step[special_points[0][0]:special_points[0][-1]],
#                                     v0=new_v[num][-1], s0=new_pos[num][-1]),
#                                     order[num][special_points[0][0]:special_points[0][-1]])
#
#         start = special_points[0][-1]
#         new_order[num][special_points[0][0]:special_points[0][-1]] = min1
#         print(min1)
#         print(order[num][special_points[0][0]:special_points[0][-1]])
#




def Fun(a, s0, delta_t, t):
    if t < delta_t:
        return s0
    else:
        return a/2*(t-delta_t)**2 + s0


def error(a, s0, delta_t, t, s):
    return sum([(Fun(a=a, s0=s0, delta_t=delta_t, t=t[i]) - s[i])**2 for i in range(len(t))])


for num in [0]:
    for i in range(len(turning_points[num])-1):
        key = turning_points[num][i]
        print("******",key)
        if switch[num][key][0] == 0:  # 启动
            a = a_value[num] * switch[num][key][1]
            print("start")
        elif switch[num][key][1] == 0:  # 停止
            a = -a_value[num] * switch[num][key][0]
            print("stop")
        print(a)
        s0 = order[num][key]
        print("s0", s0)
        print(rough_range[num][key][1])
        para = fmin(lambda delta_t: error(a=a, s0=s0, delta_t=delta_t,
                                          t=rough_range[num][key][1], s=rough_range[num][key][2]), t[key])
        print(para)
        delta_t = para[0]
        pyplot.plot(rough_range[num][key][0],
                    [Fun(a=a, s0=s0, delta_t=t[key], t=tt) for tt in rough_range[num][key][1]])
        for j in range(int(delta_t/0.02-1)-len(rough_range[num][key][0]), rough_range[num][key][0][-1]):
            if t[j] <= delta_t <= t[j+1]:
                delta_x = j
                print(delta_x, t[delta_x], delta_t, key, t[key])
                break

        if switch[num][key][0] == 0:  # 启动

            rg = (min(delta_x, rough_range[num][key][0][0]), rough_range[num][key][0][-1])
        elif switch[num][key][1] == 0:  # 停止

            rg = (rough_range[num][key][0][0], max(delta_x,rough_range[num][key][0][-1]))


        for j in range(rough_range[num][key][0][0], rough_range[num][key][0][0]):
            vt = a * (t[j] - delta_t)
            new_order[num][j] = a / 2 * (t[j] - delta_t) ** 2 + s0  #+ vt**2/2/a
            print(a / 2 * (t[j] - delta_t) ** 2 + s0 + vt**2/2/a)





# 计算实际位置
new_pos = [[order[0][0]], [order[1][0]], [], [order[3][0]], [], []]
new_v = [[0] for i in range(6)]
for num in motor:
    for i in range(1, len(time_step)+1):
        time_list, spd_list, run_type = build_math_path(new_v[num][i-1], vmax_value[num], new_pos[num][i-1],
                                                        order[num][i], a_value[num], a_value[num])
        pos_step = get_pos_by_ts(time_step[i-1], time_list, spd_list)[0]
        new_pos[num].append(new_pos[num][i-1]+pos_step)
        new_v[num].append(get_math_spd_by_ts(time_step[i-1], time_list, spd_list)[0])


num = 0

pyplot.plot( new_pos[num], 'ro')
pyplot.plot(pos[num], 'yo')
pyplot.plot( order[num], 'b')
pyplot.plot(fluctuation_points[num], [order[num][i] for i in fluctuation_points[num]], "go")
pyplot.plot(fluctuation_points_density[num])
pyplot.show()