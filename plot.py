from matplotlib import pyplot
import math
from math import sqrt
from pickle import load
from offset import get_motor_max_para, calculate_motor_send_interval, is_postive
from motor import build_math_path, get_math_spd_by_ts, get_pos_by_ts
from fuction_solver import fuction_solver

# 读最大加速度，最大速度
motor_max = get_motor_max_para("bot1.json")
vmax_value = [motor_max[num]["vel"] for num in range(1, 7)]
a_value = [motor_max[num]["acc"] for num in range(1, 7)]

# 读位置信息
with open("./data/data_for_zzh", 'rb') as f_pos:
    data = load(f_pos)[1]

# 计算理想轨迹
order = [[] for i in range(6)]
for num in [0, 1, 2, 3,5]:
    order[num] = [data[i][num + 1] for i in range(len(data)-1)]
    order[num] = [data[0][num + 1]] + order[num]

# 计算时间间隔
time_step = [data[i][0] - data[i - 1][0] for i in range(1, len(data))]

# 计算理想平均速度
v_assume = [[] for i in range(6)]
for num in [0, 1, 2, 3, 5]:
    v_assume[num] = [(data[i][num + 1] - data[i-1][num + 1]) / time_step[i] for i in range(1, len(time_step))]
    v_assume[num] = [0] + v_assume[num]


# 计算位移，并寻找启动，反向，停止点
s = [[] for i in range(6)]
switch = [{} for i in range(6)]
for i in range(len(order[0]) - 1):
    for num in [0, 1, 2, 3, 5]:
        s[num].append(order[num][i + 1] - order[num][i])
        if i > 0:
            if is_postive(s[num][i]) != is_postive(s[num][i - 1]):  # and is_postive(s[num][i-1]) != 0
                switch[num][i] = [is_postive(s[num][i - 1]), is_postive(s[num][i])]
print(switch)

# 对启动，反向，停止点进行排序
change_points = [[] for i in range(6)]
for num in [0, 1, 2, 3, 5]:
    l = list(switch[num].keys())
    l.sort()
    change_points[num] = l + [len(order[0])]

# 计算实际位置
pos = [[data[0][num + 1]] for num in [0, 1, 2, 3]]
pos.append([])
pos.append([data[0][5 + 1]])
v = [[0] for i in range(6)]
deviation = [[0] for i in range(6)]
for num in [0, 1, 3]:
    for i in range(1, len(time_step)):
        time_list, spd_list, run_type = build_math_path(v[num][i-1], vmax_value[num], pos[num][i-1], data[i-1][num + 1],
                                                        a_value[num], a_value[num])
        pos_step = get_pos_by_ts(time_step[i], time_list, spd_list)[0]
        pos[num].append(pos[num][i-1]+pos_step)
        deviation[num].append(pos[num][i] - data[i-1][num + 1])
        v[num].append(get_math_spd_by_ts(time_step[i], time_list, spd_list)[0])

# 求每一段的平均速度，与平均偏差
v_avg = [{} for i in range(6)]
deviation_avg = [{} for i in range(6)]
for num in [0, 1, 3]:
    start = 0
    for i in range(len(change_points[num])):
        end = change_points[num][i]
        if len(v[num][start:end]) != 0:
            v_avg[num][start, end] = sum(v_assume[num][start:end])/len(v_assume[num][start:end])
            deviation_avg[num][start, end] = sum(deviation[num][start:end]) / len(deviation[num][start:end])
        start = change_points[num][i]
print(v_avg)
print(deviation_avg)
#
# for num in [0]:
#     total = 0
#     for i in range(change_points[num][0],change_points[num][1]):
#         total += order[num][i]
#         if abs(total)


#
# rough_range = [{} for i in range(6)]
# for num in [0, 1, 2, 3, 5]:
#     start = 0
#     for i in range(len(change_points[num])-1):
#         key = change_points[num][i]
#         next = change_points[num][i+1]
#         t1 = abs(v_avg[num][start, key] / a_value[num])
#         t2 = abs(v_avg[num][key, next] / a_value[num])
#         mid_t = data[key - 1][0]
#         rough_range[num][key] = []
#         for j in range(int(-t1/0.02), int(t2/0.02)+1):
#             if mid_t-t1 <= data[key + j - 1][0] <= mid_t+t2:
#                 rough_range[num][key].append(key + j)
#         start = key
# print(rough_range)
#
# new_order = [[] for i in range(6)]
# for num in [0, 1, 2, 3, 5]:
#     new_order[num] = [data[i][num + 1] for i in range(len(data)-1)]
#     new_order[num] = [data[0][num + 1]] + new_order[num]
# for num in [0]:
#     for i in range(1):  # len(change_points[num]) - 1):
#         key = change_points[num][i]
#         s0 = order[num][key-1]
#         print("s0", s0)
#         print(data[key][0])
#         a, b, c, d = 0, 0, 0, 0
#         for j in range(84, 105): #rough_range[num][key]
#             a += 1
#             b += +3 * data[j][0]
#             c += +3 * data[j][0] ** 2 + 2 * (s0 - order[num][j])/a_value[num]
#             d += data[j][0] ** 3 + 2 * (s0 - order[num][j]) / a_value[num] * data[j][0]
#         print(a, b, c, d)
#         delta_t = - fuction_solver(a, b, c, d)[2].real
#         print("delta_t:", delta_t)
#         print(int(delta_t/0.02-1), rough_range[num][key][-1])
#         for j in range(int(delta_t/0.02-1)-len(rough_range[num][key]), rough_range[num][key][-1]):
#             if data[j][0] <= delta_t <= data[j + 1][0]:
#                 delta_x = j
#                 # delta_t = data[j][0]
#                 print(delta_x, delta_t, data[key][0])
#         # print(data[key][0])
#         print( int(0.19634147366478558/0.5/0.02)+1)
#         for j in range(85, 95): # rough_range[num][key][-1]
#             new_order[num][j] = a_value[num] / 2 * (data[j][0] - delta_t) ** 2 + s0
# #         mid_time = data[key-1][0]
# #
#
# # 对每段路程进行偏移
# pos_modified = [[data[0][num + 1]] for num in [0, 1, 2, 3]]
# pos_modified.append([])
# pos_modified.append([data[0][5 + 1]])
# v_modified = [[0] for i in range(6)]
# offset = 0
# for num in [0, 1, 2, 3, 5]:
#     if num in [3, 5]:
#         gap = 5e-1
#     else:
#         gap = 1e-4
#     for i in range(1, len(time_step)):
#         for rg in deviation_avg[num].keys():
#             if rg[0] < i < rg[1]:
#                 offset = -deviation_avg[num][rg]
#                 break
#         offset = 0
#         time_list, spd_list, run_type = build_math_path(v_modified[num][i-1], vmax_value[num], pos_modified[num][i-1],
#                                                         new_order[num][i] +offset, a_value[num], a_value[num])
#         pos_step = get_pos_by_ts(time_step[i], time_list, spd_list)[0]
#         pos_modified[num].append(pos_modified[num][i-1]+pos_step)
#         v_modified[num].append(get_math_spd_by_ts(time_step[i], time_list, spd_list)[0])
        # if data[i][0] - data[i - 1][0]>0.020000000000000018:
        #     print(i, ":",data[i][0] - data[i - 1][0])
        # if abs(pos[num][i] - data[i-1][num + 1]) > gap:
            # print(i, ":", num, ":", data[i+1][0]-data[i][0], pos_step, data[i+1][num + 1]-data[i][num + 1])
            # print(i, ":", num, ":",data[i][0], ":",  pos[num][i] ,":", data[i-1][num + 1],":", pos[num][i]-data[i-1][num + 1])

    # for i in range(1, len(time_step)):
    #     for rg in change_points[num][1:]:
    #         if i < rg:
    #             offset = -deviation_avg[num][rg]
    #             break
    # v_avg[] a_value[num]
    # v0 = data[83][0] * a_value[num]/2
    # print(v0)
    # for i in range(83-3,83+3):
    #     # t_step = data[i][0] - data[i - 1][0]
    #     t = data[i][0]
    #     b= data[i-1][num + 1]
    #     value += (data[i-1][num + 1] + a_value[num] * data[i][0] ** 2 / 2 - v0 * data[i][0])
    # s0 = value/6


# # 计算实际位置
# pos = [[data[0][num + 1]] for num in [0, 1, 2, 3]]
# pos.append([])
# pos.append([data[0][5 + 1]])
# v = [[0] for i in range(6)]
# deviation = [[0] for i in range(6)]
# for num in [0, 1, 2, 3, 5]:
#     for i in range(1, len(time_step)):
#         time_list, spd_list, run_type = build_math_path(v[num][i-1], vmax_value[num], pos[num][i-1],
#                                                         data[i-1][num + 1], a_value[num], a_value[num])
#         pos_step = get_pos_by_ts(time_step[i], time_list, spd_list)[0]
#         pos[num].append(pos[num][i-1]+pos_step)
#         deviation[num].append(pos[num][i] - data[i-1][num + 1])
#         v[num].append(get_math_spd_by_ts(time_step[i], time_list, spd_list)[0])
#

num = 0
# pyplot.plot([i for i in range(len(time_step))], pos_modified[num], 'ro')
pyplot.plot([i for i in range(len(time_step))], pos[num], 'yo')
pyplot.plot([i for i in range(len(time_step))], order[num][:-1], 'b*')
pyplot.show()