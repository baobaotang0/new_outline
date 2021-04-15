from math import sin, sqrt, cos, radians
from random import random, randint
from pickle import load
import os
from matplotlib import pyplot


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
            check_dict[i] = sqrt(abs(acc) / limit_acc)

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


if __name__ == '__main__':
    from sim import simxyr2plot, xyr2plot

    acc_list = [.5, .3, 60]
    vel_list = [.3, .2, 35]
    for ts, r, xy, outline in car_loader():
        x = [p[0] for p in xy]
        y = [p[1] for p in xy]
        new_t1, new_x = line_filter(ts, x, acc_list[0])
        new_t2, new_y = line_filter(ts, y, acc_list[1])
        new_t3, new_r = line_filter(ts, r, acc_list[2])
        new_t = [max(a, b, c) for a, b, c in zip(new_t1, new_t2, new_t3)]


        t_sum = [0]
        t_sum_cached = 0
        for t in ts:
            t_sum_cached += t
            t_sum.append(t_sum_cached)

        new_t_sum = [0]
        new_t_cached = 0
        for t in new_t:
            new_t_cached += t
            new_t_sum.append(new_t_cached)
        print(sum(new_t), sum(ts))
        # '''命令
        pyplot.figure(figsize=(20, 10))
        pyplot.plot([p[0] for p in outline], [p[1] for p in outline], 'b*')

        pyplot.plot(*xyr2plot(x, y, r), 'y.')
        pyplot.plot(*xyr2plot(new_x, new_y, new_r), 'r.')

        pyplot.show()

        # '''
        # '''模拟
        pyplot.figure(figsize=(20, 10))
        l1 = 0.6471
        pyplot.plot([p[0] for p in outline], [p[1] for p in outline], 'b*')

        pyplot.plot(*simxyr2plot(ts, x, y, r, vel_list, acc_list), 'y')
        pyplot.plot(*simxyr2plot(new_t, new_x, new_y, new_r, vel_list, acc_list), 'r')

        pyplot.show()
        # '''

