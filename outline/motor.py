from math import sqrt
from bisect import bisect_left, bisect_right
from pickle import load
from accelerate import  get_motor_max_para, calculate_motor_send_interval


def build_math_path(spd_in, spd_tar, pos_in, pos_end, acc, dec):
    pos = pos_end - pos_in
    spd_tar = abs(spd_tar) if spd_in >= 0 else -abs(spd_tar)
    t0 = abs(spd_in / dec)
    t1 = abs((spd_tar - spd_in) / acc)
    t2 = abs(spd_tar / acc)
    t3 = abs(spd_tar / dec)
    t4 = abs(spd_in / acc)
    if spd_in < 0:
        acc = -acc
        dec = -dec
    T_S = (acc + dec) * spd_tar * spd_tar / (2 * acc * dec)  # 高度为spd_tar 三角形面积
    pos_t = T_S - spd_in * t4 / 2
    pos_m = spd_in * t0 / 2
    pos_b = pos_m - T_S  # 三个边界面积
    if spd_tar == 0:
        run_type = '===spd zero==='
        if spd_in * t0 / 2 == pos:
            time_list = [0, t0]
            spd_list = [spd_in, 0]
        else:
            return [None], [None], run_type

    elif abs(spd_tar) >= abs(spd_in) and (pos >= pos_t >= pos_b or pos <= pos_t <= pos_b):
        run_type = '===acc==='
        t_D = (pos - pos_t) / spd_tar
        time_list = [0, t1, t1+t_D, t1+t_D+t3]
        spd_list = [spd_in, spd_tar, spd_tar, 0]
    elif abs(spd_tar) > abs(spd_in) and (pos_t >= pos >= pos_m or pos_t <= pos <= pos_m):
        run_type = '>>>acc<<<'
        spd_D = sqrt((spd_in * t4 / 2 + pos) * 2 * dec * acc / (acc + dec))
        if spd_in < 0:
            spd_D = -spd_D
        time_list = [0, (spd_D - spd_in) / acc, (spd_D - spd_in) / acc + spd_D / dec]
        spd_list = [spd_in, spd_D, 0]
    elif abs(spd_tar) < abs(spd_in) and (pos >= pos_m > pos_b or pos <= pos_m < pos_b):
        run_type = '===dec==='
        t_D = (pos - pos_m) / spd_tar
        time_list = [0, t0 - t3, t0 + t_D - t3, t0 + t_D]
        spd_list = [spd_in, spd_tar, spd_tar, 0]
    elif pos <= pos_b < pos_m or pos >= pos_b > pos_m:
        run_type = '===reverse==='
        t_D = (pos_b - pos) / spd_tar
        time_list = [0, t0, t0+t2, t0+t2+t_D, t0+t2+t_D+t3]
        spd_list = [spd_in, 0, -spd_tar, -spd_tar, 0]
    elif pos_b >= pos >= pos_m or pos_b <= pos <= pos_m:
        run_type = '>>>reverse<<<'
        spd_D = sqrt((pos_m - pos) * 2 * acc * dec / (acc + dec))
        if spd_in < 0:
            spd_D = -spd_D
        time_list = [0, t0, t0 + spd_D / acc, t0 + spd_D / acc + spd_D / dec]
        spd_list = [spd_in, 0, -spd_D, 0]
    else:
        print('something error')
        print(spd_in, spd_tar, pos)
        print(pos, pos_t, pos_m, pos_b)
        return [None], [None], None
    return time_list, spd_list, run_type


def get_math_spd_by_ts(ts_cur, ts_list, spd_list):
    action = True
    if ts_cur >= ts_list[-1]:
        spd_cur = 0
        action = False
        # print('GET_SPD', spd_cur)
    elif ts_cur in ts_list:
        idx = bisect_right(ts_list, ts_cur) - 1
        spd_cur = spd_list[idx]
    else:
        idx_r = bisect_right(ts_list, ts_cur)
        idx_l = idx_r - 1
        ts_percent = (ts_cur - ts_list[idx_l]) / (ts_list[idx_r] - ts_list[idx_l])
        spd_cur = ts_percent * (spd_list[idx_r] - spd_list[idx_l]) + spd_list[idx_l]
    return spd_cur, action


def get_pos_by_ts(ts_cur, ts_list, spd_list):
    action = True
    if ts_cur >= ts_list[-1]:
        ts_cur = ts_list[-1]
        action = False
    idx_r = bisect_left(ts_list, ts_cur)
    idx_l = idx_r - 1
    pos_cur = 0
    t0 = ts_list[0]
    v0 = spd_list[0]
    for k in range(1, idx_r):
        pos_cur += (v0 + spd_list[k]) * (ts_list[k] - t0) / 2
        t0 = ts_list[k]
        v0 = spd_list[k]
    # BUG HERE [THREAD SAFE]
    if ts_cur == 0:
        ts_percent = 0
    else:
        try:
            ts_percent = (ts_cur - ts_list[idx_l]) / (ts_list[idx_r] - ts_list[idx_l])
        except:
            print(f'[CRASH] <bisect_left> ts_list={ts_list}, ts_cur={ts_cur}')
            print(f'[CRASH] idx_right={idx_r}, idx_left={idx_l}, ts_list_len={ts_list}')
    # I WILL WAIT
    spd_cur = ts_percent * (spd_list[idx_r] - spd_list[idx_l]) + spd_list[idx_l]
    pos_end = (v0 + (spd_cur - v0) / 2) * (ts_cur - t0)
    pos_cur += pos_end
    return pos_cur, action





