from pickle import load
import json
from math import sqrt


def get_motor_max_para(json_path: str) -> object:
    motor_max = {}
    with open(json_path, 'r') as f_para:
        para = json.loads(f_para.read())
    for motor in para['motors']:
        motor_max[motor["id"]] = motor["motion"]
    return motor_max


def is_postive(num: float):
    if num > 0:
        return 1
    elif num == 0:
        return 0
    else:
        return -1


def calculate_motor_send_interval(pos_path: str, motor_max: dict):
    with open(pos_path, 'rb') as f_pos:
        pos = load(f_pos)[1]

    v = [0] * 6
    t = [0] * 6
    t_total = []
    s = [[], [], [], [], [], []]
    switch = [{}, {}, {}, {}, {}, {}]
    s_offset = [[], [], [], [], [], []]
    v_reverse_max = [{}, {}, {}, {}, {}, {}]
    vmax_value = [motor_max[num]["vel"] for num in range(1, 7)]
    a_value = [motor_max[num]["acc"] for num in range(1, 7)]

    for i in range(len(pos) - 1):
        for num in [0, 1, 2, 3, 5]:
            s[num].append(pos[i + 1][num + 1] - pos[i][num + 1])
            if i > 0:
                if is_postive(s[num][i]) != is_postive(s[num][i-1]):  # and is_postive(s[num][i-1]) != 0
                    switch[num][i-1] = [is_postive(s[num][i-1]), is_postive(s[num][i])]
    print(switch)
    for num in [0, 1, 2, 3, 5]:
        for k in switch[num].keys():
            v0 = 0
            direction = is_postive(s[num][k])
            a = direction * a_value[num]
            i = 0
            while abs(v0) < vmax_value[num]:
                # print(k-i, ":", num)
                # print("s:", s[num][k - i], "v0:", v0)
                if k-i not in v_reverse_max[num].keys():
                    _v = direction * sqrt(2 * a * s[num][k - i] + v0 ** 2)
                    v_reverse_max[num][k-i] = [_v, v0]
                else:
                    break
                v0 = _v
                i += 1
    # print(v_reverse_max)

    # print([s[i][0] for i in [0, 1, 2, 3, 5]])
    # print(s[0][81:84])

    #
    # for i in range(110): # len(s[0]):
    #     print("****************")
    #     for num in [0, 1, 2, 3, 5]:
    #         if s[num][i] != 0:
    #             direction = is_postive(s[num][i])
    #             a = direction * a_value[num]
    #             vmax = direction * vmax_value[num]
    #             t1 = (vmax - v[num]) / a
    #             s1 = (vmax + v[num]) / 2 * t1
    #             if i in v_reverse_max[num].keys():  #即将停止或反向,末速度减速
    #                 print(i, ":", num, ":即将停止或反向,末速度减速:", v_reverse_max[num][i])
    #                 if abs(v[num]) <= abs(v_reverse_max[num][i][0]) or abs(abs(v[num]) - abs(v_reverse_max[num][i][0])) <= 1e-5:
    #                     v_possible_max = direction * sqrt(2 * a*s[num][i] + v[num]**2)
    #                     if abs(v_possible_max) < abs(v_reverse_max[num][i][1]):
    #                         print("速度太低，不存在停不下来的情况，无需减速")
    #                         vt = direction * sqrt(2 * a * s[num][i] + v[num] ** 2)
    #                         t[num] = ((vt - v[num]) / a)
    #                     else:
    #                         print("为即将停止或反向而减速")
    #                         v1 = direction * sqrt(a*s[num][i] + v[num]**2/2 + v_reverse_max[num][i][1]**2/2)
    #                         if abs(v1) < vmax_value[num]:
    #                             t[num] = (2 * v1 - v[num] - v_reverse_max[num][i][1]) / a
    #                         else:
    #                             t3 = (vmax - v_reverse_max[num][i][1]) / a
    #                             s3 = (vmax + v_reverse_max[num][i][1]) / 2 * t1
    #                             s2 = s[num][i] - s1 - s3
    #                             t2 = s2 / vmax
    #                             t[num] = t1 + t2 + t3
    #                 else:
    #                     print("error:>>>>")
    #             else:  # 方向不变
    #                 print(i, ":", num, ":方向不变, v0:", v[num])
    #                 if abs(s1) > abs(s[num][i]):  # 仅加速
    #                     vt = direction * sqrt(2 * a * s[num][i] + v[num] ** 2)
    #                     t[num] = ((vt - v[num]) / a)
    #                     # v[num] = vt
    #                 else:  # 加速，匀速
    #                     s2 = s[num][i] - s1
    #                     t2 = s2 / vmax
    #                     t[num] = (t1 + t2)
    #                     # v[num] = vmax
    #         else:
    #             t[num] = 0
    #     tmax = max(t + [0.02])
    #     t_total.append(tmax)
    #     print(i, "tmax:", tmax)
    #     for num in [0, 1, 2, 3, 5]:
    #         if s[num][i] != 0:
    #             direction = is_postive(s[num][i])
    #             a = direction * a_value[num]
    #             vmax = direction * vmax_value[num]
    #             t1 = (vmax - v[num]) / a
    #             s1 = (vmax + v[num]) / 2 * t1
    #             t_vmax = vmax / a
    #             s_over = 0
    #             if t_vmax > tmax - t1:
    #                 t3 = tmax - t1
    #                 s3 = vmax * t3 - a * t3 ** 2 / 2
    #             else:
    #                 t3 = t_vmax
    #                 s3 = vmax ** 2 / 2 / a
    #             print("s:", s[num][i])
    #             if abs(s1) > abs(s[num][i]) or s1 + s3 >= s[num][i]:  # 无匀速
    #                 print(num, "无匀速")
    #                 if abs(v[num] ** 2 / 2 / a) > abs(s[num][i]) and v[num] / a < tmax:  # 仅减速，减速不到0就跑完了s, 多跑一段直到速度为0
    #                     print("仅减速,减速到0")
    #                     v[num] = 0
    #                     s_offset[num].append(0)
    #                 else:
    #                     v1 = direction * sqrt(a * s[num][i] + v[num] ** 2 / 2)
    #                     t_move = (2 * v1 - v[num])/a
    #                     if t_move <= tmax or abs(t_move - tmax) < 1e-5:  # 加速，减速到0, 等待
    #                         print(num, "加速，减速到0, 等待")
    #                         print("v:", v[num])
    #                         print("v1:", v1, "t_move:", t_move)
    #                         v[num] = 0
    #                         s_offset[num].append(0)
    #                     else:  # 加速，减速不到0
    #                         print("加速，减速不到0, v:", v[num])
    #                         a_para = -a
    #                         b_para = 2 * a * tmax
    #                         c_para = v[num] * tmax - a * tmax ** 2 / 2 - s[num][i]
    #                         delta = b_para ** 2 - 4 * a_para * c_para
    #                         if abs(delta) < 1e-10:
    #                             delta = 0
    #
    #                         tx = (-b_para + direction * sqrt(delta))/2/a_para
    #                         print("tx:", tx)
    #                         if tx < 0:
    #                             s_over = v[num] * tmax - a * tmax ** 2 / 2
    #                             v[num] = v[num] - a * tmax
    #                             s[num][i+1] -= s_over
    #                             print("最小位移大于s：", s_over)
    #                         else:
    #                             vt = v[num] + a * (2 * tx - tmax)
    #                             v[num] = vt
    #                         print(i, ":", num, ":", v[num])
    #                         s_offset[num].append(v[num] ** 2 / 2 / a)
    #             else:  # 有匀速
    #                 print("有匀速")
    #                 s3 = vmax ** 2 / 2 / a
    #                 s2 = s[num][i] - s1 - s3
    #                 t2 = s2 / vmax
    #                 if t1 + t2 + vmax / a <= tmax:  # 加速，匀速, 减速到0, 等待
    #                     v[num] = 0
    #                     s_offset[num].append(0)
    #                 else:
    #                     print(i, ":", num, ":", v[num])
    #                     t32 = (vmax * tmax - s[num][i] - (v[num]-vmax)**2 / 2 / a) / (0.5 * a)
    #                     t3 = sqrt((s[num][i] + (v[num]-vmax)**2 / 2 / a - vmax * tmax) / (-0.5 * a))
    #                     v[num] = vmax - a * t3
    #                     s_offset[num].append(v[num]**2/2/a)
    #         else:
    #             s_offset[num].append(0)
    #         print(i, ":", num, "offset:", pos[i][num + 1],  pos[i+1][num + 1], pos[i+1][num + 1]+s_offset[num][i])
    return t_total, s_offset, pos


if __name__ == '__main__':
    motor_max = get_motor_max_para("bot1.json")
    t = calculate_motor_send_interval("data_for_zzh", motor_max)
    # old_time = [load(f_pos)[1][i][0] for i in len([])]
