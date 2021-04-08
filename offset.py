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
    return switch

if __name__ == '__main__':
    motor_max = get_motor_max_para("bot1.json")
    t = calculate_motor_send_interval("data_for_zzh", motor_max)
    # old_time = [load(f_pos)[1][i][0] for i in len([])]
