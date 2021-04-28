import math
from typing import List

from build_tools import *


def line2vectors(off_xy: list):
    """
    计算折线上端点的的方向向量
    :param off_xy:
    :return:
    """
    length = len(off_xy)
    vectors = [[0, 0, 0] for i in range(length)]
    for i in range(1, length - 1):
        pre = off_xy[i - 1]
        cur = off_xy[i]
        fut = off_xy[i + 1]
        vector = get_unit_vector2(pre, cur, fut)
        vectors[i] = vector
    # first
    vectors[0] = get_unit_vector(off_xy[0], off_xy[1])
    # last
    vectors[length - 1] = get_unit_vector(off_xy[length - 2], off_xy[length - 1])
    return vectors


def vectors_rot_angle(vectors, angle):
    """
    :param vectors: 输入单位向量!
    :param angle: 角度degree
    :return:
    """
    normal_with_angle = []
    for vector in vectors:
        r_vector = rot_vectorXY(vector, angle)
        normal_with_angle.append(r_vector)
    return normal_with_angle


def line2distancelist(off_xy: list) -> list:
    length = len(off_xy)
    distance_list = [0]
    for i in range(1, length):
        vector = [a - b for a, b in zip(off_xy[i - 1], off_xy[i])]
        dis = get_vector_length(vector)
        distance_list.append(dis)
    return distance_list


def search_distance_list(value, distance_list):
    idx = 0
    for dis in distance_list:
        value -= dis
        if value < 0:
            return idx
        idx += 1
    return idx


def number_interpolation(n1, n2, percent):
    return n1 + (n2 - n1) * percent


def point_interpolation(p1, p2, percent):
    return [a + (b - a) * percent for a, b in zip(p1, p2)]


def list2D_sum(list2d, unit_vector=False):
    res = [0, 0, 0]
    for vector in list2d:
        for i in range(3):
            res[i] += vector[i]
    if unit_vector:
        res = vector2units(res)
    return res


def list2D_add(list1: list, list2: list, *, ratio1=1, ratio2=1):
    return [[a * ratio1 + b * ratio2 for a, b in zip(i, j)] for i, j in zip(list1, list2)]


def list2D_sub(list1: list, list2: list, *, ratio1=1, ratio2=1):
    return [[a * ratio1 - b * ratio2 for a, b in zip(i, j)] for i, j in zip(list1, list2)]


def get_z_on_x_from_path_xz(x_from_xy, path_xz, negative=False):
    len_xz = len(path_xz)
    min_index = 0
    max_index = len_xz - 1
    negative = -1 if negative else 1

    while max_index - min_index > 1:
        mid_index = (max_index + min_index) // 2
        if x_from_xy * negative > path_xz[mid_index][0] * negative:
            min_index = mid_index
        else:
            max_index = mid_index

    x1 = path_xz[min_index][0]
    x2 = path_xz[max_index][0]
    z1 = path_xz[min_index][2]
    z2 = path_xz[max_index][2]
    if abs(x1 - x2) < 1e-3:
        z_res = (z1 + z2) / 2
    else:
        z_res = z1 + (z2 - z1) * (x_from_xy - x1) / (x2 - x1)
    return z_res


def _get_angle(x0, y0, x1, y1):
    # 范围为0~2pi
    dx = x1 - x0
    dy = y1 - y0
    if dx == 0 and dy == 0:
        print("\033[1;31;m", "WARNING, compare same points", f"{x0}, {y0}")
        return None
    if dx == 0:
        return math.pi / 2 if dy > 0 else math.pi * 3 / 2
    if dy == 0:
        return 0 if dx > 0 else math.pi
    angle = math.atan(dy / dx)
    if dx < 0:
        angle += math.pi
    elif dx > 0 and dy < 0:
        angle += 2 * math.pi
    return angle


def cut_xy_spline(xyz_list: list, lowest, cut_angle=45):
    y_limit = lowest
    xyz_list.pop()  # 因为曲线闭合，头尾点为同一点，所以少取一位
    length = len(xyz_list)
    # Green formula
    d = 0
    for i in range(length - 1):
        d += -0.5 * (xyz_list[i + 1][1] + xyz_list[i][1]) * (xyz_list[i + 1][0] - xyz_list[i][0])
    if d > 0:
        counter_clockwise = True
    else:
        counter_clockwise = False
    # choose delete point
    delete_indices = []
    y_min_max = [xyz_list[0][1], xyz_list[0][1]]
    for i in range(length):
        if xyz_list[i][1] < y_min_max[0]:
            y_min_max[0] = xyz_list[i][1]
        elif xyz_list[i][1] > y_min_max[1]:
            y_min_max[1] = xyz_list[i][1]
        if xyz_list[i][1] < y_limit:
            delete_indices.append(i)
            continue
        if cut_angle is None or cut_angle == 0:
            continue
        if i != length - 1:
            angle = _get_angle(xyz_list[i][0], xyz_list[i][1], xyz_list[i + 1][0], xyz_list[i + 1][1])
        else:
            angle = _get_angle(xyz_list[i][0], xyz_list[i][1], xyz_list[0][0], xyz_list[0][1])
        if counter_clockwise:
            if math.cos(angle) > math.cos(math.radians(cut_angle)):
                delete_indices.append(i)
        else:
            if math.cos(angle) < -math.cos(math.radians(cut_angle)):
                delete_indices.append(i)
    if len(delete_indices) == 0:
        print('cut spline的裁剪高度参数错误！')
        return []
    # compute result
    del_min = delete_indices[-1]
    del_max = delete_indices[0]
    save_len = delete_indices[0] + length - delete_indices[-1]
    idx_pre = delete_indices[0]
    for idx in delete_indices:
        if idx - idx_pre > save_len:
            del_min = idx_pre
            del_max = idx
        idx_pre = idx
    # resort
    if del_max > del_min:
        result = xyz_list[del_min: del_max]  # BUG HERE WITH PEOPLE
    else:
        result = xyz_list[del_min:] + xyz_list[:del_max]
    if counter_clockwise:
        result.reverse()
    # insert first and last
    if result[0][1] > y_limit:
        point = result[0].copy()
        point[0] = result[0][0] - (result[0][0] - result[1][0]) / (result[0][1] - result[1][1]) * (
                result[0][1] - y_limit) * 0.8
        point[1] = y_limit
        result.insert(0, point)
    if result[-1][1] > y_limit:
        point = result[-1].copy()
        point[0] = result[-1][0] - (result[-1][0] - result[-2][0]) / (result[-1][1] - result[-2][1]) * (
                result[-1][1] - y_limit) * 0.8
        point[1] = y_limit
        result.append(point)
    return result


def xy_head_end_filter(xyz_list: List[List[float]]):
    """
    请在cut_xy_spline 后使用
    """
    length = len(xyz_list)
    max_idx = length - 1
    min_idx = 0
    for idx in range(length):
        x = xyz_list[idx][0]
        if x > xyz_list[max_idx][0]:
            max_idx = idx
        elif x < xyz_list[min_idx][0]:
            min_idx = idx
    del idx, x
    for i in range(min_idx):
        xyz_list[i][0] = xyz_list[min_idx][0]
    for i in range(max_idx + 1, length):
        xyz_list[i][0] = xyz_list[max_idx][0]


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
