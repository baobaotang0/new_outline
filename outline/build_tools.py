import math
from typing import List,Dict

def interpolate(outline: list, dis_step: float):
    vector = []
    dis = [0]
    for i in range(1, len(outline)):
        vector.append([outline[i][0] - outline[i - 1][0], outline[i][1] - outline[i - 1][1]])
        dis.append(dis[i - 1] + math.sqrt((outline[i][0] - outline[i - 1][0]) ** 2 +
                                          (outline[i][1] - outline[i - 1][1]) ** 2))
    # min_dis = min([dis[i+1]-dis[i] for i in range(len(dis)-1)])
    # if  min_dis< dis_step:
    #     import warnings
    #     warn_msg = f"dis_step too short, should bigger than {min_dis}"
    #     warnings.warn(warn_msg)
    count = int(dis[-1] // dis_step)
    j = 0
    res = []
    for i in range(count + 1):
        dis_cur = dis_step * i
        while dis_cur > dis[j + 1]:
            j += 1
        percent = (dis_cur - dis[j]) / (dis[j + 1] - dis[j])
        res.append([outline[j][0] + percent * vector[j][0], outline[j][1] + percent * vector[j][1]])
    return res


def find_horline_circle_intersection(center: list, radius: float, line_y: float):
    return [center[0] + math.sqrt(radius ** 2 - (line_y - center[1]) ** 2),
            center[0] - math.sqrt(radius ** 2 - (line_y - center[1]) ** 2)]


def build_circle(center: list, radius: float, line_num=10) -> list:
    res = []
    step = 2 * math.pi / line_num
    for i in range(line_num + 1):
        res.append([center[0] + radius * math.cos(i * step), center[1] + radius * math.sin(i * step)])
    return res


def calculate_dis(point1: list, point2: list) -> float:
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def find_horline_outline_intersection(outline: list, line_y: float, start_idx=None, end_idx=None, up=False):
    if start_idx is None:
        start_idx = 0
    if end_idx is None:
        end_idx = len(outline) - 1
    idx = []
    compare_bigger = True
    for i in range(start_idx, end_idx + 1):
        if compare_bigger:
            if outline[i][1] > line_y:
                if up:
                    idx.append([i, outline[i]])
                else:
                    idx.append([i - 1, outline[i - 1]])
                compare_bigger = False
        else:
            if outline[i][1] < line_y:
                if up:
                    idx.append([i - 1, outline[i - 1]])
                else:
                    idx.append([i, outline[i]])
                compare_bigger = True
    return idx


def find_2verline_outline_intersection(outline: list, x_range: list, start_idx=None, end_idx=None):
    if start_idx is None:
        start_idx = 0
    if end_idx is None:
        end_idx = len(outline) - 1
    x_range.sort()
    if outline[end_idx][0] < x_range[0] or outline[start_idx][0] > x_range[1]:
        raise ValueError("完全处于范围外")
    idx_left, idx_right = [start_idx, outline[start_idx]], [end_idx, outline[end_idx]]
    for i in range(start_idx, end_idx + 1):
        if outline[i][0] >= x_range[0]:
            idx_left = [i, outline[i]]
            break
    for i in range(end_idx, start_idx - 1, -1):
        if outline[i][0] <= x_range[1]:
            idx_right = [i, outline[i]]
            break
    return [idx_left, idx_right]


