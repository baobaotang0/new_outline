import math


def takeFirst(elem):
    return elem[0]


def is_postive(num: float):
    if num > 0:
        return 1
    elif num == 0:
        return 0
    else:
        return -1


def interpolate(outline: list, dis_step: float):
    vector = []
    dis = [0]
    for i in range(1, len(outline)):
        vector.append([outline[i][0] - outline[i - 1][0], outline[i][1] - outline[i - 1][1]])
        dis.append(dis[i - 1] + math.sqrt((outline[i][0] - outline[i - 1][0]) ** 2 +
                                          (outline[i][1] - outline[i - 1][1]) ** 2))
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
    if outline[0][1] > line_y:
        compare_bigger = False
    else:
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


def calculate_angle(point1: list, point2: list, smaller_than_0=True) -> float:
    '''smaller_than_0 =True：-180～180；smaller_than_0 =False：0～360'''
    vector = [point2[0] - point1[0], point2[1] - point1[1]]
    if vector[1] < 0:
        if smaller_than_0:
            angle = math.degrees(- math.acos(vector[0] / calculate_dis(point1, point2)))
        else:
            angle = math.degrees(2 * math.pi - math.acos(vector[0] / calculate_dis(point1, point2)))
    else:
        angle = math.degrees(math.acos(vector[0] / calculate_dis(point1, point2)))
    return angle


def is_clockwise(xyz_list: list):
    if xyz_list[-1] == xyz_list[0]:
        xyz_list.pop()  # 因为曲线闭合，头尾点为同一点，所以少取一位
    length = len(xyz_list)
    d = 0
    for i in range(length - 1):
        d += -0.5 * (xyz_list[i + 1][1] + xyz_list[i][1]) * (xyz_list[i + 1][0] - xyz_list[i][0])
    if d < 0:
        clockwise = True
    else:
        clockwise = False
    return clockwise


def cut_outline_with_horline(outline: list, line_y: float):
    outline = rebuild_outline(outline, line_y)
    bigger_part, smaller_part = [], []
    idx = [0]
    if outline[0][1] > line_y:
        compare_bigger = False
    else:
        compare_bigger = True
    for i in range(len(outline)):
        if compare_bigger:
            if outline[i][1] > line_y:
                smaller_part.append(outline[idx[-1]:i])
                idx.append(i)
                compare_bigger = False
        else:
            if outline[i][1] < line_y:
                bigger_part.append(outline[idx[-1]:i])
                idx.append(i)
                compare_bigger = True
        if i == len(outline) - 1:
            if compare_bigger:
                smaller_part.append(outline[idx[-1]:len(outline)])
            else:
                bigger_part.append(outline[idx[-1]:len(outline)])
    return bigger_part, smaller_part


def rebuild_outline(line: list, line_y: float):
    if not is_clockwise(line):
        line.reverse()
    boundary = find_horline_outline_intersection(line, line_y)
    if not boundary:
        res = line
    elif len(boundary) == 1:
        if line[0][0] > boundary[0][1][0] and line[-1][0] > boundary[0][1][0]:
            res = line[boundary[0][0]+1:] + line[:boundary[0][0]+1]
        else:
            res = line
    else:
        def takeSecFirst(elem):
            return elem[1][0]
        boundary.sort(key=takeSecFirst)
        res = line[boundary[0][0]+1:] + line[:boundary[0][0]+1]
    return res


def unify_list(list):
    res = []
    for i in list:
        res += i
    return res


    # if not lines:
    #     res = []
    # elif len(lines) == 1:
    #     res = lines[0]
    # else:
    #     from copy import deepcopy
    #     local_lines = deepcopy(lines)
    #     head = []
    #     while len(local_lines)>1:
    #         dis_list = []
    #         for i in local_lines[1:]:
    #             dis_list.append([calculate_dis(point1=local_lines[0][-1], point2=i[0]), i])
    #         dis_list.sort(key=takeFirst)
    #         if dis_list[0][0] > calculate_dis(point1=local_lines[0][-1], point2=local_lines[-1][-1]):
    #             head.append(local_lines[0])
    #         else:
    #             dis_list[0][1] = local_lines[0] + dis_list[0][1]
    #         local_lines.pop(0)
    #     print("head",head)
    #     print("local",local_lines[0])
    #
    #     res = local_lines[0]
    #     for i in head:
    #         res += i
    #     print(res)
    #     # if direction:
    #     #     res.reverse()