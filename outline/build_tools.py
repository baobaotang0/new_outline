import math

import numpy
from matplotlib import pyplot


def new_plot(l: list, style=None):
    if style:
        pyplot.plot([p[0] for p in l], [p[1] for p in l], style)
    else:
        pyplot.plot([p[0] for p in l], [p[1] for p in l])


def takeFirst(elem):
    return elem[0]


def is_positive(num: float):
    if num > 0:
        return 1
    elif num == 0:
        return 0
    else:
        return -1


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


def is_in_circle(center: list, radius: float, point: list):
    return is_positive(radius - calculate_dis(center, point))


def get_vector(point1, point2):
    return [point2[0] - point1[0], point2[1] - point1[1]]


def get_unit_vector(point1, point2):
    if calculate_dis(point1, point2) == 0:
        return [0, 0]
    else:
        return [get_vector(point1, point2)[0] / calculate_dis(point1, point2),
                get_vector(point1, point2)[1] / calculate_dis(point1, point2)]


def calculate_dis(point1: list, point2: list) -> float:
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)


def calculate_angle_2points(point1: list, point2: list, smaller_than_0=True) -> float:
    '''smaller_than_0 =True：-180～180；smaller_than_0 =False：0～360'''
    vector = get_vector(point1, point2)
    if vector[1] < 0:
        if smaller_than_0:
            angle = math.degrees(- math.acos(vector[0] / calculate_dis(point1, point2)))
        else:
            angle = math.degrees(2 * math.pi - math.acos(vector[0] / calculate_dis(point1, point2)))
    else:
        angle = math.degrees(math.acos(vector[0] / calculate_dis(point1, point2)))
    return angle


def interpolate_by_stepLen(outline: list, dis_step: float):
    vector = []
    dis = [0]
    for i in range(1, len(outline)):
        vector.append(get_vector(outline[i - 1], outline[i]))
        dis.append(dis[i - 1] + calculate_dis(outline[i - 1], outline[i]))
    count = int(dis[-1] // dis_step)
    j = 0
    res = []
    for i in range(count + 1):
        dis_cur = dis_step * i
        if i >= count:
            pass
        while dis_cur - dis[j + 1] > 1e-6:
            j += 1
        percent = (dis_cur - dis[j]) / (dis[j + 1] - dis[j])
        res.append([outline[j][0] + percent * vector[j][0], outline[j][1] + percent * vector[j][1]])
    if calculate_dis(res[-1], outline[-1]) > dis_step / 2:
        res.append(outline[-1])
    else:
        res[-1] = outline[-1]
    return res


def interpolate_by_stepNum(outline: list, dis_num: int):
    vector = []
    dis = [0]
    for i in range(1, len(outline)):
        vector.append(get_vector(outline[i - 1], outline[i]))
        dis.append(dis[i - 1] + calculate_dis(outline[i - 1], outline[i]))
    dis_step = dis[-1] / (dis_num + 1)
    j = 0
    res = []
    for i in range(dis_num + 2):
        dis_cur = dis_step * i

        while dis_cur - dis[j + 1] > 1e-6:
            j += 1
        percent = (dis_cur - dis[j]) / (dis[j + 1] - dis[j])
        res.append([outline[j][0] + percent * vector[j][0], outline[j][1] + percent * vector[j][1]])
    return res


def interpolate_by_stepLen_plus(outline: list, dis_step: float, additional_list: list):
    vector = []
    addition_vector = []
    dis = [0]
    for i in range(1, len(outline)):
        vector.append(get_vector(outline[i - 1], outline[i]))
        addition_vector.append(get_vector(additional_list[i - 1], additional_list[i]))
        dis.append(dis[i - 1] + calculate_dis(outline[i - 1], outline[i]))
    count = int(dis[-1] // dis_step)
    j = 0
    res, addition_res = [], []
    for i in range(count + 1):
        dis_cur = dis_step * i
        while dis_cur - dis[j + 1] > 1e-6:
            j += 1
        percent = (dis_cur - dis[j]) / (dis[j + 1] - dis[j])
        res.append([outline[j][0] + percent * vector[j][0], outline[j][1] + percent * vector[j][1]])
        addition_res.append([additional_list[j][0] + percent * addition_vector[j][0],
                             additional_list[j][1] + percent * addition_vector[j][1]])
    if calculate_dis(res[-1], outline[-1]) > dis_step / 2:
        res.append(outline[-1])
        addition_res.append(additional_list[-1])
    else:
        res[-1] = outline[-1]
        addition_res[-1] = additional_list[-1]
    return res, addition_res


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
            res = line[boundary[0][0] + 1:] + line[:boundary[0][0] + 1]
        else:
            res = line
    else:
        def takeSecFirst(elem):
            return elem[1][0]

        boundary.sort(key=takeSecFirst)
        res = line[boundary[0][0] + 1:] + line[:boundary[0][0] + 1]
    return res


def make_ends(line: list, line_y: float):
    line1 = get_line_function_with_2points(line[0], line[1])
    line2 = get_line_function_with_2points(line[-2], line[-1])
    horline = [0, 1, -line_y]
    line.insert(0, get_2line_intersection(line1, horline))
    line.append(get_2line_intersection(line2, horline))
    return line


def get_line_function_with_2points(point1: list, point2: list) -> list:
    if point1 == point2:
        raise ValueError("两点重合")
    else:
        line_sin_theta = (point1[1] - point2[1]) / calculate_dis(point1, point2)
        line_cos_theta = (point1[0] - point2[0]) / calculate_dis(point1, point2)
        res = [line_sin_theta, -line_cos_theta, -line_sin_theta * point1[0] + line_cos_theta * point1[1]]
        if line_sin_theta < 0 or (line_sin_theta == 0 and -line_cos_theta < 0):
            res = [-i for i in res]
    return res


def get_bisector(vector1: list, vector2: list, point: list) -> list:
    v1 = get_unit_vector([0, 0], vector1)
    v2 = get_unit_vector([0, 0], vector2)
    v = [v1[0] + v2[0], v1[1] + v2[1]]
    point2 = [point[0] + v[0], point[1] + v[1]]
    return get_line_function_with_2points(point2, point)


def get_2line_intersection(line1: list, line2: list):
    if line1[0] == line1[1] == 0:
        print(f"直线1{line1[0]}x+{line1[1]}y+{line1[2]}=0不存在")
        res = None
    elif line2[0] == line2[1] == 0:
        print(f"直线2{line2[0]}x+{line2[1]}y+{line2[2]}=0不存在")
        res = None
    elif line1[1] * line2[0] - line2[1] * line1[0] == 0:
        if line1[1] * line2[2] - line2[1] * line1[2] == 0:
            print("直线重合")
            res = None
        else:
            # print("两直线平行")
            res = []
    else:
        res = [-(line1[1] * line2[2] - line2[1] * line1[2]) / (line1[1] * line2[0] - line2[1] * line1[0]),
               (line1[0] * line2[2] - line2[0] * line1[2]) / (line1[1] * line2[0] - line2[1] * line1[0])]
    return res


def get_perpendicular_line_function(line, point):
    return [line[1], -line[0], -line[1] * point[0] + line[0] * point[1]]


def unify_list(l: list):
    res = []
    for i in l:
        res += i
    return res


def offset(original_line: list, radius: float, left: bool):
    if left:
        rotate = 90
    else:
        rotate = -90
    angle0 = calculate_angle_2points(original_line[0], original_line[1]) + rotate
    vector0 = [math.cos(math.radians(angle0)) * radius, math.sin(math.radians(angle0)) * radius]
    offset_hard = [[original_line[0][0] + vector0[0], original_line[0][1] + vector0[1]]]
    for i in range(1, len(original_line) - 1):
        vector1 = get_unit_vector(original_line[i], original_line[i - 1])
        vector2 = get_unit_vector(original_line[i], original_line[i + 1])
        angle0 = calculate_angle_2points(original_line[i - 1], original_line[i]) + rotate
        vector0 = [math.cos(math.radians(angle0)) * radius, math.sin(math.radians(angle0)) * radius]
        point0 = [original_line[i][0] + vector0[0], original_line[i][1] + vector0[1]]
        bisector_vector = [is_positive(rotate) * (vector1[0] + vector2[0]),
                           is_positive(rotate) * (vector1[1] + vector2[1])]
        if calculate_dis([0, 0], bisector_vector) < 0.0001:
            offset_hard.append(point0)
        else:
            bisector_angle = calculate_angle_2points([0, 0], bisector_vector)
            bisector = [math.sin(math.radians(bisector_angle)), -math.cos(math.radians(bisector_angle)),
                        -math.sin(math.radians(bisector_angle)) * original_line[i][0] +
                        math.cos(math.radians(bisector_angle)) * original_line[i][1]]
            line1 = get_line_function_with_2points(original_line[i], original_line[i - 1])
            para_line1 = [line1[0], line1[1], -line1[0] * point0[0] - line1[1] * point0[1]]
            offset_hard.append(get_2line_intersection(para_line1, bisector))
    angle0 = calculate_angle_2points(original_line[-2], original_line[-1]) + rotate
    vector0 = [math.cos(math.radians(angle0)) * radius, math.sin(math.radians(angle0)) * radius]
    offset_hard.append([original_line[-1][0] + vector0[0], original_line[-1][1] + vector0[1]])

    checklist = {}
    min_point_dis = min([calculate_dis(original_line[i], original_line[i - 1]) for i in range(len(original_line))])
    searching_len = math.ceil(radius * 2 / min_point_dis)

    for i in range(len(offset_hard) - 1):
        this_line = get_line_function_with_2points(offset_hard[i], original_line[i])
        for j in range(i + 1, min(i + searching_len + 1, len(offset_hard))):
            next_line = get_line_function_with_2points(offset_hard[j], original_line[j])
            intersection_point = get_2line_intersection(this_line, next_line)
            if intersection_point and (offset_hard[i][0] - intersection_point[0]) * \
                    (original_line[i][0] - intersection_point[0]) <= 0 and \
                    (offset_hard[j][0] - intersection_point[0]) * \
                    (original_line[j][0] - intersection_point[0]) <= 0:
                checklist[i] = j
    rebuild_list = []
    last_list = []
    for start, end in checklist.items():
        if not last_list:
            last_list = [start, end]
        else:
            if start <= last_list[1]:
                if end > last_list[1]:
                    last_list[1] = end
            else:
                rebuild_list.append(last_list)
                last_list = [start, end]
    if last_list:
        rebuild_list.append(last_list)
        for i in rebuild_list:
            start = max(i[0] - 1, 0)
            end = min(i[1] + 1, len(offset_hard) - 1)
            x = list(numpy.linspace(offset_hard[start][0], offset_hard[end][0], end - start + 1))
            y = list(numpy.linspace(offset_hard[start][1], offset_hard[end][1], end - start + 1))
            offset_hard[start:end + 1] = [[x[i], y[i]] for i in range(len(x))]

    # new_plot(original_line)
    # n = numpy.arange(len(offset_hard))
    # for i, txt in enumerate(n):
    #     pyplot.annotate(txt, (offset_hard[i][0], offset_hard[i][1]))
    #
    # for i in range(len(original_line)):
    #     new_plot([original_line[i], offset_hard[i]])
    #
    # pyplot.show()
    return offset_hard


def get_inscribed_circle(left_2point: list, right_2point: list):
    """使用前需检验重合"""
    line1 = get_line_function_with_2points(left_2point[0], left_2point[1])
    line2 = get_line_function_with_2points(right_2point[0], right_2point[1])
    apex = get_2line_intersection(line1, line2)
    if apex:
        bisector_line = get_bisector(get_vector(left_2point[1], left_2point[0]),
                                     get_vector(right_2point[1], right_2point[0]), apex)
        line1_perpen = get_perpendicular_line_function(line1, left_2point[0])
        line2_perpen = get_perpendicular_line_function(line2, right_2point[0])
        center1 = get_2line_intersection(bisector_line, line1_perpen)
        center2 = get_2line_intersection(bisector_line, line2_perpen)
        if calculate_dis(center1, left_2point[0]) < calculate_dis(center2, right_2point[0]):
            radius = calculate_dis(center1, left_2point[0])
            center = center1
            near_flag = "left"
        else:
            radius = calculate_dis(center2, right_2point[0])
            center = center2
            near_flag = "right"
    elif apex is None:
        radius = 0
        center = None
        near_flag = "overlap"
    else:
        bisector_line = [(line1[i] + line2[i]) / 2 for i in range(3)]
        line1_perpen = get_perpendicular_line_function(line1, left_2point[0])
        line2_perpen = get_perpendicular_line_function(line2, right_2point[0])
        center1 = get_2line_intersection(bisector_line, line1_perpen)
        center2 = get_2line_intersection(bisector_line, line2_perpen)
        if abs(calculate_angle_2points(center1, center2) - calculate_angle_2points(left_2point[0],
                                                                                   left_2point[1])) > 0.001:
            radius = calculate_dis(center1, left_2point[0])
            center = center1
            near_flag = "left"
        else:
            radius = calculate_dis(center2, right_2point[0])
            center = center2
            near_flag = "right"
    return radius, center, near_flag

def build_cicle(center: list, radius: float, lineNum=12):
    theta = numpy.linspace(0, math.pi * 2, lineNum)
    res = []
    for i in theta:
        res.append([center[0] + math.cos(i) * radius, center[1] + math.sin(i) * radius])
    return res


def round_connect(lines: list, smooth_length=10):
    connecting_points_id = []
    x, y = [lines[0][0][0]], [lines[0][0][1]]
    new_line = [lines[0][0]]
    for curve in lines:
        # pyplot.plot([p[0] for p in curve], [p[1] for p in curve], "-o")
        local_x = [i[0] for i in curve]
        local_y = [i[1] for i in curve]
        connecting_points_id.append(len(x) - 1)
        x += local_x[1:]
        y += local_y[1:]
        new_line += curve[1:]
    connecting_points_id.append(len(x) - 1)
    print(connecting_points_id)

    # n = numpy.arange(len(new_line))
    # for i, txt in enumerate(n):
    #     pyplot.annotate(txt, (new_line[i][0], new_line[i][1]))

    for key in connecting_points_id[1:-1]:
        rough_range = [key - 1, key + 1]
        radius, center, near_flag = get_inscribed_circle([new_line[rough_range[0]], new_line[rough_range[0] + 1]],
                                                         [new_line[rough_range[1]], new_line[rough_range[1] - 1]])
        left, right = key - 1, key + 1
        judge_line = get_line_function_with_2points(new_line[left], new_line[right])
        out_left, out_right = max(0, left - 1), min(right + 1, len(new_line) - 1)
        key_flag = is_positive(judge_line[0] * new_line[key][0] + judge_line[1] * new_line[key][1] + judge_line[2])
        left_flag = is_positive(
            judge_line[0] * new_line[out_left][0] + judge_line[1] * new_line[out_left][1] + judge_line[2])
        right_flag = is_positive(
            judge_line[0] * new_line[out_right][0] + judge_line[1] * new_line[out_right][1] + judge_line[2])
        extend_left = True
        while not ((key_flag == 0 and left_flag * right_flag == -1)
                   or key_flag * left_flag == 1 or key_flag * right_flag == 1):
            if extend_left:
                if out_left > 0:
                    if key - left >= smooth_length:
                        break
                    else:
                        left -= 1
                        out_left -= 1
                extend_left = False
            else:
                if out_right < len(new_line) - 1:
                    if right - key >= smooth_length:
                        break
                    else:
                        right += 1
                        out_right += 1
                extend_left = True

            new_radius, new_center, new_near_flag = get_inscribed_circle([new_line[left], new_line[left + 1]],
                                                                         [new_line[right], new_line[right - 1]])
            if new_radius> radius:
                rough_range = [left, right]
                center, radius, near_flag = new_center, new_radius, new_near_flag
            judge_line = get_line_function_with_2points(new_line[left], new_line[right])
            key_flag = is_positive(judge_line[0] * new_line[key][0] + judge_line[1] * new_line[key][1] + judge_line[2])
            left_flag = is_positive(
                judge_line[0] * new_line[out_left][0] + judge_line[1] * new_line[out_left][1] + judge_line[2])
            right_flag = is_positive(
                judge_line[0] * new_line[out_right][0] + judge_line[1] * new_line[out_right][1] + judge_line[2])
        left, right = rough_range
        print(rough_range)
        if near_flag == "left":
            line2 = get_line_function_with_2points(new_line[right], new_line[right - 1])
            line2_perpen_center = get_perpendicular_line_function(line2, center)
            boundary_point = get_2line_intersection(line2_perpen_center, line2)
            theta_vector1 = calculate_angle_2points(center, new_line[left])
            theta_vector2 = calculate_angle_2points(center, boundary_point)
            if abs(theta_vector2 - theta_vector1) > 180:
                print(theta_vector1, theta_vector2)
                if theta_vector1 > theta_vector2:
                    theta_vector2 += 360
                else:
                    theta_vector1 += 360
                print(theta_vector1, theta_vector2)
            theta_circle = list(numpy.linspace(theta_vector1, theta_vector2, (right - left) * 5))
            circle = [
                [center[0] + radius * math.cos(math.radians(i)), center[1] + radius * math.sin(math.radians(i))] for
                i in theta_circle]
            if circle[-1] != new_line[right]:
                round_corner = circle + [new_line[right]]
            else:
                round_corner = circle
        elif near_flag == "right":
            line1 = get_line_function_with_2points(new_line[left], new_line[left + 1])
            line1_perpen_center = get_perpendicular_line_function(line1, center)
            boundary_point = get_2line_intersection(line1_perpen_center, line1)
            theta_vector1 = calculate_angle_2points(center, boundary_point)
            theta_vector2 = calculate_angle_2points(center, new_line[right])
            if abs(theta_vector2 - theta_vector1) > 180:
                print(theta_vector1, theta_vector2)
                if theta_vector1 > theta_vector2:
                    theta_vector2 += 360
                else:
                    theta_vector1 += 360
                print(theta_vector1, theta_vector2)
            theta_circle = list(numpy.linspace(theta_vector1, theta_vector2, (right - left) * 5))
            circle = [
                [center[0] + radius * math.cos(math.radians(i)), center[1] + radius * math.sin(math.radians(i))] for
                i in theta_circle]
            if circle[0] != new_line[left]:
                round_corner = [new_line[left]] + circle
            else:
                round_corner = circle
        else:
            round_corner = [new_line[left] + new_line[right]]
        round_corner = interpolate_by_stepNum(round_corner, right - left - 1)
        new_line[left + 1: right] = round_corner[1:-1]

    return new_line
