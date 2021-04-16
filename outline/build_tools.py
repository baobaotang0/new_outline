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


def interpolate_by_stepLen(outline: list, dis_step: float):
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
    print(line1, line[0])
    print(line2, line[-1])
    horline = [0, 1, -line_y]
    line.insert(0, calculate_2line_intersection(line1, horline))
    line.append(calculate_2line_intersection(line2, horline))
    return line


def get_line_function_with_2points(point1: list, point2: list) -> list:
    if point1 == point2:
        raise ValueError("两点重合")
    else:
        line1_sin_theta = (point1[1] - point2[1]) / calculate_dis(point1, point2)
        line1_cos_theta = (point1[0] - point2[0]) / calculate_dis(point1, point2)
        return [line1_sin_theta, -line1_cos_theta, -line1_sin_theta * point1[0] + line1_cos_theta * point1[1]]


def calculate_2line_intersection(line1: list, line2: list):
    if line1[0] == line1[1] == 0:
        print(f"直线1{line1[0]}x+{line1[1]}y+{line1[2]}=0不存在")
        res = []
    elif line2[0] == line2[1] == 0:
        print(f"直线2{line2[0]}x+{line2[1]}y+{line2[2]}=0不存在")
        res = []
    elif line1[0] / math.sqrt(line1[0] ** 2 + line1[1] ** 2) == line2[0] / math.sqrt(line2[0] ** 2 + line2[1] ** 2):
        print("两直线平行")
        res = []
    else:
        res = [-(line1[1] * line2[2] - line2[1] * line1[2]) / (line1[1] * line2[0] - line2[1] * line1[0]),
               (line1[0] * line2[2] - line2[0] * line1[2]) / (line1[1] * line2[0] - line2[1] * line1[0])]
    return res


def unify_list(list):
    res = []
    for i in list:
        res += i
    return res


def round_connect(lines: list, smooth_length=10):
    def get_shape(three_points: list):
        point1, point2, point3 = three_points
        vector1 = (point1[0] - point2[0], point1[1] - point2[1])
        vector2 = (point3[0] - point2[0], point3[1] - point2[1])
        if vector2[1] + vector1[1] > 0.001:
            res = "in"
        elif vector2[1] + vector1[1] < -0.001:
            res = "out"
        else:
            res = "flat"
        return res

    from matplotlib import pyplot

    connecting_points_id = []
    x, y, new_line = [lines[0][0][0]], [lines[0][0][1]], [lines[0][0]]
    for curve in lines:
        pyplot.plot([p[0] for p in curve], [p[1] for p in curve],"-o")
        local_x = [i[0] for i in curve]
        local_y = [i[1] for i in curve]
        connecting_points_id.append(len(x) - 1)
        if abs(x[-1] - local_x[0]) < 1e-10 and abs(y[-1] - local_y[0]) < 1e-10:
            x += local_x[1:]
            y += local_y[1:]
            new_line += curve[1:]
        else:
            curve.insert(0, [x[-1], y[-1]])
            x += local_x
            y += local_y
            new_line += curve
    connecting_points_id.append(len(x) - 1)

    round_boundary = []
    for p in range(1, len(connecting_points_id) - 1):
        this_point = connecting_points_id[p]
        last_point = connecting_points_id[p - 1]
        next_point = connecting_points_id[p + 1]
        left_bds = [this_point - 1, get_shape(new_line[this_point - 2:this_point + 1])]
        right_bds = [this_point + 1, get_shape(new_line[this_point:this_point + 3])]
        for i in range(this_point - 1, last_point, -1):
            local_shape = get_shape(new_line[i - 1:i + 2])
            if local_shape != "flat" and local_shape != left_bds[1]:
                left_bds = [i, local_shape]
                break
        if left_bds == [this_point - 1, get_shape(new_line[this_point - 2:this_point + 1])]:
            left_bds = [last_point + 1, get_shape(new_line[last_point:last_point + 3])]
        for i in range(this_point + 1, next_point):
            local_shape = get_shape(new_line[i - 1:i + 2])
            if local_shape != "flat" and local_shape != right_bds[1]:
                right_bds = [i, local_shape]
                break
        if right_bds == [this_point + 1, get_shape(new_line[this_point:this_point + 3])]:
            right_bds = [next_point - 1, get_shape(new_line[next_point - 2:next_point + 1])]
        round_boundary.append(left_bds)
        round_boundary.append(right_bds)

    print(round_boundary)
    pyplot.plot([new_line[i[0]][0] for i in round_boundary], [new_line[i[0]][1] for i in round_boundary],"*")
    pyplot.show()

    round_boundary = [i[0] for i in round_boundary]


    # 倒圆角
    round_list = {}
    for key in connecting_points_id[1:-1]:
        # 第一步：直接往拐点key的两边扩smooth_length，得到两个边界点
        left, right = max(0, key - smooth_length), min(key + smooth_length, len(new_line) - 1)
        # 第二步：从拐点key出发，分别往外扩，如果碰到round_boundary中的点则停止，覆盖原边界点得到新的边界点
        for i in range(1, smooth_length + 1):
            if x[key - i] in round_boundary:
                left = key - i
                break
        for i in range(1, smooth_length + 1):
            if x[key + i] in round_boundary:
                right = key + i
                break
        line1 = get_line_function_with_2points(new_line[left], new_line[left+1])
        line2 = get_line_function_with_2points(new_line[right], new_line[right-1])
        # 第三步：比较两个边界点的斜率，如果斜率相等，截距相差很小，则直接当作重合，取过两个边界点的直线抹匀
        while abs(line1[0] - line2[0]) < 0.0001 and abs(line1[1] - line2[1]) < 0.0001:
            print("overlap")
            for i in x[line1_x0 + 1:line2_x0]:
                line12_k = math.atan((line2_y0 - line2_y0) / (line2_x0 - line2_x0))
                round_list[i] = line12_k * (i - line1_x0) + line1_y0
            continue
        else:
            # 第四步：如果两个边界点的斜率相等，截距相差比较大，则两侧都向内缩一个点，直到斜率不相等，最多缩到round_boundary，或key的相邻点
            while line1_k == line2_k:
                print("parallel")
                if left not in round_boundary and left <= key - 1:
                    left += 1
                    line1_x0 = x[left]
                    line1_k = slope[line1_x0 - 1]
                    line1_y0 = r4_motion[line1_x0]
                if right not in round_boundary and right >= key + 1:
                    right += -1
                    line2_x0 = x[right]
                    line2_k = slope[line2_x0]
                    line2_y0 = r4_motion[line2_x0]
            # 第五步：求这两个边界点的相切直线是围成一个内凹还是外凸的形状
            if line1_k > line2_k:
                d = "out"
            elif line1_k < line2_k:
                d = "in"
        # 第六步：如果这两个边界点的相切直线围成的形状和拐点key的形状不相符，则两侧都向内缩一个点，直到形状相同，最多缩到key的相邻点
        while d != p[1] and left <= key - 1 and right >= key + 1:
            # print("方向相反d", d, "p", p)
            if 0 <= left <= key - 2:
                left += 1
                line1_x0 = x[left]
                line1_k = slope[line1_x0]
                line1_y0 = r4_motion[line1_x0]
                # print(f"左边界右移至{left}")
            if len(r4_motion) - 1 >= right >= key + 2:
                right += -1
                line2_x0 = x[right]
                line2_k = slope[line2_x0 - 1]
                line2_y0 = r4_motion[line2_x0]
                # print(f"右边界左移至{right}")
            if line1_k > line2_k:
                d = "out"
            elif line1_k < line2_k:
                d = "in"
        x0 = (line1_k * line1_x0 - line2_k * line2_x0 - line1_y0 + line2_y0) / (line1_k - line2_k)
        y0 = line1_k * (x0 - line1_x0) + line1_y0
        # 第七步：如果两条切线的交点不在边界内，如果在边界的左侧，则右边界左移，反之则左边界右移
        while x0 <= line1_x0 and right > key:
            # print("交点在范围左，x0,y0:", x0, y0)
            right += -1
            line2_x0 = x[right]
            line2_k = slope[line2_x0]
            line2_y0 = r4_motion[line2_x0]
            x0 = (line1_k * line1_x0 - line2_k * line2_x0 - line1_y0 + line2_y0) / (line1_k - line2_k)
            y0 = line1_k * (x0 - line1_x0) + line1_y0
            # print(f"右边界左移至{right}")
        while x0 >= line2_x0 and left < key:
            # print("交点在范围右，x0,y0:", x0, y0)
            left += 1
            line1_x0 = x[left]
            line1_k = slope[line1_x0 - 1]
            line1_y0 = r4_motion[line1_x0]
            x0 = (line1_k * line1_x0 - line2_k * line2_x0 - line1_y0 + line2_y0) / (line1_k - line2_k)
            y0 = line1_k * (x0 - line1_x0) + line1_y0
            # print(f"右边界左移至{right}")

        # print("最终x0,y0:", x0, y0)
        # print(f"{line1_k}(t-{line1_x0})+{line1_y0}")
        # print(f"{line2_k}(t-{line2_x0})+{line2_y0}")
        # pyplot.plot(x[key - smooth_length * 2: key + smooth_length * 2 + 1],
        #             [line1_k * (i - line1_x0) + line1_y0 for i in
        #              x[key - smooth_length * 2: key + smooth_length * 2 + 1]], "--")
        # pyplot.plot(x[key - smooth_length * 2: key + smooth_length * 2 + 1],
        #             [line2_k * (i - line2_x0) + line2_y0 for i in
        #              x[key - smooth_length * 2: key + smooth_length * 2 + 1]], "--")

        # 计算相切圆
        theta_line1 = atan(line1_k)
        theta_line2 = atan(line2_k)
        if theta_line2 < 0:
            theta_line2 = pi + theta_line2
        elif theta_line1 < 0:
            theta_line1 = pi + theta_line1
        theta_mid = (theta_line1 + theta_line2) / 2
        theta_span = min(abs(theta_line2 - theta_mid), pi - abs(theta_line2 - theta_mid))
        radius_max = min(sqrt((x0 - line1_x0) ** 2 + (y0 - line1_y0) ** 2),
                         sqrt((x0 - line2_x0) ** 2 + (y0 - line2_y0) ** 2)) * tan(theta_span)
        radius = radius_max
        center_len = radius / sin(theta_span)
        boundary = [x0 - abs(radius / tan(theta_span) * cos(theta_line1)),
                    x0 + abs(radius / tan(theta_span) * cos(theta_line2))]
        # print("boundary:", boundary)
        # 如果点在圆角范围内则修成圆上的点，否则沿切线
        for i in x[line1_x0 + 1:line2_x0]:
            if i < boundary[0]:
                round_list[i] = line1_k * (i - line1_x0) + line1_y0
            elif i <= boundary[1]:
                if p[1] == "out":
                    circle_center = [x0 - cos(theta_mid) * center_len, y0 - sin(theta_mid) * center_len]
                    # print(circle_center)
                    round_list[i] = circle_center[1] + sqrt(radius ** 2 - (i - circle_center[0]) ** 2)
                else:
                    circle_center = [x0 + cos(theta_mid) * center_len, y0 + sin(theta_mid) * center_len]
                    # print(circle_center)
                    round_list[i] = circle_center[1] - sqrt(radius ** 2 - (i - circle_center[0]) ** 2)
            else:
                round_list[i] = line2_k * (i - line2_x0) + line2_y0

    pyplot.plot(round_list.keys(), round_list.values(), 'bo')

    new_r4_motion = []
    for i in x:
        if i in round_list.keys():
            new_r4_motion.append(round_list[i])
        else:
            new_r4_motion.append(degrees(f5_front_r4[i]) - 90)
    pyplot.plot(x, new_r4_motion, "-")
    pyplot.plot(x, [degrees(a) - 90 for a in f5_front_r4])
    new_r4_motion = [(a + 90) / 180 * pi for a in new_r4_motion]
    return new_r4_motion
