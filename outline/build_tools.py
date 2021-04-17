import math,numpy


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
        dis.append(dis[i - 1] + calculate_dis(outline[i-1], outline[i]))
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
    if calculate_dis(res[-1], outline[-1]) > dis_step/2:
        res.append(outline[-1])
    else:
        res[-1] = outline[-1]
    return res


def interpolate_by_stepNum(outline: list, dis_num: int):
    vector = []
    dis = [0]
    for i in range(1, len(outline)):
        vector.append(get_vector(outline[i-1], outline[i]))
        dis.append(dis[i - 1] + calculate_dis(outline[i-1], outline[i]))
    dis_step = dis[-1]/(dis_num+1)
    j = 0
    res = []
    for i in range(dis_num + 2):
        dis_cur = dis_step * i
        while dis_cur - dis[j + 1] > 1e-6:
            j += 1
        percent = (dis_cur - dis[j]) / (dis[j + 1] - dis[j])
        res.append([outline[j][0] + percent * vector[j][0], outline[j][1] + percent * vector[j][1]])
    return res


def get_vector(point1, point2):
    return [point2[0]-point1[0], point2[1]-point1[1]]


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


def calculate_angle(point1: list, point2: list, smaller_than_0=True) -> float:
    '''smaller_than_0 =True：-180～180；smaller_than_0 =False：0～360'''
    vector =  get_vector(point1,point2)
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
    horline = [0, 1, -line_y]
    line.insert(0, get_2line_intersection(line1, horline))
    line.append(get_2line_intersection(line2, horline))
    print(line[-2],line[-1])
    return line


def get_line_function_with_2points(point1: list, point2: list) -> list:
    if point1 == point2:
        raise ValueError("两点重合")
    else:
        line_sin_theta = (point1[1] - point2[1]) / calculate_dis(point1, point2)
        line_cos_theta = (point1[0] - point2[0]) / calculate_dis(point1, point2)
        res=[line_sin_theta, -line_cos_theta, -line_sin_theta * point1[0] + line_cos_theta * point1[1]]
        if line_sin_theta < 0 or ( line_sin_theta== 0 and -line_cos_theta < 0):
            res = [-i for i in res]
    return res


def get_2line_intersection(line1: list, line2: list):
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


def get_perpendicular_line_function(line, point):
    return [line[1], -line[0], -line[1] * point[0] + line[0] * point[1]]


def unify_list(list):
    res = []
    for i in list:
        res += i
    return res


def round_connect(lines: list, smooth_length=10):
    def get_shape(three_points: list):
        point1, point2, point3 = three_points
        vector1 = get_vector(point2, point1)
        vector2 = get_vector(point2, point3)
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

        pyplot.plot([p[0] for p in curve], [p[1] for p in curve], "-o")
        local_x = [i[0] for i in curve]
        local_y = [i[1] for i in curve]
        connecting_points_id.append(len(x) - 1)
        x += local_x[1:]
        y += local_y[1:]
        new_line += curve[1:]

    connecting_points_id.append(len(x) - 1)
    print(connecting_points_id)
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
    pyplot.plot([new_line[i[0]][0] for i in round_boundary], [new_line[i[0]][1] for i in round_boundary], "*")
    pyplot.show()

    round_boundary = [i[0] for i in round_boundary]

    # 倒圆角
    for key in connecting_points_id[1:-1]:
        # 第一步：直接往拐点key的两边扩smooth_length，得到两个边界点
        left, right = max(0, key - smooth_length), min(key + smooth_length, len(new_line) - 1)
        # 第二步：从拐点key出发，分别往外扩，如果碰到round_boundary中的点则停止，覆盖原边界点得到新的边界点
        for i in range(1, smooth_length + 1):
            if key - i in round_boundary:
                left = key - i
                break
        for i in range(1, smooth_length + 1):
            if key + i in round_boundary:
                right = key + i
                break
        line1 = get_line_function_with_2points(new_line[left], new_line[left + 1])
        line2 = get_line_function_with_2points(new_line[right], new_line[right - 1])

        if abs(line1[0] - line2[0]) < 0.001 and abs(line1[1] - line2[1]) < 0.001:
            if abs(line1[2] - line2[2]) < 0.001:
                # 第三步：比较两个边界点的斜率，如果斜率相等，截距相差很小，则直接当作重合，取过两个边界点的直线抹匀
                print("overlap")
                for i in range(left + 1, right):
                    vector = get_vector(new_line[left], new_line[right])
                    x[i] = vector[0] * i / (right - left) + x[left]
                    y[i] = vector[1] * i / (right - left) + y[left]
                    new_line[i] = [x[i], y[i]]
                continue
            else:
                print("paralla")
                bisector_line = [(line1[i]+line2[i])/2 for i in range(3)] # TODO:平行
        else:
            x0, y0 = get_2line_intersection(line1,line2)
            # 第四步：如果两条切线的交点不在边界内，如果在边界的左侧，则右边界左移，反之则左边界右移
            while x0 <= min(x[left:right+1]) and right > key:
                print("交点在范围左，x0,y0:", x0, y0)
                right += -1
                line2 = get_line_function_with_2points(new_line[right], new_line[right - 1])
                x0, y0 = get_2line_intersection(line1, line2)
                print(f"右边界左移至{right}")
            while x0 >= max(x[left:right+1]) and left < key:
                print("交点在范围右，x0,y0:", x0, y0)
                left += 1
                line1_x0 = x[left]
                line1 = get_line_function_with_2points(new_line[left], new_line[left + 1])
                x0, y0 = get_2line_intersection(line1, line2)
                print(f"右边界左移至{right}")
            print(left,right)
            theta_line1 = calculate_angle(new_line[left], new_line[left + 1])
            theta_line2 = calculate_angle(new_line[right], new_line[right - 1])
            theta_mid = (theta_line1 + theta_line2) / 2
            bisector_line = [math.sin(math.radians(theta_mid)), -math.cos(math.radians(theta_mid)),
                             -math.sin(math.radians(theta_mid))*x0 + math.cos(math.radians(theta_mid))*y0]
            if calculate_dis([x0,y0],new_line[left]) < calculate_dis([x0,y0],new_line[right]):
                line1_perpen = get_perpendicular_line_function(line1, new_line[left])
                center = get_2line_intersection(bisector_line, line1_perpen)
                radius = calculate_dis(center, new_line[left])
                line2_perpen_center = get_perpendicular_line_function(line2, center)
                boundary_point = get_2line_intersection(line2_perpen_center, line2)
                theta_vector1 = calculate_angle(center, new_line[left])
                theta_vector2 = calculate_angle(center, boundary_point)
                if abs(theta_vector2-theta_vector1) >180:
                    print(theta_vector1, theta_vector2)
                    if theta_vector1 >theta_vector2:
                        theta_vector2 += 360
                    else:
                        theta_vector1 += 360
                    print(theta_vector1, theta_vector2)
                theta_circle = list(numpy.linspace(theta_vector1, theta_vector2, (right-left)*5))
                circle = [[center[0]+radius*math.cos(math.radians(i)), center[1]+radius*math.sin(math.radians(i))] for i in theta_circle]
                round_corner = circle + [new_line[right]]

            else:
                line2_perpen = get_perpendicular_line_function(line2, new_line[right])
                center = get_2line_intersection(bisector_line, line2_perpen)
                radius = calculate_dis(center, new_line[right])
                line1_perpen_center = get_perpendicular_line_function(line1, center)
                boundary_point = get_2line_intersection(line1_perpen_center, line1)
                theta_vector1 = calculate_angle(center, boundary_point)
                theta_vector2 = calculate_angle(center, new_line[right])
                if abs(theta_vector2-theta_vector1) >180:
                    print(theta_vector1, theta_vector2)
                    if theta_vector1 > theta_vector2:
                        theta_vector2 += 360
                    else:
                        theta_vector1 += 360
                    print(theta_vector1, theta_vector2)
                theta_circle = list(numpy.linspace(theta_vector1, theta_vector2, (right-left)*5))
                circle = [[center[0]+radius*math.cos(math.radians(i)), center[1]+radius*math.sin(math.radians(i))] for i in theta_circle]
                round_corner = [new_line[left]] + circle
            round_corner = interpolate_by_stepNum(round_corner, right - left - 1)
            new_line[left+1: right] = round_corner[1:-1]

    return new_line
