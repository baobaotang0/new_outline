


def round_corner_helix(left_2point: list, right_2point: list) -> list:
    """使用前需检验重合"""
    line1 = get_line_function_with_2points(left_2point[0], left_2point[1])
    line2 = get_line_function_with_2points(right_2point[0], right_2point[1])
    line_check1 = get_line_function_with_2points(left_2point[0], right_2point[0])
    line_check2 = get_line_function_with_2points(left_2point[1], right_2point[1])
    line1_perpen = get_perpendicular_line_function(line1, left_2point[0])
    line2_perpen = get_perpendicular_line_function(line2, right_2point[0])
    if get_2line_intersection(line_check1, line_check2) and \
        is_positive(left_2point[0][0] - get_2line_intersection(line_check1, line_check2)[0]) *\
        is_positive(left_2point[1][0] - get_2line_intersection(line_check1, line_check2)[0]) < 0 and \
        is_positive(right_2point[0][0] - get_2line_intersection(line_check1, line_check2)[0]) * \
        is_positive(right_2point[1][0] - get_2line_intersection(line_check1, line_check2)[0]) < 0:
        mid_point = [(left_2point[1][0] + right_2point[1][0]) / 2, (left_2point[1][1] + right_2point[1][1]) / 2]
    else:
        theta_line1 = calculate_angle_2points(left_2point[0], left_2point[1])
        theta_line2 = calculate_angle_2points(right_2point[0], right_2point[1])
        bisector_line = get_bisector(get_vector(left_2point[1], left_2point[0]),
                                     get_vector(right_2point[1], right_2point[0]), )
            line1_perpen = get_perpendicular_line_function(line1, new_line[left])
            center = get_2line_intersection(bisector_line, line1_perpen)
            radius = calculate_dis(center, new_line[left])
            line2_perpen_center = get_perpendicular_line_function(line2, center)
            boundary_point = get_2line_intersection(line2_perpen_center, line2)
            theta_vector1 = calculate_angle(center, new_line[left])
            theta_vector2 = calculate_angle(center, boundary_point)
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
            round_corner = circle + [new_line[right]]

        else:
            line2_perpen = get_perpendicular_line_function(line2, new_line[right])
            center = get_2line_intersection(bisector_line, line2_perpen)
            radius = calculate_dis(center, new_line[right])
            line1_perpen_center = get_perpendicular_line_function(line1, center)
            boundary_point = get_2line_intersection(line1_perpen_center, line1)
            theta_vector1 = calculate_angle(center, boundary_point)
            theta_vector2 = calculate_angle(center, new_line[right])
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
            round_corner = [new_line[left]] + circle



    return circle






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

    connecting_points_id = []
    x, y = [lines[0][0][0]], [lines[0][0][1]]
    new_line = [lines[0][0]]
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
        for i in range(this_point, last_point, -1):
            print(left_bds)
            local_shape = get_shape(new_line[i - 1:i + 2])
            if local_shape != "flat" and local_shape != left_bds[1] and left_bds[1] != "flat":
                left_bds = [i, local_shape]
                break
            left_bds = [i , local_shape]

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

    pyplot.plot([new_line[i[0]][0] for i in round_boundary], [new_line[i[0]][1] for i in round_boundary], "*")
    n = numpy.arange(len(new_line))
    for i, txt in enumerate(n):
        pyplot.annotate(txt, (new_line[i][0], new_line[i][1]))
    # pyplot.show()
    print(round_boundary)
    round_boundary = [i[0] for i in round_boundary]
    round_list = []

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
        print(left,right)
        if abs(line1[0] - line2[0]) < 0.0001 and abs(line1[1] - line2[1]) < 0.0001:
            if abs(line1[2] - line2[2]) < 0.0001:
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
                if abs(calculate_angle_2points(new_line[left], new_line[left + 1]) -
                       calculate_angle_2points(new_line[right], new_line[right - 1])) > 0.1:
                    round_curve = round_corner_helix([new_line[left], new_line[left + 1]],
                                                [new_line[right], new_line[right - 1]])
                else:
                    mid_point = [(new_line[left + 1][0]+new_line[right - 1][0])/2,
                                 (new_line[left + 1][1]+new_line[right - 1][1])/2]
                    round_curve1 = round_corner_helix([new_line[left], new_line[left + 1]],
                                               [mid_point, new_line[left + 1]])
                    round_curve2 = round_corner_helix([mid_point, new_line[right - 1]],
                                               [new_line[right], new_line[right - 1]])
                    round_curve = round_curve1+round_curve2
        else:
            x0, y0 = get_2line_intersection(line1, line2)
            judge_line = get_line_function_with_2points(new_line[left], new_line[right])
            # 第四步：如果两条切线的交点不在边界内，如果在边界的左侧，则右边界左移，反之则左边界右移
            while is_positive(judge_line[0] * x0 + judge_line[1] * y0 + judge_line[2]) != \
                    is_positive(judge_line[0] * new_line[key][0] + judge_line[1] * new_line[key][1] + judge_line[2]) \
                    and left < key < right:
                line1 = get_line_function_with_2points(new_line[left], new_line[left + 1])
                line2 = get_line_function_with_2points(new_line[right], new_line[right - 1])
                angle_left = abs(calculate_angle_3points([x0, y0], new_line[left], new_line[key]))
                angle_right = abs(calculate_angle_3points([x0, y0], new_line[right], new_line[key]))
                if angle_left > angle_right:
                    if key < right - 1:
                        right += -1
                        line2 = get_line_function_with_2points(new_line[right], new_line[right - 1])
                        print(f"右边界左移至{right}")
                    elif left + 1 < key:
                        left += 1
                        line1 = get_line_function_with_2points(new_line[left], new_line[left + 1])
                        print(f"左边界右移至{left}")
                    else:
                        break
                else:
                    if left + 1 < key:
                        left += 1
                        line1 = get_line_function_with_2points(new_line[left], new_line[left + 1])
                        print(f"左边界右移至{left}")
                    elif key < right - 1:
                        right += -1
                        line2 = get_line_function_with_2points(new_line[right], new_line[right - 1])
                        print(f"右边界左移至{right}")
                    else:
                        break
                    if get_2line_intersection(line1, line2):
                        x0, y0 = get_2line_intersection(line1, line2)
            print(left, right)
            round_curve = round_corner_helix([new_line[left], new_line[left + 1]],
                                                [new_line[right], new_line[right - 1]])
        round_curve = interpolate_by_stepNum(round_curve, right - left - 1)
        new_line[left: right+1] = round_curve
        round_list.append([left, right, round_curve])

    new_plot(new_line)
    pyplot.axis('equal')
    pyplot.show()
    return new_line