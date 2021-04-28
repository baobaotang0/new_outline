from math import sqrt, radians, cos, sin, tan, asin, pi, degrees, atan, acos
from scipy.spatial.distance import pdist


def get_vector_length(p0):
    total = 0
    for i in p0:
        total += i ** 2
    return sqrt(total)


def Euclidean_Distance(p1, p2):
    total = 0
    for v1, v2 in zip(p1, p2):
        total += (v1 - v2) ** 2
    return sqrt(total)


def Manhattan_Distance(p1, p2):
    total = 0
    for a, b in zip(p1, p2):
        total += abs(a - b)
    return total


def get_vector(p1, p2):
    return [b - a for a, b in zip(p1, p2)]


def get_unit_vector(p1, p2):
    return vector2units(get_vector(p1, p2))


def get_unit_vector2(p0, p1, p2):
    v1 = get_vector(p0, p1)
    dis1 = get_vector_length(v1)
    v2 = get_vector(p1, p2)
    dis2 = get_vector_length(v2)

    v1 = [v * dis2 ** 2 for v in v1]
    v2 = [v * dis1 ** 2 for v in v2]

    vector = [_v1 + _v2 for _v1, _v2 in zip(v1, v2)]
    vector = vector2units(vector)
    return vector


def vector2units(vector, unit=1):
    dis = sqrt(sum([v ** 2 for v in vector])) / unit
    res = [v / dis for v in vector]
    return res


def double_check(number):
    return -1e-6 < number < 1e-6


def rot_vectorXY(vector: list, degree):
    rad = radians(degree)
    a = cos(rad)
    b = sin(rad)
    x = vector[0]
    y = vector[1]
    res = vector.copy()
    res[0] = a * x - b * y
    res[1] = a * y + b * x
    return res


def vector2radxy(vector):
    """
    可以认为是 -90 ~ 270 输出弧度
    :param vector:
    :return: -pi/2 ～ 3pi/2
    """
    x = vector[0]
    y = vector[1]
    a = sqrt(x ** 2 + y ** 2)
    rad = asin(y / a)
    if x < 0:
        rad = pi - rad
    return rad


def radxy2vector(rad):
    return [cos(rad), sin(rad), 0]


def round_corner(f5_front_r4: list, smooth_length=10):
    def takeFirst(elem):  # 用于多维列表的排序
        return elem[0]

    r4_motion = [degrees(a) - 90 for a in f5_front_r4].copy()
    x = list(range(len(r4_motion)))
    v_r4 = []
    for i in range(1, len(r4_motion)):
        v_r4.append((r4_motion[i] - r4_motion[i - 1]))

    turning_points = []  # 拐点
    round_boundary = []  # 拐点辐射范围的交界点，边界点

    # 寻找曲率较大的拐点，并判断它是外凸的还内凹的
    for i in range(1, len(r4_motion) - 1):
        cur_dir = [i, None]
        vector1 = (-1, r4_motion[i - 1] - r4_motion[i])
        vector2 = (1, r4_motion[i + 1] - r4_motion[i])
        if vector2[1] + vector1[1] > 0.001:
            cur_dir = [i, "in"]
        elif vector2[1] + vector1[1] < -0.001:
            cur_dir = [i, "out"]
        curvity = degrees(acos(abs(1 - pdist([vector1, vector2], 'cosine')[0])))
        if curvity >= 10:
            turning_points.append(cur_dir)
            continue

    turning_points.sort(key=takeFirst)


    # 有些拐点在其他拐点的辐射范围内，为防止圆角相互覆盖，界定其交界点
    i = 0
    while i < len(turning_points) - 1:
        this_point = turning_points[i]
        next_point = turning_points[i + 1]
        if next_point[0] - this_point[0] < 2 * smooth_length + 1 and this_point[1] == next_point[1]:
            local_shift = []
            for i in range(this_point[0] + 1, next_point[0]):
                vector1 = (-1, r4_motion[i - 1] - r4_motion[i])
                vector2 = (1, r4_motion[i + 1] - r4_motion[i])
                if vector2[1] + vector1[1] > 0.001:
                    cur_dir = "in"
                elif vector2[1] + vector1[1] < -0.001:
                    cur_dir = "out"
                else:
                    cur_dir = None
                if cur_dir and cur_dir != this_point[1]:
                    local_shift.append(i)
            if local_shift:
                local_shift.sort()
                round_boundary += [local_shift[0], local_shift[-1]]
            else:
                turning_points.remove(this_point)
                turning_points.remove(next_point)
                turning_points.insert(i, [int((this_point[0] + next_point[0]) / 2 + 1), this_point[1]])
        i += 1
    for i in range(len(turning_points) - 1):
        this_point = turning_points[i]
        next_point = turning_points[i + 1]
        if next_point[0] - this_point[0] < 2 * smooth_length + 1 and this_point[1] != next_point[1]:
            if next_point[0] - this_point[0] == 1:
                if this_point[0] not in round_boundary:
                    round_boundary.append(this_point[0])
                round_boundary.append(next_point[0])
            else:
                round_boundary.append(int((this_point[0] + next_point[0]) / 2))

    # 列表的第一最后一个点当然也是拐点辐射的交界点
    round_boundary.append(0)
    round_boundary.append(len(r4_motion) - 1)

    # 倒圆角
    round_list = {}
    for p in turning_points:
        # print("new", p)
        key = p[0]
        # 第一步：直接往拐点key的两边扩smooth_length，得到两个边界点
        left, right = max(0, key - smooth_length), min(key + smooth_length, len(r4_motion) - 1)
        line1_x0 = x[left]
        line2_x0 = x[right]
        # 第二步：从拐点key出发，分别往外扩，如果碰到round_boundary中的点则停止，覆盖原边界点得到新的边界点
        for i in range(1, smooth_length + 1):
            if x[key - i] in round_boundary:
                left = key - i
                line1_x0 = x[left]
                break
        for i in range(1, smooth_length + 1):
            if x[key + i] in round_boundary:
                right = key + i
                line2_x0 = x[right]
                break
        line1_k = v_r4[line1_x0]
        line2_k = v_r4[line2_x0 - 1]
        line1_y0 = r4_motion[line1_x0]
        line2_y0 = r4_motion[line2_x0]
        # 第三步：比较两个边界点的斜率，如果斜率相等，截距相差很小，则直接当作重合，取过两个边界点的直线抹匀
        if line1_k == line2_k and abs(line1_y0 - line1_k * line1_x0 - line2_y0 - line2_k * line2_x0) < 0.0001:
            print("overlap")
            for i in x[line1_x0 + 1:line2_x0]:
                line12_k = atan((line2_y0 - line2_y0) / (line2_x0 - line2_x0))
                round_list[i] = line12_k * (i - line1_x0) + line1_y0
            continue
        else:
            # 第四步：如果两个边界点的斜率相等，截距相差比较大，则两侧都向内缩一个点，直到斜率不相等，最多缩到round_boundary，或key的相邻点
            while line1_k == line2_k:
                print("parallel")
                if left not in round_boundary and left <= key - 1:
                    left += 1
                    line1_x0 = x[left]
                    line1_k = v_r4[line1_x0 - 1]
                    line1_y0 = r4_motion[line1_x0]
                if right not in round_boundary and right >= key + 1:
                    right += -1
                    line2_x0 = x[right]
                    line2_k = v_r4[line2_x0]
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
                line1_k = v_r4[line1_x0]
                line1_y0 = r4_motion[line1_x0]
                # print(f"左边界右移至{left}")
            if len(r4_motion) - 1 >= right >= key + 2:
                right += -1
                line2_x0 = x[right]
                line2_k = v_r4[line2_x0 - 1]
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
            line2_k = v_r4[line2_x0]
            line2_y0 = r4_motion[line2_x0]
            x0 = (line1_k * line1_x0 - line2_k * line2_x0 - line1_y0 + line2_y0) / (line1_k - line2_k)
            y0 = line1_k * (x0 - line1_x0) + line1_y0
            # print(f"右边界左移至{right}")
        while x0 >= line2_x0 and left < key:
            # print("交点在范围右，x0,y0:", x0, y0)
            left += 1
            line1_x0 = x[left]
            line1_k = v_r4[line1_x0 - 1]
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


    new_r4_motion = []
    for i in x:
        if i in round_list.keys():
            new_r4_motion.append(round_list[i])
        else:
            new_r4_motion.append(degrees(f5_front_r4[i]) - 90)
    new_r4_motion = [(a + 90) / 180 * pi for a in new_r4_motion]
    return new_r4_motion


def abs_r6_to_rel(abs_r6: list, r4: list):
    r6_rel_list = []
    for r4, r6abs in zip(r4, abs_r6):
        r6rel = -r4 + r6abs + pi
        while r6rel > pi:
            r6rel -= 2 * pi
        while r6rel <= -pi:
            r6rel += 2 * pi
        r6_rel_list.append(r6rel)
    return r6_rel_list


if __name__ == '__main__':
    import random

    for i in range(1000):
        x = []
        a = pi / 2
        for i in range(100):
            a -= pi / 100 * random.random()
            x.append(a)

        y = round_corner(x, 5)
        # pyplot.show()
    # pyplot.plot(y, "r")
    # pyplot.plot(x, "b")
