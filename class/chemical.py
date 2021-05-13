import copy
import math

import numpy
from matplotlib import pyplot
from shapely.geometry import MultiPoint

import build_functions
from base import Builder
from math_tools import interpolate_by_stepLen, unify_list, offset, is_positive, calculate_angle_2points, \
    cut_outline_with_horline, rebuild_outline, round_connect, interpolate_by_stepLen_plus, is_in_circle, \
    find_horline_outline_intersection, get_unit_vector, new_plot, calculate_dis


class ChemicalBuilder(Builder):
    car: list
    outline: list
    x_range: list
    idx_down_car: list
    boundary: list
    displaced_car: list
    hard_offset_car: list
    outline_left: list
    origin_left: list
    outline_right: list
    origin_right: list
    chemical_outline: list
    chemical_origin: list
    r4: list
    front_xyzr4: list
    behind_xyzr4: list
    time_step: list
    pump_on_idx = list
    pump_off_idx = list

    def __init__(self, raw_attr):
        super(ChemicalBuilder, self).__init__(raw_attr)
        self.x_range = [None, None]
        self.loose_step_len = {}
        for num in [1, 2, 3, 4, 6]:
            self.loose_step_len[num] = self.raw_attr.command_interval * self.raw_attr.motor_vel[num] * 1

    def Build(self):
        self._check_car_height()
        if not self._find_lowest_range():
            self._displace_downward()
            self._get_front_behind_xyzr4()
            self._get_time_step()
            self._build_new_command()
        return self.bots_commands

    def _check_car_height(self, rough_para=0.8):
        highest = self.raw_attr.motor_range[2][1] + self.raw_attr.l1_length
        self.car = [[p[0], p[1]] for p in self.raw_attr.raw_xy]
        for point in self.car:
            if point[1] + self.raw_attr.shoot_range_xy > highest:
                point[1] = highest - self.raw_attr.shoot_range_xy
        self.car = interpolate_by_stepLen(self.car, 0.05)
        points = MultiPoint(self.car)
        patch = points.buffer(rough_para)
        patch = patch.buffer(self.raw_attr.shoot_range_xy - rough_para)
        circle = list(patch.exterior.coords)
        self.outline = [[p[0], p[1]] for p in circle]
        self.outline = build_functions.cut_xy_spline(self.outline, 0.15)
        build_functions.xy_head_end_filter(self.outline)
        self.outline = [[p[0], p[1]] for p in self.outline]

    def _find_lowest_range(self):
        if self.outline[0][0] > self.raw_attr.motor_range[1][0]:
            self.x_range[0] = self.outline[0][0]
        else:
            self.x_range[0] = self.raw_attr.motor_range[1][0]
        if self.outline[-1][0] < self.raw_attr.motor_range[1][1]:
            self.x_range[1] = self.outline[-1][0]
        else:
            self.x_range[1] = self.raw_attr.motor_range[1][1]
        self.car = rebuild_outline(line=self.car, line_y=self.raw_attr.motor_range[2][0])  # 把数据洗成开口在左的顺时针旋转
        up_car, down_car = cut_outline_with_horline(outline=self.car, line_y=self.raw_attr.motor_range[2][0])
        up_car, down_car = unify_list(up_car), unify_list(down_car)
        if down_car[0][0] > self.x_range[1] or down_car[-1][0] < self.x_range[0]:
            # 检车是否过界
            print("车越界")
            return 1
        self.idx_down_car = [[0, down_car[0], 180], [0, down_car[0], 0]]
        for i in range(len(down_car)):
            cur_r4 = calculate_angle_2points(point1=down_car[i],
                                             point2=[self.x_range[1], self.raw_attr.motor_range[2][0]])

            if cur_r4 > self.idx_down_car[1][2]:
                self.idx_down_car[1] = [i, down_car[i], cur_r4]
            cur_r4 = calculate_angle_2points(point1=down_car[i],
                                             point2=[self.x_range[0], self.raw_attr.motor_range[2][0]])
            if cur_r4 < self.idx_down_car[0][2]:
                self.idx_down_car[0] = [i, down_car[i], cur_r4]
        self.boundary = [[[self.x_range[0], self.raw_attr.motor_range[2][0]], self.idx_down_car[0][2]],
                         [[self.x_range[1], self.raw_attr.motor_range[2][0]], self.idx_down_car[1][2]]]
        if self.boundary[0][1] < 90:
            self.boundary[0][1] = 180
        if self.boundary[1][1] > 90:
            self.boundary[1][1] = 90

    def _displace_downward(self):
        def _get_straightline_outline(origin_bds: list, r4_bds: list):
            step_num = math.ceil(max(abs(origin_bds[1][0] - origin_bds[0][0]) / self.loose_step_len[1],
                                     abs(origin_bds[1][1] - origin_bds[0][1]) / self.loose_step_len[2],
                                     abs(r4_bds[1] - r4_bds[0]) / self.loose_step_len[4]))
            x = list(numpy.linspace(origin_bds[0][0], origin_bds[1][0], step_num))
            y = list(numpy.linspace(origin_bds[0][1], origin_bds[1][1], step_num))
            r4 = list(numpy.linspace(r4_bds[0], r4_bds[1], step_num))
            outline = [[x[i] + math.cos(math.radians(r4[i])) * self.raw_attr.l1_length,
                        y[i] + math.sin(math.radians(r4[i])) * self.raw_attr.l1_length]
                       for i in range(len(r4))]
            origin = [[x[i], y[i]] for i in range(len(x))]
            return outline, origin

        def _exam_boundary(boundary: list, offset_outline: list, origin_line: list, left: bool):
            """检查offset——outline是否在轮廓线内（距离车的距离小于shoot_range_xy），如果是，剪掉越界部分"""
            cnt = len(origin_line)

            def generate_range(front: bool, turnning_point: int, length):
                if front:
                    return [0, turnning_point]
                else:
                    return [turnning_point, length]

            if left:
                idx = 0
                research_para = [1, len(offset_outline), 1]
                compare_sign = 1
                range1 = lambda x: generate_range(front=True, turnning_point=x, length=cnt)
                range2 = lambda x: generate_range(front=False, turnning_point=x, length=cnt)
            else:
                idx = -1
                research_para = [len(offset_outline) - 2, -1, -1]
                compare_sign = -1
                range1 = lambda x: generate_range(front=False, turnning_point=x, length=cnt)
                range2 = lambda x: generate_range(front=True, turnning_point=x, length=cnt)
            if abs(boundary[idx][0][0] - origin_line[idx][0]) < 2 * self.raw_attr.l1_length:  # 离得远可以不用检查
                turnning_point = None
                last_circle_flag = is_in_circle(boundary[idx][0], self.raw_attr.l1_length, offset_outline[idx])
                for i in range(research_para[0], research_para[1], research_para[2]):
                    current_circle_flag = is_in_circle(boundary[idx][0], self.raw_attr.l1_length, offset_outline[i])
                    if last_circle_flag in [1, 0] and current_circle_flag == -1:
                        turnning_point = i
                        # print("circle_intersection", i)
                        break
                    last_circle_flag = current_circle_flag
                if turnning_point:
                    bds_angle = calculate_angle_2points(origin_line[turnning_point], offset_outline[turnning_point])
                    if is_positive(bds_angle - boundary[idx][1]) in [compare_sign, 0]:
                        vector = get_unit_vector(origin_line[turnning_point], offset_outline[turnning_point])
                        vector = [vector[0] * self.raw_attr.l1_length, vector[1] * self.raw_attr.l1_length]
                        offset_outline[idx] = [origin_line[idx][0] + vector[0], origin_line[idx][1] + vector[1]]
                        outline_point = find_horline_outline_intersection(outline=self.outline,
                                                                          line_y=offset_outline[idx][1])
                        if is_positive(offset_outline[idx][0] - outline_point[idx][0]) in [compare_sign, 0]:
                            offset_outline[range1(turnning_point)[0]:range1(turnning_point)[1]] = \
                                [[i[0] + vector[0], i[1] + vector[1]] for i in
                                 origin_line[range1(turnning_point)[0]:range1(turnning_point)[1]]]
                            boundary[idx][1] = bds_angle
                        else:
                            origin_line = origin_line[range2(turnning_point)[0]:range2(turnning_point)[1]]
                            offset_outline = offset_outline[range2(turnning_point)[0]:range2(turnning_point)[1]]
                            boundary[idx][1] = bds_angle
            return boundary, offset_outline, origin_line

        # 将轮廓线向内offset，并取上半部分，得到self.displaced_car
        self.outline = interpolate_by_stepLen(self.outline, self.raw_attr.line_speed * self.raw_attr.command_interval)
        self.displaced_car = offset(original_line=self.outline, radius=self.raw_attr.l1_length, left=False,
                                    is_hard=False)

        cutting_points = find_horline_outline_intersection(outline=self.displaced_car,
                                                           line_y=self.raw_attr.motor_range[2][0],
                                                           start_idx=None, end_idx=None, up=False)

        self.displaced_car = \
            cut_outline_with_horline(outline=self.displaced_car, line_y=self.raw_attr.motor_range[2][0])[0]
        self.displaced_car = unify_list(self.displaced_car)

        if len(self.displaced_car) >= 2:
            self.offset_car = []
            for i in range(len(cutting_points) - 1):
                self.offset_car.append(self.outline[cutting_points[i][0] + 1:cutting_points[i + 1][0]])
            self.offset_car = unify_list(self.offset_car)
            self.boundary, self.offset_car, self.displaced_car = _exam_boundary(boundary=self.boundary,
                                                                                offset_outline=self.offset_car,
                                                                                origin_line=self.displaced_car,
                                                                                left=True)
            self.outline_left, self.origin_left = _get_straightline_outline(
                origin_bds=[self.boundary[0][0], self.displaced_car[0]],
                r4_bds=[self.boundary[0][1],
                        calculate_angle_2points(self.displaced_car[0], self.offset_car[0], False)])
            self.boundary, self.offset_car, self.displaced_car = _exam_boundary(boundary=self.boundary,
                                                                                offset_outline=self.offset_car,
                                                                                origin_line=self.displaced_car,
                                                                                left=False)
            self.outline_right, self.origin_right = _get_straightline_outline(
                origin_bds=[self.displaced_car[-1], self.boundary[-1][0]],
                r4_bds=[calculate_angle_2points(self.displaced_car[-1], self.offset_car[-1]),
                        self.boundary[-1][1]])
            chemical_outline = round_connect([self.outline_left, self.offset_car, self.outline_right])
            chemical_origin = round_connect([self.origin_left, self.displaced_car, self.origin_right])

        else:
            chemical_outline, chemical_origin = _get_straightline_outline(
                origin_bds=[self.boundary[0][0], self.boundary[1][0]],
                r4_bds=[self.boundary[0][1], self.boundary[1][1]])

        self.chemical_outline, self.chemical_origin = interpolate_by_stepLen_plus(chemical_outline,
                                                                                  self.raw_attr.line_speed * self.raw_attr.command_interval,
                                                                                  chemical_origin)
        self.r4 = [calculate_angle_2points(self.chemical_origin[i], self.chemical_outline[i]) for i in
                   range(len(self.chemical_origin))]

        pyplot.plot(self.r4, "r-")

        from scipy.signal import filtfilt, butter
        b, a = butter(3, 0.05)
        self.r4 = filtfilt(b, a, self.r4)

        pyplot.plot(self.r4, "y-")
        self.chemical_outline = [
            [self.chemical_origin[i][0] + self.raw_attr.l1_length * math.cos(math.radians(self.r4[i])),
             self.chemical_origin[i][1] + self.raw_attr.l1_length * math.sin(math.radians(self.r4[i]))] for i in
            range(len(self.chemical_origin))]
        self.chemical_outline, self.chemical_origin, self.r4 = interpolate_by_stepLen_plus(self.chemical_outline,
                                                                                           self.raw_attr.line_speed * self.raw_attr.command_interval,
                                                                                           self.chemical_origin, self.r4)



        new_plot(self.chemical_outline, "-bo")
        new_plot(self.chemical_origin, "-bo")
        # new_plot(chemical_outline, "-r*")
        # new_plot(chemical_origin, "-*r")

        new_plot(self.car)
        # new_plot([[self.chemical_origin[i][0] + self.raw_attr.l1_length * math.cos(math.radians(self.r4[i])),
        #            self.chemical_origin[i][1] + self.raw_attr.l1_length * math.sin(math.radians(self.r4[i]))] for i in
        #           range(len(self.chemical_origin))], "y*")

        # from matplotlib import pyplot
        # pyplot.plot([p[1][0] for p in self.idx_down_car], [p[1][1] for p in self.idx_down_car], "b+")
        new_plot(self.outline, "r--")
        # pyplot.plot(self.raw_attr.motor_range[1], [self.raw_attr.motor_range[2][0], self.raw_attr.motor_range[2][0]])
        # pyplot.plot(self.raw_attr.motor_range[1], [self.raw_attr.motor_range[2][1], self.raw_attr.motor_range[2][1]])
        pyplot.axis('equal')
        pyplot.xlim(0, 7)
        pyplot.ylim(0, 3)
        pyplot.show()

    def _get_front_behind_xyzr4(self):
        cutting_points = [1, len(self.chemical_origin)]
        for i in range(len(self.chemical_outline)):
            if calculate_dis(self.chemical_outline[i], self.chemical_outline[0]) > self.raw_attr.dis_l1l1:
                cutting_points[0] = i
                break
        for i in range(len(self.chemical_outline) - 1, -1, -1):
            if calculate_dis(self.chemical_outline[i], self.chemical_outline[-1]) > self.raw_attr.dis_l1l1:
                cutting_points[1] = i
                break
        extention_len = max(cutting_points[0], len(self.chemical_origin) - 1 - cutting_points[1])

        cut_len = math.ceil(cutting_points[0] / 4)  # 留有1/4 的余量， 剪掉这一部分防撞
        if self.outline[0][0] - self.raw_attr.dis_l1l1 > self.raw_attr.motor_range[1][0]:
            self.front_xyzr4 = [[self.chemical_origin[i][0], self.chemical_origin[i][1],
                                 0,
                                 self.r4[i]] for i in range(len(self.chemical_origin))]
            self.behind_xyzr4 = [[self.chemical_origin[i][0], self.chemical_origin[i][1],
                                  0,
                                  self.r4[i]] for i in range(len(self.chemical_origin))]
            step = self.raw_attr.dis_l1l1 / extention_len
            head = [[self.chemical_origin[0][0] - i * step] + self.behind_xyzr4[0][1:]
                    for i in range(1, extention_len + 1)]
            self.behind_xyzr4 = head + self.behind_xyzr4
            self.pump_on_idx = [0, extention_len]
        else:
            self.front_xyzr4 = [[self.chemical_origin[i][0], self.chemical_origin[i][1],
                                 0,
                                 self.r4[i]] for i in range(cut_len, len(self.chemical_origin))]
            self.behind_xyzr4 = [[self.chemical_origin[i][0], self.chemical_origin[i][1],
                                  0,
                                  self.r4[i]] for i in range(len(self.chemical_origin))]
            head = list(numpy.linspace(-self.raw_attr.motor_range[3][1],
                                       0,
                                       extention_len - cut_len + 1))
            head.pop()
            head = [[self.behind_xyzr4[0][0], self.behind_xyzr4[0][1], i, self.behind_xyzr4[0][3]] for i in head]
            self.behind_xyzr4 = head + self.behind_xyzr4
            self.pump_on_idx = [0, extention_len - cut_len]
        if self.outline[-1][0] + self.raw_attr.dis_l1l1 < self.raw_attr.motor_range[1][1]:
            step = self.raw_attr.dis_l1l1 / extention_len
            tail = [[self.chemical_origin[-1][0] + i * step] + self.front_xyzr4[-1][1:]
                    for i in range(1, extention_len + 1)]
            self.front_xyzr4 = self.front_xyzr4 + tail
            self.pump_off_idx = [len(self.front_xyzr4) - extention_len, len(self.front_xyzr4)]
        else:
            self.behind_xyzr4 = self.behind_xyzr4[:len(self.behind_xyzr4) - cut_len]
            tail = list(numpy.linspace(0,
                                       self.raw_attr.motor_range[3][1],
                                       extention_len - cut_len + 1))
            tail.pop(0)
            tail = [[self.front_xyzr4[-1][0], self.front_xyzr4[-1][1], i, self.front_xyzr4[-1][3]] for i in tail]
            self.front_xyzr4 = self.front_xyzr4 + tail
            self.pump_off_idx = [len(self.front_xyzr4) - extention_len + cut_len, len(self.front_xyzr4)]

    def _get_time_step(self):
        vel_limit = [self.raw_attr.motor_vel[1], self.raw_attr.motor_vel[2],
                     self.raw_attr.motor_vel[3], self.raw_attr.motor_vel[4]]
        time_step = {1: [], 2: []}
        self.time_step = []
        for i in range(len(self.front_xyzr4) - 1):
            t = [abs(self.front_xyzr4[i + 1][j] - self.front_xyzr4[i][j]) / vel_limit[j] for j in range(4)]
            time_step[1].append(max(t))

        for i in range(len(self.behind_xyzr4) - 1):
            t = [abs(self.behind_xyzr4[i + 1][j] - self.behind_xyzr4[i][j]) / vel_limit[j] for j in range(4)]
            time_step[2].append(max(t))
            self.time_step.append(max(time_step[1][i], time_step[2][i], self.raw_attr.command_interval))
        #
        real_x_ft = compute_one(self.time_step, [i[0] for i in self.front_xyzr4], self.raw_attr.motor_vel[1],
                               self.raw_attr.motor_acc[1])
        real_y_ft = compute_one(self.time_step, [i[1] for i in self.front_xyzr4], self.raw_attr.motor_vel[2],
                               self.raw_attr.motor_acc[2])
        real_r_ft = compute_one(self.time_step, [i[3] for i in self.front_xyzr4], self.raw_attr.motor_vel[4],
                               self.raw_attr.motor_acc[4])
        real_o_ft =[[real_x_ft[i] + self.raw_attr.l1_length * math.cos(math.radians(real_r_ft[i])),
                    real_y_ft[i] + self.raw_attr.l1_length * math.sin(math.radians(real_r_ft[i]))] for i in range(len(real_y_ft))]
        new_plot(self.outline)
        new_plot(real_o_ft)

        pyplot.show()
    def _build_new_command(self):
        bots_id = [1, 2]
        commands = {}
        for bid, order in zip(bots_id, [self.front_xyzr4, self.behind_xyzr4]):
            commands[bid] = []
            time_use = copy.deepcopy(self.time_step)
            count = len(order)
            acc_limit = [self.raw_attr.motor_acc[1], self.raw_attr.motor_acc[2],
                         self.raw_attr.motor_acc[3], self.raw_attr.motor_acc[4]]

            time_use.insert(0, 0)  # 补齐少的一位,要在reverse结束后做
            t = []
            o = [[] for i in range(6)]
            for i in range(count):
                t.append(sum(time_use[:i + 1]))
                for j in range(4):
                    o[j].append(order[i][j])

            for i in range(count):
                command = [t[i], o[0][i], o[2][i], o[2][i], o[3][i] - 90, None, 0]
                commands[bid].append(command)

        self.bots_commands = commands


from bisect import bisect_left, bisect_right


def compute_one(cmt_t, cmd1, vmax, acc):
    pos = [cmd1[0]]
    v = [0]
    offset = [0]
    for i in range(1, len(cmt_t) + 1):
        time_list, spd_list, run_type = build_math_path(v[i - 1], vmax, pos[i - 1],
                                                        cmd1[i], acc, acc)
        pos_step = get_pos_by_ts(cmt_t[i - 1], time_list, spd_list)[0]
        pos.append(pos[i - 1] + pos_step)
        offset.append(pos[i] - cmd1[i])
        v.append(get_math_spd_by_ts(cmt_t[i - 1], time_list, spd_list)[0])
    return pos


def build_math_path(spd_in, spd_tar, pos_in, pos_end, acc, dec):
    pos = pos_end - pos_in
    spd_tar = abs(spd_tar) if spd_in >= 0 else -abs(spd_tar)
    t0 = abs(spd_in / dec)
    t1 = abs((spd_tar - spd_in) / acc)
    t2 = abs(spd_tar / acc)
    t3 = abs(spd_tar / dec)
    t4 = abs(spd_in / acc)
    if spd_in < 0:
        acc = -acc
        dec = -dec
    T_S = (acc + dec) * spd_tar * spd_tar / (2 * acc * dec)  # 高度为spd_tar 三角形面积
    pos_t = T_S - spd_in * t4 / 2
    pos_m = spd_in * t0 / 2
    pos_b = pos_m - T_S  # 三个边界面积
    if spd_tar == 0:
        run_type = '===spd zero==='
        if spd_in * t0 / 2 == pos:
            time_list = [0, t0]
            spd_list = [spd_in, 0]
        else:
            return [None], [None], run_type

    elif abs(spd_tar) >= abs(spd_in) and (pos >= pos_t >= pos_b or pos <= pos_t <= pos_b):
        run_type = '===acc==='
        t_D = (pos - pos_t) / spd_tar
        time_list = [0, t1, t1 + t_D, t1 + t_D + t3]
        spd_list = [spd_in, spd_tar, spd_tar, 0]
    elif abs(spd_tar) > abs(spd_in) and (pos_t >= pos >= pos_m or pos_t <= pos <= pos_m):
        run_type = '>>>acc<<<'
        spd_D = math.sqrt((spd_in * t4 / 2 + pos) * 2 * dec * acc / (acc + dec))
        if spd_in < 0:
            spd_D = -spd_D
        time_list = [0, (spd_D - spd_in) / acc, (spd_D - spd_in) / acc + spd_D / dec]
        spd_list = [spd_in, spd_D, 0]
    elif abs(spd_tar) < abs(spd_in) and (pos >= pos_m > pos_b or pos <= pos_m < pos_b):
        run_type = '===dec==='
        t_D = (pos - pos_m) / spd_tar
        time_list = [0, t0 - t3, t0 + t_D - t3, t0 + t_D]
        spd_list = [spd_in, spd_tar, spd_tar, 0]
    elif pos <= pos_b < pos_m or pos >= pos_b > pos_m:
        run_type = '===reverse==='
        t_D = (pos_b - pos) / spd_tar
        time_list = [0, t0, t0 + t2, t0 + t2 + t_D, t0 + t2 + t_D + t3]
        spd_list = [spd_in, 0, -spd_tar, -spd_tar, 0]
    elif pos_b >= pos >= pos_m or pos_b <= pos <= pos_m:
        run_type = '>>>reverse<<<'
        spd_D = math.sqrt((pos_m - pos) * 2 * acc * dec / (acc + dec))
        if spd_in < 0:
            spd_D = -spd_D
        time_list = [0, t0, t0 + spd_D / acc, t0 + spd_D / acc + spd_D / dec]
        spd_list = [spd_in, 0, -spd_D, 0]
    else:
        print('something error')
        print(spd_in, spd_tar, pos)
        print(pos, pos_t, pos_m, pos_b)
        return [None], [None], None
    return time_list, spd_list, run_type


def get_math_spd_by_ts(ts_cur, ts_list, spd_list):
    action = True
    if ts_cur >= ts_list[-1]:
        spd_cur = 0
        action = False
        # print('GET_SPD', spd_cur)
    elif ts_cur in ts_list:
        idx = bisect_right(ts_list, ts_cur) - 1
        spd_cur = spd_list[idx]
    else:
        idx_r = bisect_right(ts_list, ts_cur)
        idx_l = idx_r - 1
        ts_percent = (ts_cur - ts_list[idx_l]) / (ts_list[idx_r] - ts_list[idx_l])
        spd_cur = ts_percent * (spd_list[idx_r] - spd_list[idx_l]) + spd_list[idx_l]
    return spd_cur, action


def get_pos_by_ts(ts_cur, ts_list, spd_list):
    action = True
    if ts_cur >= ts_list[-1]:
        ts_cur = ts_list[-1]
        action = False
    idx_r = bisect_left(ts_list, ts_cur)
    idx_l = idx_r - 1
    pos_cur = 0
    t0 = ts_list[0]
    v0 = spd_list[0]
    for k in range(1, idx_r):
        pos_cur += (v0 + spd_list[k]) * (ts_list[k] - t0) / 2
        t0 = ts_list[k]
        v0 = spd_list[k]
    # BUG HERE [THREAD SAFE]
    if ts_cur == 0:
        ts_percent = 0
    else:
        try:
            ts_percent = (ts_cur - ts_list[idx_l]) / (ts_list[idx_r] - ts_list[idx_l])
        except:
            print(f'[CRASH] <bisect_left> ts_list={ts_list}, ts_cur={ts_cur}')
            print(f'[CRASH] idx_right={idx_r}, idx_left={idx_l}, ts_list_len={ts_list}')
    # I WILL WAIT
    spd_cur = ts_percent * (spd_list[idx_r] - spd_list[idx_l]) + spd_list[idx_l]
    pos_end = (v0 + (spd_cur - v0) / 2) * (ts_cur - t0)
    pos_cur += pos_end
    return pos_cur, action
