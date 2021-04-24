import os, numpy, math

from matplotlib import pyplot
from typing import List
from vtktool.vtktool import vtk_show, point_actor, line_actor
from shapely.geometry import MultiPoint, LineString
from scipy import interpolate
from build_tools import interpolate_by_stepLen



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


def load_outline(type: str, dis: float, highest:float, rough_para=0.8 ):
    folder_path = "2dcars/"
    car_id = os.listdir(folder_path)
    for i in car_id:
        # if i not in  ["2dcar_09.npy"]:
        #     continue
        if i.endswith("npy"):
            print(i)
            path2d = "2dcars/" + i
            with open(path2d, 'rb') as f_pos:
                car = list(numpy.load(f_pos))
                for point in car:
                    if point[1] + dis > highest:
                        point[1] = highest - dis
                new_car = interpolate_by_stepLen(car, 0.05)
                points = MultiPoint(new_car)
                patch = points.buffer(rough_para)
                patch = patch.buffer(dis - rough_para)
                circle = list(patch.exterior.coords)
                res = [[p[0], p[1]] for p in circle]
                res = cut_xy_spline(res, 0.15)
                xy_head_end_filter(res)
                if type == "long":
                    res_2d = [[i[0] for i in res], [i[1] for i in res]]
                else:
                    res_2d = [[p[0], p[1]] for p in res]
                print(res[0])
                yield res_2d, new_car



if __name__ == '__main__':
    import xpcl
    folder_path = "cars"
    car_id = os.listdir(folder_path)
    rough_para = 0.8
    dis = 0.2
    highest = 2.3
    for i in car_id:
        if i[-4:] == ".npy":
            print(i)
            path = "cars/" + i
            path2d = "2dcars/2d" + i
            # car = numpy.load(path)
            # lines = xpcl.compute_line(car, dis, 0.12, highest)[0]
            # actor = point_actor(lines[2], color=[1, 1, 0])
            #
            car2 = numpy.load(path2d)

            car = numpy.load(path2d)
            # print(car)
            for point in car:
                if point[1] + dis > highest:
                    point[1] = highest - dis
            points = MultiPoint(car)
            patch = points.buffer(rough_para)
            patch = patch.buffer(dis - rough_para)

            circle = list(patch.exterior.coords)
            # print(circle)
            res = [[p[0], p[1], 0] for p in circle]
            res = cut_xy_spline(res, 0.15)
            xy_head_end_filter(res)

            actor2 = point_actor(res, color=[0, 1, 1])
            vtk_show(actor2, car2, color=[1, 0, 0])
