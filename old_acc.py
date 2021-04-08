def Fun(order, amax, vmax, time_step, v0, s0):
    new_pos = [s0]
    new_v = [v0]
    for i in range(len(time_step)):
        time_list, spd_list, run_type = build_math_path(new_v[i], vmax, new_pos[i], order[i], amax, amax)
        pos_step = get_pos_by_ts(time_step[i], time_list, spd_list)[0]
        new_pos.append(new_pos[i] + pos_step)
        new_v.append(get_math_spd_by_ts(time_step[i], time_list, spd_list)[0])
    return new_pos, new_v


def error(new_order, order, amax, vmax, time_step, v0, s0):
    real_pos, real_v = Fun(order=new_order , amax=amax, vmax=vmax, time_step=time_step, v0=v0, s0=s0)
    return sum([(real_pos[i] - order[i])**2 for i in range(len(order))])

def accelerate(order, new_order, rough_range, time_step):
    new_pos = [[order[0][0]], [order[1][0]], [], [order[3][0]], [], []]
    new_v = [[0] for i in range(6)]
    for num in motor:
        start = 0
        for i in range(len(rough_range[num])):
            special_points = rough_range[num][i]
            print(special_points)
            real_pos, real_v = Fun(order=new_order[num][start: special_points[0] + 1], amax=a_value[num],
                                   vmax=vmax_value[num], time_step=time_step[start:special_points[0]],
                                   v0=new_v[num][-1], s0=new_pos[num][-1])
            new_pos[num] = new_pos[num] + real_pos[1:]
            new_v[num] = new_v[num] + real_v[1:]
            print(len(new_pos[num]))
            min1 = fmin(lambda o: error(new_order=o, order=order[num][special_points[0]:special_points[-1]+2],
                                        amax=a_value[num], vmax=vmax_value[num],
                                        time_step=time_step[special_points[0]:special_points[-1]+1],
                                        v0=new_v[num][-1], s0=new_pos[num][-1]),
                                        order[num][special_points[0]:special_points[-1]+1])

            start = special_points[0]
            new_order[num][special_points[0]:special_points[-1]] = min1
            print(min1)
            print(order[num][special_points[0]:special_points[-1]])
        real_pos, real_v = Fun(order=new_order[num][start:], amax=a_value[num],
                               vmax=vmax_value[num], time_step=time_step[start:],
                               v0=new_v[num][-1], s0=new_pos[num][-1])
        new_pos[num] = new_pos[num] + real_pos[1:]
        new_v[num] = new_v[num] + real_v[1:]
    print(len(new_pos[0]))
    return new_pos
