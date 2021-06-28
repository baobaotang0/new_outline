
origin_down_cut_lef t =[]
for i in range(idx_r4_min_left[0] ,idx_origin_min_left[0]):
    origin_down_cut_left.append(calculate_horline_circle(center=outline[i], radius=l1_length,
                                                         line_y=motor_range[2][0])[0])
origin_down_cut_righ t =[]
for i in range(idx_origin_min_right[0], idx_r4_min_right[0]):
    origin_down_cut_right.append(calculate_horline_circle(center=outline[i], radius=l1_length,
                                                          line_y=motor_range[2][0])[1])




outline_cur.sort(key=takeFirst)

            for i in range(idx_origin_min_left[0], idx_origin_min_right[0]):
                if outline[i][0] >= outline_cur[0][0]:
                    idx_cur_left = [i, outline_cur[0]]
                    break
            for i in range(idx_origin_min_right[0], idx_origin_min_left[0], -1):
                if outline[i][0] <= outline_cur[-1][0]:
                    idx_cur_right = [i, outline_cur[-1]]
                    break

            outline_boundary.append(idx_cur_left)
            outline_boundary.append(idx_cur_right)
            outline_boundary.sort(key=takeFirst)
            print(outline_boundary)

            left_c = build_circle(center=[idx_origin_min_left[1][0] + l1_length, idx_origin_min_left[1][1]],
                                  radius=l1_length, line_num=20)
            right_c = build_circle(center=[idx_origin_min_right[1][0] - l1_length, idx_origin_min_right[1][1]],
                                   radius=l1_length, line_num=20)
            pyplot.figure(figsize=(7, 3))
            pyplot.plot([p[0] for p in outline], [p[1] for p in outline])
            pyplot.plot([p[0] for p in outline], [p[1]-l1_length for p in outline])
            pyplot.plot([p[0] for p in outline_cur], [p[1] for p in outline_cur], "*")
            pyplot.plot([p[0] for p in car], [p[1] for p in car])
            pyplot.plot([p[0] for p in origin_cur], [p[1] for p in origin_cur])
            pyplot.plot([p[0] for p in left_c], [p[1] for p in left_c])
            pyplot.plot([p[0] for p in right_c], [p[1] for p in right_c])
            pyplot.plot([p[1][0] for p in outline_boundary], [p[1][1] for p in outline_boundary], "o")
            pyplot.plot([outline[0][0], outline[-1][0]], [motor_range[2][0], motor_range[2][0]])
            pyplot.xlim(0, 7)
            pyplot.ylim(0, 3)
            pyplot.show()
            #

        else:
            outline_cur = []

for i in range(50):
    if not os.path.exists(f"car_{i}_{bid}.npy"):
        with open(f"car_{i}_{bid}.npy", "wb") as fp:
            pickle.dump((o, commands[bid], self.raw_attr.raw_xy), fp)
        break