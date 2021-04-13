from scipy.optimize import minimize
import numpy,math

def trangle_s(t1, v0, t, a):
    return v0 * t1 + 1 / 2 * a * t1 ** 2 + (v0 + a * (2 * t1 - t)) * (t - t1) + 1 / 2 * a * (t - t1) ** 2


def error(t1, v0, t, s, a, pos0):
    res = 0
    v_start = [v0]
    pos = [pos0]
    for mid_t, time_step, destination in zip(t1, t, s):
        pos.append(pos[-1] + trangle_s(t1=mid_t, v0=v_start[-1], t=time_step, a=a))
        res += (pos[-1] - destination) ** 2
        v_start.append(v_start[-1] + a * (2 * mid_t - time_step))
    return res


min1 = minimize(fun=lambda t1: abs(error(t1=t1, v0=0.07356466428863073,
                                                   t=[0.1],
                                                   s=[1.471213234277254], pos0=1.4618013019026697,
                                                   a=0.5)), method='Powell',
                x0=numpy.array([0 for i in [0.1]]), bounds=[(0, 0.1)])

print(min1)
min2 = minimize(fun=lambda t1: abs(trangle_s(t1=t1, v0=0.07356466428863073,
                                                   t=0.1,a=0.5)-(1.471213234277254-1.4618013019026697))
                                                   , method='Powell',
                x0=numpy.array([0 for i in [0.1]]), bounds=[(0, 0.1)])
print(min2)
x = error(t1=[0.07018275], v0=0.07356466428863073,
                                                   t=[0.1],
                                                   s=[1.471213234277254], pos0=1.4618013019026697,
                                                   a=0.5)
print(x)
pingfang = (1/2*0.5*0.1**2+0.07356466428863073*0.1 -(1.471213234277254-1.4618013019026697))/0.5
0.1-math.sqrt(pingfang)
print(0.1-math.sqrt(pingfang))
print((1.471213234277254-1.4618013019026697)/0.1)