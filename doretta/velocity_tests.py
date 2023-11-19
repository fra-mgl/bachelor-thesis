from numpy import pi
from doretta.utils.constants import FREQUENCY

R_AMPIO = 1.2
R_STRETTO = 0.8
V_ALTA = 0.3
V_BASSA = 0.1

def velocity_test0():
    # linear velocity, angular velocity
    v = [1 / 20 * pi] * 15 * FREQUENCY
    o = [pi / 15] * 15 * FREQUENCY
    s = "vt0"
    return v, o, s


def velocity_test1():
    # linear velocity, angular velocity
    v = [3 / 32 * pi] * 8 * FREQUENCY
    o = [pi / 8] * 8 * FREQUENCY
    v = v + [3 / 32 * pi] * 8 * FREQUENCY
    o = o + [-pi / 8] * 8 * FREQUENCY
    s = "vt1"
    return v, o, s


def velocity_test2():
    # linear velocity, angular velocity
    v = [3 / 20 * pi] * 5 * FREQUENCY
    o = [pi / 5] * 5 * FREQUENCY
    v = v + [3 / 20 * pi] * 5 * FREQUENCY
    o = o + [-pi / 5] * 5 * FREQUENCY
    s = "vt2"
    return v, o, s


def velocity_test3():
    # linear velocity, angular velocity
    v = [3 / 20 * pi] * 5 * FREQUENCY
    o = [pi / 5] * 5 * FREQUENCY
    v = v + [9 / 20 * pi] * 2 * FREQUENCY
    o = o + [- 3 * pi / 5] * 2 * FREQUENCY
    s = "vt3"
    return v, o, s


def velocity_test4():
    # linear velocity, angular velocity
    t_end = 20
    n_samples = t_end * FREQUENCY
    radius = 0.8
    o = np.linspace(0.1, 1, n_samples)
    v = radius * o
    o = o.tolist()
    v = v.tolist()

    s = "velocity_test4"
    return v, o, s

def velocity_test5A():
    # linear velocity, angular velocity
    r = R_AMPIO
    v_ref = V_BASSA
    o_ref = v_ref / r
    delta_t = pi * r / v_ref
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt5A"
    return v, o, s

def velocity_test5B():
    # linear velocity, angular velocity
    r = R_STRETTO
    v_ref = V_BASSA
    o_ref = v_ref / r
    delta_t = pi * r / v_ref
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt5B"
    return v, o, s

def velocity_test6A():
    # linear velocity, angular velocity
    r = R_AMPIO
    v_ref = V_ALTA
    o_ref = v_ref / r
    delta_t = pi * r / v_ref
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt6A"
    return v, o, s

def velocity_test6B():
    # linear velocity, angular velocity
    r = R_STRETTO
    v_ref = V_ALTA
    o_ref = v_ref / r
    delta_t = pi * r / v_ref
    v = [v_ref] * int(round(delta_t, 0)) * FREQUENCY
    o = [o_ref] * int(round(delta_t, 0)) * FREQUENCY
    s = "vt6B"
    return v, o, s