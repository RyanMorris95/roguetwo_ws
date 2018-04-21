import sys

import math
import matplotlib.pyplot as plt


k = 0.5  # control gain
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s] time difference
L = 0.31  # [m] Wheel base of vehicle
max_steer = math.radians(30.0)  # [rad] max steering angle

show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, a, delta):
    if delta >= max_steer:
        delta = max_steer
    elif delta <= -max_steer:
        delta = -max_steer

    state.x = state.x + state.v * math.cos(state.yaw) * dt
    state.y = state.y + state.v * math.sin(state.yaw) * dt
    state.yaw = state.yaw + state.v / L * math.tan(delta) * dt
    state.yaw = pi_2_pi(state.yaw)
    state.v = state.v + a * dt

    return state


def stanley_control(state, cx, cy, cyaw, pind):

    # get index of nearest point and find the cross track error
    ind, efa = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    # keep the wheels aligned with the given path
    theta_e = pi_2_pi(cyaw[ind] - state.yaw)

    # adjust the angle to intersect the path tangent
    theta_d = math.atan2(k * efa, state.v)
    delta = theta_e + theta_d

    return delta, ind


def pi_2_pi(angle):
    while (angle > math.pi):
        angle = angle - 2.0 * math.pi

    while (angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle


def calc_target_index(state, cx, cy):

    # calc frant axle position
    fx = state.x + L * math.cos(state.yaw)
    fy = state.y + L * math.sin(state.yaw)

    # search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = [math.sqrt(idx ** 2 + idy ** 2) for (idx, idy) in zip(dx, dy)]
    mind = min(d)
    ind = d.index(mind)

    tyaw = pi_2_pi(math.atan2(fy - cy[ind], fx - cx[ind]) - state.yaw)
    if tyaw > 0.0:
        mind = - mind

    return ind, mind