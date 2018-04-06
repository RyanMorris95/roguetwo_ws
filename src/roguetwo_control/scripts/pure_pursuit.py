import numpy as np 
import math 
import matplotlib.pyplot as plt 

k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
kP = 1.0  # speed propotional gain
kI = 0.0  # speed integral gain
kD = 0.0  # speed derivation gain
dt = 0.1  # [s]
L = 2.9  # [m] wheel base of vehicle

global integral_sum, last_error
integral_sum = 0
last_error = 0

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def pid_control(target, current):
	global integral_sum, last_error
	error = target - current
	p = kP * error
	i = integral_sum + kI * error * dt
	d = kD * (error - last_error) / dt

	last_error = error
	integral_sum = i

	return p + i + d


def pure_pursuit_control(state, path_x, path_y, initial_index):
	index = calculate_target_index(state, path_x, path_y)

	if initial_index >= index:
		index = initial_index

	if index < len(path_x):
		target_x = path_x[index]
		target_y = path_y[index]
	else:
		target_x = path_x[-1]
		target_y = path_y[-1]
		index = len(path_x) - 1

	alpha = math.atan2(target_y - state.y, target_x - state.x) - state.yaw

	if state.v < 0:  # going backwards
		alpha = math.pi - alpha

	lookforward = k * state.v + Lfc

	steering_angle = math.atan2(2.0 * L * math.sin(alpha) / lookforward, 1.0)

	return steering_angle, index


def calculate_target_index(state, path_x, path_y):
	# search for nearest point index
	dx = [state.x - x for x in path_x]
	dy = [state.y - y for y in path_y]
	d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]

	# index is the closest distance
	index = d.index(min(d))

	L = 0.0

	lookforward = k * state.v + Lfc

	# search look ahead target 
	while lookforward > L and (index + 1) < len(path_x):
		dx = path_x[index + 1] - path_x[index]
		dy = path_x[index + 1] - path_x[index]
		L += math.sqrt(dx ** 2 + dy ** 2)
		index += 1

	return index