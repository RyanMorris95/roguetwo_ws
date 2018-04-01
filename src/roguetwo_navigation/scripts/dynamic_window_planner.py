import math
import numpy as np
import matplotlib.pyplot as plt
import time

show_animation = True

max_speed = 0.7  # [m/s]
min_speed = -0.50  # [m/s]
max_yawrate = 45.0 * math.pi / 180.0  # [rad/s]
max_accel = 0.4  # [m/ss]
max_dyawrate = 45.0 * math.pi / 180.0  # [rad/ss]
v_reso = 0.05  # [m/s]
yawrate_reso = 0.5 * math.pi / 180.0  # [rad/s]
dt = 0.1  # [s]
predict_time = 7.0  # [s]
to_goal_cost_gain = 1.25  # allows the car to deviate far around boxes
speed_cost_gain = 1.0
robot_radius = 0.31  # [m]


def motion(x, u, dt):
    # motion model

    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    x[3] = u[0]
    x[4] = u[1]

    return x


def calc_dynamic_window(x):

    # Dynamic window from robot specification
    Vs = [min_speed, max_speed,
          -max_yawrate, max_yawrate]

    # Dynamic window from motion model
    Vd = [x[3] - max_accel * dt,
          x[3] + max_accel * dt,
          x[4] - max_dyawrate * dt,
          x[4] + max_dyawrate * dt]

    #  [vmin,vmax, yawrate min, yawrate max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
          max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

    return dw


def calc_trajectory(xinit, v, y):

    x = np.array(xinit)
    traj = np.array(x)
    time = 0
    while time <= predict_time:
        x = motion(x, [v, y], dt)
        traj = np.vstack((traj, x))
        time += dt

    #  print(len(traj))
    return traj


def calc_final_input(x, u, dw, goal, ob):

    xinit = x[:]
    min_cost = 10000.0
    min_u = u
    min_u[0] = 0.0
    best_traj = np.array([x])
    all_traj = []


    # evalucate all trajectory with sampled input in dynamic window
    for v in np.arange(dw[0], dw[1], v_reso):
        for y in np.arange(dw[2], dw[3], yawrate_reso):
            traj = calc_trajectory(xinit, v, y)
            all_traj.append(traj)
            # calc cost
            to_goal_cost = calc_to_goal_cost(traj, goal)

            speed_cost = speed_cost_gain * \
                (max_speed - traj[-1, 3])
            ob_cost = calc_obstacle_cost(traj, ob)
            #  print(ob_cost)

            final_cost = to_goal_cost + speed_cost + ob_cost

            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                min_u = [v, y]
                best_traj = traj
    #  print(min_u)
    #  input()

    return min_u, best_traj, all_traj


def calc_obstacle_cost(traj, ob):
    # calc obstacle cost inf: collision, 0:free
    skip_n = 2
    minr = float("inf")

    if len(ob) > 1:
        for ii in range(0, len(traj[:, 1]), skip_n):
            for i in range(len(ob[:, 0])):
                ox = ob[i, 0]
                oy = ob[i, 1]
                dx = traj[ii, 0] - ox
                dy = traj[ii, 1] - oy

                r = math.sqrt(dx**2 + dy**2)
                if r <= robot_radius:
                    return float("Inf")  # collisiton

                if minr >= r:
                    minr = r

    return 1.0 / minr  # OK


def calc_to_goal_cost(traj, goal):
    # calc to goal cost. It is 2D norm.

    dy = goal[0] - traj[-1, 0]
    dx = goal[1] - traj[-1, 1]
    goal_dis = math.sqrt(dx**2 + dy**2)
    cost = to_goal_cost_gain * goal_dis

    return cost


def dwa_control(x, u, goal, ob):
    # Dynamic Window control

    start_time = time.time()
    dw = calc_dynamic_window(x)

    start_time = time.time()
    u, traj, all_traj = calc_final_input(x, u, dw, goal, ob)

    return u, traj, all_traj


def plot_arrow(x, y, yaw, length=0.5, width=0.1):
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)


def main():
    print(__file__ + " start!!")
    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([0.0, 0.0, 0, 0.0, 0.0])
    # goal position [x(m), y(m)]
    goal = np.array([10, 0])

    ob = np.matrix([[5, 0],
                    [5, 0.25],
                    [5, -0.25]])

    #ob = np.load('obstacles.npy')

    # fig = plt.show()
    # plt.plot(ob[:, 0], ob[:, 1], "ok")
    # plt.show()

    u = np.array([0.0, 0.0])
    traj = np.array(x)

    print (x)

    for i in range(1000):
        u, ltraj, all_traj = dwa_control(x, u, goal, ob)
        x = motion(x, u, dt)
        traj = np.vstack((traj, x))  # store state history

        if show_animation:
            plt.cla()
            for traj in all_traj:
                plt.plot(traj[:, 0], traj[:, 1], "-r")
            plt.plot(ltraj[:, 0], ltraj[:, 1], "-g")
            plt.plot(x[0], x[1], "xr")
            plt.plot(goal[0], goal[1], "xb")
            plt.plot(ob[:, 0], ob[:, 1], "ok")
            plot_arrow(x[0], x[1], x[2])
            plt.axis("equal")
            plt.xlim(-1, goal[0] + 1)
            plt.ylim(-1, goal[1] + 1)
            plt.grid(True)
            plt.pause(0.0001)

        # check goal
        if math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2) <= robot_radius:
            print("Goal!!")
            break

    print("Done")
    if show_animation:
        plt.plot(traj[:, 0], traj[:, 1], "-r")
        plt.show()


if __name__ == '__main__':
    main()
