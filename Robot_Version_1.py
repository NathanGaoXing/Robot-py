"""
    Robot simulation

    @author: NathanGaoXing https://github.com/NathanGaoXing
    Nat cuckoo studio GuGuKid

    28/10/2021

    @Version 1.0.2
"""
"""
    numpy-1.21.3
    cycler-0.10.0 
    kiwisolver-1.3.2 
    matplotlib-3.4.3 
    pillow-8.4.0 
    pyparsing-3.0.3 
    python-dateutil-2.8.2 
    six-1.16.0
"""

import numpy as np
import matplotlib.pyplot as plt
import math

# Robot parameters
LENGTH = 0.4  # [m]
WIDTH = 0.5  # [m]
BACKTOWHEEL = 0.0  # [m]
WHEEL_LEN = 0.15  # [m]
WHEEL_WIDTH = 0.1  # [m]
TREAD = 0.35  # [m]
WB = 0.4  # [m]

# Obstacle parameters
LENGTH_obs = 1.2  # [m]
WIDTH_obs = 0.8  # [m]

# Wall parameters
LENGTH_wall = 0.2  # [m]
WIDTH_wall = 11  # [m]

# time
dt = 0.2


# robot state
class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


# odometry state
class Odometry:

    def __init__(self, l_odo=0.0, r_odo=0.0):
        self.l_odo = l_odo
        self.r_odo = r_odo


# draw robot
def plot_robot(x, y, yaw, steer=0.0, truckcolor="k-"):
    outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                        [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

    fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                         [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD,
                          -WHEEL_WIDTH - TREAD]])

    rr_wheel = np.copy(fr_wheel)

    fl_wheel = np.copy(fr_wheel)
    fl_wheel[1, :] *= -1
    rl_wheel = np.copy(rr_wheel)
    rl_wheel[1, :] *= -1

    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    Rot2 = np.array([[math.cos(steer), math.sin(steer)],
                     [-math.sin(steer), math.cos(steer)]])

    fr_wheel = (fr_wheel.T.dot(Rot2)).T
    fl_wheel = (fl_wheel.T.dot(Rot2)).T
    fr_wheel[0, :] += WB
    fl_wheel[0, :] += WB

    fr_wheel = (fr_wheel.T.dot(Rot1)).T
    fl_wheel = (fl_wheel.T.dot(Rot1)).T

    outline = (outline.T.dot(Rot1)).T
    rr_wheel = (rr_wheel.T.dot(Rot1)).T
    rl_wheel = (rl_wheel.T.dot(Rot1)).T

    outline[0, :] += x
    outline[1, :] += y
    fr_wheel[0, :] += x
    fr_wheel[1, :] += y
    rr_wheel[0, :] += x
    rr_wheel[1, :] += y
    fl_wheel[0, :] += x
    fl_wheel[1, :] += y
    rl_wheel[0, :] += x
    rl_wheel[1, :] += y

    # body
    plt.plot(np.array(outline[0, :]).flatten() - (WB / 2) * math.cos(yaw),
             np.array(outline[1, :]).flatten() - (WB / 2) * math.sin(yaw), truckcolor)
    # wheels
    plt.plot(np.array(fr_wheel[0, :]).flatten() - (WB / 2) * math.cos(yaw),
             np.array(fr_wheel[1, :]).flatten() - (WB / 2) * math.sin(yaw), truckcolor)
    plt.plot(np.array(rr_wheel[0, :]).flatten() - (WB / 2) * math.cos(yaw),
             np.array(rr_wheel[1, :]).flatten() - (WB / 2) * math.sin(yaw), truckcolor)
    plt.plot(np.array(fl_wheel[0, :]).flatten() - (WB / 2) * math.cos(yaw),
             np.array(fl_wheel[1, :]).flatten() - (WB / 2) * math.sin(yaw), truckcolor)
    plt.plot(np.array(rl_wheel[0, :]).flatten() - (WB / 2) * math.cos(yaw),
             np.array(rl_wheel[1, :]).flatten() - (WB / 2) * math.sin(yaw), truckcolor)

    # COM
    plt.plot(x, y, "b+")

    # IR
    # plt.plot([x + 0.3 * math.cos(yaw + math.pi / 4), x + 1 * math.cos(yaw + math.pi / 4)], [y + 0.3 * math.sin(yaw + math.pi / 4), y + 1 * math.sin(yaw + math.pi / 4)], "r--")
    # plt.plot([x + 0.3 * math.cos(yaw - math.pi / 4), x + 1 * math.cos(yaw - math.pi / 4)], [y + 0.3 * math.sin(yaw - math.pi / 4), y + 1 * math.sin(yaw - math.pi / 4)], "r--")


def plot_playground():
    # Playground
    plt.plot(0, 0, "b.")
    plt.plot([-3, 3], [-1, -1], "k-")
    plt.plot([-3, 3], [10, 10], "k-")

    plt.plot([-3, -3], [-1, 10], "k-")
    plt.plot([3, 3], [-1, 10], "k-")


def plot_obstacle(x, y, yaw, truckcolor="k-"):
    outline = np.array([[-LENGTH_obs / 2, LENGTH_obs / 2, LENGTH_obs / 2, -LENGTH_obs / 2, -LENGTH_obs / 2],
                        [WIDTH_obs / 2, WIDTH_obs / 2, -WIDTH_obs / 2, -WIDTH_obs / 2, WIDTH_obs / 2]])
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    outline = (outline.T.dot(Rot1)).T
    outline[0, :] += x
    outline[1, :] += y
    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)


def plot_wall(x, y, yaw, truckcolor="k-"):
    outline = np.array([[-LENGTH_wall / 2, LENGTH_wall / 2, LENGTH_wall / 2, -LENGTH_wall / 2, -LENGTH_wall / 2],
                        [WIDTH_wall / 2, WIDTH_wall / 2, -WIDTH_wall / 2, -WIDTH_wall / 2, WIDTH_wall / 2]])
    Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                     [-math.sin(yaw), math.cos(yaw)]])
    outline = (outline.T.dot(Rot1)).T
    outline[0, :] += x
    outline[1, :] += y
    plt.plot(np.array(outline[0, :]).flatten(),
             np.array(outline[1, :]).flatten(), truckcolor)


def update_state(state, v_l, v_r, odometry):
    v_robot = (v_l + v_r) / 2
    w_robot = (v_r - v_l) / (TREAD * 2)

    x_robot = state.x + v_robot * dt * math.cos(state.yaw)
    y_robot = state.y + v_robot * dt * math.sin(state.yaw)
    yaw_robot = state.yaw + w_robot * dt
    state = State(x_robot, y_robot, yaw_robot)

    l_odo = odometry.l_odo + v_l * dt
    r_odo = odometry.r_odo + v_r * dt
    odometry = Odometry(l_odo, r_odo)

    return state, odometry


def simulation_IR(state):
    x = state.x
    y = state.y
    yaw = state.yaw

    length_ir = 1.2

    l_1x = x + 0.3 * math.cos(yaw + math.pi / 4)
    l_2x = x + length_ir * math.cos(yaw + math.pi / 4)
    l_1y = y + 0.3 * math.sin(yaw + math.pi / 4)
    l_2y = y + length_ir * math.sin(yaw + math.pi / 4)

    r_1x = x + 0.3 * math.cos(yaw - math.pi / 4)
    r_2x = x + length_ir * math.cos(yaw - math.pi / 4)
    r_1y = y + 0.3 * math.sin(yaw - math.pi / 4)
    r_2y = y + length_ir * math.sin(yaw - math.pi / 4)

    plt.plot([l_1x, l_2x], [l_1y, l_2y], "r--")
    plt.plot([r_1x, r_2x], [r_1y, r_2y], "r--")

    # left
    step_l_x = (l_2x - l_1x) / 9
    l_list_x = [l_1x]
    for i in range(9):
        l_list_x.append(l_1x + step_l_x * (i + 1))

    step_l_y = (l_2y - l_1y) / 9
    l_list_y = [l_1y]
    for i in range(9):
        l_list_y.append(l_1y + step_l_y * (i + 1))

    # right
    step_r_x = (r_2x - r_1x) / 9
    r_list_x = [r_1x]
    for i in range(9):
        r_list_x.append(r_1x + step_r_x * (i + 1))

    step_r_y = (r_2y - r_1y) / 9
    r_list_y = [r_1y]
    for i in range(9):
        r_list_y.append(r_1y + step_r_y * (i + 1))

    # points
    l_ir_points = [[l_list_x[0], l_list_y[0]]]
    for i in range(9):
        l_ir_points.append([l_list_x[i + 1], l_list_y[i + 1]])

    r_ir_points = [[r_list_x[0], r_list_y[0]]]
    for i in range(9):
        r_ir_points.append([r_list_x[i + 1], r_list_y[i + 1]])

    # plot
    # plt.plot(l_list_x, l_list_y, "b.")
    # plt.plot(r_list_x, r_list_y, "b.")

    return l_ir_points, r_ir_points


def get_point(list_b):
    i = 0
    points = []
    while i < len(list_b.split(',')[1::2]):
        points.append({'x': float(list_b.split(',')[::2][i]), 'y': float(list_b.split(',')[1::2][i])})
        i += 1

    return points


def ray_casting(point, poly):
    flag = False
    j = 0
    length_poly = len(poly)
    k = length_poly - 1

    px = point[0]
    py = point[1]
    while j < length_poly:
        sx = poly[j]['x']
        sy = poly[j]['y']
        tx = poly[k]['x']
        ty = poly[k]['y']

        if (sx == px and sy == py) or (tx == px and ty == py):
            return px, py

        if (sy < py <= ty) or (sy >= py > ty):
            x = sx + (py - sy) * (tx - sx) / (ty - sy)
            if x == px:
                return px, py
            if x > px:
                flag = not flag
        k = j
        j += 1

    if flag:
        return px, py
    else:
        return "out"


def get_obstacle_poly(obstacle):
    p1 = [-LENGTH_obs / 2, -WIDTH_obs / 2]
    p2 = [-LENGTH_obs / 2, WIDTH_obs / 2]
    p3 = [LENGTH_obs / 2, WIDTH_obs / 2]
    p4 = [LENGTH_obs / 2, -WIDTH_obs / 2]

    tmp = p1[0] * math.cos(obstacle.yaw) - p1[1] * math.sin(obstacle.yaw) + obstacle.x
    p1[1] = p1[0] * math.sin(obstacle.yaw) + p1[1] * math.cos(obstacle.yaw) + obstacle.y
    p1[0] = tmp
    tmp = p2[0] * math.cos(obstacle.yaw) - p2[1] * math.sin(obstacle.yaw) + obstacle.x
    p2[1] = p2[0] * math.sin(obstacle.yaw) + p2[1] * math.cos(obstacle.yaw) + obstacle.y
    p2[0] = tmp
    tmp = p3[0] * math.cos(obstacle.yaw) - p3[1] * math.sin(obstacle.yaw) + obstacle.x
    p3[1] = p3[0] * math.sin(obstacle.yaw) + p3[1] * math.cos(obstacle.yaw) + obstacle.y
    p3[0] = tmp
    tmp = p4[0] * math.cos(obstacle.yaw) - p4[1] * math.sin(obstacle.yaw) + obstacle.x
    p4[1] = p4[0] * math.sin(obstacle.yaw) + p4[1] * math.cos(obstacle.yaw) + obstacle.y
    p4[0] = tmp

    # plt.plot(obstacle.x, obstacle.y, "r+")
    # plt.plot(p1[0], p1[1], "r.")
    # plt.plot(p2[0], p2[1], "r.")
    # plt.plot(p3[0], p3[1], "r.")
    # plt.plot(p4[0], p4[1], "r.")

    return str(p1[0]) + ',' + str(p1[1]) + ',' + str(p2[0]) + ',' + str(p2[1]) + ',' + str(p3[0]) + ',' + str(p3[1]) + ',' + str(p4[0]) + ',' + str(p4[1])


# left touch 1, right touch 2, nothing 0
def check_obstacle(l_points, r_points, poly):
    flag = 0
    for point in l_points:
        rs = ray_casting(point, poly)
        if rs != "out":
            flag = 1
    for point in r_points:
        rs = ray_casting(point, poly)
        if rs != "out":
            flag = 2

    return flag


# Robot move forward
def robot_move_froward(state, odometry):
    return update_state(state, 0.2, 0.2, odometry)


# Robot turn left
def robot_turn_left(state, odometry):
    return update_state(state, -0.2, 0.2, odometry)


# Robot turn right
def robot_turn_right(state, odometry):
    return update_state(state, 0.2, -0.2, odometry)


# turn back
def turn_back(state, odometry):

    diff = odometry.l_odo - odometry.r_odo
    if diff > 0:
        state, odometry = robot_turn_left(state, odometry)
    else:
        state, odometry = robot_turn_right(state, odometry)

    return state, odometry


# display the map
def display(obstacle_list, wall_list):
    plot_playground()
    for obstacle in obstacle_list:
        plot_obstacle(obstacle.x, obstacle.y, obstacle.yaw)

    for wall in wall_list:
        plot_wall(wall.x, wall.y, wall.yaw)

    plt.axis("equal")
    plt.grid(True)
    plt.pause(0.01)


def test(state, odometry):
    return robot_turn_left(state, odometry)


def main():
    # init
    state = State(0.0, 0.0, math.pi / 2)

    # Odometry
    odometry = Odometry(0.0, 0.0)

    # obstacles
    obstacle_1 = State(1, 1.5, 0)
    obstacle_2 = State(-0.3, 2.4, 0)
    obstacle_3 = State(-1.4, 5, 0)
    obstacle_4 = State(0.5, 7.5, math.pi / 4)
    obstacle_list = [obstacle_1, obstacle_2, obstacle_3, obstacle_4]

    # walls
    wall_l = State(-3.1, 4.5, 0)
    wall_r = State(3.1, 4.5, 0)
    wall_list = [wall_l, wall_r]

    # while odometry.l_odo < 9:
    need_check = True

    while state.y < 9:

        plt.cla()

        obstacle_poly_list = []
        for obstacle in obstacle_list:
            obstacle_poly_list.append(get_obstacle_poly(obstacle))

        # for wall in wall_list:
        #     obstacle_poly_list.append(get_obstacle_poly(wall))

        # display robot
        plot_robot(state.x, state.y, state.yaw)
        # IR line
        l_points, r_points = simulation_IR(state)

        poly_list = []
        for obstacle_poly in obstacle_poly_list:
            poly_list.append(get_point(obstacle_poly))

        # turn the robot
        for poly in poly_list:
            if check_obstacle(l_points, r_points, poly) != 0:

                limit = 1.5
                if check_obstacle(l_points, r_points, poly) == 1 and state.x <= limit:
                    while check_obstacle(l_points, r_points, poly):
                        plt.cla()
                        state, odometry = robot_turn_right(state, odometry)
                        plot_robot(state.x, state.y, state.yaw)
                        l_points, r_points = simulation_IR(state)
                        display(obstacle_list, wall_list)

                if check_obstacle(l_points, r_points, poly) == 2 and state.x >= -limit:
                    while check_obstacle(l_points, r_points, poly):
                        plt.cla()
                        state, odometry = robot_turn_left(state, odometry)
                        plot_robot(state.x, state.y, state.yaw)
                        l_points, r_points = simulation_IR(state)
                        display(obstacle_list, wall_list)

                if check_obstacle(l_points, r_points, poly) == 1 and state.x > limit:
                    while check_obstacle(l_points, r_points, poly) != 2:
                        plt.cla()
                        state, odometry = robot_turn_left(state, odometry)
                        plot_robot(state.x, state.y, state.yaw)
                        l_points, r_points = simulation_IR(state)
                        display(obstacle_list, wall_list)

                if check_obstacle(l_points, r_points, poly) == 2 and state.x < -limit:
                    while check_obstacle(l_points, r_points, poly) != 1:
                        plt.cla()
                        state, odometry = robot_turn_right(state, odometry)
                        plot_robot(state.x, state.y, state.yaw)
                        l_points, r_points = simulation_IR(state)
                        display(obstacle_list, wall_list)

        # move forward
        break_flag = False

        if need_check:
            check_forward_flag = True
            if state.x <= 0:
                while state.yaw > math.pi / 4:
                    plt.cla()
                    state, odometry = robot_turn_right(state, odometry)
                    plot_robot(state.x, state.y, state.yaw)
                    l_points, r_points = simulation_IR(state)
                    display(obstacle_list, wall_list)

                for poly in poly_list:
                    if check_obstacle(l_points, r_points, poly) == 1:
                        break_flag = True
                        check_forward_flag = False

                if check_forward_flag:
                    while state.yaw < math.pi / 2:
                        plt.cla()
                        state, odometry = robot_turn_left(state, odometry)
                        plot_robot(state.x, state.y, state.yaw)
                        simulation_IR(state)
                        display(obstacle_list, wall_list)

            elif state.x > 0:
                while state.yaw < 3 * math.pi / 4:
                    plt.cla()
                    state, odometry = robot_turn_left(state, odometry)
                    plot_robot(state.x, state.y, state.yaw)
                    l_points, r_points = simulation_IR(state)
                    display(obstacle_list, wall_list)

                for poly in poly_list:
                    if check_obstacle(l_points, r_points, poly) == 2:
                        break_flag = True
                        check_forward_flag = False

                if check_forward_flag:
                    while state.yaw > math.pi / 2:
                        plt.cla()
                        state, odometry = robot_turn_right(state, odometry)
                        plot_robot(state.x, state.y, state.yaw)
                        simulation_IR(state)
                        display(obstacle_list, wall_list)

            if check_forward_flag:
                stop_flag = False
                cnt = 0
                while not stop_flag and cnt < 5:
                    plt.cla()
                    state, odometry = robot_move_froward(state, odometry)
                    plot_robot(state.x, state.y, state.yaw)
                    l_points, r_points = simulation_IR(state)
                    display(obstacle_list, wall_list)
                    cnt = cnt + 1
                    for poly in poly_list:
                        if check_obstacle(l_points, r_points, poly) != 0:
                            stop_flag = True
                            need_check = False

            else:
                # print("Obs Ahead")
                need_check = False

        else:
            plt.cla()
            state, odometry = robot_move_froward(state, odometry)
            plot_robot(state.x, state.y, state.yaw)
            simulation_IR(state)
            display(obstacle_list, wall_list)

        # print(abs(odometry.l_odo - odometry.r_odo))
        while abs(odometry.l_odo - odometry.r_odo) > 0.000001 and not break_flag:

            plt.cla()
            state, odometry = turn_back(state, odometry)
            plot_robot(state.x, state.y, state.yaw)
            l_points, r_points = simulation_IR(state)

            for poly in poly_list:
                if check_obstacle(l_points, r_points, poly) != 0:
                    if check_obstacle(l_points, r_points, poly) == 1:
                        for i in range(3):
                            plt.cla()
                            state, odometry = robot_turn_right(state, odometry)
                            plot_robot(state.x, state.y, state.yaw)
                            simulation_IR(state)
                            display(obstacle_list, wall_list)
                    else:
                        for i in range(3):
                            plt.cla()
                            state, odometry = robot_turn_left(state, odometry)
                            plot_robot(state.x, state.y, state.yaw)
                            simulation_IR(state)
                            display(obstacle_list, wall_list)

                    for i in range(3):
                        plt.cla()
                        state, odometry = robot_move_froward(state, odometry)
                        plot_robot(state.x, state.y, state.yaw)
                        l_points, r_points = simulation_IR(state)
                        for poly_ in poly_list:
                            if check_obstacle(l_points, r_points, poly_) != 0:
                                break_flag = True
                                display(obstacle_list, wall_list)
                                break
                    break

            if not break_flag and abs(odometry.l_odo - odometry.r_odo) < 0.000001:
                need_check = True

            display(obstacle_list, wall_list)

        display(obstacle_list, wall_list)

    while True:
        plt.cla()
        plot_robot(state.x, state.y, state.yaw)
        display(obstacle_list, wall_list)


if __name__ == '__main__':
    main()
