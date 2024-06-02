import pygame
import numpy as np
from math import *
import matplotlib.pyplot as plt

WHITE = (255, 255, 255)
RED = (255, 0, 0)
BLACK = (0, 0, 0)

time_delta = 0.01

WIDTH, HEIGHT = 800, 600
pygame.display.set_caption("3D projection")
screen = pygame.display.set_mode((WIDTH, HEIGHT))

scale = 100

circle_pos = [WIDTH/2, HEIGHT/2]  # x, y

points = []

# all the cube vertices
points.append(np.matrix([-0.5, -2, 1]))
points.append(np.matrix([0.5, -2, 1]))
points.append(np.matrix([0.5,  2, 1]))
points.append(np.matrix([-0.5, 2, 1]))
points.append(np.matrix([-0.5, -2, -1]))
points.append(np.matrix([0.5, -2, -1]))
points.append(np.matrix([0.5, 2, -1]))
points.append(np.matrix([-0.5, 2, -1]))

projection_matrix = np.matrix([
    [1, 0, 0],
    [0, 1, 0]
])

projected_points = [
    [n, n] for n in range(len(points))
]

# --------------------------------------------------- change values ---------------------------------------------------
# initial setting
roll = 0
pitch = 0
yaw = 0

# target setting
target_roll = 5
target_pitch = 5
target_yaw = 5
# ---------------------------------------------------------------------------------------------------------------------


# PID Constants:
Kp = 0.7
Ki = 0.6
Kd = 0.3


# ------------------------------------------------- For PID controls --------------------------------------------------
roll_Change = 0
roll_prevChange = 0
roll_prevprevChange = 0
roll_prevError = 0
roll_integralSum = 0

pitch_Change = 0
pitch_prevChange = 0
pitch_prevprevChange = 0
pitch_prevError = 0
pitch_integralSum = 0

yaw_Change = 0
yaw_prevChange = 0
yaw_prevprevChange = 0
yaw_prevError = 0
yaw_integralSum = 0

now_time = 0
roll_I = 0
pitch_I = 0
yaw_I = 0
roll_e_prev = 0
pitch_e_prev = 0
yaw_e_prev = 0
time_prev = 0

# for plotting
roll_history = []
pitch_history = []
yaw_history = []
time_history = []
target_roll_history = []
target_pitch_history = []
target_yaw_history = []
# ---------------------------------------------------------------------------------------------------------------------


# ---------------------------------------------------- PID control ----------------------------------------------------
def roll_PID(target_point, now):
    global MV, roll_I, time_prev, roll_e_prev

    # Value of offset - when the error is equal zero
    offset = 0

    # PID calculations
    e = target_point - now

    P = e
    roll_I = roll_I + ((e + roll_e_prev)) / 2 * time_delta
    D = (e - roll_e_prev) / time_delta

    # calculate manipulated variable - MV
    MV = offset + Kp * P + Ki * roll_I + Kd * D

    # update stored data for next iteration
    roll_e_prev = e

    return MV

def pitch_PID(target_point, now):
    global MV, pitch_I, time_prev, pitch_e_prev

    # Value of offset - when the error is equal zero
    offset = 0

    # PID calculations
    e = target_point - now

    P = e
    pitch_I = pitch_I + ((e + pitch_e_prev)) / 2 * time_delta
    D = (e - pitch_e_prev) / time_delta

    # calculate manipulated variable - MV
    MV = offset + Kp * P + Ki * pitch_I + Kd * D

    # update stored data for next iteration
    pitch_e_prev = e

    return MV

def yaw_PID(target_point, now):
    global MV, yaw_I, time_prev, yaw_e_prev

    # Value of offset - when the error is equal zero
    offset = 0

    # PID calculations
    e = target_point - now

    P = e
    yaw_I = yaw_I + ((e + yaw_e_prev)) / 2 * time_delta
    D = (e - yaw_e_prev) / time_delta

    # calculate manipulated variable - MV
    MV = offset + Kp * P + Ki * yaw_I + Kd * D

    # update stored data for next iteration
    yaw_e_prev = e

    return MV
# ---------------------------------------------------------------------------------------------------------------------


def connect_points(i, j, points):
    pygame.draw.line(
        screen, BLACK, (points[i][0], points[i][1]), (points[j][0], points[j][1]))

def Plot_graph():
    fig, ax = plt.subplots(3, 1, squeeze=True)

    plt.figure(figsize=(10, 20))
    fig.subplots_adjust(hspace=5)

    # Plotting

    plt.subplot(3, 1, 1)  # nrows=1, ncols=2, index=1
    plt.plot(time_history, roll_history, linewidth=1, color='red', label='simulated roll')
    plt.plot(time_history, target_roll_history, '-', label='target roll')
    plt.title('Roll')
    plt.xlim(0, 200)
    plt.ylim(-30, 30)
    plt.xticks(np.arange(0, 201, 10))
    plt.yticks(np.arange(-30, 31, 5))
    plt.xlabel('time(s)')
    plt.ylabel('degree(*)')
    plt.legend()

    plt.subplot(3, 1, 2)  # nrows=1, ncols=2, index=2
    plt.plot(time_history, pitch_history, linewidth=1, color='green', label='simulated pitch')
    plt.plot(time_history, target_pitch_history, '-', label='target pitch')
    plt.title('Pitch')
    plt.xlim(0, 200)
    plt.ylim(-30, 30)
    plt.xticks(np.arange(0, 201, 10))
    plt.yticks(np.arange(-30, 31, 5))
    plt.xlabel('time(s)')
    plt.ylabel('degree(*)')
    plt.legend()

    plt.subplot(3, 1, 3)  # nrows=1, ncols=2, index=2
    plt.plot(time_history, yaw_history, linewidth=1, color='blue', label='simulated yaw')
    plt.plot(time_history, target_yaw_history, '-', label='target yaw')
    plt.title('Yaw')
    plt.xlim(0, 200)
    plt.ylim(-30, 30)
    plt.xticks(np.arange(0, 201, 10))
    plt.yticks(np.arange(-30, 31, 5))
    plt.xlabel('time(s)')
    plt.ylabel('degree(*)')
    plt.legend()

    plt.show()


clock = pygame.time.Clock()

# ----------------------------------------------------- Main Loop -----------------------------------------------------

while True:

    # time calculation
    time_prev = now_time
    now_time += time_delta
    print(now_time)

    # ------------------------------------------------- change values -------------------------------------------------

    if (now_time // 50) % 2 == 1:
        target_roll = 5
        target_pitch = 5
        target_yaw = 5

    else :
        target_roll = -5
        target_pitch = -5
        target_yaw = -5

    # -------------------------------------------------------------------------------------------------


    # Store values for plotting
    roll_history.append(roll)
    pitch_history.append(pitch)
    yaw_history.append(yaw)
    time_history.append(now_time)
    target_roll_history.append(target_roll)
    target_pitch_history.append(target_pitch)
    target_yaw_history.append(target_yaw)

    clock.tick(60)

    # exit conditions  ( ESC key down / time >= 200s / close window )
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            exit()

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE:
                Plot_graph()

                plt.show()
                pygame.quit()
                exit()

    if now_time >= 200:
        Plot_graph()
        plt.show()

        pygame.quit()
        exit()

    # update stuff
    rotation_z = np.matrix([[cos(yaw), -sin(yaw), 0],
                            [sin(yaw), cos(yaw), 0],
                            [0, 0, 1]])

    rotation_y = np.matrix([[cos(pitch), 0, sin(pitch)],
                            [0, 1, 0],
                            [-sin(pitch), 0, cos(pitch)]])

    rotation_x = np.matrix([[1, 0, 0],
                            [0, cos(roll), -sin(roll)],
                            [0, sin(roll), cos(roll)]])


    # PID control
    #roll
    roll_Change = roll_PID(target_roll, roll)
    roll += (roll_prevprevChange + roll_prevChange + roll_Change) / 3 * time_delta # 3-point moving average filter

    #pitch
    pitch_Change = pitch_PID(target_pitch, pitch)
    pitch += (pitch_prevprevChange + pitch_prevChange + pitch_Change) / 3 * time_delta # 3-point moving average filter

    #yaw
    yaw_Change = yaw_PID(target_yaw, yaw)
    yaw += (yaw_prevprevChange + yaw_prevChange + yaw_Change) / 3 * time_delta # 3-point moving average filter

    # for Debug
    print(roll, pitch, yaw)
    print(roll_Change, pitch_Change, yaw_Change)

    roll_prevprevChange = roll_prevChange
    roll_prevChange = roll_Change

    pitch_prevprevChange = pitch_prevChange
    pitch_prevChange = pitch_Change

    yaw_prevprevChange = yaw_prevChange
    yaw_prevChange = yaw_Change

    screen.fill(WHITE)


    # drawining
    i = 0
    for point in points:
        rotated2d = np.dot(rotation_z, point.reshape((3, 1)))
        rotated2d = np.dot(rotation_y, rotated2d)
        rotated2d = np.dot(rotation_x, rotated2d)

        projected2d = np.dot(projection_matrix, rotated2d)

        x = int(projected2d[0][0] * scale) + circle_pos[0]
        y = int(projected2d[1][0] * scale) + circle_pos[1]

        projected_points[i] = [x, y]
        pygame.draw.circle(screen, RED, (x, y), 5)
        i += 1

    for p in range(4):
        connect_points(p, (p+1) % 4, projected_points)
        connect_points(p+4, ((p+1) % 4) + 4, projected_points)
        connect_points(p, (p+4), projected_points)


    pygame.display.update()

# ---------------------------------------------------------------------------------------------------------------------



