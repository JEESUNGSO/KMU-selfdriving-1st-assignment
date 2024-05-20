import numpy as np

from planning_path_reeds_shepp import *

import matplotlib.pyplot as plt

start = [100.0, 350.0, -45]
goal = [1036, 162, 135]

# start = [-6,-7, 90]
# goal = [-6,-7, -90]

TURNING_RADIUS = 230.78846619378564 # 회전 반경

start = scale(start, TURNING_RADIUS)
goal = scale(goal, TURNING_RADIUS)


rxy = get_optimal_path(start, goal)
rx = []
ry =[]

for xyi in rxy:
    rx.extend(unscale(xyi[0], TURNING_RADIUS))
    ry.extend(unscale(xyi[1], TURNING_RADIUS))

plt.figure(figsize=(10,10))
plt.axis('equal')
plt.scatter([start[0]* TURNING_RADIUS], [start[1]* TURNING_RADIUS], 100, 'r')
plt.scatter([goal[0] * TURNING_RADIUS], [goal[1]* TURNING_RADIUS], 100, 'g')
plt.plot(rx, ry, '-ob')
plt.show()

# #
# rx1, ry1, next_ = get_list_R((0,0,0), -deg2rad(45), 100, np.pi/180)
# rx2, ry2, next_ = get_list_forward((rx1[-1], ry1[-1], next_), 20, 1)
# rx3, ry3, next_ = get_list_L((rx2[-1], ry2[-1], next_), -deg2rad(45), 100, np.pi/180)
#
#
# rx = np.concatenate((rx1, rx2, rx3))
# ry = np.concatenate((ry1, ry2, ry3))
#
#
# # print(rx1[-1], ry1[-1], rad2deg(next_))
#
# # print(rx[-1], ry[-1], M(np.rad2deg(next_)))
#
# plt.plot(rx, ry)
# plt.show()