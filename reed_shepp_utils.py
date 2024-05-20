import math
import numpy as np


# 각도에 2pi로 모듈러 연산을 수행
def M(theta): 
    theta = theta % (2*math.pi)
    if theta < -math.pi: return theta + 2*math.pi
    if theta >= math.pi: return theta - 2*math.pi
    return theta

# 극좌표계를 반환
def R(x, y):
    r = math.sqrt(x*x + y*y)
    theta = math.atan2(y, x)
    return r, theta

# p1을 기준좌표계로 p2의 좌표 및 각도를 얻음
def change_of_basis(p1, p2):
    theta1 = deg2rad(p1[2])
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    new_x = dx * math.cos(theta1) + dy * math.sin(theta1)
    new_y = -dx * math.sin(theta1) + dy * math.cos(theta1)
    new_theta = p2[2] - p1[2]
    return new_x, new_y, new_theta

# 라디안을 도로
def rad2deg(rad):
    return 180 * rad / math.pi

# 도를 라디안으로
def deg2rad(deg):
    return math.pi * deg / 180

# 오른쪽 회전 경로 리스트 반환, delta 각도 간격
def get_list_R(current, e_param, turning_radius,  delta):
    beta = np.deg2rad(current[2]) - np.pi/2
    center = [current[0] + turning_radius*np.cos(beta), current[1] + turning_radius*np.sin(beta)]
    if e_param > 0:
        thetas = np.arange(0, e_param, delta)
    else:
        thetas = -np.arange(0, -e_param, delta)
    thetas = np.append(thetas, e_param)
    rxi = center[0] + turning_radius * np.cos(np.pi/2 + np.deg2rad(current[2]) - thetas)
    ryi = center[1] + turning_radius * np.sin(np.pi/2 + np.deg2rad(current[2]) - thetas)
    return rxi, ryi, current[2] - rad2deg(thetas[-1])
    
    
# 왼쪽 회전 경로 리스트 반환, delta 각도 간격
def get_list_L(current, e_param, turning_radius,  delta):
    beta = np.deg2rad(current[2]) + np.pi/2
    center = [current[0] + turning_radius * np.cos(beta), current[1] + turning_radius * np.sin(beta)]
    if e_param > 0:
        thetas = np.arange(0, e_param, delta)
    else:
        thetas = -np.arange(0, -e_param, delta)
    thetas = np.append(thetas, e_param)
    rxi = center[0] + turning_radius * np.cos(np.deg2rad(current[2]) + thetas - np.pi/2)
    ryi = center[1] + turning_radius * np.sin(np.deg2rad(current[2]) + thetas - np.pi/2)
    return rxi, ryi, rad2deg(thetas[-1]) + current[2]



# 직진 리스트 반환, delta 점 간격
def get_list_forward(current, e_param, delta):
    if e_param > 0:
        l = np.arange(0, e_param, delta)
    else:
        l = -np.arange(0, -e_param, delta)
    l = np.append(l, e_param)
    rxi = current[0] + l*np.cos(deg2rad(current[2]))
    ryi = current[1] + l*np.sin(deg2rad(current[2]))
    return rxi, ryi, current[2]

# 회전 반경에 따라 회전반경이 1에 맞춰져있는 경로를 구한느 함수를 사용 할 수 있게 좌표값의 스케일을 변환 하는 함수
def scale(x, turning_radius):

    scale_factor = 1/turning_radius
    print(scale_factor)
    return (x[0] * scale_factor, x[1] * scale_factor, x[2])


# 결과값을 다시 스케일을 바꿔야 하는데 인풋이 리스트임
def unscale(x, TRUNING_RADIUS):
    if type(x) is tuple or type(x) is list:
        return [p / TRUNING_RADIUS for p in x]
    return x * TRUNING_RADIUS

# #==========디버깅용===========================
# import matplotlib.pyplot as plt
#
# rxi, ryi, next_ = get_list_R((0, 0, 0), deg2rad(30), 10, np.pi/180)
#
#
# #plt.axis([95,105, 345, 355])
# print(len(rxi))
# plt.plot(rxi, ryi)
# plt.show()