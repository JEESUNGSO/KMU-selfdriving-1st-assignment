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
def get_list_R(current, e_param,  delta):
    beta = np.deg2rad(current[2]) - np.pi/2
    center = [current[0] + np.cos(beta), current[1] + np.sin(beta)]
    if e_param > 0:
        thetas = np.arange(0, e_param, delta)
    else:
        thetas = -np.arange(0, -e_param, delta)
    thetas = np.append(thetas, e_param)
    rxi = center[0] + np.cos(np.pi/2 + np.deg2rad(current[2]) - thetas)
    ryi = center[1] + np.sin(np.pi/2 + np.deg2rad(current[2]) - thetas)
    return rxi, ryi, current[2] - rad2deg(thetas[-1])
    
    
# 왼쪽 회전 경로 리스트 반환, delta 각도 간격
def get_list_L(current, e_param, delta):
    beta = np.deg2rad(current[2]) + np.pi/2
    center = [current[0] + np.cos(beta), current[1] + np.sin(beta)]
    if e_param > 0:
        thetas = np.arange(0, e_param, delta)
    else:
        thetas = -np.arange(0, -e_param, delta)
    thetas = np.append(thetas, e_param)
    rxi = center[0] + np.cos(np.deg2rad(current[2]) + thetas - np.pi/2)
    ryi = center[1] + np.sin(np.deg2rad(current[2]) + thetas - np.pi/2)
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

# 꺽이는 경로를 추종하기 위해서 조금 더 마진을 추가하는 함수
def get_list_margin(path_list, margin):
    # 마지막 끝 두점으로 마진 직선 추가
    theta = np.arctan2(path_list[1][-1] - path_list[1][-2], path_list[0][-1] - path_list[0][-2])
    l = np.arange(0, margin, 1)
    lx = path_list[0][-1] + l * np.cos(theta)
    ly = path_list[1][-1] + l * np.sin(theta)
    return lx, ly, len(l) - 1


# 회전 반경에 따라 회전반경이 1에 맞춰져있는 경로를 구한느 함수를 사용 할 수 있게 좌표값의 스케일을 변환 하는 함수
def scale(x, turning_radius):

    scale_factor = 1/turning_radius
    return (x[0] * scale_factor, x[1] * scale_factor, x[2])


# 결과값을 다시 스케일을 바꿔야 하는데 인풋이 리스트임
def unscale(x, TRUNING_RADIUS):
    if type(x) is tuple or type(x) is list:
        return [p / TRUNING_RADIUS for p in x]
    return x * TRUNING_RADIUS




# 장애물에 부딫 히는 경우
def is_collision(rxi, ryi, MAP, P_ENTRY, P_END, padding):
    # padding = 장애물로부터 얼마나 떨어진 지점까지 장애물로 취급할지
    # x값이 창 범위를 벗어나는지
    if np.sum((rxi < 0 + padding).astype(int)) or np.sum((rxi > MAP[0] - padding).astype(int)):
        return True
    # y값이 창 범위를 벗어나는지
    elif np.sum((ryi < 0 + padding).astype(int)) or np.sum((ryi > MAP[1] - padding).astype(int)):
        return True

    # 주차 하는 벽에 부딪히는지 확인
    alpha = np.arctan2(P_ENTRY[1] - P_END[1], P_ENTRY[0] - P_END[0]) - np.pi
    a = np.tan(alpha)
    distance = np.abs(a*(rxi - P_ENTRY[0]) + P_ENTRY[1] - ryi) / np.sqrt(1 + a**2)
    if np.sum((distance < padding).astype(int)):
        return True
    
    # 충돌 안하면
    return False


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