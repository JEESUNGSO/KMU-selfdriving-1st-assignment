#!/usr/bin/env python
#-- coding:utf-8 --
####################################################################
# 프로그램이름 : parking.py
# 코드작성팀명 : 타 이거
####################################################################

#=============================================
# 함께 사용되는 각종 파이썬 패키지들의 import 선언부
#=============================================
import pygame
import numpy as np
import math
import rospy
from xycar_msgs.msg import xycar_motor
from planning_path_circle import *
from planning_path_line import *
#from planning_path_MPC import *
from planning_path_ES import *
from planning_path_reeds_shepp import *
from reed_shepp_utils import *
from PIDcontrol import track_one_step

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
rx, ry = [], []

# cp_index = dir_change_exist = dir_changed = speed= 0


#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표

TURNING_RADIUS = 230.78846619378564 # 회전 반경
#TURNING_RADIUS = 1


# PID 값
Kp = 0.5
Ki = 0.0000001
Kd = 0.001

# #경로생성 가중치
# wDistance = 25000
# wTheta = 200
# initial_length = 100
#
# # PID 오류 탐색 원의 크기
# DIAMETER = 200

# # cp점 도착 여부 확인 허용 오차
# close_tolerance = 20


# # MPC 알고리즘 상수
# obstacle_points = ((0,0), (1080, 0), (1200, 120), (1200, 850), (0, 850))
# step_length = 10
# wTHETA = 1
# wPOS = (1, 1)  # x: 차량 진행 방향 가중치, y: 차량 수직방향 가중치
# wU = 1
# k = 10
# iMax = 10000000
# cntMax = 100
# epsilon = 50

#=============================================
# 모터 토픽을 발행하는 함수
# 입력으로 받은 angle과 speed 값을
# 모터 토픽에 옮겨 담은 후에 토픽을 발행함.
#=============================================
def drive(angle, speed):
    xycar_msg.angle = int(angle)
    xycar_msg.speed = int(speed)
    motor_pub.publish(xycar_msg)

#=============================================
# 경로를 생성하는 함수
# 차량의 시작위치 sx, sy, 시작각도 syaw
# 최대가속도 max_acceleration, 단위시간 dt 를 전달받고
# 경로를 리스트를 생성하여 반환한다.
#=============================================
def planning(sx, sy, syaw, max_acceleration, dt):
    #global rx, ry, cp_index, dir_change_exist, wDistance, wTheta, initial_length, dir_changed, speed
    # global rx, ry, obstacle_points, step_length, wTHETA, wPOS, wU, k, iMax, cntMax
    global rx, ry

    rx, ry = [], []  # 리스트 초기화
    print("Start Planning")
    print(sx, sy)
    # 차량 기본 위치 정보 q들
    deg_car = rad2deg(np.deg2rad(syaw+90-360))
    q_car = np.array([sx, sy, deg_car])
    deg_park = rad2deg(np.arctan2(P_ENTRY[1] - P_END[1], P_ENTRY[0] - P_END[0]))
    q_park = np.array([P_ENTRY[0], P_ENTRY[1], deg_park])

    
    # 경로 생성 알고리즘, 별도로 global 변수들 설정해줘야 하고 작동 안하는 방법도 존재
    #print(f"deg_car: {np.rad2deg(rad_car)}, deg_park: {np.rad2deg(rad_park)}")
    #rx, ry = get_path_line(sx, sy, P_ENTRY[0], P_ENTRY[1])
    #rx, ry = get_path_circle(sx, sy, rad_car, P_ENTRY[0], P_ENTRY[1], TURNING_RADIUS)
    #rx, ry = get_path_MPC(q_car, q_park, obstacle_points, step_length, wTHETA, wPOS, wU, k, iMax, cntMax, epsilon)
    #rx, ry, cp_index, dir_change_exist = get_path_ES(q_car, q_park, wDistance, wTheta, initial_length)
    #rx, ry = get_path_ES(q_car, q_park, wDistance, wTheta) # 원 디버깅용

    #--------reed shepp-----------#
    # 입력값 스케일 조정, turning_radius = 1 이 될 수 있도록, 함수에서 그렇게 사용하므로
    q_car = scale(q_car, TURNING_RADIUS)
    q_park = scale(q_park, TURNING_RADIUS)

    rxy = get_optimal_path(q_car, q_park)

    for xyi in rxy:
        # 스케일 복구
        rx.extend(unscale(xyi[0], TURNING_RADIUS))
        ry.extend(unscale(xyi[1], TURNING_RADIUS))

        # xyi[2] = 직진 후진 정보 1: 직진, -1: 후진

    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, current_speed, max_acceleration, dt):
    global rx, ry, Kp, Ki, Kd, DIAMETER, D, dir_change_exist, dir_changed, speed, close_tolerance
    # 값들 전처리
    direction = np.deg2rad(-yaw)
    velocity = np.array([np.cos(direction), np.sin(direction)], dtype=float) * current_speed


    # 전진 후진 전환 존재 할때
    if dir_change_exist:
        print("방향 전환 있음")
        # cp 후 부터
        if dir_changed:
            #전진중
            u, sel_p = track_one_step([x, y], velocity, [rx[cp_index+1:], ry[cp_index+1:]], Kp, Ki, Kd, DIAMETER, dt)
            speed = +50
        # cp 전 까지
        else:
            #후진중
            u, sel_p = track_one_step([x, y], velocity, [rx[:cp_index+1], ry[:cp_index+1]], Kp, Ki, Kd, DIAMETER, dt)
            speed = -50
            # 후진시에는 조향각이 반대
            u *= -1
            # 차량이 cp점에 도착하면
            if np.linalg.norm(np.array([x,y]) - sel_p) < close_tolerance:
                dir_changed = 1
    # 전진 후진 전환 존재 안할때 (전진만 할때)
    else:
        print("방향 전환 없음")
        u, sel_p = track_one_step([x, y], velocity, [rx, ry], Kp, Ki, Kd, DIAMETER, dt)
        speed = 50

    # pid 탐색 반경 표시
    pygame.draw.circle(screen, (255,0,0), [x, y], DIAMETER, width=2)
    # 차량 방향 표시
    pygame.draw.line(screen, (0, 155, 155), [x, y], [x, y] + np.array([np.cos(direction), np.sin(direction)]) * DIAMETER, width=2)
    pygame.draw.line(screen, (0, 155, 155), [x, y], [x, y] - np.array([np.cos(direction), np.sin(direction)]) * DIAMETER, width=2)
    # 에러계산 선택점 표시
    pygame.draw.circle(screen, (0, 255, 0), sel_p, 10)

    #print("u: ", u)
    drive(u, speed)
