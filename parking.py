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
from planning_path_MPC import *
from PIDcontrol import track_one_step

#=============================================
# 모터 토픽을 발행할 것임을 선언
#============================================= 
motor_pub = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
xycar_msg = xycar_motor()

#=============================================
# 프로그램에서 사용할 변수, 저장공간 선언부
#=============================================
rx, ry = [300, 350, 400, 450], [300, 350, 400, 450]
Dqcar_cP = 1

#=============================================
# 프로그램에서 사용할 상수 선언부
#=============================================
AR = (1142, 62) # AR 태그의 위치
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표
TURNING_RADIUS = 461.5769323875713/2 # 회전 반경

# PID 값
Kp = 0.5
Ki = 0.0000001
Kd = 0.001

# PID 오류 탐색 원의 크기
DIAMETER = 200

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
    global rx, ry
    direction = np.deg2rad(-syaw)
    print("Start Planning")
    rx, ry = get_path_line(sx, sy, P_ENTRY[0], P_ENTRY[1])
    #rx, ry = get_path_circle(sx, sy, direction, P_ENTRY[0], P_ENTRY[1], TURNING_RADIUS)
    #rx, ry = gey_path_MPC()
    return rx, ry

#=============================================
# 생성된 경로를 따라가는 함수
# 파이게임 screen, 현재위치 x,y 현재각도, yaw
# 현재속도 velocity, 최대가속도 max_acceleration 단위시간 dt 를 전달받고
# 각도와 속도를 결정하여 주행한다.
#=============================================
def tracking(screen, x, y, yaw, current_speed, max_acceleration, dt):
    # yaw 값은 degree

    global rx, ry, Dqcar_cP, Kp, Ki, Kd, DIAMETER
    # 값들 전처리
    direction = np.deg2rad(-yaw)
    velocity = np.array([np.cos(direction), np.sin(direction)], dtype=float) * current_speed

    # print("direction: ", direction, "  yaw: ", yaw, "  velocity: ", velocity)

    u, sel_p = track_one_step([x,y], velocity, [rx, ry, Dqcar_cP], Kp, Ki, Kd, DIAMETER, dt)

    # pid 탐색 반경 표시
    pygame.draw.circle(screen, (255,0,0), [x, y], DIAMETER, width=2)
    # 차량 방향 표시
    pygame.draw.line(screen, (0, 155, 155), [x, y], [x, y] + np.array([np.cos(direction), np.sin(direction)]) * DIAMETER, width=2)
    pygame.draw.line(screen, (0, 155, 155), [x, y], [x, y] - np.array([np.cos(direction), np.sin(direction)]) * DIAMETER, width=2)
    # 에러계산 선택점 표시
    pygame.draw.circle(screen, (0, 255, 0), sel_p, 10)

    #print("u: ", u)
    drive(u, 50)

