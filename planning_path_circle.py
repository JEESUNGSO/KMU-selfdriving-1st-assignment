import numpy as np





def get_turning_circle(c_pos, cdirection, turning_radius, density):
    cdir_vertical = cdirection - np.pi / 2
    # 오른쪽
    turning_center_R = c_pos + np.array([np.cos(cdir_vertical), np.sin(cdir_vertical)]) * turning_radius
    turning_points_R = np.array([turning_center_R + np.array([np.cos(theta), np.sin(theta)]) \
                         * turning_radius for theta in np.linspace(0, np.pi * 2, density)])
    # 왼쪽
    turning_center_L = c_pos - np.array([np.cos(cdir_vertical), np.sin(cdir_vertical)]) * turning_radius
    turning_points_L = np.array([turning_center_L + np.array([np.cos(theta), np.sin(theta)]) \
                               * turning_radius for theta in np.linspace(0, np.pi * 2, density)])

    return turning_points_R, turning_points_L


def get_tanget_line(circle1, circle2, diameter):
    # circle1, circle2 각각 원의 중심 좌표
    # diameter : 원의 지름
    r = diameter / 2
    
    distance = np.pow((circle1[0] - circle2[0]), 2) + np.pow((circle1[1] - circle2[1]), 2)
    distance = np.sqrt(distance) # 윈의 중심 사이의 거리
    # theta : 원의 중심을 잇는 선분과 두개의 평행한 접선이 이루는 각
    tan_theta = r / distance
    theta = np.arctan(tan_theta)
    offset = np.array([np.sin(theta), -np.cos(theta)])
    point_l1_c1 = circle1 + offset
    point_l1_c2 = circle2 + offset
    point_l4_c1 = circle1 - offset
    point_l4_c2 = circle2 - offset

    # delta : 나머지 교차하는 접선과 원의 중심을 잇는 선분이 이루는 각
    sin_delta = (distance / 2) / (r)
    delta = np.arcsin(sin_delta)
    
    
    

def get_path_circle(cx, cy, cdirection, gx, gy, turning_radius):
    # 회전 반경 원 점 개수
    density = 100
    
    # 차량 위치 벡터
    start_pos = np.array([cx, cy])
    end_pos = np.array([gx, gy])

    # 출발점 회전 반경
    start_turning_R, start_turning_points_L = get_turning_circle(start_pos, cdirection, turning_radius, density)
    # 도착점 회전 반경
    end_turning_R, end_turning_L = get_turning_circle(end_pos, cdirection, turning_radius, density)

    # 접선 찾기



    return np.concatenate((start_turning_R.T[0], end_turning_R.T[0]), axis=0), \
        np.concatenate((start_turning_R.T[1], end_turning_R.T[1]), axis=0)

