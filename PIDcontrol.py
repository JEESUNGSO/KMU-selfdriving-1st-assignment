import numpy as np


def P(Kp, error):
    return -Kp * error


def I(Ki, error, accuE, dt):
    return -Ki * (accuE + dt * error)


def D(Kd, error, bef_error, dt):
    return -Kd * (error - bef_error) / dt


# 벡터 회전 함수
def rotate_by_vec(vec, ref_vec):
    theta = np.arctan2(ref_vec[1], ref_vec[0])
    R = np.array([[np.cos(-theta), -np.sin(-theta)],
                  [np.sin(-theta), np.cos(-theta)]])

    return R @ vec






def get_error(xy, lxy, velo, di):
    # 교점 후보 허용 오차
    tolerance = 8

    # 모든 path 점들과 거리 구하기
    dx = lxy[0] - xy[0]
    dy = lxy[1] - xy[1]
    distance = np.sqrt(dx ** 2 + dy ** 2)

    # 거리가 di와 가까운 점들이 아닌 점들 거리 0으로
    distance[(distance < di - tolerance)] = 0
    distance[(distance > di + tolerance)] = 0

    # 거리가 di에 가까우 점들을 순서대로 나열하고 뒤에 4개를 취하고
    # 그 중에서 가장 나중 경로인 점을 선택
    selected_point = lxy.T[np.max(np.argsort(distance)[-4:-1])]

    # 회전 시키
    car_to_point = xy - selected_point
    rotation_vector = rotate_by_vec(car_to_point, velo)


    error = rotation_vector[1]
    print(rotation_vector)

    return error, selected_point



# 대충 테스트 해볼 수 있는 pid 값
# Kp = 0.009
# Ki = 0.001
# Kd = 0.00002




accuE = bef_error = 0

def track_one_step(pos, lxy, velo, Kp, Ki, Kd, diameter, dt):
    global accuE, bef_error
    pos = np.array(pos, dtype=float)
    lxy = np.array(lxy, dtype=float)

    error, sel_p = get_error(pos, lxy, velo, diameter)


    #==P==
    p = P(Kp, error)
    #==I==
    i = I(Ki, error, accuE, dt)
    accuE += dt * error
    #==D==
    d = D(Kd, error, bef_error, dt)
    bef_error = error

    #==컨트롤 하기==
    u = p + d + i
    return u, sel_p
