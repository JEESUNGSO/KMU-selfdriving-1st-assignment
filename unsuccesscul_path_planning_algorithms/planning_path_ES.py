import numpy as np
import matplotlib.pyplot as plt

def cost(q_car, q_park, wDistance, wTheta):
    distance_abs = np.linalg.norm((q_car[0] - q_park[0]))


    theta_pow = (q_car[2] - q_park[2]) ** 2

    # 적절한 가중치 조절이 필요..
    cost = wTheta * theta_pow + wDistance / distance_abs
    print(f"theta/distance ratio = {(wTheta * theta_pow) / (wDistance / distance_abs)*100}")
    return cost


# 점들을 잇는 점의 집합 구하기
def get_line_dots(sx, sy, gx, gy):
    theta = np.arctan2(gy-sy, gx-sx)
    l = np.arange(0, np.linalg.norm(np.array([gx-sx, gy-sy])), 0.5, dtype=float)
    lx = l * np.cos(theta) + sx
    ly = l * np.sin(theta) + sy
    return lx, ly


# 디버깅용 원 그리기
# def get_turning_circle(c_pos, turning_radius, density=300):
#     turning_points_R = np.array([c_pos + np.array([np.cos(theta), np.sin(theta)]) * turning_radius for theta in np.linspace(0, np.pi * 2, density)])
#     return turning_points_R


# 탐색선 교점 존재 여부
def crossed_point(q_car, q_park, serach_length):
    # 직선 기울기
    acar = np.tan(q_car[2])
    apark = np.tan(q_park[2])

    # 교점 x좌표 구하기
    x = ((acar*q_car[0] - apark*q_car[0]) - (q_car[1] - q_park[1]))/(acar - apark)

    # 탐색 길이 안에 드는지 확인
    # 탐색 길이 안에 있음
    if np.abs(x - q_car[0]) <= serach_length * np.cos(q_car[2]):
        return np.array([x, acar * (x - q_car[0]) + q_car[1]]) # 교점 반환
    # 탐색 길이 안에 없음
    else:
        return None # 교점 없음 반환

def get_close_point(q_car, q_park,  cp, initial_length):
    # 허용 오차
    tolerance = 10

    # 앞 뒤 탐색 막대 끝점
    fend = q_car[:2] + np.array([np.cos(q_car[2]), np.sin(q_car[2])]) * initial_length
    rend = q_car[:2] - np.array([np.cos(q_car[2]), np.sin(q_car[2])]) * initial_length

    # 앞 뒤 최소 거리
    fd = np.linalg.norm(fend - cp)
    rd = np.linalg.norm(rend - cp)


    # 앞이 더 짧으면
    if fd < rd:
        return fend
    # 뒤가 더 짧으면
    elif fd > rd:
        return rend
    # 같으면
    else:
        # 원의 중심에서 먼 점을 선택
        fd = np.linalg.norm(fend - q_park[:2])
        rd = np.linalg.norm(rend - q_park[:2])
        # 앞이 더 길면
        if fd > rd:
            return fend
        # 뒤가 더 길면
        elif fd < rd:
            return rend
        else:
            print("아니 이게 같네 ㄷㄷ")
            
            


def get_path_ES(q_car, q_park, wDistance, wTheta, initial_length):
    # 영역 디버깅 용
    # lx, ly = get_turning_circle(q_car[:2], cost(q_car, q_park, wDistance, wTheta)).T
    # return lx, ly

    # 반경 계산
    d = cost(q_car, q_park, wDistance, wTheta)
    # cp 계산
    cp = q_park[:2] - np.array([np.cos(q_park[2]), np.sin(q_park[2])]) * d


    # sp 정하기
    sp = get_close_point(q_car, q_park, cp, initial_length)
    # # 차량이 내부에 있을때
    # if np.linalg.norm(q_car[:2] - q_park[:2]) <= d:
    #     sp = get_close_point(q_car, q_park, cp, initial_length)
    # # 차량이 외부에 있을때
    # else:
    #     sp = get_close_point(q_car, q_park, cp, initial_length)



    #qcar->sp->cp
    qcar_sp_x, qcar_sp_y = get_line_dots(q_car[0], q_car[1], sp[0], sp[1])
    sp_cp_x, sp_cp_y = get_line_dots(sp[0], sp[1], cp[0], cp[1])

    #cp->qpark
    cp_qpark_x, cp_qpark_y = get_line_dots(cp[0], cp[1], q_park[0], q_park[1])

    #rx, ry 만들기 및 cp_index 구하기
    rx = np.concatenate((qcar_sp_x, sp_cp_x))
    cp_index = rx.shape[0] - 1
    rx = np.concatenate((rx, cp_qpark_x))
    ry = np.concatenate((qcar_sp_y, sp_cp_y, cp_qpark_y))

    # 방향 바꿀지 결정
    # qcar -> sp 벡터 각도
    qcar2sp_theta = np.arctan2(q_car[1] - sp[1], q_car[0] - sp[0]) - np.pi
    # qcar -> sp 벡터 각도와 차량 벡터 각도의 방향이 반대일때

    print(np.rad2deg(qcar2sp_theta), np.rad2deg(q_car[2]))
    
    # 방향이 같을때
    if  abs(qcar2sp_theta - q_car[2]) < np.pi / 2: # pi/2 는 방향이 맞는 지 확인하는 대략적인 180도보다 작은 값
        dir_change_exist = 0 # cp에서 전진 후진 안바꿈
    # 방향이 다를 때
    else:
        dir_change_exist = 1 # cp에서 전진 후진 바꿈
        
    return rx, ry, cp_index, dir_change_exist
