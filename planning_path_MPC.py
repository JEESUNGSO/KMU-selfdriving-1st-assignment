import numpy as np

# 좌표계를 변환하는 함수 (==========================================이 함수 고쳐야 됨!!)============================
def To_coordinate(q_coordinates, q_):

    # q_cordinate와 어떤 q점 사이의 거리
    lengths = np.linalg.norm(q_coordinates[:, :2] - q_[:2])
    # q_cordinate와 어떤 q점(기준) 벡터의 각도
    thetas = np.arctan2(q_coordinates[:, 1] - q_[1], q_coordinates[:, 0] - q_[0])
    gammas = thetas - ((np.pi / 2) + q_coordinates[:, 2])
    lx_ = q_coordinates[:, 0] + lengths * np.sin(gammas)
    ly_ = q_coordinates[:, 1] - lengths * np.cos(gammas)

    # 좌표계 변환된 q 반환, 글로벌 각도는 유지
    q_rad = [q_[2] for i in range(len(lx_))]
    return np.array([lx_, ly_, q_rad]).T

# 장애물 점 리스트 생성
def get_points_from_dotlist(points_list):
    # 두점을 잇는 선분의 점집합 생성
    def get_dots(p1, p2):
        p1 = np.array(p1)
        p2 = np.array(p2)
        theta = np.arctan2(p1[1] - p2[1], p1[0] - p2[0])
        l = np.arange(0, np.linalg.norm(p1 - p2), 1)
        lx = p1[0] + l * np.cos(theta)
        ly = p1[1] + l * np.sin(theta)
        lxy = np.array([lx, ly]).T
        return lxy



    for i in range(len(points_list)):
        # 처음 점일 때
        if i == 0:
            result = get_dots(points_list[i], points_list[0])
        # 마지막 점일때
        elif i == len(points_list)-1:
            np.concatenate((result, get_dots(points_list[i], points_list[0])))
        # 마지막 점이 아닐 때
        else:
            np.concatenate((result, get_dots(points_list[i], points_list[i+1])))

    return result



# 비용함수, q_is: q_i 후보들, q_park: 도착지점, u_is: q_iplus1s를 만드는 정책, u_iminus1: q_i를 만들었던 정책
def cost(q_iplus1s, q_park, u_is, u_iminus1, wTHETA, wPOS, wU):
    # q_iplus1 좌표계 기준으로 변경된 q_park들
    q_parks = To_coordinate(q_iplus1s, q_park)
    eposS = [(( q_parks[:, :2]) ** 2)[:, 0] * wPOS[0], ((q_parks[:, :2]) ** 2)[:, 1] * wPOS[1]]
    ethetaS = (q_iplus1s[:, 2] - q_park[2]) ** 2
    deltauS = (u_is - u_iminus1) ** 2

    # etheta: 후보 위치와 도착지점 각도 차이, epos: 멘하턴 거리, deltau: 이전정책과 현재 정책 변화량
    return wTHETA * ethetaS + np.sum(eposS) + wU * deltauS
    

# q_car: 차량 출발 위치, q_park: 주차 위치, w~: 가중치, step_length: 한번 탐색 길이(이거는 알아내야 함), k: 비용 최대 허용 증가 배수 가중치
# iMax: 최대 경로 탐색 횟수, cntMax: 최대 경로 변경 횟수, epsilon: 근접 판정 허용치
def get_path_MPC(q_car, q_park, obstacle_points, step_length, wTHETA, wPOS, wU, k, iMax, cntMax, epsilon):
    # 결과 경로 리스트
    result = []
    # 변수 초기화
    q_i = q_car # 초기 시작점
    u_iminus1 = 0 # 초기 조정값은 0
    cost_i = np.inf
    D = 1 # 기본 진행 방향
    cnt = 0 # 진행 방향 횟수
    i = 0 # 경로 탐색 진행 횟수

    # 장애물 점의 집합 계산
    obstacles = get_points_from_dotlist(obstacle_points)

    # 경로 탐색
    while True:
        print("q_i: ", q_i)
        #경로 추가
        result.append(q_i[:2])


        # 모든 후보점 탐색, theta p_i 점에서 원을 회전시켜서 얻는 점들
        candi_us = np.array([u for u in np.linspace(-np.deg2rad(50), np.deg2rad(50), 100)])
        candi_qs = np.array([[q_i[0] + D*step_length*np.cos(q_i[2] + D*step_length*u/2), q_i[1] + D*step_length*np.sin(q_i[2] + D*step_length*u/2), q_i[2] + D*step_length*u] for u in np.linspace(-np.deg2rad(50), np.deg2rad(50), 100)])
        candi_costs = cost(candi_qs, q_park, candi_us, u_iminus1, wTHETA, wPOS, wU)

        # 후보점들 중 코스트가 최소인 q_iplus1 구하기
        q_iplus1_index = np.argmin(candi_costs)
        cost_iplus1 = candi_costs[q_iplus1_index]
        q_iplus1 = candi_qs[q_iplus1_index]
        u_i= candi_us[q_iplus1_index] # 최적 정책

        # 진행 방향 변경 지점 찾기(cost가 급격하게 상승할때, 최적 정책으로 진행하면 장애물을 만날때)
        if cost_iplus1 > k * cost_i and np.linalg.norm(obstacles - q_iplus1) < epsilon:
            D *= -1 # 방향 전환
            cnt += 1
        # 계속해서 다음 진행 방향 탐색
        else:
            q_i = q_iplus1
            u_iminus1 = u_i


        # 탐색 종료 시점 판별
        if np.linalg.norm(q_i[:2] - q_park[:2]) < epsilon or i > iMax or cnt > cntMax:
            break

    result = np.array(result).T
    return result[0], result[1]
