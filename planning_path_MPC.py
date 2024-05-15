import numpy as np

#벡터 회전 함수
def global2local(local_init_q, q):
    R = np.array([[np.cos(-local_init_q.T[2]), -np.sin(-local_init_q.T[2])],
                  [np.sin(-local_init_q.T[2]), np.cos(-local_init_q.T[2])]])

    new_vec = R @ q.T[1:2]
    return np.array([new_vec[0], new_vec[1], q.T[2] - local_init_q.T[2]]).T

def eTheta(thetai, theta_goal):
    return thetai - theta_goal

def eP(xi, yi, gxy):
    return np.array([xi - gxy[0], yi - gxy[1]]).T

def du(ui_minus1, ui):
    return ui - ui_minus1

def qi_1(qi, ui, D):
    qix = qi.T[0]
    qiy = qi.T[1]
    qitheta = qi.T[2]

    steering_input = ui.T[0]
    step_length = ui.T[1]

    qi_1x = qix + D*step_length*np.cos(qitheta + D*steering_input*step_length/2.0)
    qi_1y = qiy + D*step_length*np.sin(qitheta + D*steering_input*step_length/2.0)
    qi_1theta = qitheta + D*steering_input*step_length

    return np.array([qi_1x, qi_1y, qi_1theta])



def get_path_MPC(s_pos, s_vec, end_pos, end_vec, RTHETA_A, R_A, RU_A, wall_front_middle_point):
    # 사용할 변수 및 상수
    # rtheta = 0
    # R = 0
    # phase = "B"
    cntMax = 30 #c 최대 방향전환 횟수
    k = 30 # 최대 비용 변화 허용치
    epsilon = 0.1 # 도착 허용오차
    iMax = 0 # 최대 경로 생성 시도 횟수
    cnt = 0  # 전진 후진 변경 횟수
    i = 0  # 경로 생성 시도 횟수
    cost_qi = np.inf # i번째 최소 코스트
    D = 1 # 초기 진행방향 1: 전지 -1: 후진



    # u, 제어값 [steering_input, step_length]
    ui_minus1 = np.array([0,0]).T
    
    # qi 현재 상태 qO 도착지 상태, [x, y, rad], 차량 좌표계 기준, 일단 각도는 차이를 구해서 쓰는것 같아서 글로벌 좌표계 기준으로 했음
    qi = np.array([0, 0, 0]).T # 차량 좌표계 기준
    qO = global2local(np.array([s_pos[0], s_pos[1], np.arctan2(s_vec[1], s_vec[0])]).T, np.array([end_pos[0], end_pos[1], np.arctan2(end_vec[1], end_vec[0])]).T) # 차량 좌표계 기준으로 변환

    # 벽 차량 기준 좌표
    wall_pos = global2local(np.array([s_pos[0], s_pos[1], np.arctan2(s_vec[1], s_vec[0])]).T, np.array(wall_front_middle_point + [0]).T) # 벽 좌표 list형식

    # 스텝 반복
    while True:
        # ==== 최적 정책 u* 구하기====
        e_theta_pi_1 = eTheta(qi.T[3] - qO.T[3]) # theta 에러값
        e_p_i_1 = eP(qi.T[0], qi.T[1], end_pos) # position 에러값

        # 테스트할 정책들 -50 < steering_input < 50 탐색 간격은 1º로, step_length = 50
        list_ui = [np.array([np.deg2rad(steering_input), 50]) for steering_input in np.arange(-50, 50, 1)]
        list_cost = []  # 코스트들 저장할 변수

        # 모든 정책들의 cost 구하기
        for ui in list_ui:
            d_u = du(ui, ui_minus1) # 정책 변화량

            # 정책을 조정해서 최소화할 cost값 계산
            cost_qi_1 = RTHETA_A * e_theta_pi_1**2 + e_p_i_1.T*R_A@e_p_i_1.T + RU_A * d_u**2

            # cost값 추가
            list_cost.append(cost_qi_1)

        #cost를 최소화 하는 u와 최소 cost 구하기
        index_min = np.argmin(list_cost)
        ui = list_ui[index_min]
        cost_qi_1 = list_cost[index_min]


        #====phase B to phase A 변경 이벤트 감지====
        # 이게 뭔지 아직 잘 모르겠음 일단 패스
        # if (차량 주차구역 접근 정도 거리 휴리스틱, 주차 종류별로 다름, 주차구역 기준 좌표계) and phase == "B":
        #     phase = "A"
        #     R = R_A
        #     rtheta = RTHETA_A


        #====전후진 변경 이벤트 감지====
        margin = 10 # 벽으로 부터 떨어진 거리 여유
        if (cost_qi_1 > k * cost_qi)or (wall_pos[0] + margin):
            D *= -1 # 전진 후진 변경
            cnt += 1 # 전진 후진 변경 횟수 추가

        #====다음스텝 업데이트====
        qi = qi_1(qi, ui, D)
        qO = global2local(qi, np.array([end_pos[0], end_pos[1], np.arctan2(end_vec[1], end_vec[0])]).T)  # 차량 좌표계 기준으로 변환
        wall_pos = global2local(qi, np.array(wall_front_middle_point + [0]).T)
        cost_qi = cost_qi_1

        #====경로탐색 끝 이벤트 감지====
        if (cnt > cntMax) or (i > iMax) or (np.linalg.norm(qi - qO) < epsilon):
            break

        i += 1 #시도횟수 추가
