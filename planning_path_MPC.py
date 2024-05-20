import math
import numpy as np

class VehicleState:
    def __init__(self, x, y, theta, direction):
        self.x = x
        self.y = y
        self.theta = theta
        self.direction = direction  # 전진(+1) 또는 후진(-1)

    def is_same(self, goal, epsilon):
        # 상태가 비슷하면
        if abs(self.x - goal.x) < epsilon and abs(self.y - goal.y) < epsilon and abs(self.theta - goal.theta) < np.pi/180:  #<---------------------------각도 차이
            return True
        # 상태가 안비슷하면
        return False

def cost_function(next_state, goal_state, steering_angle, previous_steering_angle, previous_step_length, step_length, distance_weight, angle_weight, step_length_weight, policy_weight):
    distance_cost = distance_weight * ((next_state.x - goal_state.x)**2 + (next_state.y - goal_state.y)**2)
    angle_cost = angle_weight * ((next_state.theta - goal_state.theta)**2)
    step_length_cost = ((previous_step_length - step_length) ** 2) * step_length_weight
    policy_cost = policy_weight * ((steering_angle - previous_steering_angle)**2)
    #print(f"distance_cost: {distance_cost}, angle_cost: {angle_cost}, policy_cost: {policy_cost}")
    return distance_cost + angle_cost + policy_cost + step_length_cost, distance_cost, angle_cost, policy_cost, step_length_cost

def is_obstacle(next_state, obstacles):
    # 장애물에 부딪히면
    for obstacle in obstacles:
        if np.sqrt((obstacle[0] -next_state.x)**2 + (obstacle[1] - next_state.y)**2) < obstacle[2]:
            return True

    # 경계에 부딪히면
    if next_state.x < 0 or next_state.x > 60 or next_state.y < 0 or next_state.y > 25:
        return True
    # 장애물에 안부딪히면
    return False

def bicycle_model(state, steering_angle, step_length, steering_mul):
    # new_x = state.x + state.direction * step_length * math.cos(state.theta + state.direction * steering_mul * np.deg2rad(steering_angle) / 2)
    # new_y = state.y + state.direction * step_length * math.sin(state.theta + state.direction * steering_mul * np.deg2rad(steering_angle) / 2)
    # new_theta = state.theta + state.direction * steering_mul * np.deg2rad(steering_angle) / 2
    # return VehicleState(new_x, new_y, new_theta, state.direction)

    new_x = state.x + state.direction * step_length * math.cos(state.theta + state.direction * step_length * np.deg2rad(steering_angle) / 2)
    new_y = state.y + state.direction * step_length * math.sin(state.theta + state.direction * step_length * np.deg2rad(steering_angle) / 2)
    new_theta = state.theta + state.direction * step_length * np.deg2rad(steering_angle)
    return VehicleState(new_x, new_y, new_theta, state.direction)

# # bicycle_model 테스트
# import matplotlib.pyplot as plt
#
# steering_angles = np.deg2rad(np.linspace(-50, 50, 10))
# a_state = VehicleState(20, 20, np.pi/1.4, 1)
# plt.scatter([a_state.x], [a_state.y])
# for steering_angle in steering_angles:
#     next_state = bicycle_model(a_state, steering_angle, 2, 40)
#     plt.scatter([next_state.x], [next_state.y])
#
# plt.axis([0, 50, 0, 40])
# plt.show()



def find_optimal_path(start, goal, obstacles, distance_weight, angle_weight, step_length_weight, policy_weight, k, iMax, cntMax, epsilon, steering_mul, max_step_length):
    steering_angles = np.linspace(-50, 50, num=10)  # 조향 각도 후보들
    step_lengths = np.linspace(1, max_step_length, 20)

    path = [[start.x, start.y, start.direction]]
    current_state = start
    previous_steering_angle = 0
    previous_step_length = 1
    best_next_state = None
    current_cost = np.inf

    # cost 값들 그래프로 표시하기 위해서
    distance_costs = []
    angle_costs = []
    policy_costs = []
    cost_ratios = []
    step_length_costs = []




    # 종료 감지 변수들
    i = 0 # 전체 길찾기 시도 횟수
    cnt = 0 # 방향 전환 횟수
    while True:

        is_direction_changed = False
        # 도착
        if current_state.is_same(goal, epsilon):
            # 도착 했으므로 탐색 종료
            print("찾았다 ㅎㅎ")
            return path, distance_costs, angle_costs, policy_costs, cost_ratios, step_length_costs
        elif i > iMax:
            # 길 못찾음
            print("최대 탐색 한도 넘음")
            return path, distance_costs, angle_costs, policy_costs, cost_ratios, step_length_costs
        elif cnt >= cntMax:
            # 길 못찾음
            print("너무 많은 방향 전환")
            return path, distance_costs, angle_costs, policy_costs, cost_ratios, step_length_costs



        # 모든 정책들 시도하기
        min_next_cost = np.inf
        for steering_angle in steering_angles:
            for step_length in step_lengths:
                next_state = bicycle_model(current_state, steering_angle, step_length, steering_mul)

                next_cost, distance_cost, angle_cost, policy_cost, step_length_cost = cost_function(next_state, goal, steering_angle, previous_step_length, step_length, previous_steering_angle, distance_weight, angle_weight, step_length_weight, policy_weight)

                if next_cost < min_next_cost:
                    min_next_cost = next_cost
                    best_next_state = next_state
                    best_steering_angle = steering_angle
                    best_step_legnth = step_length
                    best_next_distance_cost = distance_cost
                    best_next_angle_cost = angle_cost
                    best_next_policy_cost = policy_cost


        # cost 비율 추가
        cost_ratios.append(min_next_cost/current_cost)
        distance_costs.append(best_next_distance_cost)
        angle_costs.append(best_next_angle_cost)
        policy_costs.append(best_next_policy_cost)
        step_length_costs.append(step_length_cost)

        # 최적정책이 장애물을 만날때
        if is_obstacle(best_next_state, obstacles):
            print("최적정책에 장애물 감지!        -> 방향전환")
            current_state.direction *= -1  # 장애물 회피를 위해 방향 전환
            cnt += 1  # 방향 전환 횟수 추가
        # 최적정책 cost 값이 갑자기 증가할때
        elif min_next_cost/current_cost > k:
            print("너무 값이 큼!                -> 방향전환")
            current_state.direction *= -1 # 이 방향으로 가면 너무 돌아갈 길이 김
            cnt += 1 # 방향 전환 횟수 추가

        else:
            path.append([best_next_state.x, best_next_state.y, current_state.direction])
            current_state = best_next_state
            previous_steering_angle = best_steering_angle
            previous_step_length = best_step_legnth
            current_cost = min_next_cost


        i += 1 # 길찾기 시도횟수 추가


# 예제 사용법
obstacles = []
start = VehicleState(1, 1, 0, 1)
goal = VehicleState(55, 23, np.pi/4, 1)  # 도착 지점을 그리드 크기에 맞게 수정
distance_weight = 10
angle_weight = 400
step_length_weight = 0.0000001
policy_weight = 1
k = 200
epsilon = 1
iMax = 1000
cntMAx = 100
steering_mul = 0.8
max_step_length = 2

path, distance_costs, angle_costs, policy_costs, cost_ratios, step_length_costs = find_optimal_path(start, goal, obstacles, distance_weight, angle_weight, step_length_weight, policy_weight, k, iMax, cntMAx, epsilon, steering_mul, max_step_length)


# import matplotlib.pyplot as plt
#
# plt.figure(figsize=(6,12))
#
# #=======================맵 표시================================
# plt.subplot(6,1, 1)
# plt.axis([0, 60, 0, 25])
# # 출발, 도착지점 표시
# c = plt.Circle((start.x,start.y), 1, fc='w', ec='b')
# plt.gca().add_patch(c)
# c = plt.Circle((goal.x,goal.y), 1, fc='w', ec='b')
# plt.gca().add_patch(c)
# # 경로 표시
# path = np.array(path).T
# plt.plot(path[0], path[1], '-ro', markersize=2)
# # 장애물 표시
# for obstacle in obstacles:
#     c = plt.Circle(obstacle[:2], obstacle[2], fc='w', ec='b')
#     plt.gca().add_patch(c)
#
# #===================each cost====================
# #distance
# plt.subplot(6,1, 2)
# plt.plot(distance_costs, 'r')
# #angle
# plt.subplot(6,1, 3)
# plt.plot(angle_costs, 'g')
# #policy
# plt.subplot(6,1, 4)
# plt.plot(policy_costs, 'b')
# #step_length
# plt.subplot(6,1, 5)
# print(step_length_costs[3])
# plt.plot(step_length_costs, 'r')
#
# #===================cost ratio===================
# plt.subplot(6,1, 6)
# plt.plot(cost_ratios)
#
# plt.show()
