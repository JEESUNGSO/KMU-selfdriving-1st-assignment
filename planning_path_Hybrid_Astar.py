import math
import heapq
import matplotlib.pyplot as plt
import numpy as np

class Node:
    def __init__(self, x, y, theta, cost, parent):
        self.x = x
        self.y = y
        self.theta = theta
        self.cost = cost
        self.parent = parent

    def __lt__(self, other):
        return self.cost < other.cost

def cost_function(node, goal, w1, w2, w3):
    # 현재 위치와 목표 위치 사이의 거리 제곱에 대한 가중치
    position_error = math.sqrt((node.x - goal.x)**2 + (node.y - goal.y)**2)
    # 현재 방향과 목표 방향 사이의 각도 제곱에 대한 가중치
    angle_error = abs(node.theta - goal.theta) ** 2
    # 제어 입력 변화량에 대한 가중치 (기존 코드는 0으로 가정)
    control_input_variation = 0  # 현재 상태에서의 제어 입력 변화는 0으로 가정

    # 코스트 계산
    cost = w1 * position_error**2 + w2 * angle_error**2 + w3 * control_input_variation**2
    return cost

def is_collision(x, y, obstacles):
    # 범위를 벗어나지 않는지 확인
    if x < 0 or x > 20 or y < 0 or y > 20:
        return True

    for (ox, oy, r) in obstacles:
        if math.sqrt((x - ox)**2 + (y - oy)**2) <= r:
            return True
    return False

def next_state(current, v, delta, dt, L):
    x = current.x + v * math.cos(current.theta) * dt
    y = current.y + v * math.sin(current.theta) * dt
    theta = current.theta + (v / L) * math.tan(delta) * dt
    return Node(x, y, theta, current.cost + dt, current)

def hybrid_a_star(start, goal, v, delta_options, dt, L, max_steps, obstacles, w1, w2, w3, k, cntMax):
    open_list = []
    closed_list = set()
    heapq.heappush(open_list, (0, start))
    step = 0
    cnt = 0 # 방향 변경 횟수

    while open_list and step < max_steps:
        step += 1
        _, current = heapq.heappop(open_list)

        if math.sqrt((current.x - goal.x)**2 + (current.y - goal.y)**2) < 1.0:
            path = []
            while current:
                path.append(current)
                current = current.parent
            return path[::-1]

        closed_list.add((round(current.x, 1), round(current.y, 1), round(current.theta, 1)))

        collision_and_highdeltacost = True
        base_cost = cost_function(current, goal, w1, w2, w3)

        if cnt < cntMax:
            for delta in delta_options:
                next_node = next_state(current, v, delta, dt, L)
                if (round(next_node.x, 1), round(next_node.y, 1), round(next_node.theta, 1)) not in closed_list:
                    if not is_collision(next_node.x, next_node.y, obstacles):
                        collision_and_highdeltacost = False
                        cost = cost_function(next_node, goal, w1, w2, w3)
                        if cost <= k * base_cost:
                            heapq.heappush(open_list, (cost, next_node))

        # 자식 노드들이 모두 장애물에 부딪히거나 cost가 k배 이상 증가하는 경우 방향 전환
        if collision_and_highdeltacost:
            #v = -v
            #cnt += 1
            pass
        print(len(closed_list))
    return None

def plot_path(path, obstacles):
    for (ox, oy, r) in obstacles:
        circle = plt.Circle((ox, oy), r, color='r', fill=False)
        plt.gca().add_patch(circle)

    x_values = [node.x for node in path]
    y_values = [node.y for node in path]
    plt.plot(x_values, y_values, '-bo')

    plt.plot(x_values[0], y_values[0], 'go', label='Start')
    plt.plot(x_values[-1], y_values[-1], 'yo', label='Goal')

    plt.xlim([0, 20])  # x 축 제한
    plt.ylim([0, 20])  # y 축 제한

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Hybrid A* Path')
    plt.legend()
    plt.axis('equal')
    plt.grid(True)
    plt.show()

# 초기 상태 및 목표 상태 설정
start = Node(0, 0, 0, 0, None)
goal = Node(20, 20, 0, 0, None)
v = 1.0  # 속도
delta_options = np.linspace(np.deg2rad(-50), np.deg2rad(50), 10)  # 조향 각도 옵션
dt = 1  # 시간 간격
L = 2.0  # 휠베이스
max_steps = 10000  # 최대 탐색 단계
obstacles = [(5, 5, 3), (7, 8, 3)]  # 장애물 리스트 (x, y, 반지름)

# Cost 함수에 사용할 가중치 (조정 가능)
w1 = 10  # 현재 위치와 목표 위치의 거리 제곱에 대한 가중치
w2 = 0.00000000000000000000001  # 현재 방향과 목표 방향의 각도 제곱에 대한 가중치
w3 = 0.000000000000000000005  # 제어 입력 변화량과 정책 변화율에 대한 가중치
k = 1000000000  # cost 증가 허용 배수
cntMax = 10

# 하이브리드 A* 알고리즘 실행
path = hybrid_a_star(start, goal, v, delta_options, dt, L, max_steps, obstacles, w1, w2, w3, k, cntMax)

# 경로 출력
if path:
    plot_path(path, obstacles)
else:
    print("경로를 찾을 수 없습니다.")
