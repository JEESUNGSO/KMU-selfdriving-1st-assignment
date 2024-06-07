"""
some codes is from https://github.com/nathanlct/reeds-shepp-curves

참고 논문:
OPTIMAL PATHS FOR A CAR THAT GOES BOTH FORWARDS AND BACKWARDS
J. A. REEDS AND L. A. SHEPP

옆 링크와 논문을 참고하여 공식을 적용했음 (cf http://msl.cs.uiuc.edu/~lavalle/cs326a/rs.c)

12개의 길찾기 함수(각 함수는 4가지 변형을 통해 다른 길찾기 방법을 만들어냄)는
3가지 x, y, pi인수를 가지고, (0, 0) 0도 에서 부터 도착지점까지의 경로는
PathElement로 움직임을 나타낸 리스트로 저장된다.

(실제로는 최적화 되었을 때 48개보다 적은 경우의 수를 갖지만 여기서는 48개를 모두 구한다.)
"""

from reed_shepp_utils import * # 계산시 필요한 함수들을 모아놓았음, 좌표계 변환, 각도 변환 등...
import math # 수학 함수 라이브러리
from enum import Enum # 값을 변수로 불러오게 할 수 있는 라이브러리
from dataclasses import dataclass, replace # 클래스를 데이터 저장 용도(param, steering, gear)로 사용하게 해주는 라이브러리


# 변수로써 아래 값들을 사용하게 해줌
class Steering(Enum):
    LEFT = -1
    RIGHT = 1
    STRAIGHT = 0


class Gear(Enum):
    FORWARD = 1
    BACKWARD = -1


# 경로를 가기위한 컨트롤 값들 저장
# steering이 STRAGIGHT일때     param = 얼마나 앞으로 갈지
# steering이 L, R 일때         param = 몇도나 회전할지
# param >= 0 이면              원래 gear대로
# param < 0 이면               방향 전환
@dataclass(eq=True)
class PathElement:
    param: float
    steering: Steering
    gear: Gear

    @classmethod
    def create(cls, param: float, steering: Steering, gear: Gear):
        if param >= 0:
            return cls(param, steering, gear)
        else:
            return cls(-param, steering, gear).reverse_gear()

    def __repr__(self):
        s = "{ Steering: " + self.steering.name + "\tGear: " + self.gear.name \
            + "\tdistance: " + str(round(self.param, 2)) + " }"
        return s

    def reverse_steering(self):
        steering = Steering(-self.steering.value)
        return replace(self, steering=steering)

    def reverse_gear(self):
        gear = Gear(-self.gear.value)
        return replace(self, gear=gear)


# 최적 경로를 찾기위해서 대체되어서 사용되는 각 path의 거리 
# 회전각도(rad)들과 직진 거리의 합으로 구성
def path_length(path):
    return sum([e.param for e in path])

# element들을 사용하여 위치를 이동하고 경로좌표와 나중 위치를 반화하는 함수
def move_next(current, element):
    gear = 1 if element.gear == Gear.FORWARD else -1
    if element.steering == Steering.LEFT:
        # gear * element.param 왼쪽회전 최소 회전반경으로 얼마나 돌지
        rxi, ryi, next_theta = get_list_L(current, gear * element.param, np.pi/180)
    elif element.steering == Steering.RIGHT:
        # gear * element.param 오른쪽회전 최소 회전반경으로 얼마나 돌지
        rxi, ryi, next_theta = get_list_R(current, gear * element.param, np.pi/180)
    elif element.steering == Steering.STRAIGHT:
        # gear * element.param 직선으로 얼마나 갈지
        rxi, ryi, next_theta = get_list_forward(current, gear * element.param, 0.01)


    # 경로 점들 및 다음위치 반환
    return rxi, ryi, (rxi[-1], ryi[-1], next_theta), gear


# 최적 경로 찾기, 가장 짧은 경로를 반환, 각 조작(C,L)들을 나누서 방향과 함께 나눠진 리스트로 반환 [rxi, ryi, geari]
def get_optimal_path(start, end, MAP, P_ENTRY, P_END, padding, TURNING_RADIUS):
    # turning_radius = 1인 기준으로 만들어진 함수
    turning_radius = 1

    # 최적 경로 찾기
    paths = get_all_paths(start, end)
    while True:
        collision = False
        optimal_path = paths.pop(paths.index(min(paths, key=path_length)))

        # print(optimal_path)

        # move_next의 current는 degree

        # element로 만든 경로의 집합 (rxi ,ryi, geari)
        rxy = []

        current = start # 현재 위치
        for element in optimal_path:
            # 다음 위치로 이동 및 current 값 업데이트
            rxi, ryi, current, geari = move_next(current, element)

            # 경로 리스트 추가
            rxy.append((rxi, ryi, geari))

            # 충돌이 발생하는지 확인
            if is_collision(rxi, ryi, MAP, P_ENTRY, P_END, padding, TURNING_RADIUS):
                collision = True
                break
        
        # 충돌 없는 최적경로 발견시
        if not collision:
            break
        else:
            print("충돌발생! 다음 최적 경로 탐색 합니다.")




    # 도착지점 주차 경로 추가
    # 마지막 경로의 방향이 주차진입 방향과 같을때
    if np.arctan2(rxy[-1][0][-2] - rxy[-1][0][-1], rxy[-1][1][-2] - rxy[-1][1][-1]) - M(deg2rad(end[2])) < np.pi / 2:  # pi/2는 대략적인 방향 확인값
        rxy.append((None, None, geari))
    else:
        rxy.append((None, None, -geari))



    return rxy



# 가능한 모든 후보 path들 구하기, start = (x, y, theta(rad))
def get_all_paths(start, end):
    # 베이스가 되는 12가지 path를 구하는 함수들
    path_fns = [path1, path2, path3, path4, path5, path6, \
                path7, path8, path9, path10, path11, path12]
    paths = []

    # 출발지점 기준 좌표계로 도착지점의 좌표 및 각도를 얻음
    x, y, theta = change_of_basis(start, end)

    for get_path in path_fns:
        # 베이스 path 에서 파생되는 4가지 경로들 구하기
        # 논문 참고하길 바람
        paths.append(get_path(x, y, theta))
        paths.append(timeflip(get_path(-x, y, -theta)))
        paths.append(reflect(get_path(x, -y, -theta)))
        paths.append(reflect(timeflip(get_path(-x, -y, theta))))

    # path의 이동이 없는 pathElement 필터 
    for i in range(len(paths)):
        paths[i] = list(filter(lambda e: e.param != 0, paths[i]))

    # 빈 path 제거
    paths = list(filter(None, paths))

    return paths

# 하나의 path에서 파생시킬 수 있는 다른 path들 구하기 위한 함수(timeflip, reflect)
def timeflip(path):
    """
    timeflip transform described around the end of the article
    """
    new_path = [e.reverse_gear() for e in path]
    return new_path


def reflect(path):
    """
    reflect transform described around the end of the article
    """
    new_path = [e.reverse_steering() for e in path]
    return new_path

#=======12가지 경우의 베이스 path를 구하는 함수들, 자세한건 논문 참고, 앞 숫자는 논문에 있는 공식 번호, 뒤 문자는 path가 지나는 원, 직선 조합==========
# 8.1 CSC
def path1(x, y, phi):
    phi = deg2rad(phi)
    path = []

    u, t = R(x - math.sin(phi), y - 1 + math.cos(phi))
    v = M(phi - t)

    path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
    path.append(PathElement.create(u, Steering.STRAIGHT, Gear.FORWARD))
    path.append(PathElement.create(v, Steering.LEFT, Gear.FORWARD))

    return path

# 8.2 CSC
def path2(x, y, phi):
    phi = M(deg2rad(phi))
    path = []

    rho, t1 = R(x + math.sin(phi), y - 1 - math.cos(phi))

    if rho * rho >= 4:
        u = math.sqrt(rho * rho - 4)
        t = M(t1 + math.atan2(2, u))
        v = M(t - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.FORWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.FORWARD))

    return path

# 8.3 C|C|C
def path3(x, y, phi):
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        A = math.acos(rho / 4)
        t = M(theta + math.pi/2 + A)
        u = M(math.pi - 2*A)
        v = M(phi - t - u)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.LEFT, Gear.FORWARD))

    return path

# 8.4 (1) C|CC
def path4(x, y, phi):
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        A = math.acos(rho / 4)
        t = M(theta + math.pi/2 + A)
        u = M(math.pi - 2*A)
        v = M(t + u - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.LEFT, Gear.BACKWARD))

    return path

# 8.4 (2) CC|C
def path5(x, y, phi):
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        u = math.acos(1 - rho*rho/8)
        A = math.asin(2 * math.sin(u) / rho)
        t = M(theta + math.pi/2 - A)
        v = M(t - u - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.FORWARD))
        path.append(PathElement.create(v, Steering.LEFT, Gear.BACKWARD))

    return path

# 8.7 CCu|CuC
def path6(x, y, phi):
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho <= 4:
        if rho <= 2:
            A = math.acos((rho + 2) / 4)
            t = M(theta + math.pi/2 + A)
            u = M(A)
            v = M(phi - t + 2*u)
        else:
            A = math.acos((rho - 2) / 4)
            t = M(theta + math.pi/2 - A)
            u = M(math.pi - A)
            v = M(phi - t + 2*u)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.LEFT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.BACKWARD))

    return path

# 8.8 C|CuCu|C
def path7(x, y, phi):
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)
    u1 = (20 - rho*rho) / 16

    if rho <= 6 and 0 <= u1 <= 1:
        u = math.acos(u1)
        A = math.asin(2 * math.sin(u) / rho)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(u, Steering.LEFT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.FORWARD))

    return path

#8.9 (1) C|C[pi/2]SC
def path8(x, y, phi):
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        u = math.sqrt(rho*rho - 4) - 2
        A = math.atan2(2, u+2)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi + math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.LEFT, Gear.BACKWARD))

    return path

# 8.9 (2) CSC[pi/2]|C
def path9(x, y, phi):
    phi = deg2rad(phi)
    path = []

    xi = x - math.sin(phi)
    eta = y - 1 + math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        u = math.sqrt(rho*rho - 4) - 2
        A = math.atan2(u+2, 2)
        t = M(theta + math.pi/2 - A)
        v = M(t - phi - math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.FORWARD))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.FORWARD))
        path.append(PathElement.create(v, Steering.LEFT, Gear.BACKWARD))

    return path

# 8.10 (1) C|C[pi/2]SC
def path10(x, y, phi):
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        t = M(theta + math.pi/2)
        u = rho - 2
        v = M(phi - t - math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.BACKWARD))

    return path

# 8.10 (2) CSC[pi/2]|C
def path11(x, y, phi):
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 2:
        t = M(theta)
        u = rho - 2
        v = M(phi - t - math.pi/2)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.FORWARD))
        path.append(PathElement.create(math.pi/2, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.BACKWARD))

    return path

# 8.11 C|C[pi/2]SC[pi/2]|C
def path12(x, y, phi):
    phi = deg2rad(phi)
    path = []

    xi = x + math.sin(phi)
    eta = y - 1 - math.cos(phi)
    rho, theta = R(xi, eta)

    if rho >= 4:
        u = math.sqrt(rho*rho - 4) - 4
        A = math.atan2(2, u+4)
        t = M(theta + math.pi/2 + A)
        v = M(t - phi)

        path.append(PathElement.create(t, Steering.LEFT, Gear.FORWARD))
        path.append(PathElement.create(math.pi/2, Steering.RIGHT, Gear.BACKWARD))
        path.append(PathElement.create(u, Steering.STRAIGHT, Gear.BACKWARD))
        path.append(PathElement.create(math.pi/2, Steering.LEFT, Gear.BACKWARD))
        path.append(PathElement.create(v, Steering.RIGHT, Gear.FORWARD))

    return path
