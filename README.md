# 예선 첫번째 문제(Path Planning and Path Tracking)
----
## 1. Reed Shepp Path Planning (경로 계획 알고리즘)
### 기본적인 원리
- 차량의 회전 반경과 그 원들과 접선의 조합으로 한점에서 한점으로 이동하는 경로를 생성할 수 있다.
- 총 48가지의 호와 접선의 조합이 존재하지만 베이스가 되는 12가지 경우를 계산하고 대칭 시키는 등의 연산으로 48개를 구해낼 수 있다.

### 이 알고리즘의 장점
- 가장 직관적이어서 대략적인 개념은 이해하기 쉽다.
- 도착해야 하는 목표지점에 x, y, theta 모두 만족시키게 장애물만 없다면 무조건 도착할 수 있다.
- 가중치를 조절 해야할 필요가 없다.

### 이 알고리즘의 단점
- 항상 최소 회전반경으로 회전하기 때문에 부드러운 곡선으로 경로를 생성하기 힘들다.
- 수식을 구하는 과정이 복잡하다, (일단 아직 논문을 다 이해하기 전 생각)
- 두점 사이에 어느정도 길이의 장애물이 존재하면 경로를 찾을 수 없어 몇몇 랜드마크를 지나야 한다.

### 변수 설명

```python
#--------맵 정보와 관련된 상수----------------------------
P_ENTRY = (1036, 162) # 주차라인 진입 시점의 좌표
P_END = (1129, 69) # 주차라인 끝의 좌표
MAP = (1200, 850) # 맵 사이즈


#--------주차 방법----------------------------------------
IS_FRONT = 1 # 1: 전면 주차, 0: 후면주차

#--------경로 생성 관련 상수-------------------------------
ENTRY_MARGIN = 50 # 도착지점 추가 마진 거리
TURNING_RADIUS = 270 # 경로 생성시 참조할 차량의 회전 반경
padding = 100 / TURNING_RADIUS # 장애물(벽 및 주차 벽)의 부피 추가
```

### 사용 자료 및 논문
1. [reed shepp path by nathanlct](https://github.com/nathanlct/reeds-shepp-curves/tree/master)의 파이썬 코드를 이용하였음
2. reed shepp 알고리즘의 원래 저자의 [논문](https://projecteuclid.org/journals/pacific-journal-of-mathematics/volume-145/issue-2/Optimal-paths-for-a-car-that-goes-both-forwards-and/pjm/1102645450.full)

---
## 2. PID control Path Tracking (경로 추종 알고리즘)
### 대략 적인 개념 설명
- 개념은 이 [동영상](https://www.youtube.com/watch?v=4Y7zG48uHRo)에서 잘 설명해 주고 있다.
- P제어: 차이를 좁히게 만드는 제어
- I제어: 비슷하게 따라갈때 P제어가 천천히 수렴하는 것을 해결하기 위해 오차들을 적분 하여 하는 제어
- D제어: 외부의 힘에 의해 갑작스러운 변화가 있을때 원래상태로 빠르게 돌아가게 하기 위한 제어

### 변수 설명
```python
#--------경로 추종 관련 상수--------------------------------
MARGIN_LENGTH = 200 # 끝부분 마진 거리
MARGIN_INDEX = 40 # 마진 인덱스

#<-------PID컨트롤 관련 상수-------->
DIAMETER = 150 # PID 오류 탐색 원의 크기
Kp = 0.5 # p(오류) 가중치
Ki = 0.006 # i(적분) 가중치
Kd = 0.01 # d(미분) 가중치
# 위 가중치들의 조정을 통해 최적의 제어 식을 만들어 낸다.

#-------Detect Finish-------------------------
EPSILON_X = -6 # 전후 방향 허용 오차, 좀더 들어가도 되므로 음수까지(중앙을 지나쳐야) 가야 완벽히 초록불이 들어옴
```

## 3. 작동 영상 (2024 국민대 자율주행 경진대회 시뮬레이터)
![1번째결과](https://github.com/JEESUNGSO/KMU-selfdriving-1st-assignment/assets/166119462/b96e7d79-f6fc-4fa4-93c9-cf9b43037449)
![2번째결과](https://github.com/JEESUNGSO/KMU-selfdriving-1st-assignment/assets/166119462/8601ea4f-8243-406f-b774-fa4c2aeb1d1d)
![3번째결과](https://github.com/JEESUNGSO/KMU-selfdriving-1st-assignment/assets/166119462/7410c851-a649-4af3-8233-d19c4832b34e)
![4번째결과](https://github.com/JEESUNGSO/KMU-selfdriving-1st-assignment/assets/166119462/849f3a09-4089-41be-9fe7-1f981c93195f)
![5번째결과](https://github.com/JEESUNGSO/KMU-selfdriving-1st-assignment/assets/166119462/3c262e68-45bf-4b52-8eb1-288073dd1f03)
