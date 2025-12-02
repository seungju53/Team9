#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait
from pybricks.robotics import DriveBase
from collections import deque

# --- 1. 설정 및 초기화 ---
ev3 = EV3Brick()

# 모터 및 로봇 설정
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
arm_motor = Motor(Port.B)

# [튜닝 포인트 1] 90도 회전 각도 (102~106 조절)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
robot.settings(straight_speed=100, turn_rate=100)

# 센서 설정
left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S4)
object_detector = ColorSensor(Port.S2)
ultra_sensor = UltrasonicSensor(Port.S3)

# 상수
THRESHOLD = 50
KP = 1.2
SPEED = 100
SENSOR_OFFSET = 55  # 교차로 감지 후 센터까지 더 가는 거리

# [핵심 수정] 눈 감고 주행하는 시간 (ms)
# 1000~1500 사이 추천. 1.2초 동안은 센서 무시하고 직진하여 오작동 방지
BLIND_DRIVE_TIME = 1200 

# 방향 상수
NORTH, EAST, SOUTH, WEST = 0, 1, 2, 3
DIR_NAMES = ["North", "East", "South", "West"]
TURN_TABLE = [0, 90, 190, -90]

# 맵 그래프
GRAPH = {
    1: [2, 5],
    2: [1, 6],
    5: [1, 6, 9],
    6: [2, 5, 10],
    9: [5, 10],
    10: [6, 9, 11],
    11: [10, 12],
    12: [11]
}

NODE_COORDS = {
    1: (0, 0), 2: (1, 0),
    5: (0, 1), 6: (1, 1),
    9: (0, 2), 10: (1, 2),
    11: (2, 2), 12: (2, 3)
}

# 전역 변수
visited_nodes = []
current_node = 0
current_dir = EAST

# --- 2. 기본 주행 함수 ---

def left_line_following(speed, kp):
    reflection = left_sensor.reflection()
    error = reflection - THRESHOLD
    turn_rate = kp * error
    robot.drive(speed, turn_rate)

def right_line_following(speed, kp):
    reflection = right_sensor.reflection()
    error = reflection - THRESHOLD
    turn_rate = kp * error
    robot.drive(speed, turn_rate)

def n_move(n, direction="right"):
    """n칸 이동 (Blind Drive 적용)"""
    for i in range(n):
        # 1. 출발 직후 일정 시간은 무조건 직진 (라인 탈출 및 오인식 방지)
        robot.drive(SPEED, 0)
        wait(BLIND_DRIVE_TIME) 
        
        # 2. 센서 감지 시작
        if direction == 'right':
            while right_sensor.reflection() > 55:
                left_line_following(SPEED, KP)
            while right_sensor.reflection() <= 55:
                right_line_following(SPEED, KP)
        elif direction == 'left':
            while left_sensor.reflection() > 50:
                right_line_following(SPEED, KP)
            while left_sensor.reflection() <= 50:
                left_line_following(SPEED, KP)
        
        # 교차로 중앙 정렬
        robot.straight(SENSOR_OFFSET)
    robot.stop()

def grab_object():
    arm_motor.run_until_stalled(200, then=Stop.COAST, duty_limit=80)

def release_object():
    arm_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=50)

# --- 3. 정밀 주행 및 회전 로직 ---

def turn_to_direction(target_dir):
    global current_dir
    diff = (target_dir - current_dir) % 4
    angle = TURN_TABLE[diff]
    print("TURN:", DIR_NAMES[current_dir], "->", DIR_NAMES[target_dir], "(", angle, ")")
    if angle != 0: robot.turn(angle)
    current_dir = target_dir

def move_one_step(next_node, check_sensor=True):
    """한 칸 이동 (Blind Drive + 센서 감지)"""
    global current_node, current_dir
    
    # 1. 방향 계산 및 회전
    curr_x, curr_y = NODE_COORDS[current_node]
    next_x, next_y = NODE_COORDS[next_node]
    dx, dy = next_x - curr_x, next_y - curr_y
    
    target_dir = current_dir
    if dx == 1: target_dir = EAST
    elif dx == -1: target_dir = WEST
    elif dy == 1: target_dir = SOUTH
    elif dy == -1: target_dir = NORTH
    
    print("MOVE:", current_node, "->", next_node)
    turn_to_direction(target_dir)
    
    status = "ARRIVED"
    
    # [핵심 수정] 2. 눈 감고 주행 (Blind Drive)
    # 회전 후 바로 센서를 켜면 그림자 등을 오인식하므로, 
    # 일정 거리(시간)는 무조건 직진해서 다음 노드 근처로 보냄
    robot.drive(SPEED, 0)
    wait(BLIND_DRIVE_TIME) # 약 1.2초간 센서 무시하고 직진
    
    # 3. 센서 감지 시작
    
    # (A) 흰색 구간 (물체 감지)
    # 이미 BLIND 시간 동안 꽤 왔으므로 흰색 구간이 짧거나 없을 수도 있음
    while right_sensor.reflection() > 55:
        left_line_following(SPEED, KP)
        # 물체 감지는 계속 수행
        if check_sensor and ultra_sensor.distance() < 50:
            robot.stop()
            return "FOUND"

    # (B) 검은색 구간 (교차로 도착)
    while right_sensor.reflection() <= 55:
        right_line_following(SPEED, KP)
    
    # 교차로 중앙 정렬
    robot.straight(SENSOR_OFFSET)
    
    robot.stop()
    current_node = next_node
    return status

# --- 4. 복귀 및 하차 로직 ---

def return_to_previous_node_after_grab():
    global current_dir
    print("Returning to previous node...")
    
    # 180도 회전
    reverse_dir = (current_dir + 2) % 4
    robot.turn(190) 
    current_dir = reverse_dir 
    
    # 돌아갈 때도 Blind Drive 적용된 n_move 사용
    n_move(1, direction="right")
    print("Returned to Node:", current_node)

def execute_drop_logic(color):
    global current_dir, current_node
    
    turn_to_direction(WEST) # Start 방향 보기
    
    if color == Color.RED:
        ev3.speaker.say("Red")
        robot.turn(-90) # South
        n_move(1, direction='right')
        robot.turn(9