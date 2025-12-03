#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch
from pybricks.robotics import DriveBase

# --- 1. 설정 및 초기화 ---
ev3 = EV3Brick()

# 모터 및 로봇 설정
left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
arm_motor = Motor(Port.B)

# [튜닝 포인트] wheel_diameter(바퀴지름), axle_track(바퀴간격) 확인 필요
robot = DriveBase(left_motor, right_motor, wheel_diameter=55.5, axle_track=104)
robot.settings(straight_speed=100, turn_rate=150) # 속도 설정

# 센서 설정
left_sensor = ColorSensor(Port.S1)      # 라인 트레이싱
right_sensor = ColorSensor(Port.S4)     # 교차로 감지
object_detector = ColorSensor(Port.S2)  # 물체 색상 인식
ultra_sensor = UltrasonicSensor(Port.S3) # 거리 감지

# 상수
BLACK_THRESHOLD = 35
KP = 1.2
DETECT_DIST = 100  # 100mm (10cm) 이내 감지

# --- 2. 기본 동작 함수 ---

def grab_object():
    """물체 잡기"""
    # 스톨될 때까지 모터를 돌려 꽉 잡음
    arm_motor.run_until_stalled(200, then=Stop.HOLD, duty_limit=60)

def release_object():
    """물체 놓기"""
    arm_motor.run_until_stalled(-200, then=Stop.COAST, duty_limit=60)

def line_follow_step(speed, kp):
    """라인 트레이싱 1스텝"""
    error = left_sensor.reflection() - BLACK_THRESHOLD + 10
    turn_rate = kp * error
    robot.drive(speed, turn_rate)

def left_line_following(speed):
    """
    [강의 자료 23p 참고] 왼쪽 센서를 이용한 P제어 주행
    - 왼쪽 센서가 검은색(선)을 보면 왼쪽으로, 흰색(바닥)을 보면 오른쪽으로 꺾음
    """
    # 1. 오차 계산 (현재 밝기 - 기준값)
    left_reflection = left_sensor.reflection()
    error = left_reflection - BLACK_THRESHOLD
    
    # 2. 회전량 계산 (P제어)
    turn_rate = KP * error
    
    # 3. 모터 구동 (직진 속도 + 회전량)
    robot.drive(speed, turn_rate)

def right_line_following(speed):
    """
    [강의 자료 23p 참고] 왼쪽 센서를 이용한 P제어 주행
    - 왼쪽 센서가 검은색(선)을 보면 왼쪽으로, 흰색(바닥)을 보면 오른쪽으로 꺾음
    """
    # 1. 오차 계산 (현재 밝기 - 기준값)
    right_reflection = right_sensor.reflection()
    error = right_reflection - BLACK_THRESHOLD
    
    # 2. 회전량 계산 (P제어)
    turn_rate = -KP * error
    
    # 3. 모터 구동 (직진 속도 + 회전량)
    robot.drive(speed, turn_rate)

def move_return_path(n, speed=100):
    """
    [신규] 돌아올 때 전용 이동 함수
    - '오른쪽 센서'로 주행하고, '왼쪽 센서'로 교차로(검은 선)를 감지함
    - ㅓ 자형 교차로 해결
    """
    for _ in range(n):

        # [단계 1] 현재 밟고 있는 검은 선 탈출 (왼쪽 센서 감시)
        # 출발지가 교차로 위라면, 왼쪽 센서가 흰색이 될 때까지 전진
        while left_sensor.reflection() < BLACK_THRESHOLD:
            right_line_following(speed) # <--- 주행은 오른쪽 센서로!

        # [단계 2] 다음 교차로 만날 때까지 주행
        # 왼쪽 센서가 흰색인 동안 계속 전진
        while left_sensor.reflection() > BLACK_THRESHOLD:
            right_line_following(speed) # <--- 주행은 오른쪽 센서로!
        
        # [단계 3] 교차로(왼쪽으로 뻗은 선) 감지됨 -> 정지 및 정렬
        ev3.speaker.beep()
        
        # 축 정렬을 위해 조금 더 전진
        robot.drive(speed, 0)
        wait(150) 

    robot.stop()

def move_n_cells(n, speed=100):
    """격자 n칸 이동"""
    for _ in range(n):
        # 검은 선 벗어나기
        while right_sensor.reflection() < BLACK_THRESHOLD:
             left_line_following(speed)
        # 다음 검은 선(교차로) 찾기
        while right_sensor.reflection() >= BLACK_THRESHOLD:
             left_line_following(speed)
        # 교차로 정렬
        robot.drive(speed, 0)
        wait(150)
    robot.stop()

# --- 3. 핵심 기능 함수 ---

def fetch_from_hub():
    """
    Hub에서 정면(동쪽)으로 이동하며 물체를 찾고,
    잡은 뒤 다시 Hub로 돌아오는 함수
    """
    ev3.speaker.say("Searching")
    
    robot.drive(150, 0)
    wait(200)

    while right_sensor.reflection() < BLACK_THRESHOLD:
         left_line_following(150)

    # 1. 물체 감지하며 전진
    # 최대 3칸 정도 거리 안에서 탐색 (무한정 가지 않도록 제한)
    distance_traveled = 0
    found = False

    lines_passed = 0   
    is_on_line = False # 라인 중복 카운트 방지
    
    # 예시: 5mm씩 전진하며 체크
    while distance_traveled < 1000: # 최대 1m 탐색
        if ultra_sensor.distance() < DETECT_DIST:
            found = True
            break

        if right_sensor.reflection() < BLACK_THRESHOLD:
            if not is_on_line: # 새로운 선을 밟음
                lines_passed += 1
                print("lines_passed = " , lines_passed)
                is_on_line = True
        else:
            is_on_line = False # 선을 벗어남

        # 라인 따라서 조금씩 전진
        left_line_following(100)
        wait(10)
        distance_traveled += (150 * 0.01) # 대략적인 거리 계산 (정확하진 않음)
        
    robot.stop()

    if found:
        
        ev3.speaker.beep()
        # 거리 보정 (센서와 집게 위치 차이)
        robot.straight(100) 
        grab_object()
        wait(500)
        
        # 2. Hub로 복귀
        # 180도 회전 (서쪽을 바라봄)
        robot.turn(180)
        
        return_cells = lines_passed +1
        
        print("return cell:" ,return_cells)
        # 계산된 거리만큼 복귀
        move_return_path(return_cells)

        return_cells = 0
        
        
        
    else:
        robot.turn(180)
        print("못찾음")
        move_return_path(lines_passed + 1)


    

def deliver_to_zone(color):
    """
    [요청하신 배달 함수]
    Hub(현재 서쪽을 봄) -> 색상 구역 이동 -> 놓기 -> Hub 복귀 -> 동쪽 보기
    """
    # 현재 로봇은 Fetch 후 돌아와서 '서쪽(START방향)'을 보고 있음
    
    # 1. 남쪽(아래쪽)으로 회전하여 구역 진입 준비
    robot.straight(30)
    robot.turn(-85) # 좌회전 -> 남쪽 보기

    
    
    
    if color == Color.RED:
        ev3.speaker.say("Red") 
        # 빨간 구역: 남쪽으로 1칸 이동 -> 우회전 -> 놓기
        move_n_cells(1)
        robot.straight(60)
        wait(200)
        robot.turn(90) # 우회전 (서쪽 보기)

        
        
        robot.straight(200) # 구역 진입
        release_object()
        robot.straight(-200) # 복귀
        
        

        # Hub로 복귀: 북쪽으로 1칸 이동
        robot.turn(90) # 뒤로 돌기 (북쪽 보기)



        move_n_cells(1) # Hub 도착
        
    elif color != Color.RED:
        ev3.speaker.say("Blue")
        # 파란 구역: 남쪽으로 2칸 이동 -> 우회전 -> 놓기
        move_n_cells(2)
        robot.straight(60)
        robot.turn(90)
        
        robot.straight(200)
        release_object()
        robot.straight(-200)
        
        # Hub로 복귀: 북쪽으로 2칸 이동
        robot.turn(90)
        
        move_n_cells(2) # Hub 도착

    else:
        # 색상 모름 -> 그냥 놓기
        release_object()
        robot.turn(180) # 북쪽 보기 (제자리)

    # 함수 종료 시점: 로봇은 Hub에 있으며 '북쪽'을 보고 있음
    # 다음 루틴(탐색)을 위해 '동쪽'을 보게 해야 함
    robot.straight(60)
    robot.turn(90) # 우회전 -> 동쪽 보기
    

# --- 4. 메인 실행 ---

def main():
    ev3.speaker.beep()
    # 버튼 대기
    while not Button.CENTER in ev3.buttons.pressed():
        wait(10)

    release_object() 
    wait(500)

    # [Step 1] START -> Hub (1칸 이동)
    # 초기 방향: 동쪽
    robot.straight(50) # 출발선 통과
    move_n_cells(1)    # 1칸 이동하여 Hub 도착
    
    # Hub 도착 완료. 현재 방향: 동쪽
    
    # [Step 2] 4회 반복 루틴
    for i in range(4):
        ev3.screen.print("Loop: " + str(i+1))
        
        # 1. 탐색 및 가져오기 (Hub -> Object -> Hub)
        # 이 함수가 끝나면 로봇은 Hub에서 '서쪽'을 보고 있음
        fetch_from_hub()
        
        # 2. 색상 확인
        detected_color = object_detector.color()
        
        # 3. 배달 및 복귀 (Hub -> Zone -> Hub)
        # 이 함수는 서쪽을 보는 상태에서 시작해서, 동쪽을 보는 상태로 끝남
        deliver_to_zone(detected_color)
        
        # 이제 로봇은 Hub에서 다시 동쪽을 보고 있으므로
        # 바로 다음 반복(fetch_from_hub) 실행 가능

    # [Step 3] 모든 작업 완료 후 종료
    ev3.speaker.say("Finish")
    # 필요하다면 START 지점으로 복귀하는 코드 추가

if __name__ == "__main__":
    main()
