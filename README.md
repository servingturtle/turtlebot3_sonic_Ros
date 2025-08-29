# TurtleBot3 Sonic ROS (4-Wheel Version)

TurtleBot3에 초음파 센서와 LED 피드백을 추가하고, **4개 모터로 동작하도록 수정**한 ROS2 펌웨어입니다.

## 기능

- **4개 모터 지원**: Front Left, Front Right, Rear Left, Rear Right 모터
- **초음파 센서 3개**: 좌측, 전방, 우측 거리 측정
- **RGB LED 피드백**: 초음파 센서 값에 따른 시각적 피드백
- **DYNAMIXEL Slave Protocol**: 라즈베리파이와의 통신
- **OpenManipulator 지원**: 로봇 팔 제어 및 모니터링
- **배터리 모니터링**: 전압 상태 확인 및 경고 알람

## 하드웨어 요구사항

- TurtleBot3 Burger/Waffle/Waffle Pi (4개 모터 버전)
- OpenCR 보드
- DYNAMIXEL 모터 4개 (ID: 1, 2, 3, 4)
- 초음파 센서 3개 (HC-SR04 또는 호환 모델)
- RGB LED (선택사항)

## 센서 연결

### 초음파 센서
- **좌측 센서**: 
  - Trig: D5
  - Echo: D2
- **전방 센서**: 
  - Trig: D6
  - Echo: D3
- **우측 센서**: 
  - Trig: D7
  - Echo: D4

### RGB LED
- **Red**: D9
- **Green**: D10
- **Blue**: D11

## 프로젝트 구조

```
turtlebot3_sonic_Ros/
├── turtlebot3_sonic_Ros.ino    # 메인 Arduino 스케치
├── include/                    # 헤더 파일들
│   ├── Ultrasonic.h           # 초음파 센서 헤더
│   ├── LED.h                  # LED 제어 헤더
│   └── turtlebot3/            # TurtleBot3 관련 헤더
│       ├── custom_turtlebot3.h
│       ├── custom_open_manipulator_driver.h
│       └── ...
├── src/                       # 소스 파일들
│   └── turtlebot3/
│       ├── Ultrasonic.cpp     # 초음파 센서 구현
│       ├── LED.cpp            # LED 제어 구현
│       ├── custom_turtlebot3.cpp
│       └── ...
└── README.md
```

## 컴파일 및 업로드

1. Arduino IDE에서 `turtlebot3_sonic_Ros.ino` 파일을 엽니다
2. OpenCR 보드를 선택합니다
3. 컴파일 후 업로드합니다

## Control Table 주소

### 모터 관련 주소 (4개 모터)
- **ADDR_PRESENT_POSITION_FL** (136): Front Left 모터 현재 위치
- **ADDR_PRESENT_POSITION_FR** (140): Front Right 모터 현재 위치
- **ADDR_PRESENT_POSITION_RL** (141): Rear Left 모터 현재 위치
- **ADDR_PRESENT_POSITION_RR** (142): Rear Right 모터 현재 위치

- **ADDR_PRESENT_VELOCITY_FL** (128): Front Left 모터 현재 속도
- **ADDR_PRESENT_VELOCITY_FR** (132): Front Right 모터 현재 속도
- **ADDR_PRESENT_VELOCITY_RL** (133): Rear Left 모터 현재 속도
- **ADDR_PRESENT_VELOCITY_RR** (134): Rear Right 모터 현재 속도

- **ADDR_PRESENT_CURRENT_FL** (120): Front Left 모터 현재 전류
- **ADDR_PRESENT_CURRENT_FR** (124): Front Right 모터 현재 전류
- **ADDR_PRESENT_CURRENT_RL** (125): Rear Left 모터 현재 전류
- **ADDR_PRESENT_CURRENT_RR** (126): Rear Right 모터 현재 전류

- **ADDR_PROFILE_ACC_FL** (174): Front Left 모터 프로파일 가속도
- **ADDR_PROFILE_ACC_FR** (178): Front Right 모터 프로파일 가속도
- **ADDR_PROFILE_ACC_RL** (179): Rear Left 모터 프로파일 가속도
- **ADDR_PROFILE_ACC_RR** (180): Rear Right 모터 프로파일 가속도

### 초음파 센서 주소
- **ADDR_ULTRASONIC_LEFT** (190): 좌측 센서 거리 (미터)
- **ADDR_ULTRASONIC_FRONT** (194): 전방 센서 거리 (미터)
- **ADDR_ULTRASONIC_RIGHT** (198): 우측 센서 거리 (미터)

## 라즈베리파이 연동

라즈베리파이에서는 DYNAMIXEL Slave Protocol을 통해 모터와 초음파 센서 데이터에 접근할 수 있습니다.

### 4개 모터 제어 예시
```python
# Python 예시 - 4개 모터 제어
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class FourWheelController(Node):
    def __init__(self):
        super().__init__('four_wheel_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
    def move_forward(self, speed=0.2):
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        
    def turn_left(self, angular_speed=0.5):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)
```

### 초음파 센서 데이터 접근
```python
# Python 예시 - 초음파 센서
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicSubscriber(Node):
    def __init__(self):
        super().__init__('ultrasonic_subscriber')
        self.left_sub = self.create_subscription(Range, '/ultrasonic/left', self.left_callback, 10)
        self.front_sub = self.create_subscription(Range, '/ultrasonic/front', self.front_callback, 10)
        self.right_sub = self.create_subscription(Range, '/ultrasonic/right', self.right_callback, 10)
    
    def left_callback(self, msg):
        self.get_logger().info(f'Left distance: {msg.range} m')
    
    def front_callback(self, msg):
        self.get_logger().info(f'Front distance: {msg.range} m')
    
    def right_callback(self, msg):
        self.get_logger().info(f'Right distance: {msg.range} m')
```

## LED 피드백

RGB LED는 초음파 센서 값에 따라 다음과 같이 동작합니다:

- **빨간색**: 장애물이 가까움 (거리 < 0.3m)
- **노란색**: 주의 필요 (거리 0.3m ~ 0.5m)
- **초록색**: 안전 (거리 > 0.5m)

## 4개 모터 설정

### 모터 ID 설정
4개 모터의 ID는 다음과 같이 설정되어 있습니다:
- **Front Left**: ID 1
- **Front Right**: ID 2  
- **Rear Left**: ID 3
- **Rear Right**: ID 4

### 모델 설정
Arduino 코드에서 모델을 "FourWheel"로 설정해야 합니다:
```cpp
// turtlebot3_sonic_Ros.ino
#define MODEL "FourWheel"
```

### 4-Wheel Differential Drive
이 펌웨어는 4-wheel differential drive kinematics를 사용하여 4개 모터를 제어합니다. 각 모터의 속도는 다음과 같이 계산됩니다:

- **Front Left**: `v_linear - v_angular * wheel_separation / 2`
- **Front Right**: `v_linear + v_angular * wheel_separation / 2`
- **Rear Left**: `v_linear - v_angular * wheel_separation / 2`
- **Rear Right**: `v_linear + v_angular * wheel_separation / 2`

## 문제 해결

### 모터가 연결되지 않는 경우
1. 모터 ID가 올바르게 설정되었는지 확인하세요 (1, 2, 3, 4)
2. 모터 전원이 공급되고 있는지 확인하세요
3. 통신 케이블이 올바르게 연결되었는지 확인하세요

### 부저가 주기적으로 울리는 경우
배터리 전압이 11.0V 미만으로 떨어져서 경고 알람이 울리고 있습니다. 배터리를 충전하세요.

### 컴파일 에러가 발생하는 경우
1. Arduino IDE 캐시를 정리하세요
2. OpenCR 라이브러리가 올바르게 설치되었는지 확인하세요
3. 파일 경로가 올바른지 확인하세요

## 라이선스

이 프로젝트는 Apache License 2.0 하에 배포됩니다.

## 기여

버그 리포트나 기능 요청은 GitHub Issues를 통해 제출해주세요.

