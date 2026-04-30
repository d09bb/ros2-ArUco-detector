# ros2-ArUco-detector

ROS 2 기반 ArUco 마커 인식 패키지.  
TurtleBot3의 자율주행 및 마커 기반 위치 판단 기능을 구현하기 위해 제작한다.

## 1. Project Overview

본 패키지는 카메라 이미지 토픽을 구독하여 ArUco 마커를 검출하고, 검출 결과를 ROS 2 토픽으로 발행한다.

주요 목적은 다음과 같다.

- Astra Camera 영상을 수신함.
- OpenCV 기반으로 ArUco 마커를 검출함.
- 검출된 마커의 위치 정보를 분석함.
- TurtleBot3 주행 판단 알고리즘과 연동함.
- 개신프론티어 프로젝트의 자율주행 기능 구현을 지원함.

## 2. Development Environment

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- OpenCV
- TurtleBot3
- Astra Camera
- Jetson Linux Environment

## 3. Package Structure

```text
aruco_detector/
├── CMakeLists.txt
├── package.xml
├── launch/
├── src/
└── README.md
```

## 4. ROS 2 Topics

### Subscribed Topic

```text
/camera/color/image_raw
```

카메라에서 입력되는 원본 이미지 토픽을 구독한다.

### Published Topics

```text
/aruco/image
/aruco/decision
/cmd_vel
```

- `/aruco/image` : ArUco 검출 결과가 표시된 이미지를 발행함.
- `/aruco/decision` : 마커 위치 기반 판단 결과를 발행함.
- `/cmd_vel` : TurtleBot3 주행 명령을 발행함.

## 5. Build

워크스페이스 실행 명령어

```bash
cd ~/Turtlebot3_4Way_OpenManipulater
colcon build --packages-select aruco_detector
source install/setup.bash
```

## 6. Run

Astra Camera 실행

```bash
ros2 launch astra_camera astra_mini.launch.py
```

ArUco detector 노드 실행

```bash
ros2 run aruco_detector aruco_detector_node
```

launch 파일을 사용하는 경우 다음 명령어 실행

```bash
ros2 launch aruco_detector aruco_detector.launch.py
```

## 7. Check Topics

마커 위치 기반 판단 결과를 확인한다.

```bash
ros2 topic echo /aruco/decision
```

TurtleBot3 주행 명령 토픽을 확인한다.

```bash
ros2 topic echo /cmd_vel
```

## 8. Repository

```text
https://github.com/d09bb/ros2-ArUco-detector
```
