# ROS2_AIBOT
# AI 후송 BOT

## 프로젝트 목적
우리나라는 북한과의 대치 상황에서 50만명의 군인이 필요하지만 줄어드는 출산율에 의해 가까운 미래에 많은 수의 군인이 부족할 것이다. 이러한 문제점은 군인을 대신할 수 있는 로봇을 개발하여 인력 문제를 해결할 수 있다. 로봇이 할 수 있는 군인의 역할 중에 차량을 운전하여 아군을 후송하는 기능을 구현하여 미래의 인력 부족 사태에 대비한다.

## 개발 기간
2024.03.08 ~ 2024.04.19

## 사용 언어
C, C++, Python

## 사용 서비스와 패키지
Ubunutu22.04, ROS2 Humble, Nav2, Ros_Control, Arduino IDE

## 사용 부품
Raspberry Pi 4B+, Arduino Uno, Ydlidar x4 Pro, L298N module, DC Motor, Encoder

## 로봇 이미지
![field](https://github.com/Mrbinggrae/ros_rccar/blob/main/images/%ED%9B%84%EC%86%A1%EB%A1%9C%EB%B4%87.jpg)

## 필드 이미지
![field](https://github.com/Mrbinggrae/ros_rccar/blob/bc913bfa897627aa02213c14ac80c0dc589e0fef/images/Field.jpg)

## 시연 설명
1. RIVZ2에서 Mapping된 지도에 Waypoint를 설정하면 로봇이 이동하며 부상당한 아군을 후송한 뒤 베이스로 복귀합니다.
2. Waypoint에 도착하면 0.2초 간 휴식을 가진 후 다음 Waypoint로 이동합니다.
3. 로봇은 5rad 회전마다 또는 7.5cm 이동마다 월드맵과 로봇의 위치를 동기화합니다.
4. 로봇은 장애물으로 부터 18cm 까지 접근할 수 있습니다.
5. 그 외에 목표치 도달 허용 범위, 이동 속도, 회전 속도 등의 파라미터 값을 yaml 또는 launch 파일을 이용해서 설정할 수 있습니다.


## 시연 영상
[![Video Label](http://img.youtube.com/vi/VdPso3BwWHI/0.jpg)](https://youtu.be/VdPso3BwWHI)

위 이미지를 클릭하면 시연 영상을 볼 수 있습니다.



## 구현 설명
### 1. 로봇 정의
  - docbot.urdf.xacro 파일에 로봇의 사양을 입력했습니다.
  - properties.xacro 파일에 각각의 변수와 값을 입력하여 설정값을 직관적이고 수정하기 편한 환경을 구축했습니다.
  - docbot_ros2_control.xacro 파일에는 ros_control에 대한 파라미터 값을 설정했습니다.
  - inertial_macros.xacro에는 형태(원, 원뿔, 사각형)의 관성에 대한 메크로를 만들어 docbot.urdf.xacro에서 무게, 위치 값만 입력하여 로봇의 관성이 적용되도록 했습니다.
  - 위의 내용은 아래의 링크에서 상세한 코드 내용을 살펴볼 수 있습니다.
    
    [Docbot_Description 링크](https://github.com/Mrbinggrae/ROS2_AIBOT/tree/main/docbot_description)
   
### 2. 로봇 이동
  - ROS_Controller 패키지를 활용하여 joint_state_broadcaster/JointStateBroadcaster, diff_drive_controller/DiffDriveController토픽을 퍼블리싱했습니다.
  - teleop_twist_keyboard, joy_teleop 패지키를 활용하여 키보드와 PS4 조이스틱으로 로봇 컨트롤러 구현했습니다.
  - Hardware Interface를 이용하여 시리얼 통신으로 아두이노에게 이동, 회전 명령을 지시합니다.
  - 모터의 속도를 Encoder로 측정하여 PID를 통해 보정치를 적용했습니다.
  - Encoder로 측정한 바퀴의 속도를 각속도로 변환하여 Hardware Interface를 이용하여 시리얼 통신으로 로봇의 바퀴 관절과 위치 정보를 업데이트합니다.
  - 위의 내용은 아래의 링크에서 상세한 코드 내용을 살펴볼 수 있습니다.
    
    [Docbot_Controller 링크](https://github.com/Mrbinggrae/ROS2_AIBOT/tree/main/docbot_controller)

    [Docbot_Firmware 링크](https://github.com/Mrbinggrae/ROS2_AIBOT/tree/main/docbot_firmware)

  - docbot_flyup.launch.py 실팽 파일에 로봇의 Controller, Joystick, Hardware_interface의 실행 파일이 하나의 명령어로 실행되도록 만들었습니다.
    
    [docbot_flyup.launch 링크](https://github.com/Mrbinggrae/ros_rccar/blob/main/docbot_bringup/launch/docbot_flyup.launch.py)

### 3. Slam - 지도 제작
  - toolbox 패키지에서 mapper_params_online_async.yaml 파일을 복사했습니다.
  - tf2 노드, rviz2 노드, slam_toolbox 노드를 실행하는 slam_launch.py 파일을 만들었습니다.
  - slam_launch.py를 실행하고 로봇을 이동시켜 mapping을 진행하고 pgm, yaml, data, posegraph 파일을 생성합니다. 


### 4. Navigation 및 Localization
  - nav2_bringup 패키지에서 nav2_params.yaml, navigation_launch.py, localization_launch.py 파일을 복사합니다.
  - nav2_params.yaml에서 로봇의 속도, 장애물에 접근할 수 있는 범위 등의 수치를 설정합니다.
  - docbot_nav.launch.py 파일에서 navigation_launch.py, localization_launch.py가 실행되도록 하고 관련된 파라미터값을 입력할 수 있게 구현했습니다.
  - 위의 Slam과 Navigation, Localization에 대한 상세한 코드 내용을 아래의 링크에서 살펴 볼 수 있습니다.

    [Docbot_Navigation 링크](https://github.com/Mrbinggrae/ROS2_AIBOT/tree/main/docbot_navgation)
