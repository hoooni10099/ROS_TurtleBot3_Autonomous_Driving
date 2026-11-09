# 실행 명령 정리

실습 당시 바탕화면에 띄워 두고 순서대로 실행한 명령 목록(`final instruction.txt`)을 정리했습니다.
Remote PC(Ubuntu 20.04 · ROS Noetic)와 TurtleBot3 SBC가 같은 공유기에 연결되어 있고, `ROS_MASTER_URI`는 Remote PC를 가리킵니다.

## 공통 준비

```bash
# Remote PC
roscore

# SBC 접속 (IP는 실습 환경 기준)
ssh ubuntu@<TURTLEBOT_IP>
```

> 배터리를 먼저 충전해 둘 것. 모터가 돌지 않으면 OpenCR의 RESET을 누르고 브링업을 다시 실행.

## 1. SLAM & Navigation

```bash
# [SBC] 로봇 브링업 (OpenCR, LiDAR)
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# [Remote PC] Gmapping SLAM
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_slam turtlebot3_slam.launch

# [Remote PC] 키보드로 이동하며 지도 작성
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch

# [Remote PC] 지도 저장 → ~/map.pgm, ~/map.yaml
rosrun map_server map_saver -f ~/map

# [Remote PC] SLAM 종료 후 내비게이션
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
#   RViz: 2D Pose Estimate (초기 위치) → 2D Nav Goal (목표 지점)

# 점검
rqt          # Plugins > Topics > Topic Monitor
rqt_graph    # 노드 · 토픽 연결 구조
```

## 2. VisionDrive (차선 인식 + 내비게이션)

```bash
# [SBC] 카메라 노드
roslaunch turtlebot3_autorace_camera raspberry_pi_camera_publish.launch

# [Remote PC] 카메라 내부 보정 (체커보드)
roslaunch turtlebot3_autorace_camera intrinsic_camera_calibration.launch mode:=action

# [Remote PC] 카메라 외부 보정 (Bird's-eye 투영 영역)
roslaunch turtlebot3_autorace_camera extrinsic_camera_calibration.launch mode:=action

# [Remote PC] 차선 검출
roslaunch turtlebot3_autorace_detect detect_lane.launch mode:=action

# [SBC] 로봇 브링업
roslaunch turtlebot3_bringup turtlebot3_robot.launch

# [Remote PC] 영상 확인
rqt_image_view   # /camera/image/compressed, /detect/image_lane/compressed

# [Remote PC] 차선 주행
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch
```

### 트랙 매핑 → 통합 주행

```bash
# 차선 제어 OFF 상태에서 트랙 매핑
roslaunch turtlebot3_slam turtlebot3_slam.launch
rosrun map_server map_saver -f ~/map
# SLAM 종료

# 내비게이션 실행 후 차선 제어 다시 ON
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
roslaunch turtlebot3_autorace_driving turtlebot3_autorace_control_lane.launch
```

## 보정 값 저장 위치

| 단계 | 조정 도구 | 반영 파일 |
| --- | --- | --- |
| 카메라 밝기 · 대비 | `rqt_reconfigure` + `rqt_image_view` | `camera.yaml` |
| 투영 영역(Extrinsic) | `rqt_reconfigure` (`image_projection`) | `projection.yaml` |
| 차선 HSV 범위 | `rqt_reconfigure` (`detect_lane`) | `lane.yaml` |

`rqt_reconfigure`에서 바꾼 값은 재시작하면 사라지므로, 맞춘 값을 yaml에 옮겨 적은 뒤 `mode:=action`으로 다시 실행했습니다.
