# 2D LiDAR & ZED Camera Fusion
**Ubuntu 22.04 / ROS2 Humble**

본 프로젝트는 **2D LiDAR (RPLiDAR / SLLiDAR)**와 **ZED Stereo Camera**를 활용하여 센서 데이터를 동기화하고, 카메라 영상 위에 LiDAR 포인트를 투영(fusion)하는 ROS2 기반 패키지입니다.  

---

## 📂 프로젝트 구조

```plaintext
[your_ws]
 └── src
     ├── lidar_row_data
     ├── zed_lidar_fusion
     ├── zed_row_video
     ├── rplidar_ros2        # (외부 설치 필요)
     ├── rplidar_sdk         # (외부 설치 필요)
     ├── sllidar_ros2        # (외부 설치 필요)
     └── zed-ros2-wrapper    # (외부 설치 필요)

## 1) 워크스페이스 생성 및 이동
mkdir -p ~/your_ws/src
cd ~/your_ws/src

## 2) 필요한 패키지 clone
git clone https://github.com/babakhani/rplidar_ros2.git
git clone https://github.com/Slamtec/rplidar_sdk.git
git clone https://github.com/Slamtec/sllidar_ros2.git
git clone https://github.com/stereolabs/zed-ros2-wrapper.git

## 3) 빌드
cd ~/your_ws
colcon build
source install/setup.bash

## 🚀 실행 방법
### 1) LiDAR Driver 실행
ros2 launch rplidar_ros2 rplidar_launch.py

### 2) ZED 카메라 실행
ros2 launch zed_wrapper zed_camera.launch.py

### 3) Fusion 노드 실행
ros2 run zed_lidar_fusion fusion_node

## 📌 실행 순서 다이어그램
<img width="3180" height="1592" alt="image" src="https://github.com/user-attachments/assets/15f52809-912a-4e51-93a3-985c25e1db87" />

## 🎯 실행 결과
LiDAR 포인트가 ZED 카메라 영상 위에 투영된 모습
<img width="1590" height="1266" alt="image" src="https://github.com/user-attachments/assets/55482f9c-8056-4eb5-8fc4-68315a42b09c" />


## 📐 캘리브레이션 방법

카메라와 라이다의 좌표계를 일치시키기 위해 Extrinsic Parameter를 추정해야 합니다.
최소 4쌍 이상의 대응점이 필요합니다. (라이다 포인트 ↔ 카메라 이미지 좌표)

### 권장 방법
1. 라이다 레이저가 사물과 만나는 지점에 포스트잇 부착 → 카메라에서 쉽게 확인 가능
2. 고정된 이미지(예: 사물함 모서리)에서 특징점을 선택하면 정확도 향상
3. 대응점 쌍을 이용해 solvePnP로 Extrinsic Parameter (R, t) 계산
4. 변환된 포인트를 카메라 영상 좌표계로 투영
