2D Lidar와 ZED카메라의 fusion
ubuntu22.04 / ros2 humble
---------------------------------

1) 파일 구조
[your_ws] ----> src
                  |
                  |-----> lidar_row_data
                     |--> zed_lidar_fusion
                     |--> zed_row_video
                     |--> rplidar_ros2 (설치 필요)
                     |--> rplidar_sdk (설치 필요)
                     |--> sllidar_ros2 (설치 필요)
                     |--> zed-ros2-wrapper (설치 필요)
           

2) 필요 라이브러리

   - rplidar_ros2
     (https://github.com/babakhani/rplidar_ros2.git)
     
   - rplidar_sdk
     (https://github.com/Slamtec/rplidar_sdk.git)
     
   - sllidar_ros2
     (https://github.com/Slamtec/sllidar_ros2.git)
     
   - zed-ros2-wrapper
     (https://github.com/stereolabs/zed-ros2-wrapper.git)


3)  실행 순서
<img width="3180" height="1592" alt="image" src="https://github.com/user-attachments/assets/15f52809-912a-4e51-93a3-985c25e1db87" />

4) 실행 결과
<img width="1590" height="1266" alt="image" src="https://github.com/user-attachments/assets/55482f9c-8056-4eb5-8fc4-68315a42b09c" />


# 노트
카메라와 라이다의 캘리브레이션을 진행할 때 고정되어있는 이미지에서 특징점을 찾는것이 결과가 좋음. (ex, 사물함의 모서리)
본인은 라이다의 레이저와 사물이 만나는지점에 포스트잇을 붙여 이미지 좌표를 구했음. 
카메라와 라이다에서 획득한 2D, 3D포인트의 쌍을 최소 4개 이상 획득하여 solvepnp를 통해 extrinsic parameter를 획득하여 이미지 영상에 라이다의 point를 투영
