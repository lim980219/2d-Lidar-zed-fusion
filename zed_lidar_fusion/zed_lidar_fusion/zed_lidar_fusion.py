"""
2D 라이다(LaserScan)를 카메라 이미지에 투영하여 오버레이 이미지를 퍼블리시하는 노드.

절차:
1) LaserScan의 각 빔을 z=0 평면의 LiDAR 좌표계 3D 점으로 변환
2) 제공된 3D-2D 대응점으로 solvePnP로 LiDAR->Camera 외부 파라미터 추정
3) 카메라 내부/왜곡 파라미터로 전체 LiDAR 포인트를 이미지 평면에 투영
4) 최신 카메라 프레임에 포인트를 그려 `/fusion/overlay_image`로 퍼블리시
"""

import math
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge


class ZedLidarFusionNode(Node):
    def __init__(self) -> None:
        super().__init__('zed_lidar_fusion')
        # 1) 토픽/범위 관련 파라미터 선언 및 로드
        self._declare_topics_and_ranges()
        # 2) 카메라 내부/왜곡 파라미터 구성
        self._build_camera_intrinsics()
        # 3) PnP를 이용해 LiDAR->Camera 외부 파라미터 추정
        self._estimate_extrinsics_with_pnp()
        # 4) 브리지 및 이미지 버퍼 초기화
        self._init_buffers()
        # 5) 퍼블리셔/구독자 설정
        self._setup_ros_io()

    # ------------------------ 초기화 보조 메서드 ------------------------
    def _declare_topics_and_ranges(self) -> None:
        """토픽 이름과 사용 범위/샘플링 파라미터를 선언하고 읽어오기."""
        # 파라미터 선언 (토픽 및 동작)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('image_topic', 'relay/raw_image')
        self.declare_parameter('overlay_topic', '/fusion/overlay_image')
        self.declare_parameter('min_range', 0.05)
        self.declare_parameter('max_range', 30.0)
        self.declare_parameter('sample_step', 1)

        # 파라미터 로드
        self.scan_topic = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.overlay_topic = self.get_parameter('overlay_topic').get_parameter_value().string_value
        self.min_range = self.get_parameter('min_range').get_parameter_value().double_value
        self.max_range = self.get_parameter('max_range').get_parameter_value().double_value
        self.sample_step = int(self.get_parameter('sample_step').get_parameter_value().integer_value)

    def _build_camera_intrinsics(self) -> None:
        """카메라 내부 파라미터와 왜곡 계수를 구성하기."""
        # 카메라 내부/왜곡 파라미터 선언
        self.declare_parameter('fx', 1386.72)
        self.declare_parameter('fy', 1387.37)
        self.declare_parameter('cx', 971.801)
        self.declare_parameter('cy', 524.231)
        self.declare_parameter('k1', -0.15981219987635203)
        self.declare_parameter('k2', 0.008533436786969892)
        self.declare_parameter('p1', 0.0001220868823092627)
        self.declare_parameter('p2', -8.93159246483786e-05)
        self.declare_parameter('k3', 0.00863493861153135)

        # 파라미터 로드
        fx = float(self.get_parameter('fx').get_parameter_value().double_value)
        fy = float(self.get_parameter('fy').get_parameter_value().double_value)
        cx = float(self.get_parameter('cx').get_parameter_value().double_value)
        cy = float(self.get_parameter('cy').get_parameter_value().double_value)
        k1 = float(self.get_parameter('k1').get_parameter_value().double_value)
        k2 = float(self.get_parameter('k2').get_parameter_value().double_value)
        p1 = float(self.get_parameter('p1').get_parameter_value().double_value)
        p2 = float(self.get_parameter('p2').get_parameter_value().double_value)
        k3 = float(self.get_parameter('k3').get_parameter_value().double_value)

        # 카메라 행렬/왜곡 계수 구성
        self.camera_matrix = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.array([k1, k2, p1, p2, k3], dtype=np.float32)

    def _estimate_extrinsics_with_pnp(self) -> None:
        """제공된 3D-2D 대응점을 이용해 LiDAR->Camera 외부 파라미터(rvec/tvec)를 추정하기."""
        # 3D-2D 대응점 (LiDAR 좌표계의 3D 점 ↔ 이미지 픽셀)
        object_points = np.array([
            [-1.0365231, 0.91389835, 0.00055718],
            [-0.7284165, 0.78168476, 0.00441504],
            [-0.59901148, 1.14449489, -0.00134015],
            [-0.23015732, 1.08177292, 0.00107956],
        ], dtype=np.float32)
        image_points = np.array([
            [252.0, 207.0],
            [290.0, 212.0],
            [364.0, 210.0],
            [452.0, 215.0],
        ], dtype=np.float32)

        # PnP로 외부 파라미터 추정 (LiDAR->Camera)
        success, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

        # 결과 보관 및 실패 시 안전 가드
        if not success:
            self.get_logger().error('solvePnP 실패: 대응점을 확인하세요.')
            rvec = np.zeros((3, 1), dtype=np.float32)
            tvec = np.zeros((3, 1), dtype=np.float32)

        self.rvec = rvec
        self.tvec = tvec
        self.R, _ = cv2.Rodrigues(self.rvec) if success else (np.eye(3, dtype=np.float32), None)

    def _init_buffers(self) -> None:
        """cv_bridge와 최신 이미지 버퍼를 초기화하기."""
        self.bridge = CvBridge()
        self._latest_image = None
        self._latest_cv_image = None
        self._latest_image_shape = None  # (h, w)

    def _setup_ros_io(self) -> None:
        """퍼블리셔/구독자를 생성하고 상태를 로깅하기."""
        self.overlay_pub = self.create_publisher(Image, self.overlay_topic, 10)
        self.scan_sub = self.create_subscription(LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(Image, self.image_topic, self.on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f"Subscribing scan='{self.scan_topic}', image='{self.image_topic}' -> Publishing overlay='{self.overlay_topic}'"
        )


    # 카메라 이미지 수신 콜백
    def on_image(self, msg: Image) -> None:
        # 카메라 이미지 수신 콜백: 최신 프레임을 보관
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'cv_bridge 변환 실패: {exc}')
            return
        self._latest_image = msg
        self._latest_cv_image = cv_image
        self._latest_image_shape = (cv_image.shape[0], cv_image.shape[1])


    # LaserScan 수신 콜백
    def on_scan(self, msg: LaserScan) -> None:
        # 이미지가 아직 없으면 대기
        if self._latest_cv_image is None or self._latest_image is None:
            return

        # 1) LaserScan -> LiDAR 좌표계 3D 포인트(z=0)
        points_lidar = self._build_points_from_scan(msg)
        if points_lidar.size == 0:
            return

        # 2) 카메라 앞쪽(z>0) 포인트만 사용
        points_front = self._filter_points_in_front(points_lidar)
        if points_front.size == 0:
            return

        # 3) 이미지 좌표로 투영 후, 이미지 범위 내 포인트만 유지
        img_pts = self._project_points_to_image(points_front)
        img_pts = self._clip_to_image_bounds(img_pts)
        if img_pts.shape[0] == 0:
            return

        # 4) 오버레이 생성 및 퍼블리시
        overlay = self._draw_points_on_image(img_pts)
        out_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        out_msg.header = self._latest_image.header
        self.overlay_pub.publish(out_msg)

    # ------------------------ 투영/그리기 보조 메서드 ------------------------
    def _filter_points_in_front(self, points_lidar: np.ndarray) -> np.ndarray:
        """카메라 좌표계에서 z>0 인 포인트만 필터링하기."""
        points_cam = (self.R @ points_lidar.T).T + self.tvec.reshape(1, 3)
        z_cam = points_cam[:, 2]
        mask = z_cam > 0.0
        if not np.any(mask):
            return np.empty((0, 3), dtype=np.float32)
        return points_lidar[mask]

    def _project_points_to_image(self, points_lidar: np.ndarray) -> np.ndarray:
        """LiDAR 포인트를 이미지 좌표계(u,v)로 투영하기."""
        img_pts, _ = cv2.projectPoints(points_lidar, self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs)
        return img_pts.reshape(-1, 2)

    def _clip_to_image_bounds(self, img_pts: np.ndarray) -> np.ndarray:
        """이미지 크기(h, w)를 벗어나는 포인트를 제거하기."""
        if not self._latest_image_shape:
            return np.empty((0, 2), dtype=np.float32)
        height, width = self._latest_image_shape
        in_bounds = (
            (img_pts[:, 0] >= 0)
            & (img_pts[:, 0] < width)
            & (img_pts[:, 1] >= 0)
            & (img_pts[:, 1] < height)
        )
        return img_pts[in_bounds]

    def _draw_points_on_image(self, img_pts: np.ndarray) -> np.ndarray:
        """현재 이미지에 포인트를 작은 원으로 그려 오버레이 이미지를 반환하기."""
        overlay = self._latest_cv_image.copy()
        for x_f, y_f in img_pts:
            x_i = int(round(float(x_f)))
            y_i = int(round(float(y_f)))
            cv2.circle(overlay, (x_i, y_i), 2, (0, 255, 0), -1)
        return overlay



    # LaserScan을 LiDAR 좌표(z=0) 3D 포인트 배열 (N,3)로 변환
    def _build_points_from_scan(self, scan: LaserScan) -> np.ndarray:
        angle = scan.angle_min
        angle_increment = scan.angle_increment
        ranges = scan.ranges
        min_r = self.min_range
        max_r = self.max_range
        step = max(1, self.sample_step)

        points: List[Tuple[float, float, float]] = []
        for i in range(0, len(ranges), step):
            r = ranges[i]
            if not math.isfinite(r) or r < min_r or r > max_r:
                angle += angle_increment * step
                continue
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append((x, y, 0.0))
            angle += angle_increment * step

        if not points:
            return np.empty((0, 3), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ZedLidarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
