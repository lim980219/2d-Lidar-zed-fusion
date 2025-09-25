#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


class PointCloudRelayNode(Node):
    """ROS 2 노드: PointCloud2를 구독하여 그대로 재발행."""

    def __init__(self) -> None:
        super().__init__('pointcloud_relay')

        # 파라미터 선언 (기본 토픽명 제공)
        self.declare_parameter('input_topic', '/scan')
        self.declare_parameter('output_topic', '/scan_relay')

        self.input_topic: str = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic: str = self.get_parameter('output_topic').get_parameter_value().string_value

        # 퍼블리셔/서브스크라이버 생성 (센서 데이터 QoS 사용)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self._on_pointcloud,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"Subscribing: '{self.input_topic}' -> Republishing: '{self.output_topic}'"
        )

    def _on_pointcloud(self, msg: PointCloud2) -> None:
        # 메시지를 그대로 재발행 (타임스탬프/프레임 유지)
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PointCloudRelayNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


