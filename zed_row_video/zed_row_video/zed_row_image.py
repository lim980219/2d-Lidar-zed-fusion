
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class ZEDImageRelay(Node):
    def __init__(self):
        super().__init__('zed_image_relay')

        # ZED 카메라 원본 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            '/zed/zed_node/rgb/image_rect_color',
            self.listener_callback,
            10)

        # 새로운 토픽으로 publish
        self.publisher = self.create_publisher(
            Image,
            'relay/raw_image',
            10)


    def listener_callback(self, msg: Image):
        # 그대로 publish
        self.publisher.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = ZEDImageRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
