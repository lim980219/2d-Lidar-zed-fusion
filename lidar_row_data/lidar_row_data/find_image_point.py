#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# 클릭한 좌표 저장용 리스트
clicked_points = []

def mouse_callback(event, x, y, flags, param):
    global clicked_points
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"사용자가 클릭한 좌표: ({x}, {y})")
        clicked_points.append((x, y))

def extract_points_from_bag(
    bag_path,
    image_topic,
    max_frames
):
    global clicked_points

    # rosbag reader 초기화
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3")
    converter_options = rosbag2_py.ConverterOptions("", "")
    reader.open(storage_options, converter_options)

    bridge = CvBridge()
    frame_count = 0

    # 토픽 타입 조회
    topics_and_types = reader.get_all_topics_and_types()
    topic_type_map = {t.name: t.type for t in topics_and_types}
    image_topic_type = topic_type_map.get(image_topic)
    if image_topic_type is None:
        available = ", ".join(sorted(topic_type_map.keys()))
        raise ValueError(f"지정한 이미지 토픽을 rosbag에서 찾을 수 없습니다: {image_topic}\n사용 가능 토픽: {available}")

    cv2.namedWindow("Image Viewer")
    cv2.setMouseCallback("Image Viewer", mouse_callback)

    while reader.has_next() and frame_count < max_frames:
        (topic, msg, t) = reader.read_next()
        if topic == image_topic:
            # 직렬화된 bytes → ROS2 메시지 역직렬화
            MsgType = get_message(image_topic_type)
            ros_msg = deserialize_message(msg, MsgType)

            # ROS Image → OpenCV 변환
            if image_topic_type.endswith("CompressedImage"):
                cv_img = bridge.compressed_imgmsg_to_cv2(ros_msg)
            else:
                cv_img = bridge.imgmsg_to_cv2(ros_msg, desired_encoding="bgr8")

            cv2.imshow("Image Viewer", cv_img)
            key = cv2.waitKey(500)  # 0.5초 대기 (원하면 0으로 변경 가능)
            frame_count += 1

    cv2.destroyAllWindows()
    print("사용자가 클릭한 모든 좌표:", clicked_points)


def main():
    # === 사용자 입력 ===
    bag_path = "/home/taewon/calib_bag"  # rosbag2 저장 경로
    image_topic = "/relay/raw_image"       # 이미지 토픽 이름

    extract_points_from_bag(bag_path, image_topic, max_frames=500)


if __name__ == "__main__":
    main()
