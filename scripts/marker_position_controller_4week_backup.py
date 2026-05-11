#!/usr/bin/env python3

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


class MarkerPositionController(Node):
    def __init__(self):
        super().__init__('marker_position_controller')

        self.image_topic = '/camera/color/image_raw'
        self.decision_topic = '/aruco/decision'
        self.cmd_vel_topic = '/cmd_vel'

        self.publish_cmd_vel = True

        self.deadband_px = 60
        self.stop_area_ratio = 0.12

        self.linear_speed = 0.04
        self.turn_speed = 0.18

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )

        self.decision_pub = self.create_publisher(String, self.decision_topic, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        try:
            self.parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
            self.use_new_api = True
        except AttributeError:
            self.parameters = cv2.aruco.DetectorParameters_create()
            self.detector = None
            self.use_new_api = False

        self.get_logger().info('Marker position controller started')
        self.get_logger().info(f'Subscribing image topic: {self.image_topic}')
        self.get_logger().info(f'Publishing decision topic: {self.decision_topic}')

    def detect_markers(self, gray):
        if self.use_new_api:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self.dictionary,
                parameters=self.parameters
            )
        return corners, ids

    def publish_decision(self, decision, marker_id=-1, cx=-1, error=0, area_ratio=0.0):
        msg = String()
        msg.data = (
            f'decision={decision}, '
            f'id={marker_id}, '
            f'cx={cx}, '
            f'error={error}, '
            f'area_ratio={area_ratio:.3f}'
        )
        self.decision_pub.publish(msg)

    def publish_cmd(self, decision):
        twist = Twist()

        if decision == 'CENTER':
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        elif decision == 'LEFT':
            twist.linear.x = 0.0
            twist.angular.z = self.turn_speed
        elif decision == 'RIGHT':
            twist.linear.x = 0.0
            twist.angular.z = -self.turn_speed
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {e}')
            return

        height, width, _ = frame.shape
        image_center_x = width // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect_markers(gray)

        if ids is None or len(ids) == 0:
            decision = 'LOST'
            self.publish_decision(decision)

            if self.publish_cmd_vel:
                self.publish_cmd(decision)
            return

        ids = ids.flatten()

        # 여러 마커가 보이면 가장 크게 보이는 마커를 선택
        areas = []
        for c in corners:
            area = cv2.contourArea(c[0])
            areas.append(area)

        selected_index = int(np.argmax(areas))

        marker_id = int(ids[selected_index])
        marker_corners = corners[selected_index][0]

        cx = int(np.mean(marker_corners[:, 0]))
        error = cx - image_center_x

        marker_area = cv2.contourArea(marker_corners)
        image_area = width * height
        area_ratio = marker_area / image_area

        if area_ratio >= self.stop_area_ratio:
            decision = 'STOP'
        elif error < -self.deadband_px:
            decision = 'LEFT'
        elif error > self.deadband_px:
            decision = 'RIGHT'
        else:
            decision = 'CENTER'

        self.publish_decision(decision, marker_id, cx, error, area_ratio)

        if self.publish_cmd_vel:
            self.publish_cmd(decision)

        self.get_logger().info(
            f'id={marker_id}, cx={cx}, error={error}, '
            f'area_ratio={area_ratio:.3f}, decision={decision}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerPositionController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
