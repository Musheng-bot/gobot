#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import Image


class TargetDetector(Node):
    """Small deterministic baseline: red blob -> camera-frame target pose.

    Replace this node with a learned detector without changing ObjectNav's output
    contract: geometry_msgs/PoseStamped on /object_nav/target_pose.
    """

    def __init__(self):
        super().__init__("target_detector")
        self.declare_parameter("target_depth_m", 2.0)
        self.declare_parameter("min_area_px", 250.0)
        self.declare_parameter("fx_px", 554.0)
        self.declare_parameter("cx_px", 320.0)
        self.pub = self.create_publisher(PoseStamped, "/object_nav/target_pose_camera", 10)
        self.sub = self.create_subscription(Image, "/camera/image", self.image_cb, 5)

    def image_cb(self, msg):
        if msg.encoding not in ("bgr8", "rgb8"):
            return
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        frame = frame[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 100, 70), (10, 255, 255))
        mask |= cv2.inRange(hsv, (170, 100, 70), (179, 255, 255))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area < float(self.get_parameter("min_area_px").value):
            return
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return
        u = moments["m10"] / moments["m00"]
        depth = float(self.get_parameter("target_depth_m").value)
        fx = float(self.get_parameter("fx_px").value)
        cx = float(self.get_parameter("cx_px").value)
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = "camera_optical_frame"
        pose.pose.position.x = depth
        pose.pose.position.y = -(u - cx) * depth / fx
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = TargetDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
