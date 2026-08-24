#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformListener


class TargetNavigator(Node):
    def __init__(self):
        super().__init__("target_navigator")
        self.declare_parameter("approach_offset_m", 0.7)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.action = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.busy = False
        self.sub = self.create_subscription(
            PoseStamped, "/object_nav/target_pose_camera", self.target_cb, 10
        )

    def target_cb(self, pose):
        if self.busy or not self.action.server_is_ready():
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", pose.header.frame_id, rclpy.time.Time()
            )
            target = do_transform_pose(pose, transform)
        except Exception as exc:
            self.get_logger().debug(f"target transform unavailable: {exc}")
            return
        target.pose.position.x -= float(self.get_parameter("approach_offset_m").value)
        goal = NavigateToPose.Goal()
        goal.pose = target
        self.busy = True
        future = self.action.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle or not handle.accepted:
            self.busy = False
            return
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda _: setattr(self, "busy", False))


def main(args=None):
    rclpy.init(args=args)
    node = TargetNavigator()
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
