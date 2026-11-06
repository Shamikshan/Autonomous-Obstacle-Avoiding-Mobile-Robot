#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


def create_pose(navigator: BasicNavigator, x: float, y: float, yaw_deg: float) -> PoseStamped:
    """Create a PoseStamped in the 'map' frame from (x, y, yaw_degrees)."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    yaw_rad = math.radians(yaw_deg)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
    pose.pose.orientation.w = math.cos(yaw_rad / 2.0)
    return pose


class TFMonitorNode(Node):
    """Helper node to query TF2 transforms (map -> base_footprint)."""
    def __init__(self):
        super().__init__('tf2_pose_monitor')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_robot_pose_in_map(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            return x, y
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None


def main(args=None):
    rclpy.init(args=args)

    navigator = BasicNavigator()
    tf_monitor = TFMonitorNode()

    navigator.get_logger().info("Waiting for Nav2 action servers to become active...")
    navigator.waitUntilNav2Active(localizer='controller_server')
    navigator.get_logger().info("Nav2 is ACTIVE! Starting Autonomous Obstacle-Avoiding Patrol...")

    # Define 4 patrol waypoints inside the 10m x 10m cluttered arena
    waypoints_coords = [
        (3.2, 2.8, 90.0),
        (-3.2, 2.8, 180.0),
        (-3.2, -3.0, -90.0),
        (3.2, -3.0, 0.0),
        (0.0, 0.0, 0.0),
    ]

    for idx, (wx, wy, wyaw) in enumerate(waypoints_coords, start=1):
        goal_pose = create_pose(navigator, wx, wy, wyaw)
        navigator.get_logger().info(
            f"---> Dispatching Waypoint {idx}/{len(waypoints_coords)}: (x={wx:.1f}, y={wy:.1f}, yaw={wyaw:.0f} deg)"
        )
        navigator.goToPose(goal_pose)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            rclpy.spin_once(tf_monitor, timeout_sec=0.1)
            feedback = navigator.getFeedback()

            if feedback and i % 10 == 0:
                tf_pose = tf_monitor.get_robot_pose_in_map()
                tf_str = f"TF2(map->base_footprint)=({tf_pose[0]:.2f}, {tf_pose[1]:.2f})" if tf_pose else "TF2=pending"
                navigator.get_logger().info(
                    f"[Waypoint {idx}] Distance remaining: {feedback.distance_remaining:.2f} m | {tf_str}"
                )

            time.sleep(0.1)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            navigator.get_logger().info(f"Waypoint {idx} reached successfully!")
        elif result == TaskResult.CANCELED:
            navigator.get_logger().warn(f"Waypoint {idx} was canceled.")
            break
        elif result == TaskResult.FAILED:
            navigator.get_logger().error(f"Waypoint {idx} failed! Proceeding to next waypoint...")

    navigator.get_logger().info("Patrol cycle completed!")
    tf_monitor.destroy_node()
    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()