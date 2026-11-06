#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class LidarObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_avoidance')

        # Declare parameters
        self.declare_parameter('safe_distance', 0.75)
        self.declare_parameter('linear_speed', 0.28)
        self.declare_parameter('angular_speed', 0.85)
        self.declare_parameter('drive_mode', True)

        self.safe_dist = self.get_parameter('safe_distance').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.drive_mode = self.get_parameter('drive_mode').value

        # Subscriber to LIDAR /scan
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publisher to /cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        mode_str = "ACTIVE REACTIVE DRIVING" if self.drive_mode else "PASSIVE TELEMETRY MONITOR"
        self.get_logger().info(f"LIDAR Obstacle Avoidance Node Started [{mode_str}]")

    def clean_ranges(self, ranges, r_min, r_max):
        """Filter out NaN and Inf values and clamp to valid LIDAR range."""
        cleaned = []
        for r in ranges:
            if math.isnan(r) or math.isinf(r):
                cleaned.append(r_max)
            elif r < r_min:
                cleaned.append(r_max)
            else:
                cleaned.append(min(r, r_max))
        return cleaned

    def scan_callback(self, msg: LaserScan):
        ranges = self.clean_ranges(msg.ranges, msg.range_min, msg.range_max)
        n = len(ranges)
        if n == 0:
            return

        # Note: scan angle_min is -pi (back) and angle_max is +pi (back).
        # Index n//2 corresponds to 0.0 radians (straight ahead / FRONT).
        mid = n // 2
        sector_width = n // 12  # 30 degrees per half-sector

        front_sector = ranges[mid - sector_width : mid + sector_width]
        front_left_sector = ranges[mid + sector_width : mid + 3 * sector_width]
        front_right_sector = ranges[mid - 3 * sector_width : mid - sector_width]

        min_front = min(front_sector) if front_sector else msg.range_max
        min_fleft = min(front_left_sector) if front_left_sector else msg.range_max
        min_fright = min(front_right_sector) if front_right_sector else msg.range_max

        self.get_logger().info(
            f"LIDAR Sectors [m] -> Front: {min_front:.2f} | "
            f"F-Left: {min_fleft:.2f} | F-Right: {min_fright:.2f}",
            throttle_duration_sec=1.0
        )

        if not self.drive_mode:
            return

        twist = Twist()

        # Reactive state machine logic
        if min_front > self.safe_dist and min_fleft > (self.safe_dist * 0.8) and min_fright > (self.safe_dist * 0.8):
            # Path clear: move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0
        else:
            # Obstacle detected: stop linear motion and turn toward the more open side
            twist.linear.x = 0.0
            if min_fleft > min_fright:
                twist.angular.z = self.angular_speed   # Turn Left
            else:
                twist.angular.z = -self.angular_speed  # Turn Right

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.drive_mode:
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()