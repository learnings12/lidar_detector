import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry


class PointNavigator(Node):
    def __init__(self):
        super().__init__('point_navigator')

        # Publishers and subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_danger = self.create_subscription(Bool, '/danger_flag', self.danger_callback, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)

        # State variables
        self.danger = False
        self.path_complete = False

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # Predefined waypoints (square + return home)
        self.waypoints = [
            (2.0, 1.0),
            (2.0, -1.0),
            (0.0, -1.0),
            (0.0, 0.0)   # ✅ home position
        ]
        self.current_goal_index = 0

        # Parameters
        self.linear_speed = 0.2    # m/s
        self.angular_speed = 0.5   # rad/s
        self.dist_tolerance = 0.1  # meters
        self.angle_tolerance = 0.1  # rad

    # -------------------- Callbacks --------------------

    def danger_callback(self, msg: Bool):
        self.danger = msg.data
        if self.danger:
            self.get_logger().warn("⚠️ Danger detected! Stopping robot.")

    def odom_callback(self, msg: Odometry):
        # Current position
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    # -------------------- Main Loop --------------------

    def timer_callback(self):
        if self.path_complete:
            return

        twist = Twist()

        if self.danger:
            # Stop immediately if obstacle detected
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info(f"Publishing /cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")
            return

        # Get current goal
        goal_x, goal_y = self.waypoints[self.current_goal_index]

        # Compute distance and angle to goal
        dx = goal_x - self.x
        dy = goal_y - self.y
        distance = math.sqrt(dx * dx + dy * dy)
        goal_angle = math.atan2(dy, dx)
        angle_error = goal_angle - self.yaw

        # Normalize angle error (-pi, pi)
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Debug logging
        self.get_logger().info(
            f"Waypoint {self.current_goal_index+1}/{len(self.waypoints)} "
            f"| Pos=({self.x:.2f},{self.y:.2f}), Goal=({goal_x:.2f},{goal_y:.2f}), "
            f"Dist={distance:.2f}, AngleErr={angle_error:.2f}"
        )

        if distance < self.dist_tolerance:
            # Goal reached
            self.get_logger().info(f"✅ Reached waypoint {self.current_goal_index+1}/{len(self.waypoints)}")

            self.current_goal_index += 1
            if self.current_goal_index >= len(self.waypoints):
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.path_complete = True
                self.get_logger().info("🎉 Path complete! Returned home.")
            else:
                self.get_logger().info(f"➡️ Moving to next waypoint {self.current_goal_index+1}")
        elif abs(angle_error) > self.angle_tolerance:
            # Rotate towards goal
            twist.linear.x = 0.0
            twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
        else:
            # Move forward
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        # ✅ Publish velocity and log it
        self.cmd_pub.publish(twist)
        self.get_logger().info(f"Publishing /cmd_vel: linear.x={twist.linear.x:.2f}, angular.z={twist.angular.z:.2f}")


def main():
    rclpy.init()
    node = PointNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
