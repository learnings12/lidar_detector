import rclpy 
import math 

from rclpy.node import Node 
from sensor_msgs.msg import LaserScan

class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        # Subscribe to lidar scan topic
        self.sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, 10 
        )

        # Parameters for grouping beams 
        self.angle_threshold = 5.0      # deg, max gap in angle
        self.distance_threshold = 0.3   # meters, max dist change allowed
        self.safety_radius =   2      # meters, minimum safe distance

    def lidar_callback(self, msg:LaserScan):
        angle = msg.angle_min
        obstacles = []
        current_cluster = []

        for r in msg.ranges:
            if math.isinf(r):
                angle += msg.angle_increment
                continue 

            dist = r
            ang_deg = math.degrees(angle)

            if not current_cluster:
                current_cluster.append((dist, ang_deg))
            else:
                prev_dist, prev_ang = current_cluster[-1]
                if (abs(ang_deg - prev_ang) <= self.angle_threshold
                        and abs(dist - prev_dist) <= self.distance_threshold):
                    current_cluster.append((dist, ang_deg))    
                else:
                    obstacles.append(current_cluster)
                    current_cluster = [(dist, ang_deg)]
            angle += msg.angle_increment

        if current_cluster:
            obstacles.append(current_cluster)
        
        # Log detected obstacles
        if obstacles:
            log_msg = "Obstacles detected: \n"
            danger_detected = False

            for i, cluster in enumerate(obstacles, start=1):
                dists = [p[0] for p in cluster]
                angs = [p[1] for p in cluster]

                avg_dist = sum(dists) / len(dists)
                avg_ang = sum(angs) / len(angs)

                # Check if obstacle is inside the safety radius
                if avg_dist <= self.safety_radius:
                    danger_detected = True
                    log_msg += (f"⚠️  DANGER! Object {i}: Distance {avg_dist:.2f} m "
                                f"Angle {avg_ang:.1f}° (points: {len(cluster)})\n")
                else:
                    log_msg += (f"Object {i}: Distance {avg_dist:.2f} m "
                                f"Angle {avg_ang:.1f}° (points: {len(cluster)})\n")

            if danger_detected:
                self.get_logger().warn(log_msg.strip())
            else:
                self.get_logger().info(log_msg.strip()) 
        else:
            self.get_logger().info("No obstacles detected")

def main():
    rclpy.init()
    node = LidarObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
