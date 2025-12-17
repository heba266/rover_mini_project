import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np


class ReactiveNavigator(Node):
    def __init__(self):
        super().__init__('reactive_navigator')

        # HARD-CODED GOAL (meters, odom/map frame)
        self.goal_x = 2.0
        self.goal_y = 1.5
    
        self.costmap = None
        self.pose = None

        
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(OccupancyGrid, '/costmap', self.costmap_cb, 10)

        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info(f"Reactive Navigator Started | Goal = ({self.goal_x}, {self.goal_y})")

    
    def odom_cb(self, msg):
        self.pose = msg.pose.pose

    def costmap_cb(self, msg):
        self.costmap = msg

    
    def control_loop(self):
        if self.pose is None or self.costmap is None:
            return

        # Current robot position
        rx = self.pose.position.x
        ry = self.pose.position.y

        
        dx = self.goal_x - rx
        dy = self.goal_y - ry
        dist_goal = math.sqrt(dx * dx + dy * dy)

        
        if dist_goal < 0.2:
            self.stop_robot()
            self.get_logger().info("Goal reached")
            return

        
        att_x = dx / dist_goal
        att_y = dy / dist_goal

        
        rep_x, rep_y = self.compute_repulsion(rx, ry)

        
        vx = att_x + rep_x
        vy = att_y + rep_y

        
        cmd = Twist()
        cmd.linear.x = 0.5 * math.sqrt(vx * vx + vy * vy)
        cmd.angular.z = math.atan2(vy, vx)

        self.cmd_pub.publish(cmd)

    
    def compute_repulsion(self, rx, ry):
        rep_x = 0.0
        rep_y = 0.0

        info = self.costmap.info
        width = info.width
        height = info.height
        res = info.resolution
        origin = info.origin.position

        cx = int((rx - origin.x) / res)
        cy = int((ry - origin.y) / res)

        radius = 5  # cells around robot

        for i in range(cx - radius, cx + radius):
            for j in range(cy - radius, cy + radius):
                if i < 0 or j < 0 or i >= width or j >= height:
                    continue

                idx = j * width + i
                if self.costmap.data[idx] == 100:
                    ox = origin.x + i * res
                    oy = origin.y + j * res
                    dist = math.sqrt((rx - ox)**2 + (ry - oy)**2)

                    if dist > 0.001:
                        rep_x += (rx - ox) / dist
                        rep_y += (ry - oy) / dist

        return rep_x, rep_y

   
    def stop_robot(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = ReactiveNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()