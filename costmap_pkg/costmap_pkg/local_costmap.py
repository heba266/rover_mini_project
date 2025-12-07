import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import tf2_ros
from tf_transformations import euler_from_quaternion
import numpy as np
import math

class LocalCostMap(LifecycleNode):
    def __init__(self):
        super().__init__("local_costmap")
        
        self.declare_parameter("map_resolution", 0.05)
        self.declare_parameter("map_size", 200)

        self.map_resolution = None
        self.map_size = None
        self.map_center = None
        self.map = None
        self.tf_buffer = None
        self.tf_listener = None
        self.lidar_sub_ = None
        self.map_pub_ = None

        
    def on_configure(self, state: LifecycleState):
        self.get_logger().info("Configuring...")

        self.lidar_sub_ = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.map_pub_ = self.create_lifecycle_publisher(OccupancyGrid, '/costmap', 10)

        self.map_resolution = self.get_parameter("map_resolution").value
        self.map_size = self.get_parameter("map_size").value
        self.map_center = self.map_size // 2
        self.map = np.zeros((self.map_size, self.map_size), dtype=np.int8)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
    
        self.get_logger().info("Configuring complete")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.get_logger().info("clean up...")
        if self.map_pub_:
            self.destroy_lifecycle_publisher(self.map_pub_)
        if self.lidar_sub_:
            self.destroy_subscription(self.lidar_sub_)
        self.get_logger().info("clean up complete")
        return TransitionCallbackReturn.SUCCESS


    def on_activate(self, state: LifecycleState):
        self.get_logger().info("Activating...")

        self.map_pub_.on_activate(state)
        self.get_logger().info("Activation complete")
        self.get_logger().info("Map published!")

        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: LifecycleNode):
        self.get_logger().info("Deactivating...")
        self.map_pub_.on_deactivate(state)
        self.get_logger().info("Deactivation complete")

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("Shutting down...")
        return super().on_shutdown(state)
        

    def lidar_callback(self, msg: LaserScan):
        self.map.fill(0)
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            angle = msg.angle_min + msg.angle_increment * i

            x = int(self.map_center + r*math.cos(angle)//self.map_resolution)
            y = int(self.map_center + r*math.sin(angle)//self.map_resolution) 

            if (0 <= y < self.map_size and 0 <= x < self.map_size):
                self.map[y][x] = 100
        
        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "base_link"
        grid.info.width = self.map_size
        grid.info.height = self.map_size
        grid.info.resolution = self.map_resolution
        grid.info.origin.position.x = -(self.map_center*self.map_resolution)
        grid.info.origin.position.y = -(self.map_center*self.map_resolution)
        grid.info.origin.position.z = 0.0
        grid.data = self.map.flatten().tolist()

        self.map_pub_.publish(grid)
        

def main(args=None):
    rclpy.init(args=args)
    map = LocalCostMap()
    try:
        rclpy.spin(map)
    except KeyboardInterrupt:
        pass
    finally:
        map.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()