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
        self.declare_parameter("inflation_grid_radius", 3)


        self.map_resolution = None
        self.map_size = None
        self.map_center = None
        self.map = None
        self.tf_buffer = None
        self.tf_listener = None
        self.lidar_sub_ = None
        self.map_pub_ = None
        self.obstacle_index = None
         

        
    def on_configure(self, state: LifecycleState):
        self.get_logger().info("Configuring...")

        self.lidar_sub_ = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.map_pub_ = self.create_lifecycle_publisher(OccupancyGrid, '/costmap', 10)
        

        self.map_resolution = self.get_parameter("map_resolution").value
        self.map_size = self.get_parameter("map_size").value
        self.inflation_grid_radius = self.get_parameter("inflation_grid_radius").value
        self.map_center = self.map_size // 2
        self.map = np.zeros((self.map_size, self.map_size), dtype=np.int8)
        self.obstacle_index = []


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
    
    def on_deactivate(self, state: LifecycleState):
        self.get_logger().info("Deactivating...")
        self.map_pub_.on_deactivate(state)
        self.get_logger().info("Deactivation complete")

        return TransitionCallbackReturn.SUCCESS
    
    def on_shutdown(self, state: LifecycleState):
        self.get_logger().info("Shutting down...")
        return super().on_shutdown(state)
        

    def lidar_callback(self, msg: LaserScan):
       
        try:
            t = self.tf_buffer.lookup_transform("odom", "base_link", rclpy.time.Time())
        except Exception:
            return

        robot_x = t.transform.translation.x 
        robot_y = t.transform.translation.y
        q = t.transform.rotation
        _, _, robot_yaw = euler_from_quaternion([q.x, q.y, q.z, q.w]) 

        map_origin_x = robot_x - (self.map_center*self.map_resolution)
        map_origin_y = robot_y - (self.map_center*self.map_resolution)

        self.map.fill(0)
        for i, r in enumerate(msg.ranges):
            if math.isinf(r) or math.isnan(r):
                continue
            angle = msg.angle_min + msg.angle_increment * i

            #this rotates the x and y relative to the base_link and makes it relative to odom
            rotated_x = (r*math.cos(angle) * math.cos(robot_yaw)) - (r*math.sin(angle) * math.sin(robot_yaw))
            rotated_y = (r*math.cos(angle) * math.sin(robot_yaw)) + (r*math.sin(angle) * math.cos(robot_yaw))

            odom_x = robot_x + rotated_x
            odom_y = robot_y + rotated_y

            grid_x = int(((odom_x - map_origin_x) // self.map_resolution))
            grid_y = int(((odom_y - map_origin_y) // self.map_resolution))
 
            if (0 <= grid_y < self.map_size and 0 <= grid_x < self.map_size):
                self.map[grid_y][grid_x] = 100
                self.obstacle_index.append((grid_y, grid_x))

        self.apply_inflation()
        

        grid = OccupancyGrid()
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.header.frame_id = "odom"
        grid.info.width = self.map_size
        grid.info.height = self.map_size
        grid.info.resolution = self.map_resolution
        grid.info.origin.position.x = map_origin_x
        grid.info.origin.position.y = map_origin_y
        grid.info.origin.position.z = 0.0
        grid.data = self.map.flatten().tolist()

        self.map_pub_.publish(grid)
        
    def apply_inflation(self):
        r = self.inflation_grid_radius
        for obs_y, obs_x in self.obstacle_index:

            for dy in range(-r, r+1):
                for dx in range(-r, r+1):
                    ny, nx = dy+obs_y , dx+obs_x

                    distance = math.sqrt(dx**2+dy**2)
                    if distance <= r:
                        cost = int(100 - (10*distance))

                        if 0 <= nx < self.map_size and 0 <= ny < self.map_size:
                            self.map[ny][nx] = cost if self.map[ny][nx] < cost else self.map[ny][nx]
        self.obstacle_index.clear()
        












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


        

            






