
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, Float32, Int8
import math
from tf_transformations import euler_from_quaternion

class TurtleBot3Converter(Node):
    def __init__(self):
        super().__init__('turtlebot3_converter')
        self.ticks_pub = self.create_publisher(Int16, '/front_ticks', 10)
        self.yaw_pub = self.create_publisher(Float32, '/yaw', 10)
        self.motion_pub = self.create_publisher(Int8, '/motion', 10)

        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.threshold = 0.01
        self.turn_threshold = 0.02
        self.prev_ticks = 0

    def imu_callback(self, msg):
        q = msg.orientation
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = euler[2]
        yaw_deg = math.degrees(yaw) % 360
        yaw_msg = Float32()
        yaw_msg.data = yaw_deg
        self.yaw_pub.publish(yaw_msg)

    def odom_callback(self, msg):
        # Approximate ticks from distance traveled
        dist = msg.twist.twist.linear.x * 0.1  # distance per update (rough)
        ticks_value = int(self.prev_ticks + dist * 1000)  # scale factor
        self.prev_ticks = ticks_value
        ticks_msg = Int16()
        ticks_msg.data = ticks_value
        self.ticks_pub.publish(ticks_msg)

        # Motion detection from velocities
        left_vel = msg.twist.twist.linear.x - msg.twist.twist.angular.z
        right_vel = msg.twist.twist.linear.x + msg.twist.twist.angular.z
        motion_state = self.detect_motion(left_vel, right_vel)
        motion_msg = Int8()
        motion_msg.data = motion_state
        self.motion_pub.publish(motion_msg)

    def detect_motion(self, left_vel, right_vel):
        left_moving = abs(left_vel) > self.threshold
        right_moving = abs(right_vel) > self.threshold
        if not left_moving and not right_moving:
            return 0
        vel_diff = right_vel - left_vel
        if vel_diff > self.turn_threshold:
            return 1  # turning right
        elif vel_diff < -self.turn_threshold:
            return 2  # turning left
        else:
            return 0  #moving forward

def main():
    rclpy.init()
    converter = TurtleBot3Converter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
