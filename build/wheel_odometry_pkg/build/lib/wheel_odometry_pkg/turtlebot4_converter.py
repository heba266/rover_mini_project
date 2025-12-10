import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from irobot_create_msgs.msg import WheelTicks, WheelVels
from std_msgs.msg import Int16, Float32, Int8
import math
from tf_transformations import euler_from_quaternion

class TurtleBot4Converter(Node):
    def __init__(self):
        super().__init__('turtlebot4_converter')
        self.ticks_pub = self.create_publisher(Int16, '/front_ticks', 10)
        self.yaw_pub = self.create_publisher(Float32, '/yaw', 10)
        self.motion_pub = self.create_publisher(Int8, '/motion', 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(WheelTicks, '/wheel_ticks', self.ticks_callback, 10)
        self.create_subscription(WheelVels, '/wheel_vels', self.vels_callback, 10)
        self.threshold = 0.01
        self.turn_threshold = 0.02

    def ticks_callback(self, msg):
        ticks_msg = Int16()
        ticks_value = int(msg.ticks_left)
        ticks_value = ticks_value % 65536
        if ticks_value > 32767:
            ticks_value -= 65536
        ticks_msg.data = ticks_value
        self.ticks_pub.publish(ticks_msg)


    def imu_callback(self, msg):
        self.get_logger().info(f'Yaw')
        q = msg.orientation
        
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        euler = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = euler[2]  # yaw is the third element
        
        # Convert to degrees and normalize
        yaw_deg = math.degrees(yaw) % 360
        if yaw_deg < 0:
            yaw_deg += 360
        
        yaw_msg = Float32()
        yaw_msg.data = yaw_deg
        self.yaw_pub.publish(yaw_msg)
        
        self.get_logger().info(f'Yaw: {yaw_deg:.2f}°')


    def vels_callback(self, msg):
        left_vel = msg.velocity_left
        right_vel = msg.velocity_right
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
        if right_vel > self.turn_threshold and left_vel < -self.turn_threshold:
            return 1
        elif left_vel > self.turn_threshold and right_vel < -self.turn_threshold:
            return 2
        elif abs(vel_diff) > self.turn_threshold:
            if vel_diff > 0:
                return 1
            else:
                return 2
        else:
            return 0

def main():
    rclpy.init()
    converter = TurtleBot4Converter()
    rclpy.spin(converter)
    converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()