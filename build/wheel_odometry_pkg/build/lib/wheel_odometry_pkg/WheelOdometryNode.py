import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
import math


class WheelOdometryNode(LifecycleNode):

    def __init__(self):
        super().__init__('wheel_odometry_node')
        
        self.declare_parameter('wheel_diameter', 0.24)
        self.declare_parameter('ticks_per_rev', 580.0)
        self.declare_parameter('update_rate', 10.0)
        self.declare_parameter('publish_tf', True)
        
        self.wheel_d = 0.0
        self.ticks_per_rev = 0.0
        self.update_rate = 0.0
        self.publish_tf = True
        self.MIN_ENC = -32768
        self.MAX_ENC = 32768
        self.low_wrap = 0
        self.high_wrap = 0
        self.prev_enc = None
        self.tick_mult = 0
        self.total_ticks = 0.0
        self.rot_comp = 0.0
        self.rot_start = 0.0
        self.rot_end = 0.0
        self.prev_rot = 0.0
        self.motion = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_dist = 0.0
        self.odom_pub = None
        self.tf_broadcaster = None
        self.enc_sub = None
        self.yaw_sub = None
        self.motion_sub = None
        self.timer = None

    def on_configure(self, state):
        self.get_logger().info("Configuring node")
        self.wheel_d = self.get_parameter('wheel_diameter').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.update_rate = self.get_parameter('update_rate').value
        self.publish_tf = self.get_parameter('publish_tf').value
        range_val = self.MAX_ENC - self.MIN_ENC
        self.low_wrap = int(range_val * 0.3 + self.MIN_ENC)
        self.high_wrap = int(range_val * 0.7 + self.MIN_ENC)
        self.odom_pub = self.create_lifecycle_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.enc_sub = self.create_subscription(Int16, '/front_ticks', self.handle_enc, 10)
        self.yaw_sub = self.create_subscription(Float32, '/yaw', self.handle_yaw, 10)
        self.motion_sub = self.create_subscription(Int8, '/motion', self.handle_motion, 10)
        self.timer = self.create_timer(1.0 / self.update_rate, self.publish_odom)
        self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("Activating node")
        if self.odom_pub:
            self.odom_pub.activate()
        if self.timer:
            self.timer.reset()
        self.reset_state()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info("Deactivating node")
        if self.odom_pub:
            self.odom_pub.deactivate()
        if self.timer:
            self.timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info("Cleaning up node")
        self.destroy_publisher(self.odom_pub)
        self.destroy_subscription(self.enc_sub)
        self.destroy_subscription(self.yaw_sub)
        self.destroy_subscription(self.motion_sub)
        self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        return self.on_cleanup(state)

    def reset_state(self):
        self.prev_enc = None
        self.tick_mult = 0
        self.total_ticks = 0.0
        self.rot_comp = 0.0
        self.rot_start = 0.0
        self.rot_end = 0.0
        self.prev_rot = 0.0
        self.motion = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.prev_dist = 0.0

    def handle_yaw(self, yaw_msg):
        if self.odom_pub and self.odom_pub.is_active:
            self.theta = math.radians(360.0 - yaw_msg.data)

    def handle_motion(self, motion_msg):
        if self.odom_pub and self.odom_pub.is_active:
            self.motion = motion_msg.data

    def handle_enc(self, enc_msg):
        if not (self.odom_pub and self.odom_pub.is_active):
            return
        enc = int(enc_msg.data)
        if self.prev_enc is None:
            self.prev_enc = enc
            return
        if enc < self.low_wrap and self.prev_enc > self.high_wrap:
            self.tick_mult += 1
        elif enc > self.high_wrap and self.prev_enc < self.low_wrap:
            self.tick_mult -= 1
        full_ticks = enc + (self.tick_mult * (self.MAX_ENC - self.MIN_ENC))
        if self.motion in (1, 2):
            self.rot_end = full_ticks
            self.rot_comp = (self.rot_end - self.rot_start) + self.prev_rot
        else:
            self.rot_start = full_ticks
            self.prev_rot = self.rot_comp
        self.prev_enc = enc
        self.total_ticks = full_ticks - self.rot_comp

    def publish_odom(self):
        if not (self.odom_pub and self.odom_pub.is_active):
            return
        dist = abs((math.pi * self.wheel_d * self.total_ticks) / self.ticks_per_rev)
        delta_dist = dist - self.prev_dist
        self.prev_dist = dist
        self.x += delta_dist * math.sin(self.theta)
        self.y += delta_dist * math.cos(self.theta)
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        quat = quaternion_from_euler(0.0, 0.0, self.theta)
        odom_msg.pose.pose.orientation.x = quat[0]
        odom_msg.pose.pose.orientation.y = quat[1]
        odom_msg.pose.pose.orientation.z = quat[2]
        odom_msg.pose.pose.orientation.w = quat[3]
        odom_msg.twist.twist.linear.x = delta_dist * self.update_rate
        self.odom_pub.publish(odom_msg)
        if self.publish_tf:
            tf = TransformStamped()
            tf.header.stamp = odom_msg.header.stamp
            tf.header.frame_id = "odom"
            tf.child_frame_id = "base_link"
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.translation.z = 0.0
            tf.transform.rotation.x = quat[0]
            tf.transform.rotation.y = quat[1]
            tf.transform.rotation.z = quat[2]
            tf.transform.rotation.w = quat[3]
            self.tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()