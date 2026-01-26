import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import LifecycleState
from rclpy.lifecycle import TransitionCallbackReturn
from std_msgs.msg import Int16, Float32, Int8
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
        try:
            self.odom_pub = self.create_lifecycle_publisher(Odometry,'/my_odom',10)
            self.tf_broadcaster = TransformBroadcaster(self)
            return TransitionCallbackReturn.SUCCESS
        except Exception:
            return TransitionCallbackReturn.FAILURE

    def on_activate(self, state):
        try:
            self.enc_sub = self.create_subscription(Int16,'/front_ticks',self.handle_enc,10)
            self.yaw_sub = self.create_subscription(Float32,'/yaw', self.handle_yaw, 10)
            self.motion_sub = self.create_subscription(Int8,'/motion',self.handle_motion,10)
            self.timer = self.create_timer(1,self.publish_odom)
            self.reset_state()
            return TransitionCallbackReturn.SUCCESS
        except Exception:
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state):
        if self.enc_sub:
            self.destroy_subscription(self.enc_sub)
        if self.yaw_sub:
            self.destroy_subscription(self.yaw_sub)
        if self.motion_sub:
            self.destroy_subscription(self.motion_sub)
        if self.timer:
            self.destroy_timer(self.timer)
        return TransitionCallbackReturn.SUCCESS

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
        self.theta = math.radians(360.0 - yaw_msg.data)

    def handle_motion(self, motion_msg):
        self.motion = motion_msg.data

    def handle_enc(self, enc_msg):
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
        if not self.odom_pub:
            self.get_logger().info("Not Publish")
            return
        dist = abs((math.pi * 0.24 * self.total_ticks) / 580.0)
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
        odom_msg.twist.twist.linear.x = delta_dist * 10.0
        self.odom_pub.publish(odom_msg)
        self.get_logger().info("Publish")

        if self.tf_broadcaster:
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