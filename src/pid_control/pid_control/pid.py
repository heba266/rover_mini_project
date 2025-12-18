import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from control_interfaces.action import Control
import math


class PathControl(LifecycleNode):
    def __init__(self):
        super().__init__('path_control')

        self.goal_x = 6.0
        self.goal_y = 8.0

        self.kp_linear = 1.0
        self.ki_linear = 0.1
        self.kd_linear = 0.2

        self.kp_angular = 2.0
        self.ki_angular = 0.0
        self.kd_angular = 0.3

        self.linear_error_prev = 0.0
        self.angular_error_prev = 0.0
        self.linear_error_sum = 0.0
        self.angular_error_sum = 0.0

        self.current_distance = float('inf')
        self.goal_reached = False

        self.time_prev = self.get_clock().now()

        self.cmd_pub = None
        self.odom_sub = None
        self.control_server = None

    def on_configure(self, state: LifecycleState):
        self.cmd_pub = self.create_lifecycle_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.goal_reached = False
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.cmd_pub.activate()
        self.control_server = ActionServer(self, Control, 'control', self.execute_callback)
        self.time_prev = self.get_clock().now()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState):
        self.cmd_pub.deactivate()
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState):
        self.cmd_pub = None
        self.odom_sub = None
        self.control_server = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState):
        return TransitionCallbackReturn.SUCCESS

    def execute_callback(self, goal_handle: ServerGoalHandle):
        self.goal_x = goal_handle.request.goal_pose.pose.position.x
        self.goal_y = goal_handle.request.goal_pose.pose.position.y
        self.goal_reached = False

        rate = self.create_rate(10)
        while rclpy.ok():
            if self.goal_reached:
                break

            feedback = Control.Feedback()
            feedback.distance_to_goal = self.current_distance
            goal_handle.publish_feedback(feedback)

            rate.sleep()

        goal_handle.succeed()
        result = Control.Result()
        result.success = True
        return result

    def odom_callback(self, msg):
        if self.current_state.label != 'active':
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        self.control(x, y, yaw)

    def control(self, x, y, yaw):
        now = self.get_clock().now()
        dt = (now - self.time_prev)
        self.time_prev = now

        if dt <= 0.0:
            return

        dx = self.goal_x - x
        dy = self.goal_y - y

        linear_error = math.sqrt(dx * dx + dy * dy)
        goal_angle = math.atan2(dy, dx)
        angular_error = math.atan2(math.sin(goal_angle - yaw), math.cos(goal_angle - yaw))

        self.current_distance = linear_error

        self.linear_error_sum += linear_error * dt
        self.angular_error_sum += angular_error * dt

        linear_derivative = (linear_error - self.linear_error_prev) / dt
        angular_derivative = (angular_error - self.angular_error_prev) / dt

        linear_velocity = (
            self.kp_linear * linear_error +
            self.ki_linear * self.linear_error_sum +
            self.kd_linear * linear_derivative
        )

        angular_velocity = (
            self.kp_angular * angular_error +
            self.ki_angular * self.angular_error_sum +
            self.kd_angular * angular_derivative
        )

        self.linear_error_prev = linear_error
        self.angular_error_prev = angular_error

        if linear_error < 0.1:
            linear_velocity = 0.0
            angular_velocity = 0.0
            self.goal_reached = True

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PathControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
