import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.action import ActionServer , ActionClient
from rclpy.action.server import ServerGoalHandle
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from control_interfaces.action import Control
import math
import time


class PathControl(LifecycleNode):
    def __init__(self):
        super().__init__('path_control')

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_yaw = 0.0
        self.has_goal = False

        self.kp_linear = 0.8
        self.ki_linear = 0.0
        self.kd_linear = 0.1

        self.kp_angular = 1.0
        self.ki_angular = 0.0
        self.kd_angular = 0.1

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
        self.cmd_pub = self.create_lifecycle_publisher(TwistStamped, '/cmd_vel', 10)
        
        self.goal_reached = False
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState):
        self.cmd_pub.on_activate(state)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_sub = self.create_subscription(Path, '/planner',self.path_cb,10)
        self.control_server = ActionServer(self, Control, 'control', self.execute_callback)
        self.action_client = ActionClient (self,Control,'/control' )
        self.time_prev = self.get_clock().now()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState):
        self.cmd_pub.on_deactivate(state)
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
        self.has_goal = True
        
        
        self.goal_reached = False

        #rate = rclpy.Rate(10, self.get_clock()) 


        feedback = Control.Feedback()
        while not self.goal_reached and rclpy.ok():

            feedback.distance_to_goal = self.current_distance
            goal_handle.publish_feedback(feedback)
            self.control(self.rover_x, self.rover_y, self.rover_yaw)
            #rate.sleep()
            #time.sleep(0.1)

        goal_handle.succeed()
        result = Control.Result()
        result.success = True
        self.has_goal = False
        return result

    def odom_callback(self, msg):
        if not(self.cmd_pub and self.cmd_pub.is_activated):
            return
        # if not self.has_goal: 
        #     return
        
        self.rover_x = msg.pose.pose.position.x
        self.rover_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.rover_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
      
        #self.control(self.rover_x, self.rover_y, self.rover_yaw)


    def path_cb(self, msg):
        if not(self.cmd_pub and self.cmd_pub.is_activated):
            return
        
        path = msg
        for pose in path.poses:
            self.get_logger().info(f"goal recieved = {pose.pose.position.x}")
            goal_msg = Control.Goal()
            goal_msg.goal_pose = pose

            self.action_client.wait_for_server()
            future = self.action_client.send_goal_async(goal_msg)
            #rclpy.spin_until_future_complete(self, future)
            future.add_done_callback(self.goal_respose_cb)

    def goal_respose_cb(self, future):
        result = future.result()
        if not result.accepted:
            self.get_logger().warn("goal rejected")
            return
        else:
            self.get_logger().warn("goal accepted")

    def control(self, x, y, yaw):
        self.get_logger().info(f"has goal = {self.has_goal}")
        if not self.has_goal:
            return
        now = self.get_clock().now()
        dt = (now - self.time_prev).nanoseconds / 1e9  #bec time gives a duration not a float
        self.time_prev = now

        if dt <= 0.0:
            return

        dx = self.goal_x - x
        dy = self.goal_y - y

        self.get_logger().info(f"dx={dx}")

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

        # self.linear_error_prev = linear_error
        # self.angular_error_prev = angular_error

        linear_velocity = max(min(linear_velocity, 0.5), -0.5)
        angular_velocity = max(min(angular_velocity, 1.0), -1.0)


        if linear_error < 0.1:
            linear_velocity = 0.0
            angular_velocity = 0.0
            self.goal_reached = True
            
        

            
        self.get_logger().info(f"dx={dx}")
        # twist = Twist()
        # twist.linear.x = linear_velocity
        # twist.angular.z = angular_velocity
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = "base_link"
        twist.twist.linear.x = linear_velocity
        twist.twist.angular.z = angular_velocity
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = PathControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
