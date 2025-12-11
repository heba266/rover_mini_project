import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from interface.action import Plan
import numpy as np
import heapq
import math

class Planner(LifecycleNode):
    def __init__(self):
        super().__init__('planner')
        self.declare_parameter('planner_type','AStar')
        self.declare_parameter('path_resolution', 0.05)

        self.current_costmap = None
        self.current_pose = None

        self.get_logger().info("Planner Node Created.")

        # subscriptions
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path_pub = self.create_publisher(Path, '/planner', 10)

        # action server
        self.action_server = ActionServer(self, Plan, 'plan', self.execute_callback)

    # callbacks
    def costmap_callback(self, msg):
        self.current_costmap = msg

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def execute_callback(self, goal_handle):
        goal_pose = goal_handle.request.goal_pose.pose
        goal_x = goal_pose.position.x
        goal_y = goal_pose.position.y

        if self.current_costmap is None:
            self.get_logger().info("No costmap received yet")
            goal_handle.abort()
            return None
        if self.current_pose is None:
            self.get_logger().info("No current pose received yet")
            goal_handle.abort()
            return None

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        self.get_logger().info(f"Planning from ({start_x:.2f},{start_y:.2f}) to ({goal_x:.2f},{goal_y:.2f})")

        # convert to grid
        start_grid = self.to_grid((start_x, start_y))
        goal_grid = self.to_grid((goal_x, goal_y))

        # plan
        path_grid = self.a_star(start_grid, goal_grid)
        if path_grid is None:
            self.get_logger().info("No path found")
            goal_handle.abort()
            return None

        # convert grid path to world coordinates
        path = [self.from_grid(p) for p in path_grid]

        # create Path message
        path_msg = Path()
        path_msg.header.frame_id = "odom"
        for point in path:
            pose = PoseStamped()
            pose.pose.position = point
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

        result = Plan.Result()
        result.path = path_msg
        goal_handle.succeed()
        self.get_logger().info("Path planned successfully")
        return result

    # A* algorithm
    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}

        moves = [
            (1,0,1), (-1,0,1), (0,1,1), (0,-1,1),
            (1,1,math.sqrt(2)), (-1,1,math.sqrt(2)),
            (1,-1,math.sqrt(2)), (-1,-1,math.sqrt(2))
        ]

        while open_set:
            current_f, current = heapq.heappop(open_set)
            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            x, y = current
            for dx, dy, cost in moves:
                nx, ny = x + dx, y + dy
                if not self.is_valid(nx, ny) or self.is_obstacle(nx, ny):
                    continue
                neighbor = (nx, ny)
                tentative_g = g_score[current] + cost
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return None

    def heuristic(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.sqrt(dx*dx + dy*dy)

    def is_valid(self, grid_x, grid_y):
        if self.current_costmap is None:
            return False
        return 0 <= grid_x < self.current_costmap.info.width and 0 <= grid_y < self.current_costmap.info.height

    def is_obstacle(self, grid_x, grid_y):
        if not self.is_valid(grid_x, grid_y):
            return True
        width = self.current_costmap.info.width
        idx = int(grid_y * width + grid_x)
        if idx < len(self.current_costmap.data):
            return self.current_costmap.data[idx] == 100
        return True

    # grid conversion
    def to_grid(self, pos):
        res = self.get_parameter('path_resolution').value
        return (int(pos[0] / res), int(pos[1] / res))

    def from_grid(self, grid):
        res = self.get_parameter('path_resolution').value
        pt = Point()
        pt.x = grid[0] * res
        pt.y = grid[1] * res
        pt.z = 0.0
        return pt

    # lifecycle callbacks
    def on_configure(self, state):
        self.get_logger().info("Configuring planner...")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info("Planner activated - ready for goals")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.get_logger().info("Planner deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        self.get_logger().info("Cleaning up planner...")
        if self.action_server:
            self.action_server.destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        self.get_logger().info("Shutting down planner...")
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    
    # Create planner node
    planner = Planner()
    
    # Use regular Node execution
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(planner)
    
    try:
        planner.trigger_configure()
        planner.trigger_activate()
        
        # Spin
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        planner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()