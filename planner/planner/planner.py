import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.action import ActionServer
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from interface.action import Plan
import heapq
import math


class Planner(LifecycleNode):
    def __init__(self):
        super().__init__('planner')

        self.declare_parameter('path_resolution', 0.05)
        self.declare_parameter('obstacle_weight', 0.1)

        self.current_costmap = None
        self.current_pose = None
        self.action_server = None

        self.get_logger().info('Planner Node Created')

    # callbacks
    def costmap_callback(self, msg):
        if not (self.path_pub and self.path_pub.is_activated): 
            return
        self.current_costmap = msg

    def odom_callback(self, msg):
        if not (self.path_pub and self.path_pub.is_activated): 
            return
        self.current_pose = msg.pose.pose

    # lifecycle
    def on_configure(self, state):
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/costmap', self.costmap_callback, 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.path_pub = self.create_lifecycle_publisher(Path, '/planner', 10)

        # self.action_server = ActionServer(self, Plan, 'plan', self.execute_callback)

        self.get_logger().info('Planner configured')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.path_pub.on_activate(state)

        self.action_server = ActionServer(self, Plan, 'plan', self.execute_callback)

        self.get_logger().info('Planner activated')
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state):
        self.path_pub.on_deactivate()

        if self.action_server:
            self.action_server.destroy()
            self.action_server = None

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state):
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state):
        return TransitionCallbackReturn.SUCCESS

    # action
    def execute_callback(self, goal_handle):
        if not (self.path_pub and self.path_pub.is_activated):
            goal_handle.abort()
            self.get_logger().info("Goal rejected : planner not active")
            return Plan.Result()
        if self.current_costmap is None or self.current_pose is None:
            goal_handle.abort()
            return Plan.Result()

        goal_pose = goal_handle.request.goal_pose.pose
        start = self.to_grid((
            self.current_pose.position.x,
            self.current_pose.position.y))

        goal = self.to_grid((
            goal_pose.position.x,
            goal_pose.position.y))

        path_grid = self.a_star(start, goal)
        if path_grid is None:
            goal_handle.abort()
            return Plan.Result()

        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for p in path_grid:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position = self.from_grid(p)
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)

        result = Plan.Result()
        result.path = path_msg
        goal_handle.succeed()
        return result

    # A*
    def a_star(self, start, goal):
        open_set = []
        heapq.heappush(open_set, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}
        closed = set()

        moves = [
            (1,0,1), (-1,0,1), (0,1,1), (0,-1,1),
            (1,1,math.sqrt(2)), (-1,1,math.sqrt(2)),
            (1,-1,math.sqrt(2)), (-1,-1,math.sqrt(2))
        ]

        weight = self.get_parameter('obstacle_weight').value

        while open_set:
            _, current = heapq.heappop(open_set)

            if current in closed:
                continue
            closed.add(current)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                path.reverse()
                return path

            x, y = current
            for dx, dy, move_cost in moves:
                nx, ny = x + dx, y + dy
                cell_cost = self.get_cell_cost(nx, ny)

                if cell_cost >= 100:
                    continue

                neighbor = (nx, ny)
                cost = move_cost + (cell_cost / 100.0) * weight
                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return None

    def heuristic(self, a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        return math.sqrt(dx*dx + dy*dy)

    def get_cell_cost(self, x, y):
        if self.current_costmap is None:
            return 100

        w = self.current_costmap.info.width
        h = self.current_costmap.info.height
        if x < 0 or y < 0 or x >= w or y >= h:
            return 100

        idx = y * w + x
        if idx >= len(self.current_costmap.data):
            return 100

        value = self.current_costmap.data[idx]
        if value < 0:
            return 50
        return value

    # grid conversion
    def to_grid(self, pos):
        origin = self.current_costmap.info.origin.position
        res = self.current_costmap.info.resolution
        return (
            int((pos[0] - origin.x) / res),
            int((pos[1] - origin.y) / res)
        )

    def from_grid(self, grid):
        origin = self.current_costmap.info.origin.position
        res = self.current_costmap.info.resolution
        p = Point()
        p.x = origin.x + grid[0] * res
        p.y = origin.y + grid[1] * res
        p.z = 0.0
        return p


def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    rclpy.spin(planner)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
