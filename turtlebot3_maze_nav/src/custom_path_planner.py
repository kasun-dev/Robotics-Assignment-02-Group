#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
import math
import heapq
import numpy as np
import time

class SimpleAStarPlanner(Node):
    def __init__(self):
        super().__init__('simple_astar_planner')
        self.map = None
        self.map_header = None
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.current_pose = None
        self.goal = None
        self.path = []
        self.follow_index = 0
        self.get_logger().info("SimpleAStarPlanner started")

    def map_cb(self, msg: OccupancyGrid):
        self.map = msg
        self.map_header = msg.header

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def goal_cb(self, msg: PoseStamped):
        if self.map is None:
            self.get_logger().warn("Map not received yet - cannot plan")
            return
        self.goal = msg.pose
        self.get_logger().info("New goal received, planning...")
        self.path = self.plan_path(self.current_pose, self.goal)
        self.follow_index = 0
        if self.path:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = self.map.header.frame_id
            for (x, y) in self.path:
                ps = PoseStamped()
                ps.header = path_msg.header
                ps.pose.position.x = x
                ps.pose.position.y = y
                ps.pose.orientation.w = 1.0
                path_msg.poses.append(ps)
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Planned path with {len(self.path)} points")

    def world_to_map(self, x, y):
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        res = self.map.info.resolution
        mx = int((x - origin_x) / res)
        my = int((y - origin_y) / res)
        if mx < 0 or my < 0 or mx >= self.map.info.width or my >= self.map.info.height:
            return None
        return mx, my

    def map_to_world(self, mx, my):
        origin_x = self.map.info.origin.position.x
        origin_y = self.map.info.origin.position.y
        res = self.map.info.resolution
        x = origin_x + (mx + 0.5) * res
        y = origin_y + (my + 0.5) * res
        return x, y

    def is_occupied(self, mx, my):
        idx = my * self.map.info.width + mx
        v = self.map.data[idx]
        # unknown or high occupancy counts as obstacle
        return v >= 65

    def plan_path(self, start_pose: Pose, goal_pose: Pose):
        if self.map is None or start_pose is None:
            self.get_logger().warn("Missing map or start pose")
            return []
        start_map = self.world_to_map(start_pose.position.x, start_pose.position.y)
        goal_map = self.world_to_map(goal_pose.position.x, goal_pose.position.y)
        if start_map is None or goal_map is None:
            self.get_logger().warn("Start or goal outside map")
            return []
        sx, sy = start_map
        gx, gy = goal_map

        w = self.map.info.width
        h = self.map.info.height

        # A* search on 4-connected grid
        open_set = []
        heapq.heappush(open_set, (0 + self.heuristic(sx, sy, gx, gy), 0, (sx, sy), None))
        came_from = {}
        gscore = { (sx, sy): 0 }

        while open_set:
            f, g, current, parent = heapq.heappop(open_set)
            if current in came_from:
                continue
            came_from[current] = parent
            if current == (gx, gy):
                break
            cx, cy = current
            for dx, dy in [ (1,0),(-1,0),(0,1),(0,-1) ]:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                if self.is_occupied(nx, ny):
                    continue
                tentative_g = g + 1
                if (nx, ny) in gscore and tentative_g >= gscore[(nx, ny)]:
                    continue
                gscore[(nx, ny)] = tentative_g
                heapq.heappush(open_set, (tentative_g + self.heuristic(nx, ny, gx, gy), tentative_g, (nx, ny), current))

        # reconstruct path
        node = (gx, gy)
        if node not in came_from:
            self.get_logger().warn("No path found")
            return []
        rev = []
        while node is not None:
            rev.append(node)
            node = came_from[node]
        rev.reverse()
        # convert to world coords
        path_world = [ self.map_to_world(mx, my) for (mx, my) in rev ]
        return path_world

    def heuristic(self, x1, y1, x2, y2):
        return abs(x1-x2) + abs(y1-y2)

    def control_loop(self):
        if not self.path or self.current_pose is None:
            return
        if self.follow_index >= len(self.path):
            # reached goal: stop
            t = Twist()
            self.cmd_pub.publish(t)
            return
        target = self.path[self.follow_index]
        tx, ty = target
        px = self.current_pose.position.x
        py = self.current_pose.position.y
        dx = tx - px
        dy = ty - py
        dist = math.hypot(dx, dy)
        yaw = self.get_yaw(self.current_pose.orientation)
        angle_to_target = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(angle_to_target - yaw)

        cmd = Twist()
        # simple rotation-first behaviour
        if abs(angle_diff) > 0.3:
            cmd.angular.z = 0.6 * angle_diff
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = min(0.15, 0.5 * dist)
            cmd.angular.z = 0.6 * angle_diff

        self.cmd_pub.publish(cmd)

        if dist < 0.08:
            # advance to next waypoint
            self.follow_index += 1

    def get_yaw(self, q):
        # quaternion to yaw
        x = q.x; y = q.y; z = q.z; w = q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, a):
        while a > math.pi:
            a -= 2*math.pi
        while a < -math.pi:
            a += 2*math.pi
        return a

def main(args=None):
    rclpy.init(args=args)
    node = SimpleAStarPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
