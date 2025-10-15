import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from example_interfaces.srv import SetInt32
import math
import heapq

class MapNavigator(Node):
    def __init__(self):
        super().__init__('map_navigator')

        # Publishers
        self.waypoint_pub = self.create_publisher(PointStamped, '/way_point', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_markers', 10)

        # Subscriber for robot GPS
        self.create_subscription(NavSatFix, '/gps_raw', self.gps_callback, 10)

        # Service for setting goal
        self.goal_srv = self.create_service(SetInt32, 'set_goal', self.set_goal_callback)

        # Load map
        self.nodes = self.load_nodes('/mnt/data/nodes.txt')
        self.origin = list(self.nodes.values())[0]
        self.current_pose = None

        # Graph adjacency (3m rule)
        self.graph = self.build_graph(self.nodes, threshold=3.0)

        # Path state
        self.current_path = []
        self.current_wp_idx = 0

        # Timer for waypoint publishing
        self.timer = self.create_timer(0.5, self.update_waypoint)

        self.get_logger().info("✅ MapNavigator started. Call /set_goal with goal index.")

    def load_nodes(self, path):
        nodes = {}
        with open(path, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                idx = int(parts[0])
                lat, lon = float(parts[1]), float(parts[2])
                nodes[idx] = (lat, lon)
        return nodes

    def to_local(self, lat, lon):
        # Flat Earth ENU approx
        lat0, lon0 = self.origin
        R = 6378137.0
        dx = (lon - lon0) * math.pi/180.0 * R * math.cos(lat0*math.pi/180.0)
        dy = (lat - lat0) * math.pi/180.0 * R
        return (dx, dy)

    def gps_callback(self, msg: NavSatFix):
        self.current_pose = self.to_local(msg.latitude, msg.longitude)

    def build_graph(self, nodes, threshold=3.0):
        """Connect nodes if distance <= threshold meters"""
        graph = {i: [] for i in nodes}
        enu = {i: self.to_local(lat, lon) for i, (lat, lon) in nodes.items()}
        for i in nodes:
            for j in nodes:
                if i >= j:
                    continue
                d = math.hypot(enu[i][0] - enu[j][0], enu[i][1] - enu[j][1])
                if d <= threshold:
                    graph[i].append(j)
                    graph[j].append(i)
        return graph

    def find_nearest_node(self, pose):
        if pose is None:
            return 0
        min_d = float('inf')
        nearest = None
        for idx, (lat, lon) in self.nodes.items():
            x, y = self.to_local(lat, lon)
            d = math.hypot(x - pose[0], y - pose[1])
            if d < min_d:
                min_d = d
                nearest = idx
        return nearest if nearest is not None else 0

    def plan_path(self, start_idx, goal_idx):
        def dist(a, b):
            la, lo = self.nodes[a]
            lb, lo2 = self.nodes[b]
            return math.hypot(la - lb, lo - lo2)

        pq = [(0, start_idx, [])]
        visited = set()
        while pq:
            cost, node, path = heapq.heappop(pq)
            if node in visited:
                continue
            visited.add(node)
            path = path + [node]
            if node == goal_idx:
                return path
            for nb in self.graph[node]:
                if nb not in visited:
                    heapq.heappush(pq, (cost + dist(node, nb), nb, path))
        return []

    def publish_waypoint(self, idx):
        lat, lon = self.nodes[idx]
        x, y = self.to_local(lat, lon)
        msg = PointStamped()
        msg.header.frame_id = "map"
        msg.point.x = x
        msg.point.y = y
        self.waypoint_pub.publish(msg)
        self.get_logger().info(f"📍 Publishing waypoint {idx} at local ({x:.2f}, {y:.2f})")

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        # Clear previous markers
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        # Draw all waypoints in path as green spheres
        for i, idx in enumerate(self.current_path):
            lat, lon = self.nodes[idx]
            x, y = self.to_local(lat, lon)

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "waypoints"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.3
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            marker_array.markers.append(m)

        # Highlight current target waypoint in red
        if self.current_path and self.current_wp_idx < len(self.current_path):
            idx = self.current_path[self.current_wp_idx]
            lat, lon = self.nodes[idx]
            x, y = self.to_local(lat, lon)

            m = Marker()
            m.header.frame_id = "map"
            m.header.stamp = now
            m.ns = "active_goal"
            m.id = 9999
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.0
            m.scale.x = m.scale.y = m.scale.z = 0.6
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

    def update_waypoint(self):
        if not self.current_path or self.current_pose is None:
            return
        if self.current_wp_idx >= len(self.current_path):
            return  # already finished

        target_idx = self.current_path[self.current_wp_idx]
        lat, lon = self.nodes[target_idx]
        tx, ty = self.to_local(lat, lon)
        cx, cy = self.current_pose
        d = math.hypot(tx - cx, ty - cy)

        if d < 1.0:  # within 1m → advance
            self.current_wp_idx += 1
            if self.current_wp_idx >= len(self.current_path):
                self.get_logger().info("🎯 Reached final goal!")
                return
            target_idx = self.current_path[self.current_wp_idx]

        self.publish_waypoint(target_idx)
        self.publish_markers()

    def set_goal_callback(self, request, response):
        goal_idx = request.data
        if goal_idx not in self.nodes:
            response.success = False
            response.message = f"❌ Goal {goal_idx} not found."
            return response

        # 🔄 Cancel previous plan
        self.current_path = []
        self.current_wp_idx = 0

        start_idx = self.find_nearest_node(self.current_pose)
        self.get_logger().info(f"🔄 New request: planning path from {start_idx} → {goal_idx}")

        path = self.plan_path(start_idx, goal_idx)
        if not path:
            response.success = False
            response.message = f"⚠️ No path found to {goal_idx}."
            return response

        self.current_path = path
        self.current_wp_idx = 1 if len(path) > 1 else 0
        self.publish_markers()
        self.get_logger().info(f"✅ New path planned: {path}")
        response.success = True
        response.message = f"New path planned to {goal_idx}, {len(path)} waypoints."
        return response


def main():
    rclpy.init()
    node = MapNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

