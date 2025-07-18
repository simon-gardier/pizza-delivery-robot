import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion, Twist, Pose
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from action_msgs.msg import GoalStatus
from ros2_aruco_interfaces.msg import ArucoMarkers
from std_msgs.msg import Bool, Int32MultiArray
import numpy as np
import math
import time
from rclpy.duration import Duration
import tf2_ros
import tf_transformations
from enum import Enum, auto
import heapq
from geometry_msgs.msg import TransformStamped

class Robonardo(Node):
    EXPLORE_UPDATE_PERIOD_SECONDS = 5.0
    DELIVERY_TIMER_SECONDS = 3.0
    CLIENTS_NB = 10
    MAX_ROTATION_SPEED = 0.5
    def __init__(self):
        super().__init__("robonardo")
        self.get_logger().info("Robonardo started, 🍕 on the way!")

        # Attributes setup
        self.nav_to_pose_client         = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.current_goal_handle        = None
        self.map_data                   = None
        self.robot_position             = (0.0, 0.0)
        self.delivery_requests          = set()
        self.clients_poses              = {}
        self.client_id_being_delivered  = None
        self.rotating                   = False
        self.exploration_done           = False
        self.refine_search              = False
        self.exploration_update_timer   = None

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)

        # Topics
        self.clients_found_sub = self.create_subscription(      # for clients positions
            Int32MultiArray, '/clients_found', self.client_aruco_found_callback, 10
        )
        self.delivery_sub = self.create_subscription(           # for clients waiting for delivery
            Int32MultiArray, '/delivery_locations', self.delivery_requested_callback, 10
        )
        self.map_sub = self.create_subscription(                # for map data
            OccupancyGrid, "/map", self.map_callback, 10
        )
        # self.pose_sub = self.create_subscription(               # for robot pose
        #     PoseWithCovarianceStamped, "/amcl_pose", self.robot_pose_callback, 10
        # )
        self.rotate_pub = self.create_publisher(                # for rotation
            Twist, '/cmd_vel', 10
        )

        # First rotation at the start to find any aruco hidden behind us at the start ;)
        self.do_complete_rotation()

    def resume_exploration(self):
        if self.exploration_update_timer is None and not self.exploration_done:
            self.exploration_update_timer = self.create_timer(Robonardo.EXPLORE_UPDATE_PERIOD_SECONDS, self.schedule_exploration)
            self.get_logger().info("Exploration resumed...")

    def pause_exploration_and_deliver_to(self, client_id):
        if self.exploration_update_timer is not None:
            self.exploration_update_timer.cancel()
            self.exploration_update_timer = None
        if self.current_goal_handle is not None:
            cancel_future = self.current_goal_handle.cancel_goal_async()
            def after_cancel(_):
                self.current_goal_handle = None
                if self.client_id_being_delivered is not None:
                    self.deliver_to(self.client_id_being_delivered)
            cancel_future.add_done_callback(after_cancel)
        self.get_logger().info("Exploration paused...")
        self.deliver_to(client_id)

    def map_callback(self, msg):
        """
        Update the attribute representing the map data (known regions, unknown map center position,...)
        msg is received on /map, sent from the cartographer node
        The map data, in row-major order, starting with (0,0). Occupancy probabilities are in the range [0,100]. 
        Unseen is -1. Empty cell is 0, values under 50 are to be explored, values above 50 are considered occupied.
        """
        self.map_data = msg

    def explore_to(self, pos_x_y):
        """
        Prepare a goal position message and send it to Nav2 action server
        """
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = pos_x_y[0]
        goal_msg.pose.position.y = pos_x_y[1]
        goal_msg.pose.orientation.w = 1.0

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        self.nav_to_pose_client.wait_for_server()

        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.explore_to_response_callback)

    def explore_to_response_callback(self, future):
        """
        Callback used by Nav2 to know if goal has been accepted or no
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌  Exploration goal rejected")
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.explore_to_complete_callback)

    def explore_to_complete_callback(self, future):
        """
        Callback used by Nav2 to know if the navigation was sucessfull (= goal position reached) or not.
        """
        try:
            _ = future.result().result
            self.get_logger().info(f"Exploration success...")
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            if len(self.clients_poses) < Robonardo.CLIENTS_NB:
                self.do_complete_rotation()

        except Exception as e:
            self.get_logger().error(f"❌  Exploration fail: {e}")
            
    def client_aruco_found_callback(self, msg):
        """ Store the detected markers poses. """
        new_client_found = False
        for client_id in msg.data:
            if client_id not in self.clients_poses:
                self.get_logger().info(f"✉️  Client {client_id} found...")
                new_client_found = True
            client_aruco_frame = f"aruco_{client_id}_front"
            try:
                client_transform = self.tf_buffer.lookup_transform('map', client_aruco_frame, rclpy.time.Time())
            except Exception as e:
                continue
            pose = Pose()
            pose.position.x = client_transform.transform.translation.x
            pose.position.y = client_transform.transform.translation.y
            pose.position.z = client_transform.transform.translation.z
            pose.orientation = client_transform.transform.rotation
            self.clients_poses[client_id] = pose
        if new_client_found:
            self.schedule_delivery()

    def delivery_requested_callback(self, msg):
        """ Add the new clients to the waiting list and update the navigation. """
        new_clients = set([client for client in set(msg.data) if client not in self.delivery_requests])
        if not new_clients:
            return
        self.delivery_requests.update(new_clients)
        self.get_logger().info(f"✉️  New client(s) waiting: {new_clients}, Set of all clients waiting: {self.delivery_requests}...")
        self.schedule_delivery()

    def do_complete_rotation(self):
        """
        Start a non-blocking complete rotation using a timer.
        """
        if self.rotating:
            self.get_logger().info("Already rotating, skipping rotation...")
            return
        self.rotating = True
        self.get_logger().info("Rotation started...")
        self.rotation_start_time = self.get_clock().now()
        self.rotation_timer = self.create_timer(0.1, self.do_complete_rotation_callback)

    def do_complete_rotation_callback(self):
        if (self.client_id_being_delivered is not None):
            self.rotation_timer.cancel()
            self.get_logger().info("Rotation cancelled, delivery in progress...")
            self.rotating = False
            return
        elapsed = self.get_clock().now() - self.rotation_start_time
        rotation_time = Duration(seconds=2*math.pi / Robonardo.MAX_ROTATION_SPEED + 2)
        twist = Twist()
        if elapsed < rotation_time:
            twist.angular.z = Robonardo.MAX_ROTATION_SPEED
            self.rotate_pub.publish(twist)
        else:
            self.rotation_timer.cancel()
            twist.angular.z = 0.0
            self.rotate_pub.publish(twist)
            self.rotating = False
            self.get_logger().info("Rotation finished...")
            self.schedule_delivery()

    def find_frontiers(self, map_array):
        """
        Find frontiers in the map array.
        """
        min_wall = 55
        search_radius = 4
        robot_danger_radius = 4
        ratio_of_unknown_cells = (search_radius*2)*1.5
        if self.refine_search:
            search_radius = 2
            robot_danger_radius = 1
            ratio_of_unknown_cells = (search_radius*2)
        frontiers = set()
        max_rows, max_cols = map_array.shape
        for row in range(search_radius, max_rows - search_radius):
            for col in range(search_radius, max_cols - search_radius):
                if 0 < map_array[row, col] <= min_wall:
                    neighbors = map_array[(row-search_radius):(row+search_radius)+1, (col-search_radius):(col+search_radius)+1]
                    if len([n for n in neighbors.flatten() if 0 < n <= min_wall]) < ratio_of_unknown_cells:
                        continue
                    if not self.refine_search:
                        if np.any(neighbors > min_wall):
                            continue
                    # Find a neighbor cell with value 0 that is not near a wall (value > 50)
                    found_frontier = False
                    for n_row in range(row - robot_danger_radius, row + robot_danger_radius + 1):
                        for n_col in range(col - robot_danger_radius, col + robot_danger_radius + 1):
                            if map_array[n_row, n_col] == 0:
                                danger_zone = map_array[    max(0, n_row-robot_danger_radius):min(n_row+robot_danger_radius, max_rows),
                                                            max(0, n_col-robot_danger_radius):min(n_col+robot_danger_radius, max_cols)]
                                direct_neighbors = map_array[(n_row-1):(n_row+2), (n_col-1):(n_col+2)]
                                if np.any(danger_zone > min_wall) or np.any(direct_neighbors != 0):
                                    continue
                                else:
                                    frontiers.add((n_row, n_col))
                                    found_frontier = True
                                    break
                        if found_frontier:
                            break
        return frontiers

    def choose_frontier(self, frontiers):
        map_info = self.map_data.info
        res = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y

        min_dist = float("inf")
        chosen = None

        for row, col in frontiers:
            x = col * res + origin_x
            y = row * res + origin_y
            
            # DEBUG CODE
            # Publish a static transform for current frontier as a child of the map frame 
            # (to check in RViz if all the frontiers are found and if the robot goes to the nearest)
            # frontier_tf = TransformStamped()
            # frontier_tf.header.stamp = self.get_clock().now().to_msg()
            # frontier_tf.header.frame_id = "map"
            # frontier_tf.child_frame_id = f"frontier_{row}_{col}"
            # frontier_tf.transform.translation.x = x
            # frontier_tf.transform.translation.y = y
            # frontier_tf.transform.translation.z = 0.0
            # frontier_tf.transform.rotation.x = 0.0
            # frontier_tf.transform.rotation.y = 0.0
            # frontier_tf.transform.rotation.z = 0.0
            # frontier_tf.transform.rotation.w = 1.0
            # self.static_broadcaster.sendTransform(frontier_tf)
            # END DEBUG CODE

            dist = self.distance_to_pos((row, col))
            if dist < min_dist:
                min_dist = dist
                chosen = (row, col)
        return chosen

    def schedule_exploration(self):
        """
        Exploration logic method.
        Called every 5s.
        """
        self.exploration_update_timer.cancel()
        self.exploration_update_timer = None
        if self.refine_search and len(self.clients_poses) == Robonardo.CLIENTS_NB:
            self.get_logger().info("All clients found! Stopping exploration...")
            self.current_goal_handle = None
            self.exploration_done = True
            return

        if self.current_goal_handle is not None or self.rotating:
            self.exploration_update_timer = self.create_timer(Robonardo.EXPLORE_UPDATE_PERIOD_SECONDS, self.schedule_exploration)
            return

        if self.map_data is None:
            self.get_logger().info("No map data yet...")
            self.exploration_update_timer = self.create_timer(Robonardo.EXPLORE_UPDATE_PERIOD_SECONDS, self.schedule_exploration)
            return

        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width)
        )

        frontiers = self.find_frontiers(map_array)
        self.get_logger().info(f"Found {len(frontiers)} frontiers.")
        chosen = self.choose_frontier(frontiers)
        self.get_logger().info("Exploration decision:")
        if chosen is None:
            if not self.refine_search:
                self.get_logger().warn("➡️  All frontiers visited.")
                if len(self.clients_poses) > Robonardo.CLIENTS_NB:
                    self.get_logger().warn("➡️  And no more clients to find! Stopping exploration...")
                    self.exploration_done = True
                else:
                    self.refine_search = True
                    self.get_logger().warn(f"➡️  But still {Robonardo.CLIENTS_NB - len(self.clients_poses)} clients to find, refining search...")
                    self.exploration_update_timer = self.create_timer(Robonardo.EXPLORE_UPDATE_PERIOD_SECONDS, self.schedule_exploration)
            else:
                self.get_logger().warn(f"➡️  Refined search finished with {Robonardo.CLIENTS_NB - len(self.clients_poses)} clients remaining :( Stopping exploration...")
                self.current_goal_handle = None
                self.exploration_done = True
            return
        np.set_printoptions(threshold=np.inf, linewidth=200)
        self.get_logger().info(f"\n{map_array}")
        row, col = chosen
        goal_x = (
            col * self.map_data.info.resolution + self.map_data.info.origin.position.x
        )
        goal_y = (
            row * self.map_data.info.resolution + self.map_data.info.origin.position.y
        )
        self.get_logger().info(f"➡️  Exploration to {(goal_x, goal_y)}")
        self.explore_to((goal_x, goal_y))
        self.exploration_update_timer = self.create_timer(Robonardo.EXPLORE_UPDATE_PERIOD_SECONDS, self.schedule_exploration)

    def deliver_to(self, client_id):
        """
        Prepare a goal position message and send it to Nav2 action server.
        """
        client_pose = self.clients_poses[client_id]

        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = client_pose.position.x
        goal_msg.pose.position.y = client_pose.position.y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation = client_pose.orientation

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal_msg
        self.nav_to_pose_client.wait_for_server()

        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        send_goal_future.add_done_callback(self.deliver_to_response_callback)

    def deliver_to_response_callback(self, future):
        """
        Callback used by Nav2 to know if goal has been accepted or no
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("❌  Delivery goal rejected")
            return
        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.deliver_to_complete_callback)

    def deliver_to_complete_callback(self, future):
        """
        Callback used by Nav2 to know if the navigation was sucessfull (= goal position reached) or not.
        """
        try:
            _ = future.result().result
            self.get_logger().info(f"🔔  Delivery complete, waiting {Robonardo.DELIVERY_TIMER_SECONDS} seconds before continuing...")
            time.sleep(Robonardo.DELIVERY_TIMER_SECONDS)
            self.get_logger().info(f"End of delivery...")
            if self.current_goal_handle is not None:
                self.current_goal_handle.cancel_goal_async()
                self.current_goal_handle = None
            self.client_id_being_delivered = None
            if len(self.clients_poses) < Robonardo.CLIENTS_NB:
                self.do_complete_rotation()
            else:
                self.schedule_delivery()
        except Exception as e:
            self.get_logger().info(f"❌  Navigation fail: {e}")

    def distance_to_pos(self, goal_pos):
        """
        Compute the distance from the robot to a given position using A* algorithm.

        Args:
            goal_pos (tuple[int, int]): The goal position (indexes) as a tuple (row, col) in the map array.
        Returns:
            float: The distance in meters from the robot to the goal position.
        """
        # Get map array.
        map_array = np.array(self.map_data.data).reshape(
            (self.map_data.info.height, self.map_data.info.width)
        )
        # Get dimensions
        rows, cols = map_array.shape

        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(), timeout=Duration(seconds=1.0)
            )
            self.robot_position = (transform.transform.translation.x, transform.transform.translation.y)
        except Exception as e:
            self.get_logger().error(f"❌  Could not get robot position: {e}")
            return float("inf")

        start_pos = (
            int((self.robot_position[1] - self.map_data.info.origin.position.y) / self.map_data.info.resolution),
            int((self.robot_position[0] - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        )

        def in_map(cell):
            """ Check if the cell is in the map. """
            return (0 <= cell[0] < rows) and (0 <= cell[1] < cols)

        def heuristic(a, b):
            """ Heuristic function for A* algorithm. """
            # We need the hypotenuse distance for the heuristic because we consider diagonal moves.
            return math.hypot(a[0]-b[0], a[1]-b[1])

        # Neighbor directions.
        neighbors = [(-1, 0), (1, 0), (0, -1), (0, 1),
                     (-1, -1), (-1, 1), (1, -1), (1, 1)]

        priority_queue = []
        # Start with the robot position
        heapq.heappush(priority_queue, (heuristic(start_pos, goal_pos), 0, start_pos))
        cost_so_far = {start_pos: 0}

        found = False
        while priority_queue:
            _, current_cost, current = heapq.heappop(priority_queue)
            # Stop if we reached the goal position.
            if current == goal_pos:
                found = True
                break
            # Add the neighbors to the open set.
            for dx, dy in neighbors:
                next_cell = (current[0] + dx, current[1] + dy)
                if not in_map(next_cell) or map_array[next_cell[0], next_cell[1]] != 0:
                    continue
                # Cost is not constant because we also use diagonal moves.
                move_cost = math.hypot(dx, dy)
                new_cost = current_cost + move_cost
                if next_cell not in cost_so_far or new_cost < cost_so_far[next_cell]:
                    cost_so_far[next_cell] = new_cost
                    priority = new_cost + heuristic(next_cell, goal_pos)
                    heapq.heappush(priority_queue, (priority, new_cost, next_cell))
        if found:
            # Return distance in meters for interpretation.
            return cost_so_far[goal_pos] * self.map_data.info.resolution
        else:
            # If the goal position is not reachable, return infinity.
            return float("inf")

    def global_pos_to_map_index(self, position):
        """
        Convert a global position (x, y) to a map index (row, col).
        """
        if self.map_data is None:
            return None
        res = self.map_data.info.resolution
        row = int((position[1] - self.map_data.info.origin.position.y) / res)
        col = int((position[0] - self.map_data.info.origin.position.x) / res)
        return (row, col)
    
    def schedule_delivery(self):
        if self.rotating:
            self.get_logger().info(f"Robot is rotating, delaying delivery...")
            return

        self.get_logger().info(f"Delivery decision:")
        if not self.clients_poses:
            self.get_logger().info(f"➡️  No client positions known, resuming exploration...")
            self.resume_exploration()
            return

        if not self.delivery_requests:
            if self.client_id_being_delivered is None:
                self.get_logger().info(f"➡️  No delivery requests and no delivery in progress, resuming exploration...")
                self.resume_exploration()
                return

        available_clients = [client for client in self.delivery_requests if client in self.clients_poses]
        if not available_clients:
            if self.client_id_being_delivered is None:
                self.get_logger().info(f"➡️  No client positions known for the delivery requests, resuming exploration...")
                self.resume_exploration()
            return

        def distance_to_robot(client_id):
            client_pose = self.clients_poses[client_id].position
            goal_pos = self.global_pos_to_map_index((client_pose.x, client_pose.y))
            dist = self.distance_to_pos(goal_pos)
            return dist
        closest_client_id = min(available_clients, key=distance_to_robot)
        self.delivery_requests.remove(closest_client_id)

        if self.client_id_being_delivered is not None:
            current_pose = self.clients_poses[self.client_id_being_delivered].position
            closest_pose = self.clients_poses[closest_client_id].position
            current_index = self.global_pos_to_map_index((current_pose.x, current_pose.y))
            closest_index = self.global_pos_to_map_index((closest_pose.x, closest_pose.y))
            dist_current = self.distance_to_pos(current_index)
            dist_closest = self.distance_to_pos(closest_index)
            if dist_current < dist_closest:
                self.delivery_requests.add(closest_client_id)
                self.get_logger().info(f"➡️  No client closer than client {self.client_id_being_delivered} already being delivered...")
                return
            else:
                self.get_logger().info(f"➡️  Switching delivery from {self.client_id_being_delivered} to {closest_client_id}...")
                self.delivery_requests.add(self.client_id_being_delivered)
                self.client_id_being_delivered = closest_client_id
                cancel_future = self.current_goal_handle.cancel_goal_async()
                def after_cancel(_):
                    self.current_goal_handle = None
                    self.deliver_to(self.client_id_being_delivered)
                cancel_future.add_done_callback(after_cancel)
        else:
            self.get_logger().info(f"➡️  Starting delivery for client {closest_client_id}...")
            self.client_id_being_delivered = closest_client_id
            self.pause_exploration_and_deliver_to(closest_client_id)

def main(args=None):
    rclpy.init(args=args)
    node = Robonardo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("❌  Robonardo node stopped with ctrl+c...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
