#!/usr/bin/env python3
from collections import deque
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from ros2_aruco_interfaces.msg import ArucoMarkers
import tf_transformations
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import Int32MultiArray
import math

class Aruco(Node):
    def __init__(self):
        super().__init__('aruco_transformer')
        self.get_logger().info("Aruco Node Started...")
        self.marker_history = {}
        self.marker_front_history = {}

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Topics subscriptions
        self.subscription = self.create_subscription(
            ArucoMarkers,
            '/aruco_markers',
            self.client_aruco_found_callback,
            10)

        # Topics publishers
        self.clients_found_pub = self.create_publisher(
            Int32MultiArray,
            '/clients_found',
            10)

    def client_aruco_found_callback(self, msg):
        list_of_clients_id = set()
        for i, pose in enumerate(msg.poses):
            marker_id = msg.marker_ids[i]
            if marker_id not in self.marker_history:
                self.marker_history[marker_id] = deque(maxlen=10)
                self.marker_front_history[marker_id] = deque(maxlen=10)

            ## 1. Aruco pose
            # Create PoseStamped in camera_optical_joint frame
            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.header.frame_id = 'camera_optical_joint'
            pose_stamped.pose = pose

            # Transform client aruco pose to map frame
            try:
                pose_map = self.tf_buffer.transform(pose_stamped, 'map')
            except TransformException as ex:
                continue

            self.marker_history[marker_id].append(
                pose_map
            )
            positions = self.marker_history[marker_id]
            avg_x = sum(p.pose.position.x for p in positions) / len(positions)
            avg_y = sum(p.pose.position.y for p in positions) / len(positions)
            avg_z = sum(p.pose.position.z for p in positions) / len(positions)

            qx = sum(p.pose.orientation.x for p in positions) / len(positions)
            qy = sum(p.pose.orientation.y for p in positions) / len(positions)
            qz = sum(p.pose.orientation.z for p in positions) / len(positions)
            qw = sum(p.pose.orientation.w for p in positions) / len(positions)
            norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            avg_orientation = type(positions[0].pose.orientation)()
            avg_orientation.x = qx / norm
            avg_orientation.y = qy / norm
            avg_orientation.z = qz / norm
            avg_orientation.w = qw / norm

            # Put the averaged pose in pose_map
            pose_map.pose.position.x = avg_x
            pose_map.pose.position.y = avg_y
            pose_map.pose.position.z = avg_z
            pose_map.pose.orientation = avg_orientation

            # Broadcast static transform
            static_tf = TransformStamped()
            static_tf.header.stamp = self.get_clock().now().to_msg()
            static_tf.header.frame_id = 'map'
            static_tf.child_frame_id = f'aruco_{marker_id}'
            static_tf.transform.translation.x = pose_map.pose.position.x
            static_tf.transform.translation.y = pose_map.pose.position.y
            static_tf.transform.translation.z = pose_map.pose.position.z
            static_tf.transform.rotation = pose_map.pose.orientation
            self.static_broadcaster.sendTransform(static_tf)
            self.get_logger().info(f"Broadcasted static transform for aruco_{marker_id}...")

            ## 2. Front of the Aruco pose
            front_pose_stamped = PoseStamped()
            front_pose_stamped.header = pose_stamped.header
            front_pose_stamped.pose.position.x = pose_stamped.pose.position.x         # top-down
            front_pose_stamped.pose.position.y = pose_stamped.pose.position.y         # left-right
            front_pose_stamped.pose.position.z = pose_stamped.pose.position.z - 0.5   # depth
            front_pose_stamped.pose.orientation = pose_stamped.pose.orientation
            
            # Set orientation of front_pose_stamped to point towards pose_stamped from the camera's POV
            dx = pose_stamped.pose.position.x - front_pose_stamped.pose.position.x
            dy = pose_stamped.pose.position.y - front_pose_stamped.pose.position.y
            dz = pose_stamped.pose.position.z - front_pose_stamped.pose.position.z

            # Normalize direction vector
            norm = math.sqrt(dx**2 + dy**2 + dz**2)
            if norm == 0:
                front_pose_stamped.pose.orientation = pose_stamped.pose.orientation
            else:
                # x-right, y-down, z-forward)
                direction = [dx / norm, dy / norm, dz / norm]
                # "forward" is -z
                from_vec = [0, 0, -1]
                to_vec = direction
                dot = sum([from_vec[i]*to_vec[i] for i in range(3)])
                if abs(dot - 1.0) < 1e-6:
                    # Vectors are the same, no rotation needed
                    q = [0.0, 0.0, 0.0, 1.0]
                elif abs(dot + 1.0) < 1e-6:
                    # Vectors are opposite, rotate 180 degrees around any axis
                    q = tf_transformations.quaternion_about_axis(math.pi, [1, 0, 0])
                else:
                    axis = [
                        from_vec[1]*to_vec[2] - from_vec[2]*to_vec[1],
                        from_vec[2]*to_vec[0] - from_vec[0]*to_vec[2],
                        from_vec[0]*to_vec[1] - from_vec[1]*to_vec[0]
                    ]
                    axis_norm = math.sqrt(axis[0]**2 + axis[1]**2 + axis[2]**2)
                    if axis_norm != 0:
                        axis = [a / axis_norm for a in axis]
                    angle = math.acos(max(-1.0, min(1.0, dot)))
                    q = tf_transformations.quaternion_about_axis(angle, axis)
                front_pose_stamped.pose.orientation.x = q[0]
                front_pose_stamped.pose.orientation.y = q[1]
                front_pose_stamped.pose.orientation.z = q[2]
                front_pose_stamped.pose.orientation.w = q[3]

            # Our frame axis are permuted... Simple fix by rotating the pose by 270 degrees around the Y axis
            quaternion = (
                front_pose_stamped.pose.orientation.x,
                front_pose_stamped.pose.orientation.y,
                front_pose_stamped.pose.orientation.z,
                front_pose_stamped.pose.orientation.w
            )
            roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)

            # Apply 270 degree (3pi/2) rotation around Y axis
            pitch += 3 * math.pi / 2

            # Convert back to quaternion
            new_quaternion = tf_transformations.quaternion_from_euler(roll, pitch, yaw)
            front_pose_stamped.pose.orientation.x = new_quaternion[0]
            front_pose_stamped.pose.orientation.y = new_quaternion[1]
            front_pose_stamped.pose.orientation.z = new_quaternion[2]
            front_pose_stamped.pose.orientation.w = new_quaternion[3]

            # Transform front_pose_stamped to map frame
            try:
                front_pose_map = self.tf_buffer.transform(front_pose_stamped, 'map')
            except TransformException as ex:
                continue

            self.marker_front_history[marker_id].append(
                front_pose_map
            )

            positions = self.marker_front_history[marker_id]
            avg_x = sum(p.pose.position.x for p in positions) / len(positions)
            avg_y = sum(p.pose.position.y for p in positions) / len(positions)
            avg_z = sum(p.pose.position.z for p in positions) / len(positions)

            qx = sum(p.pose.orientation.x for p in positions) / len(positions)
            qy = sum(p.pose.orientation.y for p in positions) / len(positions)
            qz = sum(p.pose.orientation.z for p in positions) / len(positions)
            qw = sum(p.pose.orientation.w for p in positions) / len(positions)
            norm = math.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            avg_orientation = type(positions[0].pose.orientation)()
            avg_orientation.x = qx / norm
            avg_orientation.y = qy / norm
            avg_orientation.z = qz / norm
            avg_orientation.w = qw / norm

            # Put the averaged pose in front_pose_map
            front_pose_map.pose.position.x = avg_x
            front_pose_map.pose.position.y = avg_y
            front_pose_map.pose.position.z = avg_z
            front_pose_map.pose.orientation = avg_orientation

            # Broadcast static transform for front_pose_stamped
            static_tf_new = TransformStamped()
            static_tf_new.header.stamp = self.get_clock().now().to_msg()
            static_tf_new.header.frame_id = 'map'
            static_tf_new.child_frame_id = f'aruco_{marker_id}_front'
            static_tf_new.transform.translation.x = front_pose_map.pose.position.x
            static_tf_new.transform.translation.y = front_pose_map.pose.position.y
            static_tf_new.transform.translation.z = front_pose_map.pose.position.z
            static_tf_new.transform.rotation = front_pose_map.pose.orientation
            self.static_broadcaster.sendTransform(static_tf_new)
            self.get_logger().info(f"Broadcasted static transform for aruco_{marker_id}_front...")

            list_of_clients_id.add(marker_id)

        if list_of_clients_id:
            msg = Int32MultiArray()
            msg.data = list(list_of_clients_id)
            self.get_logger().info(f"Sending {msg.data}...")
            self.clients_found_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Aruco()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("❌  Aruco node stopped with ctrl+c...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
