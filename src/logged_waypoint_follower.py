#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
import os
import sys
import time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import tf_transformations
import tf2_ros
from ament_index_python.packages import get_package_share_directory
import math

class YamlGpsWaypointParser:
	"""
	Parse GPS waypoints from a YAML file
	"""
	def __init__(self, file_path: str):
		with open(file_path, 'r') as f:
			self.wps_dict = yaml.safe_load(f)

	def get_waypoints(self):
		"""
		Return a list of waypoints as dicts: {latitude, longitude, yaw}
		"""
		waypoints = []
		for wp in self.wps_dict["waypoints"]:
			waypoints.append({
				"latitude": wp["latitude"],
				"longitude": wp["longitude"],
				"yaw": wp["yaw"]
			})
		return waypoints

class GpsWaypointFollower(Node):
	"""
	Follows GPS waypoints using Nav2 and tf2 for coordinate conversion
	"""
	def __init__(self, yaml_file):
		super().__init__('gps_waypoint_follower')
		self.declare_parameter('waypoints_file', yaml_file)
		self.waypoint_parser = YamlGpsWaypointParser(yaml_file)
		self.navigator = BasicNavigator()
		self.tf_buffer = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

		self.gps_fix = None
		self.create_subscription(NavSatFix, '/navsat/fix', self.gps_callback, 10)

	def gps_callback(self, msg: NavSatFix):
		self.gps_fix = msg

	def gps_to_map_pose(self, latitude, longitude, yaw=0.0):
		"""
		Convert GPS waypoint to PoseStamped in map frame using simple ENU offset
		"""
		if self.gps_fix is None:
				self.get_logger().warn("Waiting for first GPS fix...")
				return None

		# ENU approximation: simple offset from current GPS fix
		d_lat = latitude - self.gps_fix.latitude
		d_lon = longitude - self.gps_fix.longitude
		x = d_lon * 111111 * math.cos(math.radians(self.gps_fix.latitude))
		y = d_lat * 111111
		z = 0.0

		q = tf_transformations.quaternion_from_euler(0, 0, yaw)

		pose = PoseStamped()
		pose.header.frame_id = 'map'
		pose.header.stamp = self.get_clock().now().to_msg()
		pose.pose.position.x = x
		pose.pose.position.y = y
		pose.pose.position.z = z
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]

		return pose

	def start_following(self):
		"""
		Start following all GPS waypoints
		"""
		while self.gps_fix is None:
				self.get_logger().info("Waiting for GPS fix...")
				rclpy.spin_once(self, timeout_sec=0.5)

		wps = self.waypoint_parser.get_waypoints()

		self.get_logger().info(f"Loaded {len(wps)} waypoints from YAML file.")
		costmap_width = 100.0
		costmap_height = 100.0
		MIN_X = -costmap_width / 2
		MAX_X = costmap_width / 2
		MIN_Y = -costmap_height / 2
		MAX_Y = costmap_height / 2

		map_poses = []
		for i, wp in enumerate(wps):
			pose = self.gps_to_map_pose(wp['latitude'], wp['longitude'], wp['yaw'])
			if pose is None:
				continue
			x, y = pose.pose.position.x, pose.pose.position.y
			if not (MIN_X <= x <= MAX_X and MIN_Y <= y <= MAX_Y):
				self.get_logger().warn(f" Waypoint {i+1} ({x:.2f}, {y:.2f}) is OUTSIDE the map bounds!")
			else:
				self.get_logger().info(f" Waypoint {i+1} ({x:.2f}, {y:.2f}) is inside the map.")
			map_poses.append(pose)

		for i, pose in enumerate(map_poses):
			self.get_logger().info(f"[INFO] Moving to waypoint {i+1}...")
			self.navigator.goToPose(pose)

			while not self.navigator.isTaskComplete():
				rclpy.spin_once(self, timeout_sec=0.2)
				time.sleep(0.1)

			result = self.navigator.getResult()
			if result == TaskResult.SUCCEEDED:
				self.get_logger().info(f"[INFO] Waypoint {i+1} reached successfully!")
			elif result == TaskResult.CANCELED:
				self.get_logger().warn(f"[WARN] Waypoint {i+1} navigation canceled!")
			else:
				self.get_logger().warn(f"[WARN] Waypoint {i+1} navigation failed! Code: {result}")

		self.get_logger().info("[INFO] All waypoints processed.")

def main(args=None):
	rclpy.init(args=args)

	package_share = get_package_share_directory('round_bot')
	default_yaml_file = os.path.join(package_share, 'config', 'demo_waypoints.yaml')
	yaml_file = sys.argv[1] if len(sys.argv) > 1 else default_yaml_file

	follower = GpsWaypointFollower(yaml_file)
	follower.start_following()

	rclpy.shutdown()

if __name__ == "__main__":
	main()
