#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PointStamped, PoseStamped
import tf_transformations
from pyproj import Proj, Transformer

# Map origin / datum (same as in navsat_transform_node)
DATUM_LAT = 38.161491   # origin latitude
DATUM_LON = -122.454644 # origin longitude
DATUM_ALT = 0.0         # altitude in meters

# Initialize a pyproj transformer for WGS84 -> local ENU (meters)
# Using a local tangent plane with the map origin as reference
transformer = Transformer.from_crs(
	f"+proj=latlong +datum=WGS84",
	f"+proj=tmerc +lat_0={DATUM_LAT} +lon_0={DATUM_LON} +k=1 +x_0=0 +y_0=0 +datum=WGS84",
	always_xy=True,
)


class InteractiveWaypointFollower(Node):
	"""
	ROS2 node to send GPS waypoints to Nav2 from Mapviz clicks,
	converting them to map-frame meters.
	"""

	def __init__(self):
		super().__init__("interactive_waypoint_follower")
		self.navigator = BasicNavigator("basic_navigator")

		self.subscription = self.create_subscription(PointStamped,"/clicked_point",self.mapviz_callback,1)

		self.get_logger().info("Interactive waypoint follower ready. Click points in Mapviz.")

	def mapviz_callback(self, msg: PointStamped):
		"""
		Callback for clicked points from Mapviz
		Converts GPS to map-frame meters and sends waypoint to Nav2
		"""
		lat = msg.point.y
		lon = msg.point.x

		x, y = transformer.transform(lon, lat)

		self.get_logger().info(f"Received clicked point: x={x:.2f} m, y={y:.2f} m")

		yaw = getattr(msg.point, "z", 0.0)

		pose = PoseStamped()
		pose.header.frame_id = "map"
		pose.header.stamp = self.get_clock().now().to_msg()
		pose.pose.position.x = x
		pose.pose.position.y = y
		pose.pose.position.z = 0.0

		q = tf_transformations.quaternion_from_euler(0, 0, yaw)
		pose.pose.orientation.x = q[0]
		pose.pose.orientation.y = q[1]
		pose.pose.orientation.z = q[2]
		pose.pose.orientation.w = q[3]

		self.navigator.followWaypoints([pose])
		self.get_logger().info("Navigating to clicked waypoint...")

		while not self.navigator.isTaskComplete():
			rclpy.spin_once(self, timeout_sec=0.1)

		self.get_logger().info("Waypoint reached successfully!")


def main():
    rclpy.init()
    node = InteractiveWaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from nav2_simple_commander.robot_navigator import BasicNavigator
# from geometry_msgs.msg import PointStamped, PoseStamped
# import tf_transformations
# import geodesy.utm

# # Datum for your map origin (meters relative to which we offset GPS points)
# DATUM_LAT = 38.161491   # latitude of map origin
# DATUM_LON = -122.454644 # longitude of map origin
# DATUM_ALT = 0.0         # altitude in meters

# # Maximum distance from origin to stay inside global_costmap for testing
# MAX_DISTANCE_M = 10.0  # meters

# class InteractiveWaypointFollower(Node):
#     """
#     Interactive waypoint follower using Mapviz clicks, with automatic local map scaling
#     """

#     def __init__(self):
#         super().__init__("interactive_waypoint_follower")
#         self.navigator = BasicNavigator("basic_navigator")

#         self.subscription = self.create_subscription(
#             PointStamped,
#             "/clicked_point",
#             self.clicked_point_cb,
#             10
#         )
#         self.get_logger().info("Interactive waypoint follower ready. Click points in Mapviz.")

#         # Precompute datum in UTM
#         self.datum_utm = geodesy.utm.fromLatLong(DATUM_LAT, DATUM_LON)

#     def clicked_point_cb(self, msg: PointStamped):
#         # Convert GPS lat/lon → UTM
#         utm_point = geodesy.utm.fromLatLong(msg.point.y, msg.point.x)

#         # Compute local x/y relative to datum
#         x = utm_point.easting - self.datum_utm.easting
#         y = utm_point.northing - self.datum_utm.northing

#         # Clamp to maximum distance to fit inside global costmap
#         x = max(min(x, MAX_DISTANCE_M), -MAX_DISTANCE_M)
#         y = max(min(y, MAX_DISTANCE_M), -MAX_DISTANCE_M)

#         # Use z field as yaw if provided
#         yaw = getattr(msg.point, "z", 0.0)

#         # Create PoseStamped in map frame
#         pose = PoseStamped()
#         pose.header.frame_id = "map"
#         pose.header.stamp = self.get_clock().now().to_msg()
#         pose.pose.position.x = x
#         pose.pose.position.y = y
#         pose.pose.position.z = 0.0

#         q = tf_transformations.quaternion_from_euler(0, 0, yaw)
#         pose.pose.orientation.x = q[0]
#         pose.pose.orientation.y = q[1]
#         pose.pose.orientation.z = q[2]
#         pose.pose.orientation.w = q[3]

#         self.get_logger().info(f"Waypoint in map frame: x={x:.2f} m, y={y:.2f} m, yaw={yaw:.2f} rad")

#         # Send waypoint to Nav2
#         self.navigator.followWaypoints([pose])
#         self.get_logger().info("Navigating to clicked waypoint...")

#         # Wait until the robot reaches the waypoint
#         while not self.navigator.isTaskComplete():
#             rclpy.spin_once(self, timeout_sec=0.1)

#         self.get_logger().info("Waypoint reached successfully!")

# def main():
#     rclpy.init()
#     node = InteractiveWaypointFollower()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

