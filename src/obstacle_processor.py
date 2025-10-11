#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point32, TwistWithCovariance, Point
from std_msgs.msg import Header, ColorRGBA
from costmap_converter_msgs.msg import ObstacleArrayMsg, ObstacleMsg
from visualization_msgs.msg import Marker, MarkerArray
from rclpy.duration import Duration

class ObstacleProcessor(Node):
	def __init__(self):
		super().__init__('obstacle_processor')
		self.get_logger().info("Obstacle_processor initialized")

		self.obstacle_publisher = self.create_publisher(ObstacleArrayMsg, '/obstacles', 1)
		self.trajectory_publisher = self.create_publisher(MarkerArray, '/obstacle_trajectories', 1)

		self.subscription = self.create_subscription(PoseArray, '/person_pose_info_with_frame', self.pose_callback,1) # added frame id in the pose_frame_id_adder.py

	def pose_callback(self, msg: PoseArray):
		if not msg.poses:
			return

		pose = msg.poses[0]  # There is only one dynamic obstacle(standing person) added in the world 
		# idx = 4               # Known obstacle id which is in the edifice world
		idx = 4              		# Known obstacle id which is in the sonoma.sdf world
		msg_t_stamp = msg.header.stamp

		obstacle_arr_msg = ObstacleArrayMsg()
		obstacle_arr_msg.header = Header()
		obstacle_arr_msg.header.stamp = msg_t_stamp
		obstacle_arr_msg.header.frame_id = 'map'
		obstacle_arr_msg.obstacles = []

		trajectory_markers = MarkerArray()

		velocity = TwistWithCovariance()

		# Create obstacle message
		obstacle_msg = ObstacleMsg()
		obstacle_msg.header = Header()
		obstacle_msg.header.stamp = msg_t_stamp
		obstacle_msg.header.frame_id = 'map' # since map is the frame id all over used in nav2
		obstacle_msg.radius = self.calculate_radius(pose)
		obstacle_msg.id = idx
		obstacle_msg.orientation = pose.orientation
		obstacle_msg.velocities = velocity
		obstacle_msg.polygon.points = [
			Point32(x=pose.position.x + 5.6, 
					 y=pose.position.y, 
					 z=pose.position.z)
		]

		obstacle_arr_msg.obstacles.append(obstacle_msg)

		trajectory_marker = self.publish_trajectory(pose, msg_t_stamp, 'map', idx)
		trajectory_markers.markers.append(trajectory_marker)

		self.obstacle_publisher.publish(obstacle_arr_msg)
		self.trajectory_publisher.publish(trajectory_markers)

	def calculate_radius(self, pose):
		return 0.7

	def publish_trajectory(self, pose, timestamp, frame_id, idx):
		marker = Marker()
		marker.header.frame_id = frame_id
		marker.header.stamp = timestamp
		marker.ns = "obstacle_trajectory"
		marker.id = idx
		marker.type = Marker.SPHERE
		marker.action = Marker.ADD
		marker.scale.x = 0.5
		marker.scale.y = 0.5
		marker.scale.z = 0.5
		marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
			# added this 5.6 because there is an offset between the map and world
			# (5.6 in x is where the robot is spawned in the world see the navigation_launch.py file)
		marker.pose.position.x = pose.position.x + 5.6
		marker.pose.position.y = pose.position.y
		marker.pose.position.z = pose.position.z
		marker.pose.orientation = pose.orientation  
		marker.lifetime = Duration(seconds=0.2).to_msg()
		return marker

def main(args=None):
	rclpy.init(args=args)
	node = ObstacleProcessor()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
	main()
