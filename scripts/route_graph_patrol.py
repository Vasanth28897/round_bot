## ----------navigate_to_pose version(working)----------------
# !/usr/bin/env python3
import rclpy
import json
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeAndTrackRoute, NavigateToPose
from geometry_msgs.msg import PoseStamped

class RoutePatrol(Node):
    def __init__(self):
        super().__init__('route_patrol')

        graph_file = self.declare_parameter(
            'graph_file',
            '/home/vasanth/ros2_ws/install/round_bot/share/round_bot/graphs/edifice_graph.geojson'
        ).value

        with open(graph_file) as f:
            data = json.load(f)

        self.nodes = sorted([
            {
                'id': feat['properties']['id'],
                'x': feat['geometry']['coordinates'][0],
                'y': feat['geometry']['coordinates'][1]
            }
            for feat in data['features']
            if feat['geometry']['type'] == 'Point'
        ], key=lambda n: n['id'])

        self.get_logger().info(f'Loaded {len(self.nodes)} nodes from graph')

        self._route_client = ActionClient(
            self, ComputeAndTrackRoute, 'compute_and_track_route')
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

        self._current_node_index = 0
        self._current_goal_handle = None

        self.get_logger().info('Waiting for action servers...')
        self._route_client.wait_for_server()
        self._nav_client.wait_for_server()
        self.get_logger().info('Servers ready! Starting patrol...')

        self.send_next_goal()

    def send_next_goal(self):
        if self._current_node_index >= len(self.nodes):
            self.get_logger().info('All nodes visited! Restarting from node 0...')
            self._current_node_index = 0

        node = self.nodes[self._current_node_index]
        self.get_logger().info(
            f'Going to node {node["id"]} at ({node["x"]:.2f}, {node["y"]:.2f}) '
            f'[{self._current_node_index + 1}/{len(self.nodes)}]'
        )

        # Use navigate_to_pose which uses the full BT navigator stack
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = node['x']
        goal.pose.pose.position.y = node['y']
        goal.pose.pose.orientation.w = 1.0

        future = self._nav_client.send_goal_async(
            goal, feedback_callback=self.feedback_cb)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        self._current_goal_handle = future.result()
        if not self._current_goal_handle.accepted:
            self.get_logger().error('Goal rejected! Trying next node...')
            self._current_node_index += 1
            self.send_next_goal()
            return
        self.get_logger().info('Goal accepted!')
        result_future = self._current_goal_handle.get_result_async()
        result_future.add_done_callback(self.result_cb)

    def feedback_cb(self, feedback):
        fb = feedback.feedback
        self.get_logger().info(
            f'Distance remaining: {fb.distance_remaining:.2f}m',
            throttle_duration_sec=3.0)

    def result_cb(self, future):
        status = future.result().status
        node = self.nodes[self._current_node_index]
        if status == 4:  # SUCCEEDED
            self.get_logger().info(f'Reached node {node["id"]}!')
        else:
            self.get_logger().warn(
                f'Failed to reach node {node["id"]} with status {status}, moving to next...')

        self._current_node_index += 1
        self.send_next_goal()

def main():
    rclpy.init()
    node = RoutePatrol()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

## -----------navigate through poses version(not working as expected)----------------
# #!/usr/bin/env python3
# import rclpy
# import json
# from rclpy.node import Node
# from rclpy.action import ActionClient
# from nav2_msgs.action import NavigateThroughPoses
# from geometry_msgs.msg import PoseStamped

# class RoutePatrol(Node):
#     def __init__(self):
#         super().__init__('route_patrol')

#         graph_file = self.declare_parameter(
#             'graph_file',
#             '/home/vasanth/ros2_ws/install/round_bot/share/round_bot/graphs/edifice_graph.geojson'
#         ).value

#         with open(graph_file) as f:
#             data = json.load(f)

#         self.nodes = sorted([
#             {
#                 'id': feat['properties']['id'],
#                 'x': feat['geometry']['coordinates'][0],
#                 'y': feat['geometry']['coordinates'][1]
#             }
#             for feat in data['features']
#             if feat['geometry']['type'] == 'Point'
#         ], key=lambda n: n['id'])

#         self.get_logger().info(f'Loaded {len(self.nodes)} nodes from graph')

#         self._client = ActionClient(
#             self, NavigateThroughPoses, 'navigate_through_poses')

#         self.get_logger().info('Waiting for navigate_through_poses server...')
#         self._client.wait_for_server()
#         self.get_logger().info('Server ready! Starting patrol...')

#         self.send_all_poses()

#     def build_poses(self):
#         poses = []
#         for node in self.nodes:
#             pose = PoseStamped()
#             pose.header.frame_id = 'map'
#             pose.pose.position.x = node['x']
#             pose.pose.position.y = node['y']
#             pose.pose.position.z = 0.0
#             pose.pose.orientation.w = 1.0
#             poses.append(pose)
#         return poses

#     def send_all_poses(self):
#         poses = self.build_poses()
#         self.get_logger().info(
#             f'Sending {len(poses)} waypoints to navigate_through_poses...')

#         goal = NavigateThroughPoses.Goal()
#         goal.poses = poses

#         future = self._client.send_goal_async(
#             goal, feedback_callback=self.feedback_cb)
#         future.add_done_callback(self.goal_response_cb)

#     def goal_response_cb(self, future):
#         goal_handle = future.result()
#         if not goal_handle.accepted:
#             self.get_logger().error('Goal rejected!')
#             return
#         self.get_logger().info('Goal accepted! Robot navigating through all waypoints...')
#         result_future = goal_handle.get_result_async()
#         result_future.add_done_callback(self.result_cb)

#     def feedback_cb(self, feedback):
#         fb = feedback.feedback
#         self.get_logger().info(
#             f'Poses remaining: {fb.number_of_poses_remaining}, '
#             f'Distance remaining: {fb.distance_remaining:.2f}m',
#             throttle_duration_sec=3.0)

#     def result_cb(self, future):
#         status = future.result().status
#         if status == 4:  # SUCCEEDED
#             self.get_logger().info('Completed all waypoints! Restarting patrol...')
#         else:
#             self.get_logger().warn(
#                 f'Navigation ended with status {status}. Restarting...')
#         # Loop forever
#         self.send_all_poses()

# def main():
#     rclpy.init()
#     node = RoutePatrol()
#     rclpy.spin(node)

# if __name__ == '__main__':
#     main()