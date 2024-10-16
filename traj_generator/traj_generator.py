import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from people_msgs.msg import People, Person
from nav2_msgs.srv import SetInitialPose
from nav2_msgs.action import ComputePathThroughPoses, SmoothPath
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

class TrajGeneratorNode(Node):
    def __init__(self):
        super().__init__('traj_generator')

        # Initialize the service client for /set_initial_pose
        self.initial_pose_client = self.create_client(SetInitialPose, '/set_initial_pose')
        while not self.initial_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/set_initial_pose service not available, waiting...')

        # Initialize the action clients for /compute_path_through_poses and /smooth_path
        self.path_action_client = ActionClient(self, ComputePathThroughPoses, '/compute_path_through_poses')
        self.smooth_path_client = ActionClient(self, SmoothPath, '/smooth_path')

        # Initialize the publisher for the /plan topic
        self.plan_publisher = self.create_publisher(Path, '/plan', QoSProfile(depth=10))

        self.people_publisher = self.create_publisher(People, '/people', 10)

        # Call the actions and service in a sequence
        self.set_initial_pose()
        self.publish_people()
        self.compute_path_through_poses()

    def set_initial_pose(self):
        request = SetInitialPose.Request()
        request.pose.header.frame_id = "map"
        request.pose.pose.pose.position.x = 0.0
        request.pose.pose.pose.position.y = 4.0
        request.pose.pose.pose.position.z = 0.01
        request.pose.pose.pose.orientation.x = 0.0
        request.pose.pose.pose.orientation.y = 0.0
        request.pose.pose.pose.orientation.z = -0.7071068
        request.pose.pose.pose.orientation.w = 0.7071068
        request.pose.pose.covariance = [
            0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ]

        self.initial_pose_client.call_async(request)
        self.get_logger().info('Set initial pose called')

    def publish_people(self):
        msg = People()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()

        person = Person()
        person.name = 'person1'
        person.position.x = 0.0
        person.position.y = 0.0
        person.position.z = 0.0
        person.velocity.x = 0.0
        person.velocity.y = 1.0
        person.velocity.z = 0.0
        msg.people.append(person)

        self.people_publisher.publish(msg)
        self.get_logger().info('Published people message')

    def compute_path_through_poses(self):
        if not self.path_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ComputePathThroughPoses action server not available!')
            return

        goal_msg = ComputePathThroughPoses.Goal()
        goal_msg.goals = []

        # Define multiple goal poses
        # for x, y in [(0.0, 2.0), (-0.8, 0.0), (-0.8, -1.0)]:
        for x, y in [(0.0, 2.0), (-0.8, 0.0)]:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'map'
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.orientation.z = -0.7071068
            pose_stamped.pose.orientation.w = 0.7071068
            goal_msg.goals.append(pose_stamped)

        self.get_logger().info('Sending goal to ComputePathThroughPoses')
        send_goal_future = self.path_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.compute_path_through_poses_response)

    def compute_path_through_poses_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('ComputePathThroughPoses goal was rejected!')
            return

        self.get_logger().info('ComputePathThroughPoses goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.on_path_computed)

    def on_path_computed(self, future):
        result = future.result().result
        self.get_logger().info('Path computed successfully! Sending to SmoothPath')

        # Send the path from ComputePathThroughPoses to SmoothPath
        self.smooth_path(result.path)

    def smooth_path(self, path):
        if not self.smooth_path_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('SmoothPath action server not available!')
            return

        goal_msg = SmoothPath.Goal()
        goal_msg.path = path

        self.get_logger().info('Sending path to SmoothPath')
        send_goal_future = self.smooth_path_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.smooth_path_response)

    def smooth_path_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('SmoothPath goal was rejected!')
            return

        self.get_logger().info('SmoothPath goal accepted!')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.on_path_smoothed)

    def on_path_smoothed(self, future):
        result = future.result().result
        self.get_logger().info('Path smoothed successfully! Publishing to /plan')

        # Publish the smoothed path to /plan
        self.plan_publisher.publish(result.path)
        self.get_logger().info('Smoothed path published to /plan')

        # Schedule shutdown after path is published
        self.get_logger().info('Shutting down the node')
        self.shutdown()

    def shutdown(self):
        # Destroy the node after shutting down
        self.get_logger().info('Destroying node and shutting down')
        self.destroy_node()
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = TrajGeneratorNode()
    rclpy.spin(node)  # Keeps the node alive until shutdown is called

if __name__ == '__main__':
    main()
