#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from motion_planning.util import pose_to_constraints
from tf_transformations import quaternion_from_euler


class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner')

        # MoveIt2 action client
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("MoveGroup action server available!")

        # Create goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "arm"  # your MoveIt2 planning group
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 60.0

        # Target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.0
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.6

        roll = 0
        pitch = 0
        yaw = 0
        qx, qy, qz, qw = quaternion_from_euler(roll, pitch, yaw)
        target_pose.pose.orientation.x = qx
        target_pose.pose.orientation.y = qy
        target_pose.pose.orientation.z = qz
        target_pose.pose.orientation.w = qw        

        # Convert PoseStamped to Constraints using utility function
        goal_constraint = pose_to_constraints(target_pose, link_name="tool0")
        goal_msg.request.goal_constraints.append(goal_constraint)

        self._send_goal(goal_msg)
        

    def _send_goal(self, goal_msg):
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected!")
            return
        self.get_logger().info("Goal accepted. Waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._get_result_callback)

    def _get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Plan status: {result.error_code.val}")
        # The trajectory is in result.trajectory
        # Optionally, send to /execute_trajectory action next

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
