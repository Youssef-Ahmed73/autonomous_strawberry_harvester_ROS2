# ashr_autonomy/autonomus_node/states/state_approach.py

from ashr_autonomy.autonomus_node.states.state_base import StateBase
import copy
import math

class StateApproach(StateBase):
    def __init__(self):
        super().__init__('APPROACH')
        self.motion_complete = False

    def enter(self, context):
        context.get_logger().info("Entering APPROACH state.")
        
        # Open the gripper fully before getting near the plant
        context.set_end_effector(scissors_open=True, door_open=True, iris_open=True)
        self.motion_complete = False

    def execute(self, context):
        if self.motion_complete:
            return 'ENGULF'

        if context.active_target is None:
            context.get_logger().error("No active target found. Returning to IDLE.")
            return 'IDLE'

        # 1. Calculate the Pre-Grasp Pose
        pre_grasp_pose = self.calculate_pre_grasp(context.active_target, context.pre_grasp_distance)

        # 2. Plan and Execute
        context.get_logger().info(f"Moving to pre-grasp offset ({context.pre_grasp_distance}m)...")
        success = context.plan_and_execute_pose(pre_grasp_pose)

        if success:
            context.get_logger().info("Approach motion complete.")
            self.motion_complete = True
            return 'ENGULF'
        else:
            context.get_logger().warn("Failed to reach pre-grasp pose. Returning to IDLE.")
            return 'IDLE'

    def exit(self, context):
        pass

    def calculate_pre_grasp(self, target_pose_stamped, offset_distance):
        """
        Shifts the pose backward along its local Z-axis by the offset_distance.
        Assumes the Z-axis of the target pose points INTO the strawberry.
        """
        offset_pose = copy.deepcopy(target_pose_stamped)
        
        # Quaternion math to find the directional vector
        q = target_pose_stamped.pose.orientation
        
        # The local Z-axis vector rotated by the quaternion
        # (Formulas derived from standard quaternion to rotation matrix conversion)
        z_x = 2 * (q.x * q.z + q.w * q.y)
        z_y = 2 * (q.y * q.z - q.w * q.x)
        z_z = 1 - 2 * (q.x**2 + q.y**2)
        
        # Shift the position backward (subtracting the vector * distance)
        offset_pose.pose.position.x -= z_x * offset_distance
        offset_pose.pose.position.y -= z_y * offset_distance
        offset_pose.pose.position.z -= z_z * offset_distance
        
        return offset_pose