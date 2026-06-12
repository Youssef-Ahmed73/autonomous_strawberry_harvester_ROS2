# ashr_autonomy/autonomus_node/states/state_retreat.py

from ashr_autonomy.autonomus_node.states.state_base import StateBase
import copy

class StateRetreat(StateBase):
    def __init__(self):
        super().__init__('RETREAT')

    def enter(self, context):
        context.get_logger().info("Entering RETREAT state. Clearing the canopy...")

    def execute(self, context):
        # 1. Calculate the retreat pose
        retreat_pose = self.calculate_retreat_pose(context.active_target, context.retreat_distance)

        # 2. Plan and Execute
        success = context.plan_and_execute_pose(retreat_pose)

        if success:
            context.get_logger().info("Retreat complete.")
            # Clear the target so we don't accidentally harvest it again
            context.active_target = None 
            return 'IDLE'
        else:
            context.get_logger().error("Retreat failed! Operator intervention required.")
            return None # Halt state machine

    def exit(self, context):
        pass

    def calculate_retreat_pose(self, target_pose_stamped, offset_distance):
        """Shifts the pose backward along local Z-axis."""
        offset_pose = copy.deepcopy(target_pose_stamped)
        q = target_pose_stamped.pose.orientation
        
        z_x = 2 * (q.x * q.z + q.w * q.y)
        z_y = 2 * (q.y * q.z - q.w * q.x)
        z_z = 1 - 2 * (q.x**2 + q.y**2)
        
        offset_pose.pose.position.x -= z_x * offset_distance
        offset_pose.pose.position.y -= z_y * offset_distance
        offset_pose.pose.position.z -= z_z * offset_distance
        
        return offset_pose