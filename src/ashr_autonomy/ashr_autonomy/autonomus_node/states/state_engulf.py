# ashr_autonomy/autonomus_node/states/state_engulf.py

from ashr_autonomy.autonomus_node.states.state_base import StateBase
import copy
import time

class StateEngulf(StateBase):
    def __init__(self):
        super().__init__('ENGULF')
        self.motion_complete = False
        self.iris_closed = False
        self.action_start_time = 0

    def enter(self, context):
        context.get_logger().info("Entering ENGULF state. Plunging into fruit...")
        self.motion_complete = False
        self.iris_closed = False

    def execute(self, context):
        # 1. Execute the plunge motion
        if not self.motion_complete:
            # We move directly to the raw target pose (center of the fruit)
            # You can offset this forward if the center of your chamber is deeper than the grasp_target_link
            engulf_pose = copy.deepcopy(context.active_target)
            
            success = context.plan_and_execute_pose(engulf_pose)
            if success:
                self.motion_complete = True
                self.action_start_time = time.time()
                # Close the Iris to secure the fruit, keep Scissors and Door open
                context.set_end_effector(scissors_open=True, door_open=True, iris_open=False)
                context.get_logger().info("Target engulfed. Closing Iris.")
            else:
                context.get_logger().error("Engulf motion failed! Retreating safely.")
                return 'RETREAT'
                
        # 2. Wait a moment for the Iris physical actuation
        if self.motion_complete and not self.iris_closed:
            if time.time() - self.action_start_time > 1.0: # 1 second for Iris to close
                self.iris_closed = True
                return 'INSPECT'

        return None

    def exit(self, context):
        pass