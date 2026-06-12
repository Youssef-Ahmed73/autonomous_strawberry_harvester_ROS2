# ashr_autonomy/autonomus_node/states/state_drop_off.py

from ashr_autonomy.autonomus_node.states.state_base import StateBase
import time

class StateDropOff(StateBase):
    def __init__(self):
        super().__init__('DROP_OFF')
        self.at_drop_off = False
        self.drop_start_time = 0

    def enter(self, context):
        context.get_logger().info("Entering DROP_OFF state. Basket full!")
        self.at_drop_off = False

    def execute(self, context):
        if not self.at_drop_off:
            context.arm_group.set_start_state_to_current_state()
            # Command MoveIt to go to a predefined joint configuration
            context.arm_group.set_named_target("drop_off")
            
            plan_result = context.arm_group.plan()
            if plan_result:
                context.moveit.execute(plan_result.trajectory, blocking=True)
                self.at_drop_off = True
                
                # Open the door to drop the strawberries
                context.set_end_effector(scissors_open=True, door_open=False, iris_open=True)
                self.drop_start_time = time.time()
                context.get_logger().info("Dropping payload...")
            else:
                context.get_logger().error("Cannot plan path to drop_off!")
                return None

        # Wait for the door to open and fruit to fall
        if self.at_drop_off and (time.time() - self.drop_start_time > 2.0):
            # Reset counter and close the door
            context.current_harvests = 0
            context.set_end_effector(scissors_open=True, door_open=True, iris_open=True)
            return 'IDLE'

        return None

    def exit(self, context):
        pass