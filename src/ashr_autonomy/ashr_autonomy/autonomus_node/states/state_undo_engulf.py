# ashr_autonomy/autonomus_node/states/state_undo_engulf.py

from ashr_autonomy.autonomus_node.states.state_base import StateBase
import time

class StateUndoEngulf(StateBase):
    def __init__(self):
        super().__init__('UNDO_ENGULF')
        self.open_start_time = 0

    def enter(self, context):
        context.get_logger().info("Strawberry unripe. Entering UNDO_ENGULF...")
        # Open everything to safely back away
        context.set_end_effector(scissors_open=True, door_open=True, iris_open=True)
        self.open_start_time = time.time()

    def execute(self, context):
        # Give the Iris time to physically clear the fruit
        if time.time() - self.open_start_time > 1.0:
            return 'RETREAT'
            
        return None

    def exit(self, context):
        pass