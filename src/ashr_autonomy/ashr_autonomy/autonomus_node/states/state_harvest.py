# ashr_autonomy/autonomus_node/states/state_harvest.py

from ashr_autonomy.autonomus_node.states.state_base import StateBase
import time

class StateHarvest(StateBase):
    def __init__(self):
        super().__init__('HARVEST')
        self.cut_start_time = 0

    def enter(self, context):
        context.get_logger().info("Entering HARVEST state. Cutting stem...")
        # Close the scissors. Iris remains closed.
        context.set_end_effector(scissors_open=False, door_open=True, iris_open=False)
        self.cut_start_time = time.time()
        context.current_harvests += 1
        context.get_logger().info(f"Harvest count: {context.current_harvests}/{context.harvests_before_dropoff}")

    def execute(self, context):
        # Wait 1.5 seconds for the physical blades to close and cut
        if time.time() - self.cut_start_time > 1.5:
            return 'RETREAT'
            
        return None

    def exit(self, context):
        pass