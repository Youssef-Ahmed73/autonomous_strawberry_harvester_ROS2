from ashr_autonomy.autonomus_node.states.state_base import StateBase
from ashr_interfaces.srv import GetHarvestTarget

class StateIdle(StateBase):
    def __init__(self):
        super().__init__('IDLE')
        self.future = None
        self.request_sent = False

    def enter(self, context):
        context.get_logger().info("Entering IDLE state.")
        self.request_sent = False
        self.future = None

    def execute(self, context):
        if context.current_harvests >= context.harvests_before_dropoff:
            return 'DROP_OFF'

        # 1. Send the request once
        if not self.request_sent:
            if not context.target_client.wait_for_service(timeout_sec=1.0):
                context.get_logger().warn("Waiting for target server...")
                return None

            request = GetHarvestTarget.Request()
            self.future = context.target_client.call_async(request)
            self.request_sent = True
            return None

        # 2. Check if the future is done on subsequent timer ticks
        if self.future.done():
            result = self.future.result()
            if result is not None and result.success:
                context.get_logger().info("Target received.")
                context.active_target = result.target_pose
                return 'APPROACH'
            else:
                # Server replied but had no target. Reset to ask again.
                self.request_sent = False
                return None

        # Still waiting for future to complete, loop again
        return None

    def exit(self, context):
        pass