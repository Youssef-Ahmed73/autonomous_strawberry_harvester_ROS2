from ashr_autonomy.autonomus_node.states.state_base import StateBase
from ashr_interfaces.srv import CheckRipeness

class StateInspect(StateBase):
    def __init__(self):
        super().__init__('INSPECT')
        self.future_1 = None
        self.future_2 = None
        self.request_sent = False

    def enter(self, context):
        context.get_logger().info("Entering INSPECT state. Verifying ripeness from both ESP cameras...")
        self.request_sent = False
        self.future_1 = None
        self.future_2 = None

    def execute(self, context):
        if not self.request_sent:
            if not context.ripeness_client_1.wait_for_service(timeout_sec=1.0) or \
               not context.ripeness_client_2.wait_for_service(timeout_sec=1.0):
                context.get_logger().warn("One or both ripeness services offline. Waiting...")
                return None
            
            req = CheckRipeness.Request()
            self.future_1 = context.ripeness_client_1.call_async(req)
            self.future_2 = context.ripeness_client_2.call_async(req)
            self.request_sent = True
            context.get_logger().info("Ripeness requests sent to both cameras.")
            return None

        if self.future_1.done() and self.future_2.done():
            result_1 = self.future_1.result()
            result_2 = self.future_2.result()
            
            if result_1 is not None and result_2 is not None:
                avg_ripeness = (result_1.ripeness_percentage + result_2.ripeness_percentage) / 2.0
                is_ripe = result_1.is_ripe and result_2.is_ripe
                
                context.get_logger().info(f"ESP1 Ripeness: {result_1.ripeness_percentage}%, Ripe: {result_1.is_ripe}")
                context.get_logger().info(f"ESP2 Ripeness: {result_2.ripeness_percentage}%, Ripe: {result_2.is_ripe}")
                context.get_logger().info(f"Combined Average: {avg_ripeness}%, Final Decision: Ripe={is_ripe}")
                
                if is_ripe:
                    return 'HARVEST'
                else:
                    return 'UNDO_ENGULF'
            else:
                context.get_logger().error("One or both ripeness services failed. Failing safe.")
                return 'UNDO_ENGULF'

        return None

    def exit(self, context):
        pass