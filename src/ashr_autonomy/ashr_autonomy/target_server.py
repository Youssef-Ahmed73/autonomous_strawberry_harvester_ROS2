import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ashr_interfaces.srv import GetHarvestTarget

class TargetServer(Node):
    def __init__(self):
        super().__init__('target_server_node')
        
        self.latest_pose = None
        
        self.target_sub = self.create_subscription(
            PoseStamped,
            '/target_pose',
            self.pose_callback,
            10
        )
        
        self.srv = self.create_service(
            GetHarvestTarget,
            '/get_harvest_target',
            self.service_callback
        )

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg

    def service_callback(self, request, response):
        if self.latest_pose is not None:
            response.success = True
            response.target_pose = self.latest_pose
            self.latest_pose = None
        else:
            response.success = False
            
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TargetServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()