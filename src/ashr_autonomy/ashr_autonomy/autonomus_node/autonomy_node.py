#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import tf2_ros
from tf2_ros import Buffer, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ashr_interfaces.srv import CheckRipeness, GetHarvestTarget

from ashr_autonomy.autonomus_node.states.state_idle import StateIdle
from ashr_autonomy.autonomus_node.states.state_approach import StateApproach
from ashr_autonomy.autonomus_node.states.state_engulf import StateEngulf
from ashr_autonomy.autonomus_node.states.state_inspect import StateInspect
from ashr_autonomy.autonomus_node.states.state_harvest import StateHarvest
from ashr_autonomy.autonomus_node.states.state_undo_engulf import StateUndoEngulf
from ashr_autonomy.autonomus_node.states.state_retreat import StateRetreat
from ashr_autonomy.autonomus_node.states.state_drop_off import StateDropOff

# MoveIt 2 Python API
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState
from moveit.planning import PlanRequestParameters

class AutonomyFSM(Node):
    def __init__(self):
        super().__init__('autonomy_fsm_node')

        # Load YAML Parameters
        self.declare_parameter('harvests_before_dropoff', 10)
        self.declare_parameter('pre_grasp_distance', 0.15)
        self.declare_parameter('engulf_plunge_depth', 0.18)
        self.declare_parameter('retreat_distance', 0.20)
        self.declare_parameter('cartesian_speed_fraction', 0.1)

        self.harvests_before_dropoff = self.get_parameter('harvests_before_dropoff').value
        self.pre_grasp_distance = self.get_parameter('pre_grasp_distance').value
        self.engulf_plunge_depth = self.get_parameter('engulf_plunge_depth').value
        self.retreat_distance = self.get_parameter('retreat_distance').value
        self.cartesian_speed_fraction = self.get_parameter('cartesian_speed_fraction').value

        self.current_harvests = 0
        self.active_target = None # Will store the PoseStamped from target server

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.ee_pub = self.create_publisher(
            JointTrajectory,
            '/ee_controller/joint_trajectory',
            10
        )

        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        
        self.ripeness_client_1 = self.create_client(
            CheckRipeness, 
            '/esp1/verify_ripeness',
            callback_group=self.service_cb_group
        )
        
        self.ripeness_client_2 = self.create_client(
            CheckRipeness, 
            '/esp2/verify_ripeness',
            callback_group=self.service_cb_group
        )
        
        self.target_client = self.create_client(
            GetHarvestTarget,
            '/get_harvest_target',
            callback_group=self.service_cb_group
        )

        # Initialize MoveIt 2 Python API
        self.get_logger().info("Initializing MoveIt 2...")
        try:
            self.moveit = MoveItPy(node_name="autonomy_fsm_node")
            self.arm_group = self.moveit.get_planning_component("arm")
            self.get_logger().info("MoveIt 2 successfully initialized for group: 'arm'")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize MoveIt 2: {str(e)}")

        self.states = {}
        self.current_state = None
        
        # FSM Tick
        self.timer = self.create_timer(0.1, self.step_fsm)

        # Register all states
        self.add_state(StateIdle())
        self.add_state(StateApproach())
        self.add_state(StateEngulf())
        self.add_state(StateInspect())
        self.add_state(StateHarvest())
        self.add_state(StateUndoEngulf())
        self.add_state(StateRetreat())
        self.add_state(StateDropOff())

        # Start the FSM!
        self.set_initial_state('IDLE')

    def add_state(self, state_obj):
        self.states[state_obj.name] = state_obj

    def set_initial_state(self, state_name: str):
        self.current_state = state_name
        self.states[self.current_state].enter(self)

    def transition_to(self, next_state_name: str):
        if next_state_name == self.current_state:
            return
            
        self.get_logger().info(f"Transitioning: {self.current_state} -> {next_state_name}")
        self.states[self.current_state].exit(self)
        self.current_state = next_state_name
        self.states[self.current_state].enter(self)

    def step_fsm(self):
        if self.current_state is None:
            return

        next_state = self.states[self.current_state].execute(self)
        
        if next_state is not None:
            self.transition_to(next_state)

    def set_end_effector(self, scissors_open: bool, door_open: bool, iris_open: bool):
        msg = JointTrajectory()
        msg.joint_names = ['Active_Scissors_Gear_Joint', 'Door_Joint', 'Iris_Active_Gear_Joint']
        
        point = JointTrajectoryPoint()
        
        scissors_pos = 0.0 if scissors_open else 1.0  # 0.5 is your URDF max, adjust if 1.0 isn't strictly used by HW mapping
        door_pos = 0.0 if door_open else 1.0          # URDF says -3.14 to 0, ensure controller mapping aligns with 0-1 if mapped
        iris_pos = 0.0 if iris_open else 1.0          

        point.positions = [scissors_pos, door_pos, iris_pos]
        point.time_from_start.sec = 1 
        msg.points.append(point)
        
        self.ee_pub.publish(msg)


    # ==========================================
    # MoveIt Helper Methods for the States
    # ==========================================
    def plan_and_execute_pose(self, target_pose_stamped, planner_id="PTP"):
        """Plans and executes a trajectory to a PoseStamped."""
        self.get_logger().info(f"Planning trajectory using {planner_id}...")
        self.arm_group.set_start_state_to_current_state()
        
        # 1. Set the physical goal
        self.arm_group.set_goal_state(pose_stamped_msg=target_pose_stamped, pose_link="grasp_target_link")
        
        # 2. Instantiate parameters (One argument, confirmed)
        plan_params = PlanRequestParameters(self.moveit)
        plan_params.planner_id = planner_id
        plan_params.planning_pipeline = "pilz_industrial_motion_planner"
        
        # 3. Pass the parameters using the exact keyword (Confirmed)
        plan_result = self.arm_group.plan(parameters=plan_params)
        
        if plan_result:
            self.get_logger().info("Plan successful. Executing...")
            success = self.moveit.execute("arm", plan_result.trajectory, blocking=True)
            return success
        else:
            self.get_logger().error("Planning failed.")
            return False


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyFSM()
    
    # Needs MultiThreadedExecutor so the MoveIt execution blocking doesn't freeze the Service Callbacks
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()