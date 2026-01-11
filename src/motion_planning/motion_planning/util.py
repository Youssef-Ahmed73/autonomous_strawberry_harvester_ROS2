#!/usr/bin/env python3
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive

def pose_to_constraints(target_pose, link_name="tool0"):
    goal_constraint = Constraints()

    # Position constraint
    pos_constraint = PositionConstraint()
    pos_constraint.header.frame_id = target_pose.header.frame_id
    pos_constraint.link_name = link_name
    pos_constraint.target_point_offset.x = 0.0
    pos_constraint.target_point_offset.y = 0.0
    pos_constraint.target_point_offset.z = 0.0

    primitive = SolidPrimitive()
    primitive.type = SolidPrimitive.SPHERE
    primitive.dimensions = [0.01]
    pos_constraint.constraint_region.primitives.append(primitive)
    pos_constraint.constraint_region.primitive_poses.append(target_pose.pose)
    pos_constraint.weight = 1.0

    # Orientation constraint
    ori_constraint = OrientationConstraint()
    ori_constraint.header.frame_id = target_pose.header.frame_id
    ori_constraint.link_name = link_name
    ori_constraint.orientation = target_pose.pose.orientation
    ori_constraint.absolute_x_axis_tolerance = 0.1
    ori_constraint.absolute_y_axis_tolerance = 0.1
    ori_constraint.absolute_z_axis_tolerance = 0.1
    ori_constraint.weight = 1.0

    # Add constraints
    goal_constraint.position_constraints.append(pos_constraint)
    goal_constraint.orientation_constraints.append(ori_constraint)
    return goal_constraint
