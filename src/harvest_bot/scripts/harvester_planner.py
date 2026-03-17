#!/usr/bin/env python3
import rclpy
import math
import numpy as np
import time
import copy

from rclpy.node import Node
from rclpy.action import ActionClient
from builtin_interfaces.msg import Duration 
from control_msgs.action import GripperCommand 
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState

if not hasattr(np, 'float'):
    np.float = float
import tf_transformations

# MoveIt Messages
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint, CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive

class MoveItWrapper:
    def __init__(self, node, group_name="ur_manipulator"):
        self.node = node
        self.group_name = group_name
        self.velocity_scaling = 0.1
        self.acceleration_scaling = 0.1
        
        self.move_action_client = ActionClient(node, MoveGroup, '/move_action')
        self.execute_action_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        
        self.current_joints = {}
        self.joint_sub = node.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        
    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos

    def get_current_joint_values(self):
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        values = []
        for name in joint_names:
            if name in self.current_joints:
                values.append(self.current_joints[name])
        return values

    def set_max_velocity_scaling_factor(self, factor):
        self.velocity_scaling = factor

    def set_max_acceleration_scaling_factor(self, factor):
        self.acceleration_scaling = factor
    
    def move_to_joints(self, joint_values):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.max_velocity_scaling_factor = self.velocity_scaling
        goal_msg.request.max_acceleration_scaling_factor = self.acceleration_scaling
        goal_msg.request.allowed_planning_time = 10.0 # Increased for long moves
        goal_msg.request.num_planning_attempts = 15
        goal_msg.request.planner_id = "RRTConnectkConfigDefault"
        
        # Widen the workspace to allow the high "Candle" move
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.min_corner.z = -1.0
        goal_msg.request.workspace_parameters.min_corner.x = -2.0
        goal_msg.request.workspace_parameters.min_corner.y = -2.0
        goal_msg.request.workspace_parameters.max_corner.x = 2.0
        goal_msg.request.workspace_parameters.max_corner.y = 2.0
        goal_msg.request.workspace_parameters.max_corner.z = 2.5 # Allow high reach
        
        jc_list = []
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        for i, val in enumerate(joint_values):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = val
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight = 1.0
            jc_list.append(jc)
        goal_msg.request.goal_constraints = [Constraints(joint_constraints=jc_list)]

        if not self.move_action_client.wait_for_server(timeout_sec=2.0): return False
        
        future = self.move_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        if not future.done() or not future.result().accepted: return False
        
        res_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, res_future, timeout_sec=15.0)
        if not res_future.done(): return False
        
        result = res_future.result().result
        return (result.error_code.val == 1)

    def get_ik_joints(self, target_pose):
        from moveit_msgs.srv import GetPositionIK
        if not hasattr(self, 'ik_client'):
            self.ik_client = self.node.create_client(GetPositionIK, '/compute_ik')
        
        if not self.ik_client.wait_for_service(timeout_sec=1.0): return None

        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state.is_diff = True
        
        p = PoseStamped()
        p.header.frame_id = "base_link"
        p.pose = target_pose.pose
        req.ik_request.pose_stamped = p
        
        req.ik_request.avoid_collisions = False 
        req.ik_request.timeout = Duration(sec=1, nanosec=0)

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()

        if response.error_code.val == 1:
            return response.solution.joint_state.position[:6]
        else:
            return None

class HarvestStateMachine(Node):
    def __init__(self):
        super().__init__('harvest_state_machine')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group = MoveItWrapper(self, "ur_manipulator")
        self.debug_pub = self.create_publisher(PoseStamped, '/debug_wrist_goal', 10)
        self.scene_pub = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.co_pub = self.create_publisher(CollisionObject, '/collision_object', 10)

    def clear_scene(self):
        self.get_logger().info("Cleaning Planning Scene...")
        co = CollisionObject()
        co.operation = CollisionObject.REMOVE
        for name in ["green_pipe", "pipe", "obstacle", "ground", "table", "box"]:
            co.id = name
            self.co_pub.publish(co)
        time.sleep(0.5)

    def get_stable_target(self, target_frame, samples=20, timeout=5.0):
        self.get_logger().info(f"Collecting {samples} samples for stability...")
        points = []
        start_time = time.time()
        
        while len(points) < samples:
            if time.time() - start_time > timeout: break
            try:
                if self.tf_buffer.can_transform('base_link', target_frame, rclpy.time.Time()):
                    trans = self.tf_buffer.lookup_transform('base_link', target_frame, rclpy.time.Time())
                    p = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
                    points.append(p)
            except Exception: pass
            rclpy.spin_once(self, timeout_sec=0.05) 
            
        if not points: return None

        avg_point = np.mean(points, axis=0)
        
        if avg_point[2] < 0.10: 
            self.get_logger().warn(f"Rejecting target Z={avg_point[2]:.2f} (Too low)")
            return None
        
        dist = math.sqrt(avg_point[0]**2 + avg_point[1]**2)
        if dist > 1.2:
            self.get_logger().warn(f"Rejecting target Dist={dist:.2f}m (Too far)")
            return None

        self.get_logger().info(f"Target locked at: {avg_point}")
        return avg_point

    def move_to_unfold(self, target_xyz):
        """
        Executes the 'Candle Maneuver':
        1. Retract to Candle (Safe)
        2. Spin to Target Pan (Zero radius)
        3. Extend to Ready (Down)
        """
        self.get_logger().info("Executing 3-Stage Candle Move...")
        
        # Calculate Targets
        tx, ty, tz = target_xyz
        target_pan = math.atan2(ty, tx)
        
        current_joints = self.move_group.get_current_joint_values()
        if current_joints is None: return False
        current_pan = current_joints[0]
        
        # Smart Unwind Calculation
        diff = target_pan - current_pan
        while diff > math.pi: diff -= 2 * math.pi
        while diff < -math.pi: diff += 2 * math.pi
        final_pan = current_pan + diff
        
        # --- STAGE 1: RETRACT TO CANDLE ---
        # Keep current Pan, Lift arm Straight Up
        candle_joints = [
            current_pan, # Stay at current rotation
            -1.57,       # Shoulder Up (Vertical)
            -0.10,       # Elbow (Almost Straight) - Avoids 0.0 singularity
            -1.57,       # Wrist 1
            1.57,        # Wrist 2
            0.0          # Wrist 3
        ]
        self.get_logger().info("  1. Retracting to CANDLE (Stick Mode)...")
        self.move_group.set_max_velocity_scaling_factor(0.3)
        if not self.move_group.move_to_joints(candle_joints):
            self.get_logger().error("  Failed to Retract.")
            return False

        # --- STAGE 2: SPIN IN PLACE ---
        # Move ONLY the Base Pan to the target angle
        spin_joints = list(candle_joints)
        spin_joints[0] = final_pan
        
        self.get_logger().info(f"  2. Spinning Base ({math.degrees(current_pan):.0f} -> {math.degrees(final_pan):.0f})...")
        if not self.move_group.move_to_joints(spin_joints):
            self.get_logger().error("  Failed to Spin.")
            return False

        # --- STAGE 3: EXTEND TO READY ---
        # Lower arm to reach position
        ready_joints = [
            final_pan,
            -1.57, # Shoulder Up
            -1.57, # Elbow 90
            -1.57, # Wrist Level
            1.57,  # Pointer Forward
            0.0
        ]
        
        self.get_logger().info("  3. Extending to READY POSE...")
        success = self.move_group.move_to_joints(ready_joints)
        self.move_group.set_max_velocity_scaling_factor(1.0)
        return success

    def execute_pick_sequence(self, target_xyz):
        tx, ty, tz = target_xyz
        self.get_logger().info(f"Targeting Berry at: {tx:.2f}, {ty:.2f}, {tz:.2f}")
        self.get_logger().info("Searching for JOINT SPACE Solution...")
        
        STANDOFF = 0.20
        approach_yaw = math.atan2(ty, tx)
        approach_vector = np.array([math.cos(approach_yaw), math.sin(approach_yaw), 0.0])
        wrist_pos = np.array([tx, ty, tz]) - (approach_vector * STANDOFF)

        z_axis = approach_vector / np.linalg.norm(approach_vector)
        world_down = np.array([0.0, 0.0, -1.0])
        x_axis = np.cross(z_axis, world_down)
        if np.linalg.norm(x_axis) < 0.01: x_axis = np.array([1.0, 0.0, 0.0])
        x_axis = x_axis / np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)
        y_axis = y_axis / np.linalg.norm(y_axis)
        
        solution_found = False
        
        for i in range(12):
            roll = i * (math.pi / 6)
            
            c = math.cos(roll)
            s = math.sin(roll)
            x_f = x_axis * c - y_axis * s
            y_f = x_axis * s + y_axis * c
            
            R = np.eye(4)
            R[0:3, 0] = x_f
            R[0:3, 1] = y_f
            R[0:3, 2] = z_axis
            q = tf_transformations.quaternion_from_matrix(R)

            target_pose = PoseStamped()
            target_pose.header.frame_id = "base_link"
            target_pose.pose.position.x = wrist_pos[0]
            target_pose.pose.position.y = wrist_pos[1]
            target_pose.pose.position.z = wrist_pos[2]
            
            target_pose.pose.orientation.x = q[0]
            target_pose.pose.orientation.y = q[1]
            target_pose.pose.orientation.z = q[2]
            target_pose.pose.orientation.w = q[3]
            
            self.debug_pub.publish(target_pose)

            joint_solution = self.move_group.get_ik_joints(target_pose)
            
            if joint_solution:
                self.get_logger().info(f"✅ IK Solution Found (Roll {math.degrees(roll):.0f}). Moving Joints...")
                if self.move_group.move_to_joints(joint_solution):
                    self.get_logger().info("✅ HARVEST COMPLETE!")
                    solution_found = True
                    break
        
        if not solution_found:
             self.get_logger().error("CRITICAL: Unreachable. Target is likely too low/far.")

    def run_harvest_loop(self):
        scan_joints = [-1.53, -1.40, -2.17, -1.13, 1.55, 0.01] 
        self.clear_scene()

        while rclpy.ok():
            self.get_logger().info("--- STARTING NEW CYCLE ---")
            
            self.get_logger().info("Moving to Scan Position...")
            self.move_group.move_to_joints(scan_joints)
            
            self.get_logger().info("Scanning for berries...")
            time.sleep(2.0) 
            target_xyz = self.get_stable_target('detected_berry')
            
            if target_xyz is None:
                self.get_logger().info("No valid berries found. Retrying...")
                continue 
            
            self.get_logger().info("Berry Found!")
            
            if self.move_to_unfold(target_xyz):
                self.execute_pick_sequence(target_xyz)
            else:
                self.get_logger().error("Failed to Unfold. Skipping.")

def main():
    rclpy.init()
    node = HarvestStateMachine()
    try:
        node.run_harvest_loop()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()