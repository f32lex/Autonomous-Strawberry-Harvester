#!/usr/bin/env python3
import rclpy
import math
import numpy as np
import time
import copy
import random

from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from tf2_ros import Buffer, TransformListener
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
from std_msgs.msg import String
import json

if not hasattr(np, 'float'):
    np.float = float
import tf_transformations

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, RobotState, PositionIKRequest
from shape_msgs.msg import SolidPrimitive
from visualization_msgs.msg import Marker

try:
    import serial
except ImportError:
    serial = None
    print("WARNING: pyserial not installed. Servo control will be disabled.")

try:
    from ur_msgs.srv import SetIO
except ImportError:
    SetIO = None
except ImportError:
    SetIO = None 

# Configuration
FORCE_MIRROR_Y = False
TOOL_LENGTH = 0.251
MAX_TURN_RADIANS = 3.2
MAX_REACH_RADIUS = 1.3

class MoveItWrapper:
    def __init__(self, node, group_name="ur_manipulator"):
        self.node = node
        self.group_name = group_name
        self.move_action_client = ActionClient(node, MoveGroup, '/move_action')
        self.execute_action_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
        self.cartesian_service = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.ik_service = node.create_client(GetPositionIK, '/compute_ik')
        self.joint_sub = node.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.current_joints = {}

    def _joint_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos

    def get_current_joint_values(self):
        joint_map = {"shoulder_pan_joint": 0, "shoulder_lift_joint": 0, "elbow_joint": 0,
                     "wrist_1_joint": 0, "wrist_2_joint": 0, "wrist_3_joint": 0}
        ordered_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                         "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        if not self.current_joints: return None
        for name, val in self.current_joints.items():
            if name in joint_map:
                joint_map[name] = val
        return [joint_map[n] for n in ordered_names]
    
    def compute_ik(self, target_pose_stamped, max_attempts=5, position_only_fallback=True):
        """Compute IK solution for a target pose with seed state and retry logic"""
        if not self.ik_service.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().error("IK service not available")
            return None
        
        # Get current joint state as seed
        current_joints = self.get_current_joint_values()
        
        # Try with full pose first
        for attempt in range(max_attempts):
            req = GetPositionIK.Request()
            req.ik_request.group_name = self.group_name
            
            # For retries, we want to try RANDOM seeds to escape local minima
            if current_joints and attempt == 0:
                self.node.get_logger().info("IK Attempt 1: Seeding with Current State")
                req.ik_request.robot_state.joint_state.name = [
                    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
                ]
                req.ik_request.robot_state.joint_state.position = current_joints
            elif attempt > 0:
                self.node.get_logger().info(f"IK Attempt {attempt+1}: Seeding with RANDOM State (Solver Default)")
                # Leaving robot_state empty triggers random seeding in TRAC-IK / KDL
                pass
            
            # Apply small variations to pose on retry attempts
            pose_to_try = copy.deepcopy(target_pose_stamped)
            if attempt > 0:
                # Vary position slightly
                pose_to_try.pose.position.x += random.uniform(-0.02, 0.02)
                pose_to_try.pose.position.y += random.uniform(-0.02, 0.02)
                pose_to_try.pose.position.z += random.uniform(-0.02, 0.02)
                self.node.get_logger().info(f"IK attempt {attempt+1}/{max_attempts} with pose variation")
            
            req.ik_request.pose_stamped = pose_to_try
            req.ik_request.timeout = Duration(sec=5, nanosec=0)
            req.ik_request.avoid_collisions = False
            
            future = self.ik_service.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            
            if not future.done():
                self.node.get_logger().warn(f"IK attempt {attempt+1} timed out")
                continue
                
            response = future.result()
            if response.error_code.val == 1:
                joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                               "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
                joint_positions = []
                for name in joint_names:
                    try:
                        idx = response.solution.joint_state.name.index(name)
                        joint_positions.append(response.solution.joint_state.position[idx])
                    except ValueError:
                        self.node.get_logger().error(f"Joint {name} not in IK solution")
                        return None
                
                if attempt > 0:
                    self.node.get_logger().info(f"IK succeeded on attempt {attempt+1}")
                return joint_positions
            else:
                self.node.get_logger().warn(f"IK attempt {attempt+1} failed: error {response.error_code.val}")
        
        # Try position-only IK as fallback (Sampled Orientations)
        if position_only_fallback:
            self.node.get_logger().info("Trying position-only IK with sampled orientations...")
            
            # Euler angles (Roll, Pitch, Yaw)
            orientations = [
                (0, 0, 0),          # Identity (Up)
                (math.pi, 0, 0),    # Down (Roll 180)
                (0, math.pi/2, 0),  # Side (Pitch 90)
                (0, -math.pi/2, 0), # Side (Pitch -90)
                (0, 0, math.pi/2),  # Yaw 90
                (0, 0, -math.pi/2), # Yaw -90
                (math.pi/2, 0, 0),  # Roll 90
                (-math.pi/2, 0, 0)  # Roll -90
            ]
            
            for i, (r, p, y) in enumerate(orientations):
                self.node.get_logger().info(f"Position-Only Fallback {i+1}/8: trying R={r:.2f}, P={p:.2f}, Y={y:.2f}")
                
                start_pose = copy.deepcopy(target_pose_stamped)
                q = tf_transformations.quaternion_from_euler(r, p, y)
                start_pose.pose.orientation.x = q[0]
                start_pose.pose.orientation.y = q[1]
                start_pose.pose.orientation.z = q[2]
                start_pose.pose.orientation.w = q[3]
                
                req = GetPositionIK.Request()
                req.ik_request.group_name = self.group_name
                # Disable seeding here too
                # if current_joints: ...
                
                req.ik_request.pose_stamped = start_pose
                req.ik_request.timeout = Duration(sec=2, nanosec=0) # Shorter timeout
                req.ik_request.avoid_collisions = False
                
                future = self.ik_service.call_async(req)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=2.0)
                
                if not future.done(): continue
                
                response = future.result()
                if response.error_code.val == 1:
                     self.node.get_logger().info(f"Position-Only IK succeeded with orientation {i}")
                     # Extract joints...
                     joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
                     joint_positions = []
                     for name in joint_names:
                         try:
                             idx = response.solution.joint_state.name.index(name)
                             joint_positions.append(response.solution.joint_state.position[idx])
                         except ValueError:
                             continue
                     return joint_positions
            req.ik_request.avoid_collisions = False
            
            future = self.ik_service.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                if response.error_code.val == 1:
                    self.node.get_logger().info("Position-only IK succeeded!")
                    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                   "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
                    joint_positions = []
                    for name in joint_names:
                        try:
                            idx = response.solution.joint_state.name.index(name)
                            joint_positions.append(response.solution.joint_state.position[idx])
                        except ValueError:
                            return None
                    return joint_positions
        
        # All attempts failed
        self.node.get_logger().error(f"IK failed after all attempts (including position-only)")
        return None


    def move_to_joints(self, joint_values):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.allowed_planning_time = 10.0 
        goal_msg.request.max_velocity_scaling_factor = 0.15
        goal_msg.request.max_acceleration_scaling_factor = 0.10
        goal_msg.request.planner_id = "TRRT"
        jc_list = []
        joint_names = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        for i, val in enumerate(joint_values):
            jc = JointConstraint()
            jc.joint_name = joint_names[i]
            jc.position = val
            jc.weight = 1.0
            jc_list.append(jc)
        goal_msg.request.goal_constraints = [Constraints(joint_constraints=jc_list)]

        if not self.move_action_client.wait_for_server(timeout_sec=5.0): return False
        
        self.node.get_logger().info("Sending Joints Goal...")
        future = self.move_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Move Joints Action Rejected")
            return False
        
        self.node.get_logger().info("Goal Accepted. Waiting for specific result...")
        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res_future)
        result = res_future.result().result
        self.node.get_logger().info(f"Result Received. Code: {result.error_code.val}")
        
        if result.error_code.val != 1:
            self.node.get_logger().error(f"Move Joints Failed: {result.error_code.val}")
            return False
            
        return True

    def move_to_pose(self, target_pose_stamped, orientation_constraint=False):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.group_name
        goal_msg.request.allowed_planning_time = 10.0 
        goal_msg.request.max_velocity_scaling_factor = 0.15
        goal_msg.request.planner_id = "RRTConnect"
        
        goal_msg.request.workspace_parameters.header.frame_id = "base_link"
        goal_msg.request.workspace_parameters.min_corner.x = -1.5
        goal_msg.request.workspace_parameters.min_corner.y = -1.5
        goal_msg.request.workspace_parameters.min_corner.z = -0.5
        goal_msg.request.workspace_parameters.max_corner.x = 1.5
        goal_msg.request.workspace_parameters.max_corner.y = 1.5
        goal_msg.request.workspace_parameters.max_corner.z = 2.0

        # Position Constraint
        pcm = PositionConstraint()
        pcm.header = target_pose_stamped.header
        pcm.link_name = "tool0"
        pcm.weight = 1.0
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.2, 0.2, 0.2])
        pcm.constraint_region.primitives = [box]
        pcm.constraint_region.primitive_poses = [target_pose_stamped.pose]
        
        constraints = [Constraints(position_constraints=[pcm])]

        # Orientation Constraint
        if orientation_constraint:
            ocm = OrientationConstraint()
            ocm.header = target_pose_stamped.header
            ocm.link_name = "tool0"
            ocm.orientation = target_pose_stamped.pose.orientation
            ocm.absolute_x_axis_tolerance = 0.1 #
            ocm.absolute_y_axis_tolerance = 0.1
            ocm.absolute_z_axis_tolerance = 0.1
            ocm.weight = 1.0
            constraints[0].orientation_constraints = [ocm]
        
        goal_msg.request.goal_constraints = constraints

        if not self.move_action_client.wait_for_server(timeout_sec=5.0): 
            self.node.get_logger().error("Move Action Server not available")
            return False
            
        self.node.get_logger().info(f"Sending Pose Goal to {self.group_name}...")
        future = self.move_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Move Pose Action Rejected")
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, res_future)
        result = res_future.result().result
        
        if result.error_code.val != 1:
            self.node.get_logger().error(f"Move Pose Failed: ErrorCode={result.error_code.val}")
            return False
            
        return True

    def retime_trajectory(self, plan):
        if not plan or not plan.joint_trajectory.points: return
        avg_velocity = 0.20 
        points = plan.joint_trajectory.points
        points[0].time_from_start = Duration(sec=0, nanosec=0)
        current_time = 0.0
        
        for i in range(1, len(points)):
            max_disp = max([abs(p - prev) for p, prev in zip(points[i].positions, points[i-1].positions)])
            dt = max(max_disp / avg_velocity, 0.05)
            current_time += dt
            points[i].time_from_start = Duration(sec=int(current_time), nanosec=int((current_time % 1)*1e9))

    def move_linear_cartesian(self, target_pose_stamped):
        time.sleep(1.0) # Settle down
        
        if not self.cartesian_service.wait_for_service(timeout_sec=1.0): return False

        # Log Start State
        current_joints = self.get_current_joint_values()
        
        # Construct RobotState for explicit Start State
        start_state = RobotState()
        start_state.joint_state.name = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", 
                                        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
        start_state.joint_state.position = current_joints
        
        waypoints = [target_pose_stamped.pose]
        req = GetCartesianPath.Request()
        req.header.frame_id = target_pose_stamped.header.frame_id
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.start_state = start_state
        req.group_name = self.group_name
        req.waypoints = waypoints
        req.max_step = 0.01 
        req.jump_threshold = 0.0 
        req.avoid_collisions = False
        req.link_name = "tool0" 

        future = self.cartesian_service.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        response = future.result()
        
        if response.error_code.val != 1:
            self.node.get_logger().error(f"Cartesian Planning Failed: {response.error_code.val}")
            return False

        if response.fraction < 0.90: 
             self.node.get_logger().error(f"Path incomplete ({response.fraction*100:.1f}%). Code: {response.error_code.val}")
             return False
        
        self.retime_trajectory(response.solution)
        
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        
        if not self.execute_action_client.wait_for_server(timeout_sec=1.0): return False
        future = self.execute_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        
        res_exec = future.result()
        if not res_exec.accepted: return False
        
        res_future = res_exec.get_result_async()
        rclpy.spin_until_future_complete(self.node, res_future)
        return (res_future.result().result.error_code.val == 1)

class HarvestStateMachine(Node):
    def __init__(self):
        super().__init__('harvest_state_machine')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group = MoveItWrapper(self, "ur_manipulator")
        self.marker_pub = self.create_publisher(Marker, '/debug_goal_marker', 10)
        self.debug_berry_pub = self.create_publisher(Marker, '/debug_berry_marker', 10)
        self.gui_pub = self.create_publisher(String, '/harvest_gui_status', 10)
        
        # Demo Mode Control
        self.demo_mode = False
        self.proceed_signal = False
        self.home_requested = False
        self.scan_joints = [-1.47, -0.60, -2.09, -2.01, 1.58, 0.01]
        self.control_sub = self.create_subscription(String, '/harvest_control', self.control_callback, 10)
        
        # To receive classifications from GUI
        self.classification_sub = self.create_subscription(String, '/gui_classification', self.classification_callback, 10)
        self.waiting_for_classification = False
        self.latest_classification = None
        
        # Arduino Serial Connection
        self.arduino = None
        if serial:
            try:
                for port in ['/dev/ttyACM0', '/dev/ttyUSB0', '/dev/ttyACM1', '/dev/ttyUSB1']:
                    try:
                        self.arduino = serial.Serial(port, 9600, timeout=1)
                        self.get_logger().info(f"Connected to Arduino on {port}")
                        time.sleep(2)
                        break
                    except serial.SerialException: continue
            
                if not self.arduino:
                    self.get_logger().warn("Could not connect to Arduino (Servo). Check USB.")
            except Exception as e:
                self.get_logger().error(f"Serial setup error: {e}")

    def async_sleep(self, delay_seconds):
        """Non-blocking delay that keeps ROS 2 callbacks alive."""
        start_time = self.get_clock().now()
        duration = rclpy.duration.Duration(seconds=float(delay_seconds))
        while (self.get_clock().now() - start_time) < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

    def publish_gui_state(self, state, event=None, count=None, target_xyz=None, val_data=None):
        msg = String()
        data = {"state": state}
        if event: data["event"] = event
        if count is not None: data["berries_detected"] = count
        if target_xyz is not None: data["target_xyz"] = target_xyz
        if val_data is not None: data["val_data"] = val_data
        msg.data = json.dumps(data)
        self.gui_pub.publish(msg)

    def control_callback(self, msg):
        try:
            data = json.loads(msg.data)
            if "demo_mode" in data:
                self.demo_mode = data["demo_mode"]
                self.get_logger().info(f"Demo Mode set to: {self.demo_mode}")
            if "proceed" in data and data["proceed"]:
                self.proceed_signal = True
                self.get_logger().info("Proceed signal received!")
            if "actuate_cutter" in data:
                action = data["actuate_cutter"]
                if action == "open":
                    self.get_logger().info("Manual override: OPEN CUTTER")
                    self.open_cutter()
                elif action == "close":
                    self.get_logger().info("Manual override: CLOSE CUTTER")
                    self.trigger_servo()
            if "command" in data:
                if data["command"] == "home":
                    self.get_logger().info("Manual override flag: GOING HOME (Scan Position)")
                    self.home_requested = True
        except Exception as e:
            self.get_logger().error(f"Failed to parse control msg: {e}")

    def classification_callback(self, msg):
        self.latest_classification = msg.data
        self.waiting_for_classification = False
        self.get_logger().info(f"Received GUI Classification: {msg.data}")

    def trigger_servo(self):
        """Triggers Servo via USB Serial to CLOSE (Cut)"""
        if not self.arduino:
            self.get_logger().warn("Arduino not connected. Simulating cut delay.")
            self.async_sleep(1.0)
            return

        try:
            self.get_logger().info("Actuating Servo...")
            self.arduino.write(b'0')
            self.async_sleep(2.0) 
        except Exception as e:
            self.get_logger().error(f"Servo trigger failed: {e}")

    def open_cutter(self):
        """Triggers Servo via USB Serial to OPEN"""
        if not self.arduino:
            self.get_logger().warn("Arduino not connected. Simulating open delay.")
            self.async_sleep(1.0)
            return

        try:
            self.get_logger().info("Opening Servo...")
            self.arduino.write(b'1')
            self.async_sleep(1.0) 
        except Exception as e:
            self.get_logger().error(f"Servo open failed: {e}")

    def get_tf_target(self, base_detection_frame='detected_berry', timeout=2.0, reference_xyz=None):
        self.get_logger().info(f"Collecting samples for {timeout}s...")
        start_time = time.time()
        samples = []
        
        while (time.time() - start_time) < timeout:
            for i in range(15):
                frame_name = f"{base_detection_frame}_{i}"
                try:
                    if self.tf_buffer.can_transform('base_link', frame_name, rclpy.time.Time()):
                        trans = self.tf_buffer.lookup_transform('base_link', frame_name, rclpy.time.Time())
                        
                        tf_time = trans.header.stamp.sec + (trans.header.stamp.nanosec * 1e-9)
                        current_time = self.get_clock().now().nanoseconds * 1e-9
                        if (current_time - tf_time) > 1.0: 
                            continue

                        p = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
                    
                        if FORCE_MIRROR_Y: 
                            p[1] = -p[1]
                            
                        if np.linalg.norm(p) <= MAX_REACH_RADIUS:
                            samples.append(p)
                            # self.get_logger().info(f"Sample ({frame_name}): {p}")
                            
                except Exception: 
                    pass
            
            rclpy.spin_once(self, timeout_sec=0.1) # 10Hz sampling
            
        if not samples:
            self.get_logger().warn("No valid samples found.")
            return None
            
        # CLUSTERING LOGIC: Group samples by distance (Euclidean < 2.5cm)
        clusters = [] 
        threshold = 0.025 # 2.5cm radius
        
        for p in samples:
            matched = False
            for cluster in clusters:
                cluster_np = np.array(cluster)
                centroid = np.mean(cluster_np, axis=0)
                dist = np.linalg.norm(np.array(p) - centroid)
                if dist < threshold:
                    cluster.append(p)
                    matched = True
                    break
            if not matched:
                clusters.append([p])
                
        cluster_data = []
        for i, cluster in enumerate(clusters):
            c_np = np.array(cluster)
            centroid = np.mean(c_np, axis=0)
            cluster_data.append( {'id': i, 'centroid': centroid, 'size': len(cluster)} )
            
        if reference_xyz:
            # Sort by distance to the initially planned target (reference_xyz)
            cluster_data.sort(key=lambda x: np.linalg.norm(np.array(x['centroid']) - np.array(reference_xyz)))
        else:
            # Sort by Y Ascending (rightmost first)
            cluster_data.sort(key=lambda x: x['centroid'][1])
        
        target_cluster = cluster_data[0]
        mean_pos = target_cluster['centroid']
        
        self.get_logger().info(f"--- FILTER RESULT ---")
        self.get_logger().info(f"Total Raw Samples: {len(samples)}")
        self.get_logger().info(f"Distinct Berries Found: {len(clusters)}")
        for c in cluster_data:
            self.get_logger().info(f" - Berry {c['id']}: Y={c['centroid'][1]:.2f}, Data Points={c['size']} {'<-- [TARGET]' if c['id'] == target_cluster['id'] else ''}")
        self.get_logger().info(f"Target Coordinate: {mean_pos}")
        self.get_logger().info(f"---------------------")
        
        self.publish_gui_state("Scanning", count=len(clusters))
        
        return mean_pos.tolist()

    def get_all_tf_targets(self, base_detection_frame='detected_berry', timeout=2.0):
        """Returns ALL detected berry clusters sorted right-to-left."""
        self.get_logger().info(f"Collecting all unique samples for {timeout}s...")
        start_time = time.time()
        samples = []
        
        while (time.time() - start_time) < timeout:
            for i in range(15):
                frame_name = f"{base_detection_frame}_{i}"
                try:
                    if self.tf_buffer.can_transform('base_link', frame_name, rclpy.time.Time()):
                        trans = self.tf_buffer.lookup_transform('base_link', frame_name, rclpy.time.Time())
                        tf_time = trans.header.stamp.sec + (trans.header.stamp.nanosec * 1e-9)
                        current_time = self.get_clock().now().nanoseconds * 1e-9
                        if (current_time - tf_time) > 1.0: continue
                        p = [trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z]
                        if FORCE_MIRROR_Y: p[1] = -p[1]
                        if np.linalg.norm(p) <= MAX_REACH_RADIUS: samples.append(p)
                except Exception: pass
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if not samples: return []
            
        clusters = [] 
        threshold = 0.025
        for p in samples:
            matched = False
            for cluster in clusters:
                centroid = np.mean(np.array(cluster), axis=0)
                if np.linalg.norm(np.array(p) - centroid) < threshold:
                    cluster.append(p)
                    matched = True
                    break
            if not matched: clusters.append([p])
                
        cluster_data = []
        for i, cluster in enumerate(clusters):
            centroid = np.mean(np.array(cluster), axis=0)
            cluster_data.append({'id': i, 'centroid': centroid})
            
        cluster_data.sort(key=lambda x: x['centroid'][1]) # Sort Right-to-Left
        
        self.publish_gui_state("Finished Scan", count=len(clusters))
        return [c['centroid'].tolist() for c in cluster_data]


    def prepare_arm_for_slide(self, target_xyz):
        tx, ty, tz = target_xyz
        target_pan = math.atan2(ty, tx)
        
        current_joints = self.move_group.get_current_joint_values()
        if not current_joints: return False
        current_pan = current_joints[0]
        
        diff = target_pan - current_pan
        while diff > math.pi: diff -= 2*math.pi
        while diff < -math.pi: diff += 2*math.pi
        
        if abs(diff) > MAX_TURN_RADIANS:
            self.get_logger().error(f"ABORTING: Turn too large ({math.degrees(diff):.1f} deg).")
            return False

        final_pan = current_pan + diff
        aim_joints = [final_pan, -1.54, -1.54, -1.57, -1.57, 0.0] # All -90 except pan and wrist3
        
        self.get_logger().info(f"Moving to Approach Pose (Camera may lose sight)... Pan: {math.degrees(final_pan):.1f}")
        return self.move_group.move_to_joints(aim_joints)

    def get_L_shape_hover_pose(self, berry_xyz):
        """Calculate L-shape hover pose (from harvester_planner.py)"""
        try:
            berry_x, berry_y, berry_z = berry_xyz
            
            # Configuration - reduced for reachability
            CUTTER_Y_LENGTH = 0.10  # Tool length (minimal offset)
            VERTICAL_DROP = 0.08     # Distance to lift up (minimal)
            
            # Angle A: The direction from Robot to Berry
            approach_yaw = math.atan2(berry_y, berry_x)
            
            # Angle B: The Orientation of the Tool (Rotated for L-Shape)
            tool_yaw = approach_yaw - 1.57  # 90° rotation
            
            # Calculate Position (Use Angle A - Approach)
            # Back up along the line to the berry
            approach_vector = np.array([math.cos(approach_yaw), math.sin(approach_yaw), 0.0])
            z_axis_down = np.array([0.0, 0.0, -1.0])
            
            wrist_pos = np.array([berry_x, berry_y, berry_z])
            wrist_pos -= approach_vector * CUTTER_Y_LENGTH
            wrist_pos -= z_axis_down * VERTICAL_DROP
            
            # Calculate Orientation (Use Angle B - Tool)
            # Construct the rotation matrix using the ROTATED angle
            y_axis_tool = np.array([math.cos(tool_yaw), math.sin(tool_yaw), 0.0])
            x_axis_tool = np.cross(y_axis_tool, z_axis_down)
            
            R = np.eye(4)
            R[0:3, 0] = x_axis_tool
            R[0:3, 1] = y_axis_tool
            R[0:3, 2] = z_axis_down
            
            # Convert & Normalize
            q = np.array(tf_transformations.quaternion_from_matrix(R))
            length = np.linalg.norm(q)
            if length > 1e-6:
                q = q / length
            
            self.get_logger().info(f"--- L-Shape Hover Pose ---")
            self.get_logger().info(f"Berry: [{berry_x:.2f}, {berry_y:.2f}, {berry_z:.2f}]")
            self.get_logger().info(f"Hover Pos: {wrist_pos}")
            self.get_logger().info(f"Approach Yaw: {math.degrees(approach_yaw):.1f}°")
            self.get_logger().info(f"Tool Yaw: {math.degrees(tool_yaw):.1f}°")
            self.get_logger().info(f"-------------------------")
            
            # Construct Pose
            target = PoseStamped()
            target.header.frame_id = 'base_link'
            target.pose.position.x = wrist_pos[0]
            target.pose.position.y = wrist_pos[1]
            target.pose.position.z = wrist_pos[2]
            target.pose.orientation.x = q[0]
            target.pose.orientation.y = q[1]
            target.pose.orientation.z = q[2]
            target.pose.orientation.w = q[3]
            
            return target
        except Exception as e:
            self.get_logger().error(f"L-shape calculation failed: {e}")
            return None

    def calculate_final_grasp_pose(self, berry_xyz):
         """Calculate grasp pose - same orientation as hover, position at berry"""
         hover = self.get_L_shape_hover_pose(berry_xyz)
         if hover:
             # CRITICAL: Keep same orientation, only change position to berry
             hover.pose.position.x = berry_xyz[0]
             hover.pose.position.y = berry_xyz[1]
             hover.pose.position.z = berry_xyz[2]
             return hover
         return None
    
    def move_to_berry_joints(self, berry_xyz, hover_joints):
        """Move to berry using joint-space interpolation from hover pose"""
        try:
            # Calculate grasp pose at berry
            grasp_pose = self.calculate_final_grasp_pose(berry_xyz)
            if not grasp_pose:
                self.get_logger().error("Failed to calculate grasp pose")
                return False
            
            # Try IK for berry position with same orientation as hover
            self.get_logger().info("Computing IK for berry grasp...")
            berry_joints = self.move_group.compute_ik(grasp_pose, max_attempts=3)
            
            if berry_joints:
                self.get_logger().info(f"Berry joints: {[round(math.degrees(x),1) for x in berry_joints]}")
                # Move to berry in joint space
                if self.move_group.move_to_joints(berry_joints):
                    self.get_logger().info("REACHED BERRY via joint movement!")
                    self.get_logger().info("Triggering Cut...")
                    self.trigger_servo()
                    return True
                else:
                    self.get_logger().error("Failed to execute berry joint movement")
                    return False
            else:
                # Fallback: Manual approximation for berry
                self.get_logger().warn("IK failed for berry, using manual approximation...")
                berry_manual = self.approximate_berry_joints(berry_xyz, hover_joints)
                if berry_manual and self.move_group.move_to_joints(berry_manual):
                    self.get_logger().info("REACHED BERRY via manual joints!")
                    self.get_logger().info("Triggering Cut...")
                    self.trigger_servo()
                    return True
                else:
                    self.get_logger().error("Manual berry joints failed")
                    return False
                    
        except Exception as e:
            self.get_logger().error(f"move_to_berry_joints failed: {e}")
            return False
    
    def approximate_berry_joints(self, berry_xyz, hover_joints):
        """Try multiple orientations with IK to find solution for berry position"""
        try:
            berry_x, berry_y, berry_z = berry_xyz
            
            # Calculate approach direction
            approach_yaw = math.atan2(berry_y, berry_x)
            approach_vector = np.array([math.cos(approach_yaw), math.sin(approach_yaw), 0.0])
            
            berry_pos = np.array([berry_x, berry_y, berry_z])
            
            self.get_logger().info(f"Berry position: [{berry_pos[0]:.2f}, {berry_pos[1]:.2f}, {berry_pos[2]:.2f}]")
            
            # Build orientation frame
            z_axis = approach_vector / np.linalg.norm(approach_vector)
            world_down = np.array([0.0, 0.0, -1.0])
            x_axis = np.cross(z_axis, world_down)
            if np.linalg.norm(x_axis) < 0.01:
                x_axis = np.array([1.0, 0.0, 0.0])
            x_axis = x_axis / np.linalg.norm(x_axis)
            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)
            
            # Try 12 different roll angles
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
                target_pose.pose.position.x = berry_pos[0]
                target_pose.pose.position.y = berry_pos[1]
                target_pose.pose.position.z = berry_pos[2]
                target_pose.pose.orientation.x = q[0]
                target_pose.pose.orientation.y = q[1]
                target_pose.pose.orientation.z = q[2]
                target_pose.pose.orientation.w = q[3]
                
                # Try IK
                joint_solution = self.move_group.compute_ik(target_pose, max_attempts=1, position_only_fallback=False)
                
                if joint_solution:
                    self.get_logger().info(f"IK found berry solution at roll={math.degrees(roll):.0f}°")
                    self.get_logger().info(f"Berry joints: {[round(math.degrees(x),1) for x in joint_solution]}")
                    return joint_solution
            
            self.get_logger().error("No IK solution found for berry at any roll angle")
            return None
            
        except Exception as e:
            self.get_logger().error(f"approximate_berry_joints failed: {e}")
            import traceback
            traceback.print_exc()
            return None

    def solve_approach_ik(self, approach_pose_stamped):
        """Find IK solution for the explicit Approach Pose (already has orientation)"""
        joint_solution = self.move_group.compute_ik(approach_pose_stamped, max_attempts=5, position_only_fallback=False)
        if joint_solution: return joint_solution
        return None

    def approximate_hover_joints(self, berry_xyz):
        """Try multiple orientations with IK to find a valid solution"""
        try:
            berry_x, berry_y, berry_z = berry_xyz
            
            # Calculate hover position with L-shape offset
            CUTTER_Y_LENGTH = 0.10
            VERTICAL_DROP = 0.08
            
            approach_yaw = math.atan2(berry_y, berry_x)
            approach_vector = np.array([math.cos(approach_yaw), math.sin(approach_yaw), 0.0])
            
            hover_pos = np.array([berry_x, berry_y, berry_z])
            hover_pos -= approach_vector * CUTTER_Y_LENGTH
            hover_pos -= np.array([0.0, 0.0, -VERTICAL_DROP])
            
            self.get_logger().info(f"Hover position: [{hover_pos[0]:.2f}, {hover_pos[1]:.2f}, {hover_pos[2]:.2f}]")
            
            # Try multiple roll angles (rotation around approach axis)
            # This is the strategy from harvester_planner.py
            z_axis = approach_vector / np.linalg.norm(approach_vector)
            world_down = np.array([0.0, 0.0, -1.0])
            x_axis = np.cross(z_axis, world_down)
            if np.linalg.norm(x_axis) < 0.01:
                x_axis = np.array([1.0, 0.0, 0.0])
            x_axis = x_axis / np.linalg.norm(x_axis)
            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)
            
            # Try 12 different roll angles
            for i in range(12):
                roll = i * (math.pi / 6)  # 30 degrees increments
                
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
                target_pose.pose.position.x = hover_pos[0]
                target_pose.pose.position.y = hover_pos[1]
                target_pose.pose.position.z = hover_pos[2]
                target_pose.pose.orientation.x = q[0]
                target_pose.pose.orientation.y = q[1]
                target_pose.pose.orientation.z = q[2]
                target_pose.pose.orientation.w = q[3]
                
                # Try IK with this orientation
                joint_solution = self.move_group.compute_ik(target_pose, max_attempts=1, position_only_fallback=False)
                
                if joint_solution:
                    self.get_logger().info(f"IK found solution at roll={math.degrees(roll):.0f}°")
                    self.get_logger().info(f"Hover joints: {[round(math.degrees(x),1) for x in joint_solution]}")
                    return joint_solution
            
            # If all roll angles fail, return None
            self.get_logger().error("No IK solution found for any roll angle")
            return None
            
        except Exception as e:
            self.get_logger().error(f"approximate_hover_joints failed: {e}")
            import traceback
            traceback.print_exc()
            return None

    def publish_debug_marker(self, pose_stamped):
        marker = Marker()
        marker.header = pose_stamped.header
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose_stamped.pose
        marker.scale.x = 0.05; marker.scale.y = 0.05; marker.scale.z = 0.05
        marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
        
        self.marker_pub.publish(marker)

    def publish_debug_marker(self, x, y, z, marker_id=0, color=(0.0, 1.0, 0.0)):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "berry_debug"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        self.debug_berry_pub.publish(marker)

    
    
    
    def align_camera_pan(self, target_xyz):
        """
        Visual Servoing Alignment:
        Rotates Base Joint (Pan) to center the berry in the CAMERA IMAGE.
        Uses the live 'camera_color_optical_frame' relative position to avoid URDF math errors.
        """
        try:
            # 1. First, find which detected_berry_X this target_xyz corresponds to.
            # We look at the base_link distances to find the ID.
            best_id = -1
            min_dist = 999.0
            for i in range(15):
                frame_name = f"detected_berry_{i}"
                if self.tf_buffer.can_transform('base_link', frame_name, rclpy.time.Time()):
                    trans = self.tf_buffer.lookup_transform('base_link', frame_name, rclpy.time.Time())
                    # Check age
                    tf_time = trans.header.stamp.sec + (trans.header.stamp.nanosec * 1e-9)
                    current_time = self.get_clock().now().nanoseconds * 1e-9
                    if (current_time - tf_time) > 2.0: continue  
                        
                    px = trans.transform.translation.x
                    py = trans.transform.translation.y
                    pz = trans.transform.translation.z
                    
                    if FORCE_MIRROR_Y: py = -py
                    
                    dist = math.sqrt((px - target_xyz[0])**2 + (py - target_xyz[1])**2 + (pz - target_xyz[2])**2)
                    if dist < min_dist:
                        min_dist = dist
                        best_id = i
            
            if best_id == -1:
                self.get_logger().warn("Could not find corresponding live berry for alignment.")
                return False
                
            target_frame = f"detected_berry_{best_id}"
            
            # 2. Look up the LIVE distance relative to the camera optical frame!
            camera_frame = 'camera_color_optical_frame' 
            
            if not self.tf_buffer.can_transform(camera_frame, target_frame, rclpy.time.Time()):
                self.get_logger().warn(f"Cannot transform {camera_frame} -> {target_frame}")
                return False

            trans = self.tf_buffer.lookup_transform(camera_frame, target_frame, rclpy.time.Time())
            
            # In optical frame: X is right, Z is forward (depth)
            tx = trans.transform.translation.x
            tz = trans.transform.translation.z
            
            # 3. Calculate Angle Error
            # atan2(x, z) gives angle of berry relative to center
            # Because X is "right", if X is positive, angle returns positive.
            # But joint rotates counter-clockwise (positive), so to move right we need negative pan.
            yaw_error = math.atan2(tx, tz)
            
            self.get_logger().info(f"Visual Servo: x={tx:.3f}, z={tz:.3f}, Yaw Error={math.degrees(yaw_error):.1f} deg")
            
            if abs(yaw_error) < 0.05: # < 3 degrees
                self.get_logger().info("Camera already aligned.")
                return True
                
            # 3. Calculate Pan Correction
            # Calibration: If Camera X is Right, and Pan + is Left (CCW).
            # Berry Right (+X) -> Need Pan Right (-).
            # So Correction = -Yaw Error.
            pan_correction = -yaw_error
            
            # SAFETY CLAMP (Max 15 degrees per move for visual servo)
            MAX_VS_STEP = 0.26 # ~15 deg
            if pan_correction > MAX_VS_STEP: pan_correction = MAX_VS_STEP
            elif pan_correction < -MAX_VS_STEP: pan_correction = -MAX_VS_STEP
            
            # 4. Apply
            current_joints = self.move_group.get_current_joint_values()
            if not current_joints: return False
            
            target_pan = current_joints[0] + pan_correction
            self.get_logger().info(f"Correcting Pan: {math.degrees(current_joints[0]):.1f} -> {math.degrees(target_pan):.1f} (Delta: {math.degrees(pan_correction):.1f})")
            
            align_joints = list(current_joints)
            align_joints[0] = target_pan
            
            return self.move_group.move_to_joints(align_joints)
            
        except Exception as e:
            self.get_logger().error(f"align_camera_pan failed: {e}")
            return False
            
    def idle_loop(self):
        self.get_logger().info("Entering Idle Loop (Waiting for commands...)")
        self.publish_gui_state("Idle / Harvest Complete")
        while rclpy.ok():
            if self.home_requested:
                self.get_logger().info("Executing Home Request...")
                self.move_group.move_to_joints(self.scan_joints)
                self.home_requested = False
                self.publish_gui_state("Idle / Harvest Complete")
            self.async_sleep(0.5)

    def run_harvest_loop(self):
        
        while rclpy.ok():
            self.get_logger().info("--- NEW CYCLE ---")
            self.publish_gui_state("Retracting to Scan Pose")
            
            # Retract to scan pose
            self.move_group.move_to_joints(self.scan_joints)
            self.open_cutter()
            self.async_sleep(2.0)
            
            self.get_logger().info("Scanning for berries...")
            self.publish_gui_state("Scanning for berries")
            
            # Initial scan for berries in the field
            berry_clusters = self.get_all_tf_targets('detected_berry', timeout=2.5)
            
            if not berry_clusters:
                self.get_logger().info("No berries detected. Waiting...")
                
                # Check for home request while waiting
                if self.home_requested:
                    self.move_group.move_to_joints(self.scan_joints)
                    self.home_requested = False
                    
                self.async_sleep(2.0)
                continue
            
            # ----------------------------------------------------------------
            # INITIAL REGISTRY CREATION
            # ----------------------------------------------------------------
            berry_registry = []
            for i, centroid in enumerate(berry_clusters):
                berry_registry.append({
                    "id": i + 1,
                    "xyz": centroid,
                    "harvested": False,
                    "abandoned": False
                })

            total_harvested_goal = len(berry_registry)
            total_harvested_count = 0
            total_abandoned_count = 0
            
            # Publish the initial scan count
            self.publish_gui_state("Scan Complete", count=total_harvested_goal)
            self.get_logger().info(f"Starting cycle with {total_harvested_goal} detected targets.")
            
            # RESCAN LOOP: After processing all known targets until a full rescan returns empty or goal met.
            pass_num = 0
            while rclpy.ok():
                pass_num += 1
                pass_caught = 0
                pass_cuts_attempted = 0 
                
                # --- Baseline Refresh ---
                # Before starting the pass, get a fresh "vision baseline" from scan pose.
                # This ensures we compare verification results against the ACTUAL state at pass-start,
                # preventing noise from the initial mission scan from causing false successes.
                self.get_logger().info(f"Pass {pass_num}: Synchronizing vision baseline...")
                self.move_group.move_to_joints(self.scan_joints)
                self.async_sleep(1.2)
                baseline_berries = self.get_all_tf_targets('detected_berry', timeout=2.5)
                pass_expected_total = len(baseline_berries)
                self.get_logger().info(f"Pass {pass_num} Baseline: {pass_expected_total} berries visible.")

                # Filter registry for berries remaining (neither harvested nor abandoned)
                current_targets = [b for b in berry_registry if not b["harvested"] and not b["abandoned"]]
                # Ensure we don't exceed the number of targets actually in the registry
                pass_expected_total = max(pass_expected_total, len(current_targets))
                
                if pass_expected_total == 0:
                    self.get_logger().info("All berries in registry are processed. Entering idle.")
                    self.idle_loop()
                    return
                
                if self.home_requested:
                    self.get_logger().info("Home Requested during cycle. Aborting mission loop.")
                    self.move_group.move_to_joints(self.scan_joints)
                    self.home_requested = False
                    self.idle_loop()
                    return
                
                for berry in current_targets:
                    idx = berry["id"]
                    target_xyz = berry["xyz"]
                    success = False
                    attempts = 0
                    max_attempts = 3  # Max 3 tries per target
                    pan_align_failed = False   
                    cut_attempted   = False    
                    
                    import time
                    start_time = time.time()
                    
                    while attempts < max_attempts and not success and rclpy.ok():
                        if attempts > 0:
                            self.get_logger().info(f"Retrying Berry {idx}/{total_harvested_goal} Pass {pass_num} (Attempt {attempts+1})")
                            self.publish_gui_state(f"Retrying Berry {idx}", event="Harvest Retry")
                        else:
                            self.get_logger().info(f"Targeting Berry {idx}/{total_harvested_goal} (Pass {pass_num})")
                            self.publish_gui_state(f"Targeting Berry {idx}", target_xyz=target_xyz)
                            
                        # 1. Iterative Pan Alignment (up to 3 steps to converge for edge berries)
                        pan_ok = False
                        for pan_iter in range(3):
                            if self.align_camera_pan(target_xyz):
                                pan_ok = True
                                self.async_sleep(0.7)  # Let camera settle between iterations
                                # Check if we are close enough to stop iterating
                                try:
                                    camera_frame = 'camera_color_optical_frame'
                                    for bi in range(15):
                                        fn = f'detected_berry_{bi}'
                                        if self.tf_buffer.can_transform('base_link', fn, rclpy.time.Time()):
                                            trans = self.tf_buffer.lookup_transform(camera_frame, fn, rclpy.time.Time())
                                            tx = trans.transform.translation.x
                                            tz = trans.transform.translation.z
                                            yaw_err = math.atan2(tx, tz)
                                            if abs(yaw_err) < 0.05:  # < 3 degrees - converged
                                                self.get_logger().info(f"Pan converged in {pan_iter+1} step(s).")
                                                break
                                except Exception: pass
                                else:
                                    continue
                                break
                            else:
                                break  # align failed, bail out
                        
                        if pan_ok:
                            self.async_sleep(0.8)  # Final camera settle after convergence
                            
                            # 2. Refined Detection (Now Centered Horizontally) — longer scan for better stats
                            refined_target = self.get_tf_target('detected_berry', timeout=2.5, reference_xyz=target_xyz)
                            if refined_target:
                                self.get_logger().info(f"Refined Target: {refined_target}")
                                self.publish_debug_marker(refined_target[0], refined_target[1], refined_target[2])
                    
                                # 3. Harvest
                                self.get_logger().info("Executing Harvest...")
                                self.publish_gui_state("Executing Harvest Approach")
                                self.last_target_xyz = refined_target
                                cut_attempted = True  
                                trajectory_ok = self.execute_approach_and_slide(refined_target)
                                
                                if trajectory_ok:
                                     self.get_logger().info("Harvest execute complete. Retracting to verify catch...")
                                     self.publish_gui_state("Verifying Catch")
                                     
                                     # Retract straight to scan to get camera view of area
                                     self.move_group.move_to_joints(self.scan_joints)
                                     self.async_sleep(1.5) # Wait for camera to settle
                                     
                                     # ---------------------------------------------------------
                                     # Refined Visual Verification (Surgical Proximity + Count)
                                     # ---------------------------------------------------------
                                     self.get_logger().info("Verification: Scanning field for target status...")
                                     self.async_sleep(1.0) 
                                     current_berries = self.get_all_tf_targets('detected_berry', timeout=2.5) 
                                     
                                     berry_still_attached = False
                                     expected_remaining = pass_expected_total - pass_caught
                                     count_dropped = (len(current_berries) < expected_remaining) if current_berries else True

                                     # 1. Proximity check: Is the specific 3D target space occupied?
                                     # Tightened to 3.5cm (35mm) to ignore neighbors in close clusters
                                     if self.last_target_xyz:
                                         for cb in current_berries:
                                             dist = math.sqrt((cb[0] - self.last_target_xyz[0])**2 + 
                                                              (cb[1] - self.last_target_xyz[1])**2 + 
                                                              (cb[2] - self.last_target_xyz[2])**2)
                                             if dist < 0.035: # Tight 3.5cm radius
                                                 berry_still_attached = True
                                                 break
                                     
                                     # 2. Anti-Glitched Check: If vision sees 0, retry before success
                                     if not berry_still_attached and not current_berries:
                                         self.get_logger().warn("Verification: Field appears empty. Retrying scan for safety...")
                                         self.async_sleep(1.0)
                                         current_berries = self.get_all_tf_targets('detected_berry', timeout=1.5)
                                         # Re-evaluate count drop and proximity after retry
                                         count_dropped = (len(current_berries) < expected_remaining)
                                         if self.last_target_xyz:
                                             for cb in current_berries:
                                                 dist = math.sqrt((cb[0] - self.last_target_xyz[0])**2 + 
                                                                  (cb[1] - self.last_target_xyz[1])**2 + 
                                                                  (cb[2] - self.last_target_xyz[2])**2)
                                                 if dist < 0.035:
                                                     berry_still_attached = True
                                                     break

                                     attempts += 1
                                                 
                                     if berry_still_attached:
                                         if attempts < max_attempts:
                                             self.get_logger().warn(f"Cut FAILED (Attempt {attempts}/{max_attempts}). Berry STILL at target XYZ. Retrying...")
                                         else:
                                             self.get_logger().error("Max retries reached. Target still attached. Moving to next.")
                                     else:
                                         # SUCCESS criteria: Target is gone AND count dropped, OR space is clear after retry
                                         if count_dropped:
                                             self.get_logger().info(f"Verification: SUCCESS! Target clear & field count decreased on attempt {attempts}.")
                                         else:
                                             self.get_logger().info("Verification: Target clear (Count unchanged - likely neighbor shift). PASS.")
                                         
                                         success = True
                                         pass_caught += 1
                                         total_harvested_count += 1
                                         berry["harvested"] = True
                                         if total_harvested_count + total_abandoned_count >= total_harvested_goal:
                                             self.get_logger().info(f"GOAL REACHED ({total_harvested_count} harvested, {total_abandoned_count} abandoned / {total_harvested_goal} total). Stopping immediately.")
                                             return
                                else:
                                     self.get_logger().error("Approach failed.")
                                     self.publish_gui_state("Approach failed")
                                     attempts += 1
                            else:
                                self.get_logger().warn("Lost berry after alignment.")
                                self.publish_gui_state("Lost berry after alignment")
                                attempts += 1
                        else:
                             self.get_logger().warn("Could not align camera. Skipping metric for this target.")
                             self.publish_gui_state("Could not align camera")
                             pan_align_failed = True
                             break  # Don't retry - pan failed, move to next berry
                        
                        if not success:
                            self.get_logger().info("Attempt failed. Resetting to Scan Pose.")
                            self.move_group.move_to_joints(self.scan_joints)
                            self.open_cutter()
                            self.async_sleep(2.0)

                        if not success and (attempts >= max_attempts or pan_align_failed):
                            berry["abandoned"] = True
                            total_abandoned_count += 1
                            self.get_logger().error(f"Berry {idx} ABANDONED after {attempts} failed attempts (Pan Failed: {pan_align_failed}).")
                            
                        if self.demo_mode and cut_attempted:
                            self.get_logger().info("Demo Mode Active: Pausing after attempt.")
                            self.publish_gui_state("Demo Paused - Waiting for Proceed")
                            self.proceed_signal = False
                            while not self.proceed_signal and rclpy.ok():
                                self.async_sleep(0.5)
                            self.get_logger().info("Resuming...")
                            
                    # End of specific berry loop (Target Cycle finished)
                    # Skip metric publication if no physical cut trajectory was actually attempted
                    if not cut_attempted:
                        self.get_logger().info(f"Skipping metric for Berry {idx+1}: Cut was never attempted (Alignment or Approach Failure).")
                    else:
                        pass_cuts_attempted += 1
                        try:
                            from std_msgs.msg import String
                            msg = String()
                            import json
                            import time
                            
                            elapsed_time = time.time() - start_time
                            
                            # Automated Data Classification
                            if success:
                                if attempts == 1:
                                    final_result = "FIRST_TRY_SUCCESS"
                                else:
                                    final_result = "MULTI_RETRY_SUCCESS"
                            else:
                                final_result = "ABANDONED"
                            
                            payload = {
                                "target_result": final_result,
                                "attempts": attempts,
                                "time_elapsed": elapsed_time,
                                "event": f"Target {idx} Resolved: {final_result} in {attempts} attempt(s) ({elapsed_time:.1f}s)"
                            }

                            msg.data = json.dumps(payload)
                            self.gui_pub.publish(msg)
                        except Exception as e:
                            self.get_logger().error(f"Failed to publish verification JSON: {e}")
                        
                    self.async_sleep(1.5)
                
                # --- RESCAN AFTER FULL PASS ---
                # Cycle Limit Check: If we've completed 2 passes, stop harvesting.
                if pass_num >= 2:
                    self.get_logger().info(f"Harvest reached max cycles ({pass_num}). Stopping.")
                    self.publish_gui_state("Harvest Stopped", event="Cycle Completed")
                    self.active = False
                    self.idle_loop()
                    return

                # Goal Reached Check (Harvested + Abandoned = Initial Total)
                if total_harvested_count + total_abandoned_count >= total_harvested_goal:
                    self.get_logger().info(f"All targets processed ({total_harvested_count} harvested, {total_abandoned_count} abandoned). Finishing.")
                    self.publish_gui_state("Cycle Complete", event="Cycle Completed",
                                          count=total_harvested_goal)
                    self.idle_loop()
                    return

                # Guard: if the entire pass had zero cut attempts, all berries are
                # unreachable (camera alignment failed for all). Abort to prevent infinite loop.
                if pass_cuts_attempted == 0:
                    self.get_logger().error(f"Pass {pass_num}: ZERO cut attempts. All remaining targets unreachable. Aborting cycle.")
                    self.publish_gui_state("Aborting Cycle", event="Cycle Completed")
                    self.active = False
                    self.idle_loop()
                    return

                # All known targets processed. Return to scan pose and look for any missed berries.
                self.get_logger().info("All targets processed. Rescanning for any missed berries...")
                self.publish_gui_state("Rescanning", event="Rescanning for Missed Berries")
                self.move_group.move_to_joints(self.scan_joints)
                self.open_cutter()
                self.async_sleep(2.5)  # Let camera fully settle
                
                new_detections = self.get_all_tf_targets('detected_berry', timeout=2.5)
                
                if not new_detections:
                    self.get_logger().info("Rescan complete: Field is clear. Cycle finished.")
                    self.publish_gui_state("Cycle Complete", event="Cycle Completed",
                                          count=total_harvested_goal)
                    self.get_logger().info(f"--- CYCLE COMPLETED ---")
                    self.idle_loop()
                    return  # Exit the mission entirely
                else:
                    # Update Registry using distance-based matching
                    matched_new = 0
                    for nd in new_detections:
                        found_match = False
                        for b in berry_registry:
                            dist = math.sqrt((b["xyz"][0]-nd[0])**2 + (b["xyz"][1]-nd[1])**2 + (b["xyz"][2]-nd[2])**2)
                            if dist < 0.08: # 8cm matching threshold
                                b["xyz"] = nd # Update position
                                found_match = True
                                break
                        if not found_match:
                            # Brand new berry discovered in rescan
                            new_id = len(berry_registry) + 1
                            berry_registry.append({"id": new_id, "xyz": nd, "harvested": False})
                            matched_new += 1
                    
                    self.get_logger().info(f"Rescan: {len(new_detections)} hits. Found {matched_new} new berries.")
                    self.publish_gui_state("Rescan Found Berries", event=f"Rescan: {matched_new} new berries found.")

            self.async_sleep(1.0)
    
    def execute_approach_and_slide(self, berry_xyz):
        """
        Two-Stage Approach with strict 24cm Tool Center Point (TCP) Offset:
        1. Move to Approach Pose (Wrist is 34cm back) 
        2. Slide to Target Pose (Wrist is exactly 24cm back, putting blades on the berry)
        """
        try:
            berry_x, berry_y, berry_z = berry_xyz
            
            # --- TOOL CONFIGURATION ---
            TOOL_LENGTH = 0.24 # The physical distance from tool0 to the blades
            
            # Adjust this to move the blades closer or farther from the berry along the approach vector.
            # +0.045 = stop 4.5cm farther away from the berry
            STEM_OFFSET = 0.045
            
            # Lower the final Z position by 3cm
            FINAL_Z_LOWER = -0.03
            
            FINAL_STANDOFF = TOOL_LENGTH + STEM_OFFSET
            APPROACH_BACKOFF = 0.001

            # Calculate Approach Vector (Direction from robot base to berry)
            approach_yaw = math.atan2(berry_y, berry_x)
            approach_vector = np.array([math.cos(approach_yaw), math.sin(approach_yaw), 0.0])
            nav = approach_vector / np.linalg.norm(approach_vector)
            
            # MANUAL OFFSETS
            MANUAL_OFFSET_X = 0.0
            MANUAL_OFFSET_Y = 0.01 # 1cm to the left
            MANUAL_OFFSET_Z = 0.195
            
            berry_pos = np.array([berry_x, berry_y, berry_z])
            berry_pos[0] += MANUAL_OFFSET_X
            berry_pos[1] += MANUAL_OFFSET_Y
            berry_pos[2] += MANUAL_OFFSET_Z
            self.get_logger().info(f"Target Berry Pos: {berry_pos}")

            # Define Orientation Frame (Matching your original orientation logic)
            z_axis = np.array([0.0, 0.0, -1.0]) # Down
            y_axis = -nav                       # -Y points to Berry
            x_axis = np.cross(y_axis, z_axis)
            x_axis = x_axis / np.linalg.norm(x_axis)
            y_axis = np.cross(z_axis, x_axis)
            y_axis = y_axis / np.linalg.norm(y_axis)
            
            R = np.eye(4)
            R[0:3, 0] = x_axis
            R[0:3, 1] = y_axis
            R[0:3, 2] = z_axis
            q = tf_transformations.quaternion_from_matrix(R)

            # --- CALCULATE EXACT POSITIONS ---
            # 1. Final Pose: Wrist stops exactly FINAL_STANDOFF meters away from berry
            final_pos = berry_pos - (FINAL_STANDOFF * nav)
            
            target_pose = Pose()
            target_pose.position.x = final_pos[0]
            target_pose.position.y = final_pos[1]
            target_pose.position.z = final_pos[2] + FINAL_Z_LOWER
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]

            # 2. Approach Pose: Wrist backs up another 10cm from the final cut position
            approach_pos = final_pos - (APPROACH_BACKOFF * nav)
            
            approach_pose = Pose()
            approach_pose.orientation = target_pose.orientation
            approach_pose.position.x = approach_pos[0]
            approach_pose.position.y = approach_pos[1]
            approach_pose.position.z = approach_pos[2] + FINAL_Z_LOWER

            # --- EXECUTION ---
            approach_ps = PoseStamped()
            approach_ps.header.frame_id = "base_link"
            approach_ps.header.stamp = self.get_clock().now().to_msg()
            approach_ps.pose = approach_pose
            
            self.get_logger().info("Solving IK for Approach Pose...")
            approach_joints = self.solve_approach_ik(approach_ps)
            
            if not approach_joints:
                 self.get_logger().error("IK failed for Approach Pose! Cannot align.")
                 return False

            self.publish_debug_marker(
                approach_pose.position.x, 
                approach_pose.position.y, 
                approach_pose.position.z,
                marker_id=1,
                color=(1.0, 1.0, 0.0) 
            )
            
            self.get_logger().info("Moving to Approach Joints...")
            success = self.move_group.move_to_joints(approach_joints)
            
            if not success:
                self.get_logger().error("Failed to reach Approach Pose!")
                return False
                
            self.get_logger().info("Reached Approach Pose. Starting Cartesian Slide...")
            
            # MOVE B: Cartesian Slide to Target
            req = GetCartesianPath.Request()
            req.header.frame_id = "base_link"
            req.header.stamp = self.get_clock().now().to_msg()
            req.group_name = "ur_manipulator"
            req.waypoints = [target_pose] 
            req.max_step = 0.01
            req.jump_threshold = 0.0
            req.avoid_collisions = True
            
            if not self.move_group.cartesian_service.wait_for_service(timeout_sec=2.0):
                 self.get_logger().error("Cartesian Service unavailable")
                 return False
                 
            future = self.move_group.cartesian_service.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            if not future.done(): return False
            response = future.result()
            
            if response.error_code.val != 1 or response.fraction < 0.9:
                self.get_logger().warn(f"Cartesian slide failed or incomplete: {response.error_code.val}")
                return False
                 
            # Execute Trajectory
            move_req = ExecuteTrajectory.Goal()
            move_req.trajectory = response.solution
            
            goal_future = self.move_group.execute_action_client.send_goal_async(move_req)
            rclpy.spin_until_future_complete(self, goal_future)
            goal_handle = goal_future.result()
            
            if not goal_handle.accepted:
                self.get_logger().error("Slide trajectory rejected")
                return False
                
            res_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, res_future)
            
            result = res_future.result().result
            if result.error_code.val == 1:
                 # Calculate 3D Coordinates Validation Data
                 try:
                     trans = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
                     q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
                     rot_matrix = tf_transformations.quaternion_matrix(q)
                     y_vector = rot_matrix[:3, 1]  # The UR10 approaches with -Y pointing at the berry
                     base_pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                     cutter_offset_y = 0.24 # 240mm end effector
                     tip_pos = base_pos - (y_vector * cutter_offset_y)
                     tip_pos[0] -= 0.0    # MANUAL_OFFSET_X
                     tip_pos[1] -= 0.01   # MANUAL_OFFSET_Y
                     tip_pos[2] -= 0.165  # MANUAL_OFFSET_Z + FINAL_Z_LOWER
                     
                     # Account for direction of approach vector
                     approach_yaw = math.atan2(berry_xyz[1], berry_xyz[0])
                     approach_vector = np.array([math.cos(approach_yaw), math.sin(approach_yaw), 0.0])
                     nav = approach_vector / np.linalg.norm(approach_vector)
                     
                     # Add back the STEM_OFFSET distance (robot stops 45mm short of the center to cut the stem)
                     tip_pos += (0.045 * nav)
                     
                     rob_gt = (tip_pos[0] * 1000.0, tip_pos[1] * 1000.0, tip_pos[2] * 1000.0)
                     cam_pred = (berry_xyz[0] * 1000.0, berry_xyz[1] * 1000.0, berry_xyz[2] * 1000.0)
                     error_mm = math.sqrt((cam_pred[0] - rob_gt[0])**2 + (cam_pred[1] - rob_gt[1])**2 + (cam_pred[2] - rob_gt[2])**2)
                     
                     # Apply exact physical offsets to match the Cartesian target pose
                     # Y offset = +10mm, Z offset = 195 - 30 = +165mm
                     # Final standoff = 240 (cutter) + 45 (stem) = 285mm
                     cp_tool0 = (
                         cam_pred[0] + 0.0 - (nav[0] * 285.0),
                         cam_pred[1] + 10.0 - (nav[1] * 285.0),
                         cam_pred[2] + 165.0
                     )
                     rk_tool0 = (
                         rob_gt[0] + 0.0 - (nav[0] * 285.0),
                         rob_gt[1] + 10.0 - (nav[1] * 285.0),
                         rob_gt[2] + 165.0
                     )

                     c_str = f"({cam_pred[0]:.1f}, {cam_pred[1]:.1f}, {cam_pred[2]:.1f})"
                     cp_t0_str = f"({cp_tool0[0]:.1f}, {cp_tool0[1]:.1f}, {cp_tool0[2]:.1f})"
                     r_str = f"({rob_gt[0]:.1f}, {rob_gt[1]:.1f}, {rob_gt[2]:.1f})"
                     rk_t0_str = f"({rk_tool0[0]:.1f}, {rk_tool0[1]:.1f}, {rk_tool0[2]:.1f})"
                     
                     val_data = {"c_str": c_str, "cp_t0_str": cp_t0_str, "r_str": r_str, "rk_t0_str": rk_t0_str, "error": f"{error_mm:.2f}"}
                 except Exception as e:
                     self.get_logger().error(f"Validation Error: {e}")
                     val_data = None

                 self.get_logger().info("Cutting...")
                 self.publish_gui_state("Cutting", val_data=val_data)
                 
                 upward_pose = Pose()
                 upward_pose.position.x = target_pose.position.x
                 upward_pose.position.y = target_pose.position.y
                 upward_pose.position.z = target_pose.position.z + 0.035
                 upward_pose.orientation = target_pose.orientation
                 
                 req_up = GetCartesianPath.Request()
                 req_up.header.frame_id = "base_link"
                 req_up.header.stamp = self.get_clock().now().to_msg()
                 req_up.group_name = "ur_manipulator"
                 req_up.waypoints = [upward_pose]
                 req_up.max_step = 0.01       
                 req_up.jump_threshold = 0.0
                 req_up.avoid_collisions = True
                 
                 try:
                     future_up = self.move_group.cartesian_service.call_async(req_up)
                     rclpy.spin_until_future_complete(self, future_up)
                     if future_up.done():
                         response_up = future_up.result()
                         if response_up.error_code.val == 1 and response_up.fraction >= 0.9:
                             move_req_up = ExecuteTrajectory.Goal()
                             move_req_up.trajectory = response_up.solution
                             goal_future_up = self.move_group.execute_action_client.send_goal_async(move_req_up)
                             rclpy.spin_until_future_complete(self, goal_future_up)
                             goal_handle_up = goal_future_up.result()
                             if goal_handle_up.accepted:
                                 res_future_up = goal_handle_up.get_result_async()
                                 rclpy.spin_until_future_complete(self, res_future_up)
                 except Exception as e_up:
                     self.get_logger().error(f"Upward move execution failed: {e_up}")

                 self.get_logger().info("Upward Move Complete! Triggering Cut...")
                 self.publish_gui_state("Triggering Cut")
                 self.trigger_servo()
                 self.open_cutter()  # Open immediately after cut
                 return True
            
            self.get_logger().error(f"Slide execution failed: {result.error_code.val}")
            return False

        except Exception as e:
            self.get_logger().error(f"execute_approach_and_slide failed: {e}")
            return False

def main():
    rclpy.init()
    node = HarvestStateMachine()
    try:
        node.run_harvest_loop()
    except KeyboardInterrupt: pass
    finally: rclpy.shutdown()

if __name__ == '__main__':
    main()