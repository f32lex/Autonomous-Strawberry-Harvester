#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
import numpy as np

# Patch for numpy >= 1.24 removing np.float which crashes tf_transformations -> transforms3d
if not hasattr(np, 'float'):
    np.float = float

import tf_transformations
import csv
import os
from datetime import datetime
import math

class CoordinateValidator(Node):
    def __init__(self):
        super().__init__('coordinate_validator')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Tool Offset (Distance from tool0 wrist mount to the actual cutting blades)
        self.cutter_offset_z = 0.24 # 24cm (240mm)
        
        self.csv_data = []
        self.csv_headers = [
            "Camera Prediction (Xc, Yc, Zc) in mm",
            "CP tool0 Position (mm)",
            "Robot Kinematics (Xr, Yr, Zr) in mm",
            "RK tool0 Position (mm)",
            "Euclidean Error (mm)"
        ]
        
        self.get_logger().info("=========================================")
        self.get_logger().info("COORDINATE VALIDATION TOOL STARTED")
        self.get_logger().info("=========================================")
        
        # Start the interactive loop
        self.create_timer(1.0, self.interactive_loop)
        self.loop_active = False

    def get_ground_truth(self):
        """Reads kinematics for tool0, extends by cutter offset, returns X,Y,Z in mm relative to base_link"""
        try:
            now = rclpy.time.Time()
            if self.tf_buffer.can_transform('base_link', 'tool0', now, rclpy.time.Duration(seconds=1.0)):
                trans = self.tf_buffer.lookup_transform('base_link', 'tool0', now)
                
                # We need to project the tool0 position FOREWARD along its Z axis by cutter_offset_z
                q = [trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w]
                rot_matrix = tf_transformations.quaternion_matrix(q)
                
                # The tool vector points along the local Z axis [0, 0, 1]
                z_vector = rot_matrix[:3, 2]
                
                base_pos = np.array([trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z])
                tip_pos = base_pos + (z_vector * self.cutter_offset_z)
                
                # Convert meters to millimeters for thesis readability
                return (tip_pos[0] * 1000.0, tip_pos[1] * 1000.0, tip_pos[2] * 1000.0)
            else:
                self.get_logger().error("Cannot transform base_link -> tool0. Is UR running?")
                return None
        except Exception as e:
            self.get_logger().error(f"TF Error: {e}")
            return None

    def get_camera_prediction(self):
        """Reads highest confidence berry from vision, returns X,Y,Z in mm"""
        try:
            now = rclpy.time.Time()
            
            # Find any active detected_berry tf frames
            # In a real validation scenario, you'd likely isolate a single berry in view,
            # or we can just grab detected_berry_1
            target_frame = 'detected_berry_1'
            
            if self.tf_buffer.can_transform('base_link', target_frame, now, rclpy.time.Duration(seconds=1.0)):
                trans = self.tf_buffer.lookup_transform('base_link', target_frame, now)
                
                cam_x = trans.transform.translation.x * 1000.0
                cam_y = trans.transform.translation.y * 1000.0
                cam_z = trans.transform.translation.z * 1000.0
                return (cam_x, cam_y, cam_z)
            else:
                self.get_logger().warn(f"Cannot find '{target_frame}' in TF tree. Is the camera seeing a berry?")
                return None
        except Exception as e:
            self.get_logger().error(f"TF Error: {e}")
            return None

    def calculate_error(self, cam_pos, rob_pos):
        """Euclidean distance root((Xc-Xr)^2 + ... )"""
        return math.sqrt((cam_pos[0] - rob_pos[0])**2 + (cam_pos[1] - rob_pos[1])**2 + (cam_pos[2] - rob_pos[2])**2)

    def interactive_loop(self):
        if self.loop_active: return
        self.loop_active = True
        
        while rclpy.ok():
            print("\n-----------------------------------------")
            print("To collect a data point:")
            print("1. Jog the robot manually so the cutter tip touches the center of the target berry.")
            print("2. Ensure the camera can see the berry (or keep the robot still while the camera locks on).")
            
            val = input("\nPress [Enter] to capture point, or type 'q' to save and quit: ")
            if val.lower() == 'q':
                self.save_csv()
                break
            
            # Sample Data
            rob_gt = self.get_ground_truth()
            if not rob_gt:
                print("Failed to get robot ground truth. Skipping point.")
                continue
                
            cam_pred = self.get_camera_prediction()
            if not cam_pred:
                print("Failed to get camera prediction. Skipping point.")
                continue
                
            error_mm = self.calculate_error(cam_pred, rob_gt)
            
            approach_yaw = math.atan2(cam_pred[1], cam_pred[0])
            approach_vector = np.array([math.cos(approach_yaw), math.sin(approach_yaw), 0.0])
            nav = approach_vector / np.linalg.norm(approach_vector)

            # Apply identical manual offsets used in harvest.py's execute_approach_and_slide
            # MANUAL_OFFSET_Y = 10mm | MANUAL_OFFSET_Z = 195mm | FINAL_Z_LOWER = -30mm -> +165mm
            # FINAL_STANDOFF = 240mm (cutter) + 45mm (stem offset) = 285mm
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
            
            print(f"\n--- DATA COLLECTED ---")
            print(f"Camera Prediction:    {c_str}")
            print(f"CP tool0 Position:    {cp_t0_str}")
            print(f"Robot Kinematics:     {r_str}")
            print(f"RK tool0 Position:    {rk_t0_str}")
            print(f"Euclidean Error:      {error_mm:.2f}")
            
            self.csv_data.append([
                c_str,
                cp_t0_str,
                r_str,
                rk_t0_str,
                f"{error_mm:.2f}"
            ])
            
        # Exit node
        import sys
        sys.exit(0)

    def save_csv(self):
        if not self.csv_data:
            print("No data collected. Exiting without saving.")
            return
            
        default_dir = "/home/f32lex/strawberry_ws/rosbags"
        default_file = f"thesis_coordinate_validation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        default_path = os.path.join(default_dir, default_file)
        
        print(f"\nDefault Save Path: {default_path}")
        user_input = input("Enter a custom path/filename (or press Enter to use default): ").strip()
        
        if not user_input:
            final_path = default_path
        elif not user_input.endswith('.csv'):
            final_path = user_input + '.csv'
        else:
            final_path = user_input
            
        # Ensure the directory exists if they typed a new path
        save_dir = os.path.dirname(final_path)
        if save_dir and not os.path.exists(save_dir):
            try:
                os.makedirs(save_dir)
            except Exception as e:
                print(f"Error creating directory {save_dir}: {e}. Falling back to default.")
                final_path = default_path

        try:
            with open(final_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(self.csv_headers)
                writer.writerows(self.csv_data)
                
            print(f"\nSUCCESS: Data saved to {final_path}")
            print("This file contains the exact table format requested for your thesis!")
        except Exception as e:
            print(f"Failed to save CSV: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CoordinateValidator()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
