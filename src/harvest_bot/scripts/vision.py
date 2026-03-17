#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
# Patch for numpy >= 1.24 removing np.float
if not hasattr(np, 'float'):
    np.float = float

import ultralytics.utils.loss
import torch.nn as nn
if not hasattr(ultralytics.utils.loss, 'BCEDiceLoss'):
    class BCEDiceLoss(nn.Module):
        def __init__(self, *args, **kwargs):
            super().__init__()
    ultralytics.utils.loss.BCEDiceLoss = BCEDiceLoss

if not hasattr(ultralytics.utils.loss, 'MultiChannelDiceLoss'):
    class MultiChannelDiceLoss(nn.Module):
        def __init__(self, *args, **kwargs):
            super().__init__()
    ultralytics.utils.loss.MultiChannelDiceLoss = MultiChannelDiceLoss

from ultralytics import YOLO
from tf2_ros import TransformBroadcaster
import tf_transformations
import math
import threading
import time

class VisionBroadcaster(Node):
    def __init__(self):
        super().__init__('vision')
        
        # Initialize YOLO model (Strawberry/Peduncle Detection)
        self.model = YOLO('/home/f32lex/strawberry_ws/src/harvest_bot/models/best.pt') 
        
        # Class IDs
        self.ripe = 0  
        self.peduncle = 1
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.bridge = CvBridge()
        self.color_sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, 10)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        
        # Publisher 
        self.pub = self.create_publisher(PoseStamped, '/detected_strawberry_pose', 10)
        self.target_pub = self.create_publisher(String, '/harvest_targets', 10)
        
        # Data Storage
        self.lock = threading.Lock()
        self.latest_color_image = None
        self.latest_depth_image = None
        self.camera_intrinsics = None
        self.camera_distortion = None  # D vector for lens undistortion
        self.visualization_image = None
        
        # Threading
        self.running = True
        self.inference_thread = threading.Thread(target=self.inference_loop, daemon=True)
        self.inference_thread.start()
        self.create_timer(0.033, self.display_callback)
        self.get_logger().info("Vision Broadcaster Initialized (Threaded). Waiting for images...")
        self.last_image_time = time.time()
        self.create_timer(2.0, self.watchdog_callback)

    def watchdog_callback(self):
        if time.time() - self.last_image_time > 3.0:
             self.get_logger().warn("No images received from camera for >3 seconds! Check connection.")

    def info_callback(self, msg):
        if self.camera_intrinsics is None:
            self.camera_intrinsics = np.array(msg.k).reshape((3, 3))
            self.camera_distortion = np.array(msg.d)  # [k1, k2, p1, p2, k3]
            self.get_logger().info(f"Camera Intrinsics Loaded. Distortion Coeffs: {self.camera_distortion}")

    def depth_callback(self, msg):
        with self.lock:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def color_callback(self, msg):
        self.last_image_time = time.time()
        with self.lock:
            self.latest_color_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def display_callback(self):
        try:
            vis_img = None
            with self.lock:
                if self.visualization_image is not None:
                     vis_img = self.visualization_image.copy()
            
            if vis_img is not None:
                cv2.imshow("YOLO Vision Broadcaster", vis_img)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().warn(f"Display Error: {e}")

    def inference_loop(self):
        """
        Runs YOLO inference in a separate loop to avoid blocking ROS callbacks.
        """
        while self.running and rclpy.ok():
            color_img = None
            depth_img = None
            intrinsics = None
            
            with self.lock:
                if self.latest_color_image is not None:
                    color_img = self.latest_color_image.copy()
                if self.latest_depth_image is not None:
                    depth_img = self.latest_depth_image.copy()
                intrinsics = self.camera_intrinsics
                dist_coeffs = self.camera_distortion

            if color_img is None or depth_img is None or intrinsics is None:
                time.sleep(0.1)
                continue

            # Run YOLO
            results = self.model(color_img, imgsz=640, verbose=False)
            
            # Prepare visualization
            vis_img = color_img.copy()
            
            berry_count = 0 # Track berries
            peduncle_count = 0 # Track peduncles
            target_list = [] # Store all targets for JSON
            
            for result in results:
                boxes = result.boxes
                masks = result.masks
                
                # Retrieve class names from the model
                class_names = result.names 
                
                for i, box in enumerate(boxes):
                    cls_id = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    # Determine by name or fallback to ID
                    class_name = class_names[cls_id].lower() if class_names else ""
                    
                    is_peduncle = ('peduncle' in class_name or 'stem' in class_name)
                    is_berry = ('berry' in class_name or 'ripe' in class_name or 'strawberry' in class_name)
                    
                    # Fallbacks if names are missing
                    if not is_peduncle and not is_berry:
                        if cls_id == self.peduncle: is_peduncle = True
                        elif cls_id == self.ripe: is_berry = True
                        
                    if is_berry and conf > 0.8:
                        valid = True
                    elif is_peduncle and conf > 0.4:
                        valid = True
                    else:
                        valid = False

                    if valid:
                        # Default Bounding Box
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        u = (x1 + x2) // 2
                        v = (y1 + y2) // 2
                        angle = 0.0
                        
                        # Default Box Points for visualization
                        box_points = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.int32)
                        
                        # Override with OBB if mask is available
                        if masks is not None and i < len(masks.xy):
                            contour = masks.xy[i].astype(np.int32)
                            if len(contour) > 0:
                                rect = cv2.minAreaRect(contour)
                                box_center, box_size, angle_obb = rect
                                
                                # Standardize angle
                                if box_size[0] < box_size[1]:
                                    angle_obb += 90
                                angle = angle_obb
                                
                                box_points_rect = cv2.boxPoints(rect)
                                box_points = np.int32(box_points_rect)
                                
                                u, v = int(box_center[0]), int(box_center[1])
                                
                        # Visuals
                        if is_berry:
                            cv2.drawContours(vis_img, [box_points], 0, (0, 255, 0), 2)
                            cv2.circle(vis_img, (u, v), 5, (0, 0, 255), -1)
                        else:  # Peduncle
                            cv2.drawContours(vis_img, [box_points], 0, (255, 0, 0), 2)
                            cv2.circle(vis_img, (u, v), 5, (255, 0, 0), -1)
                        
                        # Safety Check
                        h, w, _ = color_img.shape
                        if u >= w or v >= h: continue
                        
                        # 1. READ DEPTH (Z) — Prefer mask-based sampling over rectangular window
                        h_d, w_d = depth_img.shape
                        depth_mm = 0
                        
                        # Primary: Use the YOLO segmentation mask to sample depth ONLY inside the berry
                        if masks is not None and i < len(masks.xy):
                            contour = masks.xy[i].astype(np.int32)
                            if len(contour) > 4:
                                mask_canvas = np.zeros((h_d, w_d), dtype=np.uint8)
                                cv2.fillPoly(mask_canvas, [contour], 255)
                                berry_depth_pixels = depth_img[mask_canvas > 0]
                                valid_mask_depths = berry_depth_pixels[berry_depth_pixels > 0]
                                if len(valid_mask_depths) > 3:
                                    # Use median depth (berry center, not front surface)
                                    depth_mm = float(np.median(valid_mask_depths))
                        
                        # Fallback: 51x51 window around bbox center
                        if depth_mm == 0:
                            u_clipped = int(np.clip(u, 0, w_d - 1))
                            v_clipped = int(np.clip(v, 0, h_d - 1))
                            depth_mm = depth_img[v_clipped, u_clipped]
                            
                        if depth_mm == 0:
                            window = depth_img[max(0,v-25):min(h_d,v+26), max(0,u-25):min(w_d,u+26)]
                            valid_pixels = window[window > 0]
                            if len(valid_pixels) == 0:
                                continue
                            depth_mm = np.median(valid_pixels)
                        
                        z_metric = depth_mm / 1000.0
                        
                        if z_metric > 1.5:
                            continue
                        
                        # 2. Compute 3D coordinates using lens-corrected pixel coordinates
                        fx = intrinsics[0, 0]
                        fy = intrinsics[1, 1]
                        cx = intrinsics[0, 2]
                        cy = intrinsics[1, 2]
                        
                        if dist_coeffs is not None and np.any(dist_coeffs != 0):
                            # Undistort the center pixel to remove radial/tangential lens error
                            raw_pt = np.array([[[float(u), float(v)]]], dtype=np.float32)
                            undist_pt = cv2.undistortPoints(raw_pt, intrinsics, dist_coeffs, P=intrinsics)
                            u_corr = undist_pt[0, 0, 0]
                            v_corr = undist_pt[0, 0, 1]
                        else:
                            u_corr, v_corr = float(u), float(v)
                        
                        # ─── Lateral Correction ──────────────────────────────────────────────
                        # Clustered berries have a systematic
                        # rightward centroid bias because the more-visible berry skews the
                        # YOLO mask center pixel to the right. Shift u_corr left to compensate.
                        # Tune this value: negative = shift left (corrects rightward offset).
                        # For single-strand berries this has negligible effect since their
                        # masks are well-centered. Typical calibrated range: -5 to -20 px.
                        LATERAL_PIXEL_OFFSET = 0  # pixels: Set to 0 to align exactly with berry center
                        u_corr += LATERAL_PIXEL_OFFSET
                        # ─────────────────────────────────────────────────────────────────────
                        
                        x_metric = (u_corr - cx) * z_metric / fx
                        y_metric = (v_corr - cy) * z_metric / fy
                        
                        if is_berry:
                            self.get_logger().info(f"Broadcast BERRY_{berry_count}: XYZ=({x_metric:.2f}, {y_metric:.2f}, {z_metric:.2f}) | Conf: {conf:.2f} | Angle: {angle:.1f}deg")
                            self.broadcast_frame(x_metric, y_metric, z_metric, angle, obj_id=berry_count, is_berry=True)
                            target_list.append({"id": berry_count, "type": "berry", "x": x_metric, "y": y_metric, "z": z_metric, "conf": conf, "angle": angle})
                            berry_count += 1
                        else:
                            self.get_logger().info(f"Broadcast PEDUNCLE_{peduncle_count}: XYZ=({x_metric:.2f}, {y_metric:.2f}, {z_metric:.2f}) | Conf: {conf:.2f}")
                            self.broadcast_frame(x_metric, y_metric, z_metric, angle, obj_id=peduncle_count, is_berry=False)
                            target_list.append({"id": peduncle_count, "type": "peduncle", "x": x_metric, "y": y_metric, "z": z_metric, "conf": conf, "angle": angle})
                            peduncle_count += 1
                        
                        # Project the 3D point back to a 2D pixel for visual targeting
                        val_u = int((x_metric * fx / z_metric) + cx)
                        val_v = int((y_metric * fy / z_metric) + cy)
                        
                        # Draw a small target dot for where the robot will aim
                        cv2.circle(vis_img, (val_u, val_v), 5, (0, 255, 255), -1)
                        
                        # Display Text
                        label = "Berry" if is_berry else "Peduncle"
                        text_str = f"{label}: {conf:.2f}"
                        color = (0, 255, 0) if is_berry else (255, 0, 0)
                        
                        # Offset the text slightly differently so they don't overlap as badly if they are near each other
                        y_offset = max(0, y1 - 20) if is_berry else max(0, y2 + 20)
                        cv2.putText(vis_img, text_str, (x1, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            
            # Update Visualization
            with self.lock:
                self.visualization_image = vis_img
                
            # Broadcast Telemetry
            target_msg = String()
            target_msg.data = json.dumps(target_list)
            self.target_pub.publish(target_msg)

    def broadcast_frame(self, x, y, z, angle=0.0, obj_id=0, is_berry=True):
        """
        Broadcasts the relative Camera Optical Frame.
        """
        t = TransformStamped()
        
        t.header.stamp = self.get_clock().now().to_msg()
        # Use camera_color_optical_frame as requested for RealSense alignment
        t.header.frame_id = 'camera_color_optical_frame' 
        
        # Append the unique ID to the frame name
        prefix = "detected_berry" if is_berry else "detected_peduncle"
        t.child_frame_id = f'{prefix}_{obj_id}'

        # Direct Mapping (Optical Frame)
        t.transform.translation.x = float(x)
        t.transform.translation.y = float(y)
        t.transform.translation.z = float(z)

        # Orientation: Use angle derived from mask's Oriented Bounding Box
        q = tf_transformations.quaternion_from_euler(0, 0, math.radians(angle))
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = VisionBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
        node.inference_thread.join(timeout=1.0)
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
