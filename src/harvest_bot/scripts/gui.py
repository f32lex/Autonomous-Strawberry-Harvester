#!/usr/bin/env python3
import sys
import json
import rclpy
import math
import csv
from datetime import datetime
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QLabel, QTextEdit, QFileDialog, QGroupBox, QGridLayout, QCheckBox, QInputDialog
from PyQt5.QtCore import QTimer, pyqtSignal, QObject, Qt, QPoint
from PyQt5.QtGui import QPainter, QColor, QFont, QPen
import subprocess
import signal
import os
import threading
import queue
import pyqtgraph as pg  

class RosNode(Node):
    def __init__(self, signals):
        super().__init__('harvest_gui')
        self.signals = signals
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.sub = self.create_subscription(String, '/harvest_gui_status', self.status_callback, 10)
        self.target_sub = self.create_subscription(String, '/harvest_targets', self.targets_callback, 10)
        self.control_pub = self.create_publisher(String, '/harvest_control', 10)
        self.class_pub = self.create_publisher(String, '/gui_classification', 10)
        
        # Thread-safe queue for UI-to-ROS communication
        self.cmd_queue = queue.Queue()
        self.timer = self.create_timer(0.1, self.process_queue)

    def process_queue(self):
        while not self.cmd_queue.empty():
            cmd_dict = self.cmd_queue.get()
            if "classification" in cmd_dict:
                msg = String()
                msg.data = cmd_dict["classification"]
                self.class_pub.publish(msg)
            else:
                msg = String()
                msg.data = json.dumps(cmd_dict)
                self.control_pub.publish(msg)

    def status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.signals.status_updated.emit(data)
        except Exception as e:
            self.signals.log_message.emit(f"[GUI ERROR] Failed to parse JSON: {e} | Raw Data: {msg.data}")
            self.get_logger().error(f"Error parsing status: {e}")

    def targets_callback(self, msg):
        try:
            targets = json.loads(msg.data)
            self.signals.targets_updated.emit(targets)
        except Exception as e:
            self.get_logger().error(f"Error parsing targets JSON: {e}")

class GuiSignals(QObject):
    status_updated = pyqtSignal(dict)
    targets_updated = pyqtSignal(list)
    log_message = pyqtSignal(str)  

class MapWidget(QWidget):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(400, 400)
        self.targets = [] # List of dicts {"id", "type", "x", "y", "z", "conf"}
        self.robot_eff_pos = (0, 0)
        
    def update_data(self, targets, robot_eff_pos):
        self.targets = targets
        self.robot_eff_pos = robot_eff_pos
        self.update()
        
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        
        painter.fillRect(self.rect(), QColor("#1e1e1e"))
        
        w = self.width()
        h = self.height()
        cx = w / 2
        cy = h / 2
        
        painter.setPen(QPen(QColor("#333333"), 1, Qt.DashLine))
        for i in range(0, w, 50): painter.drawLine(i, 0, i, h)
        for i in range(0, h, 50): painter.drawLine(0, i, w, i)
        
        scale = 150 
        rx = cx
        ry = h - 50
        
        painter.setPen(QPen(QColor("#555555"), 1, Qt.DashLine))
        for r_m in [0.5, 1.0, 1.5, 2.0]:
            r_px = int(r_m * scale)
            painter.drawArc(int(rx - r_px), int(ry - r_px), r_px*2, r_px*2, 0, 180 * 16)
            painter.setPen(QPen(QColor("#aaaaaa"), 1))
            painter.drawText(int(rx + r_px + 5), int(ry - 5), f"{r_m}m")
            painter.setPen(QPen(QColor("#555555"), 1, Qt.DashLine))

        def to_screen(r_x, r_y):
            px = rx + r_x * scale
            py = ry - r_y * scale
            return int(px), int(py)

        painter.setBrush(QColor("#4287f5"))
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawRect(int(rx - 15), int(ry - 15), 30, 30)
        painter.setBrush(QColor("#ffffff"))
        painter.drawPolygon(
            QPoint(int(rx), int(ry - 20)),
            QPoint(int(rx - 8), int(ry - 5)),
            QPoint(int(rx + 8), int(ry - 5))
        )
        
        # Draw Targets
        for t in self.targets:
            px, py = to_screen(t['x'], t['y'])
            
            if t['type'] == 'berry':
                painter.setBrush(QColor("#f54242"))
                painter.setPen(QPen(QColor("#ffffff"), 1))
                painter.drawEllipse(px - 10, py - 10, 20, 20)
                # Label
                painter.setPen(QColor("#ffffff"))
                painter.drawText(px + 12, py + 5, f"B{t['id']}")
            else:
                painter.setBrush(QColor("#42f5aa"))
                painter.setPen(QPen(QColor("#ffffff"), 1))
                painter.drawRect(px - 6, py - 6, 12, 12)
                # Label
                painter.setPen(QColor("#ffffff"))
                painter.drawText(px + 10, py + 5, f"S{t['id']}")
            
        ex, ey = self.robot_eff_pos
        px, py = to_screen(ex, ey)
        painter.setBrush(QColor("#4287f5"))
        painter.setPen(QPen(QColor("#ffffff"), 2))
        painter.drawEllipse(px - 10, py - 10, 20, 20)
        painter.setPen(QPen(QColor("#4287f5"), 2, Qt.DashLine))
        painter.drawLine(int(rx), int(ry), px, py)


class HarvestGUI(QMainWindow):
    def __init__(self, ros_node, signals):
        super().__init__()
        self.ros_node = ros_node
        self.signals = signals
        
        self.procs = {
            'hw_control': None, 'hw_moveit': None, 'sim': None,
            'vision': None, 'harvest': None, 'bag_play': None, 'bag_rec': None,
            'val': None
        }
        
        self.setWindowTitle("Harvesting Bot Dashboard")
        self.setMinimumSize(1100, 800)
        self.setStyleSheet("background-color: #121212; color: #ffffff;")
        
        central = QWidget()
        self.setCentralWidget(central)
        layout = QHBoxLayout(central)
        
        # --- LEFT PANEL (Map & Data Logs) ---
        left_panel = QVBoxLayout()
        title_map = QLabel("Live Berry Map")
        title_map.setFont(QFont("Arial", 16, QFont.Bold))
        self.map_widget = MapWidget()
        left_panel.addWidget(title_map)
        left_panel.addWidget(self.map_widget)
        
        title_logs = QLabel("Event Logs")
        title_logs.setFont(QFont("Arial", 14, QFont.Bold))
        self.txt_logs = QTextEdit()
        self.txt_logs.setReadOnly(True)
        self.txt_logs.setStyleSheet("background-color: #1e1e1e; border: 1px solid #333; font-family: monospace;")
        self.txt_logs.setMaximumHeight(150)
        left_panel.addWidget(title_logs)
        left_panel.addWidget(self.txt_logs)
        
        title_telemetry = QLabel("Target Monitor")
        title_telemetry.setFont(QFont("Arial", 14, QFont.Bold))
        self.txt_target_logs = QTextEdit()
        self.txt_target_logs.setReadOnly(True)
        self.txt_target_logs.setStyleSheet("background-color: #1a1a2e; border: 1px solid #4287f5; font-family: monospace; color: #42f5aa;")
        left_panel.addWidget(title_telemetry)
        left_panel.addWidget(self.txt_target_logs)
        
        # --- RIGHT PANEL (Controls & Metrics) ---
        right_panel = QVBoxLayout()
        
        # OBJECTIVE METRICS
        metrics_group = QGroupBox("Objective Metrics")
        metrics_group.setStyleSheet("QGroupBox { font-size: 14px; font-weight: bold; border: 1px solid #333; margin-top: 10px; }")
        metrics_layout = QVBoxLayout(metrics_group)
        
        self.lbl_state = QLabel("State: Idle")
        self.lbl_state.setFont(QFont("Arial", 14))
        self.lbl_state.setStyleSheet("color: #42f5aa;")

        self.lbl_efficiency = QLabel("Harvest Efficiency: 0.0%")
        self.lbl_efficiency.setFont(QFont("Arial", 12))
        self.lbl_efficiency.setStyleSheet("color: #4287f5;")
        
        self.lbl_act = QLabel("Average Cycle Time: 0.0s")
        self.lbl_act.setFont(QFont("Arial", 12))
        self.lbl_act.setStyleSheet("color: #b359f5;")
        
        # DATA STORAGE FOR PLOT AND CSV
        self.targets_engaged = 0
        self.total_attempts = 0
        self.total_time_spent = 0.0
        self.total_berries_in_field = 0  # Set from scan detection count
        
        self.caught_1st = 0
        self.caught_retries = 0
        self.missed_cut = 0
        self.dropped_fruit = 0
        
        self.cycle_caught = 0
        self.cycle_missed = 0
        self.cycle_dropped = 0
        self.cycles = 0
        
        self.csv_data = [] # Stores rows for CSV export
        self.val_csv_data = [] # Stores validation coordinate tuples
        
        self.lbl_attempts = QLabel("Attempts: 0 | Catches: 0 | Misses: 0 | Drops: 0")
        self.lbl_attempts.setFont(QFont("Arial", 12))
        self.lbl_success_rate = QLabel("Harvest Success Rate (Yield): 0.0%")
        self.lbl_success_rate.setFont(QFont("Arial", 16, QFont.Bold))
        self.lbl_success_rate.setStyleSheet("color: #f5a142;") 
        
        metrics_layout.addWidget(self.lbl_state)
        metrics_layout.addWidget(self.lbl_efficiency)
        metrics_layout.addWidget(self.lbl_act)
        metrics_layout.addWidget(self.lbl_attempts)
        metrics_layout.addWidget(self.lbl_success_rate)
        
        # LIVE GRAPH SETUP (Dual-Panel Count Comparison)
        self.graph_layout = pg.GraphicsLayoutWidget()
        self.graph_layout.setBackground('#1e1e1e')
        self.graph_layout.setMinimumHeight(290)

        # --- TOP PANEL: Harvest Progress ---
        self.thsr_plot = self.graph_layout.addPlot(title="Harvest Progress: Harvested vs Total in Field")
        self.thsr_plot.showGrid(x=True, y=True, alpha=0.3)
        self.thsr_plot.setLabel('left', 'Berries')
        self.thsr_plot.setLabel('bottom', 'Berries Targeted')
        self.thsr_plot.setYRange(0, 12)
        self.thsr_plot.addLegend(offset=(5, 5))
        self.thsr_ceiling_line = None  # Set dynamically from scan count

        self.thsr_harvest_line = self.thsr_plot.plot(
            pen=pg.mkPen('#42f5aa', width=3), stepMode='right',
            symbol='o', symbolBrush='#42f5aa', symbolSize=7, name="Harvested")

        self.graph_layout.nextRow()

        # --- BOTTOM PANEL: Attempt Efficiency (HE) ---
        self.he_plot = self.graph_layout.addPlot(title="Harvest Efficiency (HE): Harvested vs Total Attempts")
        self.he_plot.showGrid(x=True, y=True, alpha=0.3)
        self.he_plot.setLabel('left', 'Count')
        self.he_plot.setLabel('bottom', 'Berries Targeted')
        self.he_plot.setYRange(0, 12)
        self.he_plot.addLegend(offset=(5, 5))

        self.he_harvest_line = self.he_plot.plot(
            pen=pg.mkPen('#42f5aa', width=3),
            symbol='o', symbolBrush='#42f5aa', symbolSize=7, name="Harvested")
        self.he_attempts_line = self.he_plot.plot(
            pen=pg.mkPen('#f5a142', width=2, style=pg.QtCore.Qt.DashLine),
            symbol='t', symbolBrush='#f5a142', symbolSize=7, name="Total Attempts")

        # Shared data arrays
        self.plot_x = []
        self.plot_harvested = []
        self.plot_attempts = []

        metrics_layout.addWidget(self.graph_layout)


        # BUTTONS: Logging & Export
        so4_layout = QHBoxLayout()
        
        btn_drop = QPushButton("Correction: Berry Dropped")
        btn_drop.setStyleSheet("background-color: #f5a142; color: #121212; font-weight: bold; padding: 5px;")
        btn_drop.clicked.connect(self.log_correction_dropped)
        
        so4_layout.addWidget(btn_drop)
        
        ctrl_layout = QHBoxLayout()
        self.btn_export = QPushButton("Save Data")
        self.btn_export.setStyleSheet("background-color: #333333; color: #ffffff; font-weight: bold; padding: 5px;")
        self.btn_export.clicked.connect(self.export_csv)
        
        self.btn_reset = QPushButton("Reset Data")
        self.btn_reset.setStyleSheet("background-color: #666666; color: #ffffff; font-weight: bold; padding: 5px;")
        self.btn_reset.clicked.connect(self.reset_metrics)
        
        ctrl_layout.addWidget(self.btn_export)
        ctrl_layout.addWidget(self.btn_reset)
        
        metrics_layout.addLayout(so4_layout)
        metrics_layout.addLayout(ctrl_layout)
        
        right_panel.addWidget(metrics_group)

        # NODE CONTROL
        control_group = QGroupBox("Node Control")
        control_group.setStyleSheet("QGroupBox { font-size: 14px; font-weight: bold; border: 1px solid #333; margin-top: 10px; }")
        grid = QGridLayout()
        
        def make_btn(text, color, func):
            b = QPushButton(text)
            b.setStyleSheet(f"background-color: {color}; padding: 5px; font-weight: bold;")
            b.clicked.connect(func)
            return b

        self.btn_hardware = make_btn("Launch Hardware", "#333", self.launch_real_hardware)
        self.btn_stop_hardware = make_btn("Stop Hardware", "#a33", self.stop_real_hardware)
        self.btn_sim = make_btn("Launch Sim", "#333", self.launch_simulation)
        self.btn_stop_sim = make_btn("Stop Sim", "#a33", self.stop_simulation)
        self.btn_vision = make_btn("Start Vision", "#333", self.launch_vision)
        self.btn_stop_vision = make_btn("Stop Vision", "#a33", self.stop_vision)
        self.btn_harvest = make_btn("Start Harvest", "#333", self.launch_harvesting)
        self.btn_stop_harvest = make_btn("Stop Harvest", "#a33", self.stop_harvesting)
        self.btn_kill = make_btn("KILL ALL PROCESSES SAFELY", "#f54242", self.kill_all_processes)

        self.btn_home = make_btn("Home (Scan Position)", "#4287f5", self.manual_home)

        self.btn_record_bag = make_btn("Record Rosbag", "#333", self.record_rosbag)
        self.btn_play_bag = make_btn("Play Rosbag", "#333", self.play_rosbag)
        self.btn_stop_bag = make_btn("Stop Rosbag", "#a33", self.stop_rosbag)

        grid.addWidget(self.btn_hardware, 0, 0)
        grid.addWidget(self.btn_stop_hardware, 0, 1)
        grid.addWidget(self.btn_sim, 0, 2)
        grid.addWidget(self.btn_stop_sim, 0, 3)
        grid.addWidget(self.btn_vision, 1, 0)
        grid.addWidget(self.btn_stop_vision, 1, 1)
        grid.addWidget(self.btn_harvest, 1, 2)
        grid.addWidget(self.btn_stop_harvest, 1, 3)
        grid.addWidget(self.btn_home, 2, 0, 1, 4)
        grid.addWidget(self.btn_record_bag, 3, 0)
        grid.addWidget(self.btn_play_bag, 3, 1)
        grid.addWidget(self.btn_stop_bag, 3, 2, 1, 2)
        grid.addWidget(self.btn_kill, 4, 0, 1, 4)
        
        control_group.setLayout(grid)
        right_panel.addWidget(control_group)

        # DEMO MODE
        demo_group = QGroupBox("Demo Control")
        demo_group.setStyleSheet("QGroupBox { font-size: 14px; font-weight: bold; border: 1px solid #333; margin-top: 10px; }")
        d_layout = QHBoxLayout()
        self.chk_demo = QCheckBox("Enable Demo Mode (Pause after Attempt)")
        self.chk_demo.setStyleSheet("font-size: 14px;")
        self.chk_demo.toggled.connect(self.toggle_demo_mode)
        
        self.btn_proceed = QPushButton("PROCEED")
        self.btn_proceed.setStyleSheet("background-color: #4287f5; color: #fff; font-weight: bold; padding: 10px; font-size: 14px;")
        self.btn_proceed.clicked.connect(self.trigger_proceed)
        self.btn_proceed.setEnabled(False) 
        
        d_layout.addWidget(self.chk_demo)
        d_layout.addWidget(self.btn_proceed)
        demo_group.setLayout(d_layout)
        right_panel.addWidget(demo_group)
        
        layout.addLayout(left_panel, 1)
        layout.addLayout(right_panel, 1)
        
        self.cycles = 0 # Track cycles for CSV logging
        
        self.signals.status_updated.connect(self.on_status_updated)
        self.signals.targets_updated.connect(self.on_targets_updated)
        self.signals.log_message.connect(self.txt_logs.append)
        
        self.last_targets = []
        
        self.tf_timer = QTimer()
        self.tf_timer.timeout.connect(self.update_tf)
        self.tf_timer.start(100)
        
        # Sync all metric labels to their correct names on startup
        self.reset_metrics()

    
    # MATH & CSV EXPORT LOGIC
    
    def update_metrics(self, attempts_used, final_result):
        self.targets_engaged += 1
        self.total_attempts += attempts_used
        if final_result == "SUCCESS":
            self.successful_targets += 1
            
        rate = 0.0
        efficiency = 0.0
        if self.targets_engaged > 0:
            rate = (self.successful_targets / self.targets_engaged) * 100.0
            efficiency = (self.total_attempts / self.targets_engaged)
            
        # Update UI text
        self.lbl_attempts.setText(f"Targets: {self.targets_engaged} | Catches: {self.successful_targets} | Attempts: {self.total_attempts}")
        self.lbl_success_rate.setText(f"Target Success Rate: {rate:.1f}%")
        self.lbl_efficiency.setText(f"Efficiency: {efficiency:.2f} attempts/target")
        
        if rate >= 80.0:
            self.lbl_success_rate.setStyleSheet("color: #42f5aa;") # Green
        else:
            self.lbl_success_rate.setStyleSheet("color: #f5a142;") # Orange
            
        # Update the Live Graph
        self.plot_x.append(self.targets_engaged)
        self.plot_y.append(rate)
        self.data_line.setData(self.plot_x, self.plot_y)
        
        # Add Berry ID label to the point
        b_id = getattr(self, 'current_target_b_id', "?")
        lbl = pg.TextItem(f"B{b_id}", color=(220, 220, 220), anchor=(0.5, 1.5))
        lbl.setPos(self.targets_engaged, rate)
        self.graph_widget.addItem(lbl)
                
        self.txt_logs.append(f"Recorded Result: {final_result} in {attempts_used} attempts.")
        
    def log_classification(self, outcome):
        self.ros_node.cmd_queue.put({"classification": outcome})
        self.txt_logs.append(f"[SYS] Sending GUI Outcome: {outcome}")
        
    def log_correction_dropped(self):
        # Allow the user to manually convert the LAST caught berry into a "Dropped" berry.
        if self.caught_1st > 0 or self.caught_retries > 0:
            assigned = False
            # Deduct from whichever pool got the last point
            if self.csv_data and self.csv_data[-1][4] == "FIRST_TRY_SUCCESS":
                self.caught_1st -= 1
                if self.cycle_caught > 0: self.cycle_caught -= 1
                assigned = True
            elif self.csv_data and self.csv_data[-1][4] == "MULTI_RETRY_SUCCESS":
                self.caught_retries -= 1
                if self.cycle_caught > 0: self.cycle_caught -= 1
                assigned = True
                
            if assigned:
                self.dropped_fruit += 1
                self.cycle_dropped += 1
                
                # Update last CSV entry
                if self.csv_data:
                    self.csv_data[-1][4] = "DROPPED_ON_FLOOR"
                    
                self.txt_logs.append("[USER CORRECTION] Converted last 'Caught' to 'Dropped on Floor'.")
                self.recalculate_objective_metrics()
            else:
                self.txt_logs.append("[WARN] Could not identify last catch to correct.")
        else:
            self.txt_logs.append("[WARN] No successful catches to correct yet.")
            
    def recalculate_objective_metrics(self):
        total_caught = self.caught_1st + self.caught_retries
        
        # Harvest Success Rate: harvested / total_berries_in_field
        # We strictly prioritize the initial scan total to avoid misleading drops
        # if the robot "targets" more berries during rescan.
        field_total = self.total_berries_in_field if self.total_berries_in_field > 0 else self.targets_engaged
        thsr = (total_caught / field_total * 100.0) if field_total > 0 else 0.0
        
        # Harvest Efficiency: harvested / total_attempts
        he = (total_caught / self.total_attempts * 100.0) if self.total_attempts > 0 else 0.0
        act = (self.total_time_spent / self.targets_engaged) if self.targets_engaged > 0 else 0.0
        
        field_str = f"/{self.total_berries_in_field}" if self.total_berries_in_field > 0 else ""
        self.lbl_attempts.setText(f"Harvested: {total_caught}{field_str} | Targeted: {self.targets_engaged} | Attempts: {self.total_attempts} | Drops: {self.dropped_fruit}")
        self.lbl_success_rate.setText(f"Harvest Success Rate (Yield): {thsr:.1f}%")
        self.lbl_success_rate.setStyleSheet("color: #42f5aa;" if thsr >= 80 else "color: #f5a142;")
        self.lbl_efficiency.setText(f"Harvest Efficiency (HE): {he:.1f}%")
        self.lbl_act.setText(f"ACT (Speed): {act:.1f}s")

        # --- UPDATE DUAL-PANEL COUNT GRAPHS ---
        # Update ceiling line if we know the field total
        if self.total_berries_in_field > 0:
            if self.thsr_ceiling_line is None:
                self.thsr_ceiling_line = self.thsr_plot.addItem(
                    pg.InfiniteLine(pos=self.total_berries_in_field, angle=0,
                                    pen=pg.mkPen('#ff4444', width=2, style=pg.QtCore.Qt.DashLine),
                                    label=f'Field Total ({self.total_berries_in_field})', labelOpts={'color': '#ff4444'}))
            max_y = self.total_berries_in_field + 2
            self.thsr_plot.setYRange(0, max_y)
            self.he_plot.setYRange(0, max(max_y, self.total_attempts + 1))

        # Append or update last data point
        if len(self.plot_x) > 0 and self.plot_x[-1] == self.targets_engaged:
            self.plot_harvested[-1] = total_caught
            self.plot_attempts[-1] = self.total_attempts
        else:
            self.plot_x.append(self.targets_engaged)
            self.plot_harvested.append(total_caught)
            self.plot_attempts.append(self.total_attempts)

        self.thsr_harvest_line.setData(self.plot_x, self.plot_harvested)
        self.he_harvest_line.setData(self.plot_x, self.plot_harvested)
        self.he_attempts_line.setData(self.plot_x, self.plot_attempts)
        
    def reset_metrics(self):
        self.targets_engaged = 0
        self.total_attempts = 0
        self.caught_1st = 0
        self.caught_retries = 0
        self.missed_cut = 0
        self.dropped_fruit = 0
        self.total_berries_in_field = 0
        self.thsr_ceiling_line = None
        
        self.cycle_caught = 0
        self.cycle_missed = 0
        self.cycle_dropped = 0
        self.cycles = 0
        self.csv_data.clear()
        
        self.plot_x.clear()
        self.plot_harvested.clear()
        self.plot_attempts.clear()
        
        self.thsr_harvest_line.setData([], [])
        self.he_harvest_line.setData([], [])
        self.he_attempts_line.setData([], [])
        self.thsr_plot.clear()
        self.he_plot.clear()
        # Re-add the base plot lines after clear()
        self.thsr_harvest_line = self.thsr_plot.plot(
            pen=pg.mkPen('#42f5aa', width=3), stepMode='right',
            symbol='o', symbolBrush='#42f5aa', symbolSize=7, name="Harvested")
        self.he_harvest_line = self.he_plot.plot(
            pen=pg.mkPen('#42f5aa', width=3),
            symbol='o', symbolBrush='#42f5aa', symbolSize=7, name="Harvested")
        self.he_attempts_line = self.he_plot.plot(
            pen=pg.mkPen('#f5a142', width=2, style=pg.QtCore.Qt.DashLine),
            symbol='t', symbolBrush='#f5a142', symbolSize=7, name="Total Attempts")
        self.thsr_plot.setYRange(0, 12)
        self.he_plot.setYRange(0, 12)
                
        self.lbl_attempts.setText("Harvested: 0 | Targeted: 0 | Attempts: 0 | Drops: 0")
        self.lbl_success_rate.setText("Harvest Success Rate (Yield): 0.0%")
        self.lbl_efficiency.setText("Harvest Efficiency (HE): 0.0%")
        self.lbl_act.setText("Average Cycle Time (Speed): 0.0s")
        
        self.total_time_spent = 0.0
        
        self.lbl_success_rate.setStyleSheet("color: #f5a142;") 
        self.txt_logs.append("[SYSTEM] Objective Metrics and Graph Data Reset.")
        self.txt_target_logs.clear()
        
    def export_csv(self):
        if len(self.csv_data) == 0:
            self.txt_logs.append("[WARN] No data to export yet.")
            return
            
        path, _ = QFileDialog.getSaveFileName(self, "Save Data", "harvest_data.csv", "CSV Files (*.csv)")
        if path:
            try:
                total_caught = self.caught_1st + self.caught_retries
                field_total = self.total_berries_in_field if self.total_berries_in_field > 0 else self.targets_engaged
                hsr = (total_caught / field_total * 100.0) if field_total > 0 else 0.0
                he = round((total_caught / self.total_attempts * 100.0) if self.total_attempts > 0 else 0.0, 2)
                act  = round((self.total_time_spent / self.targets_engaged) if self.targets_engaged > 0 else 0.0, 2)

                with open(path, 'w', newline='') as f:
                    writer = csv.writer(f)

                    # ── RUN SUMMARY ──────────────────────────────────────────
                    writer.writerow(["=== RUN SUMMARY ==="])
                    writer.writerow(["Metric", "Value"])
                    writer.writerow(["Export Time",         datetime.now().strftime("%Y-%m-%d %H:%M:%S")])
                    writer.writerow(["Berries in Field",    self.total_berries_in_field])
                    writer.writerow(["Berries Targeted",    self.targets_engaged])
                    writer.writerow(["Total Harvested",     total_caught])
                    writer.writerow(["Total Attempts",      self.total_attempts])
                    writer.writerow(["Dropped on Floor",    self.dropped_fruit])
                    writer.writerow(["Harvest Success Rate (HSR %)", hsr])
                    writer.writerow(["Harvest Efficiency (HE %)",    he])
                    writer.writerow(["Avg Cycle Time (ACT s)",        act])
                    writer.writerow([])  # blank separator

                    # ── PER-TARGET DETAIL ─────────────────────────────────────
                    writer.writerow(["=== PER-TARGET LOG ==="])
                    writer.writerow(["Timestamp", "Cycle", "Target_#",
                                     "Attempts_Used", "Result",
                                     "HSR_%", "HE_%", "ACT_s",
                                     "Cumulative_Harvested", "Berries_In_Field"])
                    writer.writerows(self.csv_data)

                # Save target monitor TXT alongside
                target_path = path.replace('.csv', '_targets.txt') if path.endswith('.csv') else path + '_targets.txt'
                with open(target_path, 'w') as f:
                    f.write(self.txt_target_logs.toPlainText())
                    
                self.txt_logs.append(f"[SYSTEM] Exported:\n- {path}\n- {target_path}")
                
                # Save Validation CSV alongside
                if len(self.val_csv_data) > 0:
                    val_path = path.replace('.csv', '_validation.csv') if path.endswith('.csv') else path + '_validation.csv'
                    with open(val_path, 'w', newline='') as vf:
                        v_writer = csv.writer(vf)
                        v_writer.writerow(["Camera Prediction (Xc, Yc, Zc) in mm", "CP tool0 Position (mm)", "Robot Kinematics (Xr, Yr, Zr) in mm", "RK tool0 Position (mm)", "Euclidean Error (mm)"])
                        v_writer.writerows(self.val_csv_data)
                    self.txt_logs.append(f"- {val_path}")

            except Exception as e:
                self.txt_logs.append(f"[ERROR] Failed to save: {e}")

    # ROS 2 LOGIC
    
    def update_tf(self):
        targets_for_map = []
        eff_pos = (0, 0)
        try:
            if self.ros_node.tf_buffer.can_transform('base_link', 'tool0', rclpy.time.Time()):
                trans = self.ros_node.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
                eff_pos = (trans.transform.translation.x, trans.transform.translation.y)
        except Exception: pass

        for t in self.last_targets:
            t_id = t.get('id', 0)
            t_type = t.get('type', 'berry')
            frame_name = f"detected_{t_type}_{t_id}"
            
            try:
                if self.ros_node.tf_buffer.can_transform('base_link', frame_name, rclpy.time.Time()):
                    trans = self.ros_node.tf_buffer.lookup_transform('base_link', frame_name, rclpy.time.Time())
                    if (self.ros_node.get_clock().now().nanoseconds * 1e-9 - (trans.header.stamp.sec + trans.header.stamp.nanosec * 1e-9)) < 1.0:
                        targets_for_map.append({
                            "id": t_id,
                            "type": t_type,
                            "x": trans.transform.translation.x,
                            "y": trans.transform.translation.y,
                            "conf": t.get("conf", 0.0)
                        })
            except Exception: pass
            
        # Map widget updates
        self.last_map_targets = targets_for_map
        self.map_widget.update_data(targets_for_map, eff_pos)
        
    def on_targets_updated(self, targets):
        self.last_targets = targets
        
    def on_status_updated(self, data):
        if "state" in data:
            self.lbl_state.setText(f"State: {data['state']}")
        
        if "berries_detected" in data:
            count = int(data["berries_detected"])
            self.txt_logs.append(f"[SCAN] YOLO sees {count} berries in field.")
            
        if "target_xyz" in data and len(data["target_xyz"]) == 3:
            tx, ty, tz = data["target_xyz"]
            best_target = None
            min_dist = 999.0
            
            for t in getattr(self, 'last_map_targets', []):
                if t.get('type') != 'berry': continue
                dist = math.sqrt((t['x'] - tx)**2 + (t['y'] - ty)**2)
                if dist < min_dist:
                    min_dist = dist
                    best_target = t
                    
            if best_target and min_dist < 0.15: # 15cm geometric tolerance
                b_id = best_target['id']
                b_conf = best_target.get('conf', 0.0)
                self.current_target_b_id = b_id
                
                # Check for nearby stem
                stem_id = None
                stem_conf = 0.0
                min_s_dist = 999.0
                for t in getattr(self, 'last_map_targets', []):
                    if t.get('type') == 'peduncle':
                        s_dist = math.sqrt((t['x'] - tx)**2 + (t['y'] - ty)**2)
                        if s_dist < min_s_dist:
                            min_s_dist = s_dist
                            stem_id = t['id']
                            stem_conf = t.get('conf', 0.0)
                
                out_msg = f"--- COMPUTED TARGET ---\nTarget: Berry {b_id} | Confidence: {b_conf:.2f}\nCoordinates: X={tx:.2f}, Y={ty:.2f}, Z={tz:.2f}"
                if stem_id is not None and min_s_dist < 0.10: 
                    out_msg += f"\nAssociated Stem: Peduncle {stem_id} | Confidence: {stem_conf:.2f}"
                out_msg += "\n-----------------------"
                
                self.txt_target_logs.append(out_msg)

        if "val_data" in data:
            v_data = data["val_data"]
            self.val_csv_data.append([v_data["c_str"], v_data["cp_t0_str"], v_data["r_str"], v_data["rk_t0_str"], v_data["error"]])
            self.txt_logs.append(f"[3D COORDINATES VALIDATION] Coordinates Extracted! Error: {v_data['error']}mm")

        if "event" in data:
            event = data["event"]
            self.txt_logs.append(f"[{self.ros_node.get_clock().now().to_msg().sec}] {event}")
            
            # Cycle Completed event clears cycle counters
            # Trigger manual verification popup on any completion event
            if event in ["Cycle Completed", "Max Cycles Reached", "Cycle Aborted", "Harvest Stopped", "Aborting Cycle"]:
                # Manual Verification Prompt before resetting cycle counters
                actual_count, ok = QInputDialog.getInt(self, "Cycle Verification", 
                                                      f"Cycle {self.cycles + 1} finished.\nAuto-registered harvests: {self.cycle_caught}\nEnter ACTUAL harvest count (items in basket):", 
                                                      self.cycle_caught, 0, 100, 1)
                
                if ok:
                    if actual_count != self.cycle_caught:
                        diff = actual_count - self.cycle_caught
                        self.txt_logs.append(f"[MANUAL VERIFICATION] User corrected harvest count: {self.cycle_caught} -> {actual_count} (Delta: {diff:+})")
                        
                        # Adjust global counters to keep cumulative metrics and graphs accurate
                        # We apply the delta to caught_1st as the primary bucket
                        self.caught_1st += diff
                        
                        # Log the correction to CSV as a special "CORRECTION" row
                        timestamp = datetime.now().strftime("%H:%M:%S")
                        thsr = ( (self.caught_1st + self.caught_retries) / self.total_berries_in_field * 100.0) if self.total_berries_in_field > 0 else 0.0
                        self.csv_data.append([timestamp, self.cycles + 1, "N/A", "N/A", f"MANUAL_CORRECTION_{diff:+}", round(thsr, 2), "N/A", "N/A", (self.caught_1st + self.caught_retries), self.total_berries_in_field])
                        
                        self.recalculate_objective_metrics()
                    else:
                        self.txt_logs.append("[MANUAL VERIFICATION] Count confirmed as accurate by user.")

                self.cycles += 1
                
                self.txt_logs.append(f"[CYCLE {self.cycles}] Final Results - Caught: {actual_count if ok else self.cycle_caught} | Missed Cut: {self.cycle_missed} | Dropped: {self.cycle_dropped}")
                
                # Reset cycle counters for next run
                self.cycle_caught = 0
                self.cycle_missed = 0
                self.cycle_dropped = 0
                
                # Unlink current items so next cycle draws new bars
                self.current_bg_success = None
                self.current_bg_missed = None
                self.current_bg_dropped = None
                self.current_cycle_label = None
                
        # Target Auto-Verification logic (individual berry attempts)
        if "target_result" in data and "attempts" in data:
            result = data["target_result"]
            attempts_used = data["attempts"]
            elapsed = data.get("time_elapsed", 0.0)
            
            self.targets_engaged += 1
            self.total_attempts += attempts_used
            self.total_time_spent += elapsed
            
            if result == "FIRST_TRY_SUCCESS":
                self.caught_1st += 1
                self.cycle_caught += 1
            elif result == "MULTI_RETRY_SUCCESS":
                self.caught_retries += 1
                self.cycle_caught += 1
            elif result == "ABANDONED":
                self.missed_cut += 1
                self.cycle_missed += 1
                
            self.recalculate_objective_metrics()
            
            # Log to CSV immediately per attempt logged
            try:
                thsr = float(self.lbl_success_rate.text().split(': ')[1].replace('%', ''))
                fasr = float(self.lbl_efficiency.text().split(': ')[1].replace('%', ''))
                act = float(self.lbl_act.text().split(': ')[1].replace('s', ''))
            except Exception as e:
                self.txt_logs.append(f"[ERROR] Failed to parse metrics for CSV: {e}")
                thsr, fasr, act = 0.0, 0.0, 0.0
                
            timestamp = datetime.now().strftime("%H:%M:%S")
            total_caught = self.caught_1st + self.caught_retries
            self.csv_data.append([timestamp, self.cycles, self.targets_engaged, attempts_used, result, round(thsr, 2), round(fasr, 2), round(act, 2), total_caught, self.total_berries_in_field])
    
    # PROCESS MANAGEMENT LOGIC

    def _start_process(self, key, cmd, log_msg):
        if self.procs[key] is not None:
            self.txt_logs.append(f"[WARN] Process '{key}' is already running.")
            return
        self.txt_logs.append(log_msg)
        try: self.procs[key] = subprocess.Popen(cmd, preexec_fn=os.setsid)
        except Exception as e: self.txt_logs.append(f"[ERROR] Failed to start {key}: {e}")

    def _stop_process(self, key, log_msg):
        proc = self.procs[key]
        if proc:
            self.txt_logs.append(log_msg)
            try:
                pgid = os.getpgid(proc.pid)
                # Escalate kill signals to prevent zombie ROS2 nodes
                os.killpg(pgid, signal.SIGINT)
                try:
                    proc.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.txt_logs.append(f"[WARN] {key} resisting SIGINT. Escalating to SIGQUIT...")
                    os.killpg(pgid, signal.SIGQUIT)
                    try:
                        proc.wait(timeout=1)
                    except subprocess.TimeoutExpired:
                        self.txt_logs.append(f"[ERROR] {key} hung. Forcing SIGKILL.")
                        os.killpg(pgid, signal.SIGKILL)
                        proc.wait(timeout=1)
            except Exception as e:
                self.txt_logs.append(f"[ERROR] Issue stopping {key}: {e}")
            finally:
                self.procs[key] = None

    def launch_real_hardware(self):
        self._start_process('hw_control', ["ros2", "launch", "ur_robot_driver", "ur_control.launch.py", "ur_type:=ur10", "robot_ip:=192.168.11.100", "launch_rviz:=false"], "Launching Hardware Controller...")
        self._start_process('hw_moveit', ["ros2", "launch", "strawberry_moveit_config", "ur_moveit.launch.py", "ur_type:=ur10", "use_fake_hardware:=false", "launch_rviz:=true"], "Launching Hardware MoveIt/RViz...")
    def stop_real_hardware(self):
        self._stop_process('hw_moveit', "Stopping Hardware MoveIt...")
        self._stop_process('hw_control', "Stopping Hardware Controller...")
    def launch_simulation(self): self._start_process('sim', ["ros2", "launch", "harvest_bot", "simulation.launch.py"], "Launching Simulation...")
    def stop_simulation(self): self._stop_process('sim', "Stopping Simulation cleanly...")
    def launch_vision(self): self._start_process('vision', ["ros2", "run", "harvest_bot", "vision.py"], "Starting Vision...")
    def stop_vision(self): self._stop_process('vision', "Stopping Vision Broadcaster...")
    def launch_harvesting(self):
        # Allow the user to manually define total strawberries in field
        count, ok = QInputDialog.getInt(self, "Metrics Setup", "Enter the exact physical total of berries in the workspace:", 3, 1, 100, 1)
        if ok:
            self.total_berries_in_field = count
            self.txt_logs.append(f"[USER SETUP] Objective Physical Berry Total Set To: {count}")
        else:
            self.txt_logs.append("[USER SETUP] Defaulting Physical Berry Total To: 3")
            self.total_berries_in_field = 3
        
        self.recalculate_objective_metrics()

        # Ensures any previous harvest instance is fully dead before starting a new one
        if self.procs['harvest'] is not None:
            self.txt_logs.append("[WARN] Harvest already tracked - stopping it first...")
            self.stop_harvesting()
        # Extra pkill safety net for orphan harvest nodes from previous sessions
        subprocess.run(["pkill", "-9", "-f", "harvest.py"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        import time; time.sleep(1.5)  # Wait for ROS2 to clean up action server registrations
        self._start_process('harvest',
                            ["ros2", "run", "harvest_bot", "harvest.py"],
                            "Starting Harvesting Logic...")

    def stop_harvesting(self):
        self._stop_process('harvest', "Stopping Harvesting Logic...")
        # Hard sweep in case the process escaped its process group
        subprocess.run(["pkill", "-9", "-f", "harvest.py"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    def toggle_demo_mode(self, checked):
        state = "true" if checked else "false"
        self.txt_logs.append(f"Demo Mode set to: {state}")
        self.btn_proceed.setEnabled(checked)
        self.ros_node.cmd_queue.put({"demo_mode": checked})

    def trigger_proceed(self):
        self.txt_logs.append("Sending PROCEED command to harvest logic...")
        self.ros_node.cmd_queue.put({"proceed": True})

    def kill_all_processes(self):
        self.txt_logs.append("Executing strict shutdown of all nodes...")
        for key in list(self.procs.keys()): self._stop_process(key, f"Stopping tracked process: {key}")
        kill_cmds = [["pkill", "-9", "-f", "ros2"], ["pkill", "-9", "-f", "harvest.py"], ["pkill", "-9", "-f", "vision.py"], ["pkill", "-9", "-f", "rviz2"], ["pkill", "-9", "-f", "ur_robot_driver"], ["pkill", "-9", "-f", "realsense"]]
        for cmd in kill_cmds: subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["sh", "-c", "rm -rf /dev/shm/fastrtps*"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.run(["ros2", "daemon", "stop"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        self.txt_logs.append("All processes terminated and memory cleaned.")

    def manual_home(self):
        self.ros_node.cmd_queue.put({"command": "home"})
        self.txt_logs.append("[USER COMMAND] Requesting Robot Return to Home (Scan Pos)")

    def record_rosbag(self):
        bag_dir = os.path.expanduser(f"~/strawberry_ws/rosbags/session_{datetime.now().strftime('%Y%m%d_%H%M%S')}")
        cmd = [
            "ros2", "bag", "record", "-a", "-o", bag_dir,
            "-x", ".*image_raw.*|.*image_rect_raw.*"
        ]
        self._start_process('rosbag_record', cmd, f"Starting Rosbag Record to: {bag_dir} (Excluding raw images)")

    def play_rosbag(self):
        bag_dir = QFileDialog.getExistingDirectory(self, "Select Rosbag Directory", os.path.expanduser("~/strawberry_ws/rosbags"))
        if bag_dir:
            self._start_process('rosbag_play', ["ros2", "bag", "play", bag_dir], f"Playing Rosbag: {bag_dir}")
        else:
            self.txt_logs.append("Rosbag play cancelled: No directory selected.")

    def stop_rosbag(self):
        if self.procs.get('rosbag_record'):
            self._stop_process('rosbag_record', "Stopping Rosbag Record...")
        if self.procs.get('rosbag_play'):
            self._stop_process('rosbag_play', "Stopping Rosbag Play...")

    def closeEvent(self, event):
        self.kill_all_processes()
        event.accept()

def spin_thread_func(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass

def main():
    rclpy.init()
    app = QApplication(sys.argv)
    signals = GuiSignals()
    node = RosNode(signals)
    gui = HarvestGUI(node, signals)
    gui.show()
    
    spin_thread = threading.Thread(target=spin_thread_func, args=(node,), daemon=True)
    spin_thread.start()
    
    app.aboutToQuit.connect(node.destroy_node)
    app.aboutToQuit.connect(rclpy.shutdown)
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()