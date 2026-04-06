# Autonomous Strawberry Harvester

## Project Overview
The Autonomous Strawberry Harvester is a cutting-edge robotic system designed to harvest strawberries autonomously. Leveraging the capabilities of the Robot Operating System (ROS) 2, this project incorporates advanced technologies including computer vision for accurate fruit detection, inverse kinematics for precise end-effector positioning, hand-eye calibration for enhanced operational accuracy, and motion planning to navigate the harvesting environment efficiently.

## Features
- **Autonomous Navigation**: The robot can navigate through fields using sensors and cameras.
- **Computer Vision**: Employs image processing algorithms for identifying ripe strawberries.
- **Inverse Kinematics**: Calculates the movements required for the robot's arm to reach and grab the fruit.
- **Hand-Eye Calibration**: Ensures the robot's visual perception is accurately aligned with its physical actions.
- **Motion Planning**: Efficiently plans paths to avoid obstacles and optimize harvesting routes.

## Architecture
The architectural design of the system is modular, consisting of various ROS 2 nodes communicating through topics and services. Major components include:
- **Sensor Node**: Processes data from cameras and sensors.
- **Vision Node**: Implements computer vision techniques to detect and classify strawberries.
- **Kinematics Node**: Handles all calculations related to arm movements.
- **Motion Planning Node**: Plans the trajectory for harvesting actions.

## Installation Instructions
1. Clone the repository:
   ```bash
   git clone https://github.com/f32lex/Autonomous-Strawberry-Harvester.git
   cd Autonomous-Strawberry-Harvester
   ```
2. Ensure you have ROS 2 installed on your system. Follow the [ROS 2 installation guidelines](https://docs.ros.org/en/foxy/Installation.html).
3. Install required dependencies:
   ```bash
   sudo apt-get install -y <dependencies>
   ```
4. Build the package:
   ```bash
   colcon build
   ```

## Usage Guide
To launch the robot:
```bash
ros2 launch autonomous_strawberry_harvester main.launch.py
```

## System Requirements
- ROS 2 (Foxy or newer)
- Ubuntu 20.04 or newer
- Compatible sensors (e.g., RGB camera, LiDAR)
- NVIDIA GPU recommended for computer vision processing.

## Package Descriptions
- **autonomous_strawberry_harvester**: The main package containing all functionalities.
- **vision_processing**: Sub-package dedicated to vision algorithms.
- **motion_control**: Manages the robotic arm's movements.

## Configuration Notes
Configure parameters in the `config` directory to tune system performance based on specific operational conditions, including:
- Camera settings
- Arm calibration values
- Motion planning parameters

For detailed parameter descriptions, refer to the respective YAML files in the `config` folder.