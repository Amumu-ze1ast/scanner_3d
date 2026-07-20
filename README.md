# scanner_3d

This repository contains the implementation for the adaptive 3D scanning framework
combining robotic manipulation with dual-rotation turntable reconstruction.

## Paper

Adaptive 3D Scanning Framework Combining Robotic Object Manipulation and Precision
Turntable Reconstruction
Amanuel Tereda, Dr. Sun Yi
North Carolina A&T State University, Department of Mechanical Engineering
MECC 2026

## Repository Structure

scanner_3d
    flexbe_kinovagen3_behaviors
        flexbe_kinovagen3_flexbe_behaviors  - FlexBE behavior files
        flexbe_kinovagen3_flexbe_states     - FlexBE custom state implementations
        LICENSE
        README.md
    flexbe_kinovagen3_flexbe_states
        src                                 - Source files for custom states
        CMakeLists.txt
        package.xml
        setup.py
        LICENSE
        README.md
    scanner_3d_new
        config                              - Configuration files
        launch                              - ROS launch files
        meshes                              - 3D mesh files
        rviz                                - RViz configuration files
        scripts                             - Python scripts for scanning and IK
        sensors                             - Sensor configuration files
        urdf                                - Robot URDF and XACRO files
        world                               - Gazebo world files
        CMakeLists.txt
        export.log
        package.xml

## System Requirements

- Ubuntu 20.04 LTS
- ROS Noetic
- Gazebo 11.11.0
- RTAB-Map 0.20.23
- FlexBE 2.3.3
- TRAC-IK
- Python 3.8+

## Hardware

- Kinova Gen3 7-DOF Manipulator
- Robotiq 2-Finger Gripper
- Intel RealSense D435 RGB-D Camera
- Intel RealSense D410 depth sensor
- NEMA 17 Stepper Motors (dual-motor turntable)
- Arduino Mega 2560 with DRV8825 drivers

## Installation

1. Create a ROS workspace if you do not have one

cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src

2. Clone FlexBE repositories

git clone https://github.com/flexbe/flexbe_behavior_engine.git
git clone https://github.com/flexbe/flexbe_app.git

3. Clone this repository

git clone https://github.com/Amumu-ze1ast/scanner_3d.git

4. Install dependencies

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y

5. Build the workspace

catkin_make
source devel/setup.bash

6. Source the workspace on every new terminal or add to bashrc

source ~/catkin_ws/devel/setup.bash

To permanently add to bashrc:

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

## Usage

1. Launch the Gazebo simulation

roslaunch scanner_3d_new simulation.launch

2. Launch RTAB-Map for robotic subsystem

roslaunch rtabmap_ros rtabmap.launch rgb_topic:/robot1_kinova/camera/color/image_raw

3. Launch RTAB-Map for turntable subsystem

roslaunch rtabmap_ros rtabmap.launch rgb_topic:/robot2_turntable/camera/color/image_raw

4. Launch FlexBE

roslaunch flexbe_app flexbe_full.launch

5. Run the scanning behavior through FlexBE dashboard

## Key Features

- Dual-rotation turntable with independent camera and platform motor control
- Robotic manipulation for autonomous object flipping and bottom surface exposure
- Real-time 3D reconstruction using dual RTAB-Map instances
- HSV-based color filtering for artifact removal
- FlexBE state machine with autonomous error recovery
- Sequential and simultaneous scanning modes
- 95% surface coverage vs 65% conventional turntable-only operation

## Results

Method                  Coverage    Scan Time
Robot Only              70%         10s
Turntable Only          65%         15s
Hybrid with manipulation  95%       25s

## Contact

Amanuel Abrdo Tereda
aatereda@aggies.ncat.edu
Portfolio: https://amumu-portfolio.vercel.app
North Carolina A&T State University

## License

See LICENSE file for details.
