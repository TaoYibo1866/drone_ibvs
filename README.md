# Drone Simulator
## Abstract
+ This is a drone simulator project based on ethz-asl/rotors_simulator.
+ An image based position controller is implemented in drone_ibvs.
+ A drone velocity and gimbal position joystick control interface is implemented.
## Install
__(tested on ROS Kinetic + Gazebo7.0)__
+ install ethz-asl/rotors_simulator https://github.com/ethz-asl/rotors_simulator (installation guide: https://github.com/gsilano/BebopS)
+ sudo apt-get install ros-kinetic-joint-state-controller  
+ sudo apt-get install ros-kinetic-joint-state-publisher
## Usage
+ roslaunch drone_gazebo mav_with_gimbal_camera_joy_control.launch
+ roslaunch drone_gazebo mav_with_front_camera_traversing_gate_joy_control.launch
+ roslaunch drone_ibvs mav_ibvs_mission.launch
