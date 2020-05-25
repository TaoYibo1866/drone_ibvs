# Drone Simulator
## Abstract
+ This is a drone and gimbal camera simulation project based on ethz-asl/rotors_simulator.  
+ Add gimbal dynamic and drone velocity controller to the origin simulator.  
+ An image based position controller is implemented in TaoYibo1866/drone_visual_servo.
+ A drone velocity and gimbal position control joystick-interface is implemented.
## Install
__(tested on ROS Kinetic + Gazebo7.0)__
+ install ethz-asl/rotors_simulator https://github.com/ethz-asl/rotors_simulator (installation guide: https://github.com/gsilano/BebopS)
+ install dependencies  
sudo apt-get install ros-kinetic-joint-state-controller  
sudo apt-get install ros-kinetic-joint-state-publisher
## Usage
+ roslaunch drone_gazebo mav_and_gimbal_joy_control.launch
+ roslaunch drone_visual_servo mav_ibvs_joy_console.launch
