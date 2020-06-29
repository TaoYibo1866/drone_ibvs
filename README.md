# Drone Simulator
## Abstract
+ This is a drone simulator project based on ethz-asl/rotors_simulator.  
+ Add gimbal camera and drone velocity controller.  
+ An image based position controller is implemented in TaoYibo1866/drone_visual_servo.
+ A drone velocity and gimbal position control joystick-interface is implemented.
## Install
__(tested on ROS Kinetic + Gazebo7.0)__
+ install ethz-asl/rotors_simulator https://github.com/ethz-asl/rotors_simulator (installation guide: https://github.com/gsilano/BebopS)
## Usage
+ roslaunch drone_gazebo mav_with_gimbal_camera_joy_control.launch
+ roslaunch drone_visual_servo mav_ibvs_mission.launch
