# Environment ---

This is Alfie, a humanoid robot with:
 - two, 6dof arms, each with a 1dof gripper
 - 1dof back that raises and lowers the arms
 - 3dof neck
 - meccanum base
 - stereo cameras
 - respeaker microphone with angle sound source
 - amplified speaker

This solution runs on a 
 - Jetson Orin NX 16gb developer kit
 - Ubuntu 22.04
 - Jetpack 6.2.1 


We are actively working on an nvidia gr00t n1.6 solution to run on alfie to complete a challenge to identify, localize, pick up, then set down a soda can
https://github.com/NVIDIA/Isaac-GR00T

# ROS2 ---
Coding standards, domain knowledge, and preferences that AI should follow.
 - when you make a new msg, add the entry to cmakeLists.txt
 - when you make a new srv, add the entry to cmakeLists.txt
 - when you make a new python node, add the entry to setup.py