# Arm_2022_23

### Summary
* This repo contains code for control of the robotic arm on the Saddleback College Robotics Club's rover "Helios".
* The repo consists of backend code to interface with the motor controllers, as well as control code that interfaces with a gamepad controller and simulation.

### Control Modes
* The program has three methods of control: Closed loop, Simulation, Open loop.
* Closed loop is used for the manual control of the arm. In this case being controlled via a standard PS5 gamepad controller.
* Simulation consists of a digital twin of the arm. This being an inverse kinematic model developed in Unity Engine. Given this control mode, the real robotic arm will attempt to move to the position of the arm in the simulation. This simulation was used to aid the development of software systems for the arm, prior to manufacturing.
* Open loop mode is just for educational purposes, regarding the idea of a closed loop system.

### Real Rover and Arm
<img src="repo_images/real-rover.jpg" width="50%"> 

### Simulated Arm
<img src="repo_images/robot-arm-sim.gif">
