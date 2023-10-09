# Robotic Cup Stacker
The project aims to utilise a WidowX 250 Robot Arm 6DOF and UR3 in cohesion to collaboratively work together to pick and place cups for cup stacking. With the aim to assist humans in setting up cup stacking, this project will reduce the time taken and effort to prepare and organise. The project intends to maintain safe application and performance with the use of barriers and signs, collision detection and planning to ensure that the models are safely operated and avoid collision within the system. A functioning e-stop will be implemented through hardware use of an Arduino, simulated via GUI, to cease all operations of the robotic system. The WidowX 250 Robot Arm 6DOF by Trossen Robotics was chosen as the custom robotic arm for the project. This was the chosen arm due to its small size and its 6 DOF configuration. It weighs ~3kg, a reach of 650mm and a working payload of 250g, which is ideal for the task of pick and placing cups. The robots 6 DOF configuration allows for it to reach anything in a 650mm radius, ideal for smaller light applications. This robotic arm also comes with its own built in 2-finger gripper, which is perfect for picking up cups. This arm can also be easily customised due to its software features such as Python/MATLAB-ROS APIs, given ROS packages and Gazebo configuration files for simulations.

## Running the Code
To Launch the MATLAB Simulator run the code from the MATLAB <em>CupStacker.m</em> Class file.

```
test
```