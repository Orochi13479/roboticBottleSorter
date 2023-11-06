# Robotic Cup Stacker
The project aims to utilise a WidowX 250 Robot Arm 6DOF and UR3 in cohesion to collaboratively work together to pick and place cups for cup stacking. With the aim to assist humans in setting up cup stacking, this project will reduce the time taken and effort to prepare and organise. The project intends to maintain safe application and performance with the use of safety equipment such as barriers, warning signs, fire extinguishers and sirens. Collision detection and path planning will be implemented to ensure that the robot models are to be safely operated and avoid collision within the system. In addition, to incoporate further safety functionality, the robotic arm will react to an asynchronous stop signal by a user, when the robot system is sensing an object or someone will be impleneted. This is in the event of an external object or someone enters within the reach or limits of the operating robotic arms. 

The system will encorporate a functioning e-stop that will be implemented through hardware use of an Arduino, and simulated via GUI, to cease all operations of the robotic system. The functioning e-stop button will have two actions, in which the system will completely stop the operations once pressed, and can resume after an e-stop event is engaged by the user. 

The MATLAB graphical user interface includes an advanced developed "teach" function that will allow for jogging and manipulating the WidowX 250 robot arm. This encompasses the individual joints movements of the arm as well any cartesian movements [x,y,z]

The WidowX 250 Robot Arm 6DOF by Trossen Robotics was chosen as the custom robotic arm for the project. This was the chosen arm due to its small size and its 6 DOF configuration. It weighs ~3kg, a reach of 650mm and a working payload of 250g, which is ideal for the task of pick and placing cups. The robots 6 DOF configuration allows for it to reach anything in a 650mm radius, ideal for smaller light applications. This robotic arm also comes with its own built in 2-finger gripper, which is perfect for picking up cups. This arm can also be easily customised due to its software features such as Python/MATLAB-ROS APIs, given ROS packages and Gazebo configuration files for simulations.

## Running the Code
To Launch the MATLAB Simulator run the code from the MATLAB <em>CupStacker.m</em> Class file.
There are also supplementary files for demonstration of particular features such as collision detection and light curtains, these will all have the suffix of Demo.m.
Also located in realRobot.m is an pseudo implementation of the simualtion on the real robot, specifically the UR3e.

### IMPORTANT ###
Inside the robotics toolbox please delete/comment out line 117-118 in the animate.m file within the rvctools/robot/@Seriallink/animate.m Directory.
```
if q(L) > 0
    set(h.pjoint(L), 'Matrix', diag([1 1 q(L) 1]));
else
    % if length is zero the matrix is singular and MATLAB complains
    %error('Prismatic length must be > 0');
end
```
Change to:
```
if q(L) > 0
    % set(h.pjoint(L), 'Matrix', diag([1 1 q(L) 1]));
else
    % if length is zero the matrix is singular and MATLAB complains
    %error('Prismatic length must be > 0');
end
```