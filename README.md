# Pearl-ur5e
Catkin workspace for the PeARL laboratory at University of Massachusetts - Lowell

Managed be Brendan Hertel (brendan_hertel@student.uml.edu)

Labarotory Setup:
We have a Universal Robots UR5e 6 DOF arm attached to the LAN through ethernet. The pendant connected to the UR5e is running Polyscope version 5.2.1.61344. Attached to the UR5e is a Robotiq 2f_85 gripper. To connect ROS to the gripper, run the command

rosrun brendan_ur5e brendan_ur5e.launch ip:=XXX.XXX.XXX.XXX

where XXX.XXX.XXX.XXX is the ip address of the robot. This launch file establishes the connection to the robot, initializes moveit with the robot's model, and brings up the Rviz workspace containing the robot model. For proper operation, use Ubuntu 16.04 and ROS Kinetic--these are the versions needed to use the UR5e.

11/3/2019: Created repository with current state of local catkin_ws.

11/26/2019: pushed new changes to repository. Changes included current state of demonstration recorder node and test nodes associated with demonstration recorder.
