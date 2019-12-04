# Pearl-ur5e
Catkin workspace for the PeARL laboratory at University of Massachusetts - Lowell

Managed be Brendan Hertel (brendan_hertel@student.uml.edu)

Labarotory Setup:
We have a Universal Robots UR5e 6 DOF arm attached to the LAN through ethernet. The pendant connected to the UR5e is running Polyscope version 5.2.1.61344. Attached to the UR5e is a Robotiq 2f_85 gripper. To connect ROS to the ur5e, run the command

rosrun brendan_ur5e brendan_ur5e.launch ip:=XXX.XXX.XXX.XXX

where XXX.XXX.XXX.XXX is the ip address of the robot. This launch file establishes the connection to the robot, initializes moveit with the robot's model, and brings up the Rviz workspace containing the robot model. For proper operation, use Ubuntu 16.04 and ROS Kinetic--these are the versions needed to use the UR5e.

11/3/2019: Created repository with current state of local catkin_ws.

11/26/2019: pushed new changes to repository. Changes included current state of demonstration recorder node and test nodes associated with demonstration recorder.

12/3/2019: pushed new changes to repository. Changes included current state of demonstration recorder node and attempts to update/fix urdf files associated with gripper. Currently running into issues establishing gripper model connection.

12/4/2019: pushed new changes to repository. Added in new model for ur5e with robotiq arg2f_85 gripper attached (ur5e_gripper_no_macros.urdf.xacro) and launch file to launch urdf files (launch_urdf.launch in pkg brendan_ur5e). To use the new launch file run the command

rosrun brendan_ur5e launch_urdf.launch

By default it launches the ur5e_gripper_no_macros.urdf.xacro file, however by specifying the parameter model:={filename}, where {filename} is the name of the urdf file you wish to launch in rviz, you can launch any urdf file.
