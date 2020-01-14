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

How to use demo recorder:
1. Connect to the UR5e using the launch file as above.
2. Run `rosrun brendan_ur5e demo_record_v3.py` This will start the node, which waits for a command to start recording data from certain topics. The following data is recorded (Note: for all data, n is the number of data points):
   - Time data
     - From the header of /joint_states.
     - Time data is synchronous across all recorded data.
     - Recorded as a 2 x n matrix. The two time values are seconds and nanoseconds, stored as integers.
   - Joint State data
     - From the body of /joint_states.
     - The number of joints of the UR5e robot is 6. For the following bullet points, j is the number of joints.
     - Joint position as taken from the body of the /joint_states message. Recorded as a j x n matrix of floats.
     - Effort values as taken from the body of the /joint_states message. Recorded as a j x n matrix of floats.
   - End Effector Spatial data
     - From the body of /tf.
     - /tf publishes translations (x, y, z) and rotations (x, y, z, w) for a total of 7 values.
     - Recorded as a 7 x n matrix of floats. 
   - Other End Effector data
     - Other end effector data includes the readings of the force  and torque sensor data.
     - (Not yet implemented) Data on the gripper status, as open/close.
     - Force data as taken from the /wrench message, which publishes x, y, z values. Stored as a 3 x n matrix of floats.
     - Torque data as taken from the /wrench message, which publishes x, y, z values. Stored as a 3 x n matrix of floats.
3. To start recording a demo, open a separate terminal window and source properly. Run `rosrun brendan_ur5e start_demo.py 1` The demo recording begins.
4. Perform the demo.
5. To end the demo recording, open a separate terminal window and source properly. Run `rosrun brendan_ur5e end_demo.py 1` This gracefully stops the demo. There may be a delay from the end of the demo and from when the recorder finishes processing the queue of messages that have built up. (The longer the demo, the larger the queue.) This is something I hope to return to in the future to improve, but for now, just wait until the queue is processed. It is recommended that if you do not wish to save the demo, force quit out of the demo recorder terminal.
6. To save the demo, respond to the "Would you like to save this demo? (y/n)" prompt in the terminal. Please only input y to save or n to discard the demo. The saved demo will have the title "recorded demo WWW MMM DD HH:MM:SS YYYY.h5" where WWW is the three letter abbreviation day of the week, MMM is the three letter abbreviation for the month, DD is the date of the month, HH is the hour, MM is the minute, SS is the second, and YYYY is the year of the start of the demo_recorder_v3.py node. The file will be saved in the directory that the demo_recorder_v3.py was run in.
7. The terminal will prompt with "Would you like to start another demo? (y/n)" This functionality has technically been implemented, although not functioning as intended due to the queue sizes. This demo recorder has done its job, and you may forcefully exit the program. To record another demo, please start from Step 2.

The structure of the demo within the .h5 is according to the hdf5 standards. A flowchart of the strucure is as below:
![Demo Recorder structure](https://github.com/brenhertel/Pearl-ur5e/blob/master/pictures/hdf5%20demo%20recorder%20flowchart.png)

The shape of the stored arrays is as follows:
- time_data: {time_secs, time_nsecs} x n
- joint_positions: {shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint} x n
- joint_effort: {shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint} x n
- pos_rot_data: {transformX, transformY, transformZ, rotationX, rotationY, rotationZ, rotationW} x n
- force_data: {x, y, z} x n
- torque_data: {x, y, z} x n

12/4/2019: pushed new changes to repository. Added in new model for ur5e with robotiq arg2f_85 gripper attached (ur5e_gripper_no_macros.urdf.xacro) and launch file to launch urdf files (launch_urdf.launch in pkg brendan_ur5e). To use the new launch file run the command

    rosrun brendan_ur5e launch_urdf.launch

By default it launches the ur5e_gripper_no_macros.urdf.xacro file, however by specifying the parameter model:={path/filename}, where {path/filename} is the filepath from the current directory to the urdf file you wish to launch in rviz, you can launch any urdf file.

12/12/2019: pushed new changes to repository. Updates with changes to URDF of UR5e and Robotiq gripper together.

12/23/2019: pushed new changes to repository. Updates to organization of git repository with forks from other repositories included as submodules.

12/24/2019: pushed new changes to repository. Updates to URDF files while attempting to resolve issues within moveit launches of URDF files. Changes have been made to launch_urdf.launch file defaults for testing purposes.

12/26/2019: pushed new changes to repository. Created demo_xyz_playback.py executable. This executes a trajectory of xyz coordinates from a .h5 file with the structure of the flowchart above (technically only needs the same paths to pos_rot_data and joint_positions). To use the demo playback, follow these steps
1. Connect to the UR5e using the launch file as above.
2. Run `rosrun brendan_ur5e demo_xyz_playback.py` and follow the prompts, making sure that the behavior is as expected before proceeding to the next step. (Note: this node executes trajectories on a real robot, so make sure you are ready to hit the emergency stop when using it.)
3. When prompts are finished, the script ends and may be run again if necessary.  
Notes of demo_xyz_playback:
- Make sure you are in "remote control" mode on the pendant
- If the apparatus changes, change the setup for table and wall within the node
- Wall and table dimensions are not exact, to get better executions these can be exactly measured out--probably down to 5mm tolerance
- Using Ctrl-D to exit the node is slightly more graceful than Ctrl-C
- Something to return to in the future: the script uses the initial joint_position for initial position instead of an xyz coordinate. Revisit to switch over to xyz coordinate to be compliant with xyz trajectory deformations.

1/9/2020: pushed new changes to repository. Created lte.py executable. This executable performs Laplacian trajectory editing in python as outlined by www.itr.ei.tum.de/fileadmin/w00bok/www/CodeExamples/laplacianHardConstraints.m.

1/14/2020: pushed new changes to repository. Returned to demo_xyz_playblack.py script to fix initial state problems. Currently using a workaround where the arm will travel to the initial joint position, then to the initial xyz position before starting the trajectory. Need to implement a system that avoids joint states entirely in the future. Finished lte.py executable. To perform lte from another file, be sure to `import lte` and use the function with syntax `new_traj = perform_lte(traj)` to create a new trajectory. The LASA dataset (https://cs.stanford.edu/people/khansari/download.html) was used to test the executable. The LASA dataset was first converted to the .h5 file format, which can be found here: https://github.com/brenhertel/Pearl-ur5e/blob/master/h5%20files/lasa_dataset.h5. The lasa_dataset.h5 file follows the structure as outlined below:
![lasa dataset structure](https://github.com/brenhertel/Pearl-ur5e/blob/master/pictures/lasa_dataset%20flowchart.png)

Notes:
- For the sake of simplicity, not all 25 shapes and 7 demonstrations are shown in the flowchart, but pos, t, vel, acc, and dt data is stored for every demonstration of every shape.
- Array shape may not be consistent across platforms, arrays may be transposed as to what is described below.
- The shape of the stored arrays is as follows:
  - pos: {x, y} x 1000
  - t: t x 1000
  - vel: {vel_x, vel_y} x 1000
  - acc: {acc_x, acc_y} x 1000
  - dt: dt x 1
