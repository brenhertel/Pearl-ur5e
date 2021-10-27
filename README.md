# Pearl-ur5e
Catkin workspace for the PeARL laboratory at University of Massachusetts - Lowell

Managed by Brendan Hertel (brendan_hertel@student.uml.edu)

Labarotory Setup:
We have a Universal Robots UR5e 6 DOF arm attached to the LAN through ethernet. Lab Computer is running 32-bit Ubuntu 16.04 LTS with ROS kinetic. The pendant connected to the UR5e is running Polyscope version 5.2.1.61344. Attached to the UR5e is a Robotiq 2f_85 gripper. To connect ROS to the ur5e, run the command

    roslaunch brendan_ur5e brendan_ur5e.launch ip:=XXX.XXX.XXX.XXX

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

The structure of the demo within the .h5 is according to the hdf5 standards. A flowchart of the structure is as below:
![Demo Recorder structure](https://github.com/brenhertel/Pearl-ur5e/blob/master/brendan_ur5e/pictures/hdf5%20demo%20recorder%20flowchart.png)

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

1/14/2020: pushed new changes to repository. Returned to demo_xyz_playblack.py script to fix initial state problems. Currently using a workaround where the arm will travel to the initial joint position, then to the initial xyz position before starting the trajectory. Need to implement a system that avoids joint states entirely in the future. Finished lte.py executable. To perform lte from another file, be sure to `import lte` and use the function with syntax `new_traj = lte.perform_lte(traj)` to create a new trajectory. The LASA dataset (https://cs.stanford.edu/people/khansari/download.html) was used to test the executable. The LASA dataset was first converted to the .h5 file format, which can be found here: https://github.com/brenhertel/Pearl-ur5e/blob/master/h5%20files/lasa_dataset.h5. The lasa_dataset.h5 file follows the structure as outlined below:\
![lasa dataset structure](https://github.com/brenhertel/Pearl-ur5e/blob/master/brendan_ur5e/pictures/lasa_dataset%20flowchart.png)

Notes:
- For the sake of simplicity, not all 25 shapes and 7 demonstrations are shown in the flowchart, but pos, t, vel, acc, and dt data is stored for every demonstration of every shape.
- Array shape may not be consistent across platforms, arrays may be transposed as to what is described below.
- The shape of the stored arrays is as follows:
  - pos: {x, y} x 1000
  - t: t x 1000
  - vel: {vel_x, vel_y} x 1000
  - acc: {acc_x, acc_y} x 1000
  - dt: dt x 1

1/17/2020: pushed new changes to repository. Created ja.py executable. This implements Jerk-Accuracy trajectory deformations as found in: https://www.mathworks.com/matlabcentral/fileexchange/58403-kinematic-filtering-for-human-and-robot-trajectories. To perform ja from another file, use `import lte` and `new_traj = ja.perform_ja(traj)`. Same testing was used as in lte. Note: default lambda value is 1/20 of the number of elements in the given trajectory. Installed pydmps as forked from https://github.com/studywolf/pydmps. Created dmp.py file to interface with pydmps. To create Dynamic Movement Primitive deformation, use `import dmp` and call `new_traj = dmp.perform_dmp(traj)`. Note: currently, this implementation has the problem of if the start is close to the end the algorithm has trouble moving away from the start and then returning to goal. (See pictures folder for details). Note that in the saved pictures, dark blue is the original trajectory, green is lte, red is ja, and light blue is dmp. Created perform_all_deformations function in the perform_deformations.py file. This will perform all three algorithms given a 1D trajectory. Use syntax `[lte_traj, ja_traj, dmp_traj] = perform_all_deformations(traj, initial, end, (optional)lambda)`. If no lambda is specified ja will use default lambda value as described above. Created preprocessing.py file which implements preprocessing for data. For 1D data use `[resample_data, start, end] = preprocess_1d(traj, num_points, (optional)start, (optional)end)`, and for N-D data use `[resample_data, actual_start, actual_end] = preprocess_nd(data, num_points, (optional)start, (optional)end)`. Note that the N-D function simple calls the 1D function N times, and they therefore have the same functionality. Preprocessing does two things: shaves off any data at the start or end of the given data that is unused, and resamples the data num_points times from a time-independent spline representation of the data.

1/18/2020: pushed new changes to repository. Created display_h5_data.py executable which displays the arrays in tf_info on plots (see demo recorder above for explanation of tf_info). Attempted to record, preprocess, and playback a demonstration on the robot. Ran into issues executing the demontration with the demo_xyz_playback.py script. Will work towards resolving issues.

1/23/2020: pushed new changes to repository. Issues with demo_xyz_playback.py script stem from preprocessing script--unsure of why, will continue testing. Used deform_lasa.py script to deform all trajectories in the LASA dataset, the results of which can be found in the pictures folder.

2/7/2020: pushed new changes to repository. Added playback_lte.py script. This is similar to the demo_xyz_playback.py script but uses the lte code to deform the demo according to Laplacian Trajectory Editing. In the future functionality to add fixed points to the trajectory will be added, as of right now the trajectory deforms to the same start and end positions. Warning: Using LTE only works on the xyz coordinates of a trajectory. Running orientation values through LTE leads to discontinuities, which will cause the full playback to fail. Current functionality just performs LTE on xyz position coordinates. Additionally, do not use splining to preprocess orientation data, as the same problems will happen in playback. Because of this, the number of xyz coordinate points must be the same as the number of orientation coordinate points, which leads to demos with a large number of waypoints (I was testing with a demo that got trimmed down to ~3000 waypoints after bad_preprocessing). I will add in a dmp_playback.py and ja_playback.py scripts later, that have the same idea of deforming the demo with different algorithms.

2/7/2020: pushed new changes to repository. Recorded demo for 3D testing of similarity mapping. Added plot_3d.py to plot 3D trajectories. Small changes to demo_record_v3.py for easier use.

7/17/2020: pushed new changes to repository. Added different implementation of DMP, which correspond's to the method proposed by Pastor et al in their 2009 paper.  Added playback_ja.py and playback_dmp.py scripts, which are essentially copies of playback_lte.py except they use the different LfD representations. Currently you cannot call these scripts with rorsun--I do not know why yet. For now, if they need to be called, call them with the python command (i.e. python /path/to/playback_ja.py). Current functionality just performs DMP and JA on xyz position coordinates. Additionally, do not use splining to preprocess orientation data, as the same problems will happen in playback. Because of this, the number of xyz coordinate points must be the same as the number of orientation coordinate points, which leads to demos with a large number of waypoints (I was testing with a demo that got trimmed down to ~3000 waypoints after bad_preprocessing). Also added playback_with_repro.py script. This script takes data from 2 .h5 files. One has the original demo data, and the other has reproduction data. This script will move the robot with xyz data from the reproduction file, but orientation data from the original demo file. Please ensure the xyz data contains the same number of points as the demo data, otherwise the script will not work.

7/17/2020: pushed new changes to repository. Added script playback_with_repro_with_slerp.py. This script is almost identical to playback_with_repro.py, but uses scipy's implementation of slerp (https://docs.scipy.org/doc/scipy/reference/generated/scipy.spatial.transform.Slerp.html) to interpolate quaternion data from the demonstration file. Recorded demonstrations for pushing using the UR5e.

10/2/2020: pushed new changes to repository. Have added the ability to send and recieve data over internet through socket connections using 2 scripts: publish_gripper_position.py and listen_gripper_position.py. It should be noted that the UR5e seems to need to be in "Remote Control" mode for the socket connections to work. publish_gripper_position publishes the current gripper position over the topic /gripper_data/position (using Int32), and listen_gripper_position.py listens to the topic /gripper_sends/position (also using Int32), and sends values (in the range [0, 100]) to the gripper, which it will then move to. It should be noted that future work could include setting the speed and force of the gripper with similar scripts, but since these cannot be published back so I did not see a point.Thank you to https://dof.robotiq.com/discussion/1649/control-gripper-via-ur-controller-client-interface, which is where I found the code to make this possible. For anyone implementing this on their own, the IP addresses should be changed for it to work. I will create a launch file to launch both of these nodes at some point, and include that launch file in one of my own so gripper control launches with robot control. Additionally need to start recording gripper position in the demo recorder.

10/23/2020: pushed new changes to repository. Have been struggling with finding the best method to properly integrate gripper control. The problem arises is that you cannot listen to gripper status and publish new commands to the gripper at the same time (only the most recent socket connection works). In order to work around this problem, I have decided to use an action server which takes commands, and then starts either gripper listening or control based on those commands. Using a service blocked the code from continuing, but if it had worked would've been much simpler. The action server is unfinished as of right now (gripper_srv_handler.py) but I will continue working on it.

10/23/2020: pushed new changes to repository. Created file demo_js_playback.py which plays back the joint states from a demonstration. Also created playback_downsampled_repro_with_slerp.py which uses slerp to playback a reproduction of less points than the original demonstration on the robot.

10/7/2021: pushed new changes to repository. Finished creating file demo_record_v4.py, which is an improved version of the demo recorder, and should not have as many issues. This version listens to all data seperately (/joint_state, /tf, /wrench, and /gripper - a topic I've created which publishes the position of the gripper, more details in a future update), records them as text files, and then once the demonstration is finished combines these text files into a .h5 file. The structure of this file is as listed below. Note that while all time arrays are stored seperately, they are all identical.

![Demo Recorder structure v4](https://github.com/brenhertel/Pearl-ur5e/blob/master/brendan_ur5e/pictures/demo%20recorder%20v4%20flowchart.drawio.png)

The shape of the stored arrays is as follows:
- joint_time: {time_secs, time_nsecs} x n
- joint_positions: {shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint} x n
- joint_velocities: {shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint} x n
- joint_effort: {shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint} x n
- transform_time: {time_secs, time_nsecs} x n
- transform_positions: {transformX, transformY, transformZ} x n
- transform_orientations: {rotationX, rotationY, rotationZ, rotationW} x n
- wrench_time: {time_secs, time_nsecs} x n
- wrench_force: {x, y, z} x n
- wrench_torque: {x, y, z} x n
- gripper_time: {time_secs, time_nsecs} x n
- gripper_position: {x} x n


10/27/2021: pushed new changes to repository. Mostly focused on finalizing demo recorder and incorporating gripper status into demo recorder. Currently, there are a few issues. One issue is that the gripper socket connection can only work when the robot is in remote control mode, but the robot should not be used by a human in remote control mode. To get around this, I put the robot in remote control mode, then when the gripper script is executed on the robot, I set the robot to freedrive mode. This has some obvious safety concerns, so please be careful when connected to the robot. I end freedrive mode when the socket connection is shut down by connecting with telnet (telnet must be installed, please see https://www.journaldev.com/28614/telnet-command-linux-unix if you are confused) and executing the `end_freedrive_mode()` command. For details on the freedrive mode mechanism, see https://forum.universal-robots.com/t/setting-digital-input-to-freedrive-through-java/2113/9. This change was made to the publish_gripper_position.py node. For the listen node, no human should have to drive the robot and freedrive mode is not necessary. Additionally, I tested the new demo recorder, and everything seems to be working, with results much quicker and easier than before. This version is faster, records more data, and is more robust than v3, and should be used going forward.
