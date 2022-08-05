#! /usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import jerk_profiles
from smoothing_core_v2 import *
import math

# initialize moveit_commander + node
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group", anonymous=True)

# initialize moveit
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
# set trajectory display topic
display_trajectory_publisher = rospy.Publisher("/move_group/display_planned_path", moveit_msgs.msg.DisplayTrajectory, queue_size=1)

# add table and wall to planning scene with delays
rospy.sleep(0.5)
add_table(robot, scene)
rospy.sleep(0.5)
add_wall(robot, scene)

# custom poses go here
throw_back_pose = [0, 0, math.radians(-150), math.radians(-90), math.radians(-90), 0]
throw_forward_pose = [0, math.radians(-40), math.radians(-20), math.radians(-127), math.radians(-90), 0]

# set the arm's goal to a pose and create a trajectory
group.set_joint_value_target(throw_back_pose)
plan1 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory

# smooth trajectory
traj = generate_smooth_trajectory(plan1, jerk_profiles.UDDU)  # different jerk profiles can be used from jerk_profiles.py
print("Total time: " + str(plan1.joint_trajectory.points[-1].time_from_start.to_sec()))
print("MoveIt Final Position:")
print(plan1.joint_trajectory.points[-1].positions)
print("Smoothed Final Position:")
print(traj.joint_trajectory.points[-1].positions)
print("Smoothed Final Velocities:")
print(traj.joint_trajectory.points[-1].velocities)

# send the new trajectory to the /move_group/display_planned_path topic to display in RViz/rqt
publish_trajectory(display_trajectory_publisher, traj, robot)
print "Press 'Enter' to execute plan"
raw_input()
group.execute(traj, wait=True)

rospy.sleep(1)

# send another pose goal
group.set_joint_value_target(throw_forward_pose)
plan2 = group.plan()  # type: moveit_msgs.msg.RobotTrajectory

# smooth trajectory
traj2 = generate_smooth_trajectory(plan2, jerk_profiles.UDDU)  # different jerk profiles can be used from jerk_profiles.py
print("Total time: " + str(plan2.joint_trajectory.points[-1].time_from_start.to_sec()))
print("MoveIt Final Position:")
print(plan2.joint_trajectory.points[-1].positions)
print("Smoothed Final Position:")
print(traj2.joint_trajectory.points[-1].positions)
print("Smoothed Final Velocities:")
print(traj2.joint_trajectory.points[-1].velocities)

# display trajectory
publish_trajectory(display_trajectory_publisher, traj2, robot)
print "Press 'Enter' to execute plan"
raw_input()
group.execute(traj2, wait=True)

rospy.sleep(1)

# gracefully shut down moveit commander
moveit_commander.roscpp_shutdown()

