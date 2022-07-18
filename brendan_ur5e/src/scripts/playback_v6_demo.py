#!/usr/bin/env python

#Used https://github.com/ros-planning/moveit_tutorials/blob/master/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py as template. See what they do for better explanations. There are parts here--such as setup--which are almost exactly as in that document.

#necessary imports
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import h5py
import roslib
import numpy as np
import preprocessing
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Int32
import lte
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

#not actually sure why this is here or why its necessary, but I suppose its a good way to test tolerances
def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True

#How the robot is understood and controlled
class MoveGroupPythonInterface(object):
  def __init__(self):
    super(MoveGroupPythonInterface, self).__init__()
    #the moveit_commander is what is responsible for sending info the moveit controllers
    moveit_commander.roscpp_initialize(sys.argv)
    #initialize node
    rospy.init_node('joint_playback', anonymous=True)
    #Instantiate a `RobotCommander`_ object. Provides information such as the robot's kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()
    #Instantiate a `PlanningSceneInterface`_ object. This provides a remote interface for getting, setting, and updating the robot's internal understanding of the surrounding world:
    scene = moveit_commander.PlanningSceneInterface()
    #Instantiate a `MoveGroupCommander`_ object.  This object is an interface to a planning group (group of joints), which in our moveit setup is named 'manipulator'
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #Create a `DisplayTrajectory`_ ROS publisher which is used to display trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
 
    #Get all the info which is carried with the interface object
    #We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    #print "Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    #print  End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    #print  Available Planning Groups:", robot.get_group_names()

    # Misc variables
    self.box_name1 = ''
    self.box_name2 = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def goto_joint_state(self, js_array):
    # To start the playback of the demo, go to the initial demo position, which can be interpreted as the 0th set of joint states
    # I use joint states instead of cartesians because cartesians will fail if the current state is too far away from the goal state, whereas joint states will simply execute
    joint_goal = self.move_group.get_current_joint_values()
    joint_goal[0] = js_array[0]
    joint_goal[1] = js_array[1]
    joint_goal[2] = js_array[2]
    joint_goal[3] = js_array[3]
    joint_goal[4] = js_array[4]
    joint_goal[5] = js_array[5]
    # go to the initial position
    # The go command can be called with joint values, poses, or without any parameters if you have already set the pose or joint target for the group
    self.move_group.go(joint_goal, wait=True)
    # Calling ``stop()`` ensures that there is no residual movement
    #self.move_group.stop()


  def wait_for_state_update(self, box_name, box_is_known=False, box_is_attached=False, timeout=4):
    #either this times out and returns false or the object is found within the planning scene and returns true
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      attached_objects = self.scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0
      is_known = box_name in self.scene.get_known_object_names()
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    return False


  def add_table(self, timeout=4):
    #define a box for the table below the robot
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    #box origin (default = {0, 0, 0, 0, 0, 0, 0})
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = -0.1
    self.box_name1 = "table"
    #add box to planning scene and specify dimensions
    self.scene.add_box(self.box_name1, box_pose, size=(10, 10, 0.1))
    #wait for the box to be added in or to timeout
    return self.wait_for_state_update(self.box_name1, box_is_known=True, timeout=timeout)

  def add_wall(self, timeout=4):
    #Same as above with different dimensions
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = self.robot.get_planning_frame()
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.y = -0.15 # next to the robot
    self.box_name2 = "wall"
    self.scene.add_box(self.box_name2, box_pose, size=(10, 0.02, 10))
    return self.wait_for_state_update(self.box_name2, box_is_known=True, timeout=timeout)
 
  def remove_workspace(self, timeout=4):
    #remove each object from the planning scene, waiting for scene to update before moving on
    self.scene.remove_world_object(self.box_name1)
    self.wait_for_state_update(self.box_name1, box_is_attached=False, box_is_known=False, timeout=timeout)
    self.scene.remove_world_object(self.box_name2)
    return self.wait_for_state_update(self.box_name2, box_is_attached=False, box_is_known=False, timeout=timeout) 

  def playback_joints(self, fname):
    hf = h5py.File(fname, 'r')
    
    js = hf.get('joint_state_info')
    joint_time = np.array(js.get('joint_time'))
    joint_pos = np.array(js.get('joint_positions'))
    joint_vel = np.array(js.get('joint_velocities'))
    joint_eff = np.array(js.get('joint_effort'))
    
    gr = hf.get('gripper_info')
    gr_time = np.array(gr.get('gripper_time'))
    gr_pos = np.array(gr.get('gripper_position'))
    
    print(np.shape(joint_pos))
    print(np.shape(gr_pos))
    
    #pub = rospy.Publisher('/gripper_sends/position', Int32)
    
    for i in range(0, len(joint_pos), 40):
      self.goto_joint_state(joint_pos[i])
      #pub.publish(gr_pos[i])
    

def main():
  fname = 'recorded_demo 2022-06-03 15:10:10.h5'
  try:
    print "Playing back a demo (joint/gripper states)"
    print "Press Ctrl-D to exit at any time"
    print "Press 'Enter' to begin"
    raw_input()
    ur5e_arm = MoveGroupPythonInterface()
    #table and wall have to be added in separately--for some reason adding them together didn't work
    print "Press 'Enter' to add in table and wall"
    raw_input()
    ur5e_arm.add_table()
    ur5e_arm.add_wall()
    print "Press 'Enter' to begin playback"
    raw_input()
    ur5e_arm.playback_joints(fname)
    print "Execution complete"
    ur5e_arm.remove_workspace()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
