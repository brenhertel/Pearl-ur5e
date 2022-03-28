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
import roslib
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Int32
from geometry_msgs.msg import PointStamped
from scipy.interpolate import UnivariateSpline
from tf.msg import tfMessage

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

  def plan_traj(self, traj):
    (pts, dims) = np.shape(traj)
    waypoints = []
    wpose = self.move_group.get_current_pose().pose
    for i in range(pts):
      wpose.position.x = traj[i, 0]
      wpose.position.y = traj[i, 1]
      wpose.position.z = traj[i, 2]
      waypoints.append(copy.deepcopy(wpose))
    # We want the Cartesian path to be interpolated at a resolution of 1 mm which is why we will specify 0.001 as the eef_step in Cartesian translation. We will disable the jump threshold by setting it to 0.0, ignoring the check for infeasible jumps in joint space.
    (plan, fraction) = self.move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.001,       # eef_step
                                       0.0)       # jump_threshold
    # Note: We are just planning, not asking move_group to actually move the robot yet:
    print('Planning for %f %% of waypoints achieved' % (fraction * 100.0))
    return plan, fraction
    
  def display_trajectory(self, plan):
    #ask rviz to display the trajectory
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = self.robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    self.display_trajectory_publisher.publish(display_trajectory);

  def execute_plan(self, plan):
    #execute given plan
    self.move_group.execute(plan, wait=True)

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
    box_pose.pose.position.z = -0.07
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

class GripperControl(object):
    
    def __init__(self):
        self.pub = rospy.Publisher('/gripper_sends/position', Int32, queue_size=100)
        
    def close_gripper(self):
        self.pub.publish(99)
        rospy.sleep(5.0)
        
    def open_gripper(self):
        self.pub.publish(1)
        rospy.sleep(5.0)
        
    def gripper_goto(self, pos):
        self.pub.publish(pos)
        rospy.sleep(5.0)

def get_cup_pos():
    #return [0.5, -0.2, 0.069]
    try:
        msg = rospy.wait_for_message('/object/position_local', PointStamped, timeout=5)
    except rospy.ROSException:
        rospy.loginfo('Timeout Exceeded, no message in /object/position_local')
        return get_cup_pos()
    except rospy.ROSInterruptException:
        return None
    except KeyboardInterrupt:
        return None
    return [msg.point.x, msg.point.y, 0.069]

def get_pos():
    try:
        tfmsg = rospy.wait_for_message('/tf', tfMessage, timeout=5)
    except rospy.ROSException:
        rospy.loginfo('Timeout Exceeded, no message in /tf')
    if tfmsg.transforms[0].child_frame_id == 'tool0_controller':
        return [-tfmsg.transforms[0].transform.translation.x, -tfmsg.transforms[0].transform.translation.y, tfmsg.transforms[0].transform.translation.z]
    else:
        return get_pos()

def smooth_resample_traj(traj, n=200):
    (pts, dims) = np.shape(traj)
    in_time = np.linspace(0, 1, pts)
    out_time = np.linspace(0, 1, n)
    
    out_traj = np.zeros((n, dims))
    
    for d in range(dims):
        spl = UnivariateSpline(in_time, traj[:, d], k=5)
        spl.set_smoothing_factor(0.001)
        new_vals = spl(out_time)
        out_traj[:, d] = new_vals
    return out_traj

def generate_traj(end_pos):
    start_pos = get_pos()
    
    print('trajectory start: ' + str(start_pos) + ', end: ' + str(end_pos))
    
    mid_height = 0.13
    
    leg1z = np.arange(start_pos[2], mid_height, 0.001) if mid_height > start_pos[2] else np.arange(start_pos[2], mid_height, -0.001)
    leg1z = np.reshape(leg1z, (len(leg1z), 1))
    leg1x = start_pos[0] * np.ones(np.shape(leg1z))
    leg1y = start_pos[1] * np.ones(np.shape(leg1z))
    
    leg1 = np.hstack((leg1x, leg1y, leg1z))
    
    leg2x = np.linspace(start_pos[0], end_pos[0], 2*len(leg1z))
    leg2x = np.reshape(leg2x, (len(leg2x), 1))
    leg2y = np.linspace(start_pos[1], end_pos[1], 2*len(leg1z))
    leg2y = np.reshape(leg2y, (len(leg2y), 1))
    leg2z = leg1z[-1] * np.ones(np.shape(leg2y))
    
    leg2 = np.hstack((leg2x, leg2y, leg2z))
    
    leg3z = np.arange(leg2z[-1], end_pos[2], 0.001) if end_pos[2] > leg2z[-1] else np.arange(leg2z[-1], end_pos[2], -0.001)
    leg3z = np.reshape(leg3z, (len(leg3z), 1))
    leg3x = end_pos[0] * np.ones(np.shape(leg3z))
    leg3y = end_pos[1] * np.ones(np.shape(leg3z))
    
    leg3 = np.hstack((leg3x, leg3y, leg3z))
    
    traj = np.vstack((leg1, leg2, leg3))
    traj = smooth_resample_traj(traj)
    
    #plt.figure()
    #plt.plot(traj[:, 0])
    #plt.title('x')
    #plt.figure()
    #plt.plot(traj[:, 1])
    #plt.title('y')
    #plt.figure()
    #plt.plot(traj[:, 2])
    #plt.title('z')
    #plt.show()
    
    return traj
   
def go_to_position(arm, end_pos):
    print('Going to position ' + str(end_pos))
    traj = generate_traj(end_pos)
    cartesian_plan, fraction = arm.plan_traj(traj)
    arm.display_trajectory(cartesian_plan)
    print "Press 'Enter' to execute planned trajectory"
    raw_input()
    arm.execute_plan(cartesian_plan)
    return

def go_to_cup(arm):
    goal_pos = [-0.5027, 0.4520, 0.0764]
    gc = GripperControl()
    gc.open_gripper()
    go_to_position(arm, get_cup_pos())
    gc.gripper_goto(27)
    go_to_position(arm, goal_pos)
    gc.open_gripper()


def main():
  try:
    print "Press Ctrl-D to exit at any time"
    print "Press 'Enter' to begin"
    raw_input()
    ur5e_arm = MoveGroupPythonInterface()
    #table and wall have to be added in separately--for some reason adding them together didn't work
    ur5e_arm.add_table()
    ur5e_arm.add_wall()
    print "Press 'Enter' to begin planning playback"
    raw_input()
    go_to_cup(ur5e_arm)
    #cartesian_plan, fraction = ur5e_arm.plan_cartesian_path()
    #print "Press 'Enter' to display planned trajectory"
    #raw_input()
    #ur5e_arm.display_trajectory(cartesian_plan)
    #print "Press 'Enter' to execute planned trajectory"
    #raw_input()
    #ur5e_arm.execute_plan(cartesian_plan)
    print "Execution complete"
    print "Press 'Enter' to exit'"
    raw_input()
    ur5e_arm.remove_workspace()
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  rospy.init_node('go_to_cup', anonymous=True)
  main()
