import numpy as np
import rospy
from geometry_msgs.msg import PointStamped
import std_msgs
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from pid import PID


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
    
    self.x_PID = PID(kp=0.001, ki=1e-6, kd=1e-6)
    self.y_PID = PID(kp=0.001, ki=1e-6, kd=1e-6)
    self.z_PID = PID(kp=0.001, ki=1e-6, kd=1e-6)
    
  def goto_start(self):
    wpose = self.move_group.get_current_pose().pose
    wpose.orientation.x = -1.0 / (2**0.5)
    wpose.orientation.y = 0.0
    wpose.orientation.z = 0.0
    wpose.orientation.w = 1.0 / (2**0.5)
    waypoints = []
    waypoints.append(copy.deepcopy(wpose))
    (start_plan, start_fraction) = self.move_group.compute_cartesian_path(waypoints, 0.001, 0.0)
    self.move_group.execute(start_plan, wait=True);

  def goto_xyz(self, msg):
    dt = 0.1
    time = msg.header.stamp.secs + (1e-9 * msg.header.stamp.nsecs)
    dx = self.x_PID.calc_control(msg.point.x, time) * dt
    dy = self.y_PID.calc_control(msg.point.y, time) * dt
    dz = self.z_PID.calc_control(msg.point.z, time) * dt
    print('Position Changes')
    print([dx, dy, dz])
    wpose = self.move_group.get_current_pose().pose
    dist = (dx**2 + dy**2 + dz**2)**0.5
    if dist > 0.01 and dist < 0.3:
        print('Planning')
        wpose.position.x =  wpose.position.x + dx
        wpose.position.y =  wpose.position.y + dy
        wpose.position.z =  wpose.position.z + dz
        waypoints = []
        waypoints.append(copy.deepcopy(wpose))
        (start_plan, start_fraction) = self.move_group.compute_cartesian_path(waypoints, 0.001, 0.0)
        self.move_group.execute(start_plan, wait=True);

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
    
def point_follower():

    rospy.init_node('follow_object', anonymous=True)
    
    mgi = MoveGroupPythonInterface()
    
    #table and wall have to be added in separately--for some reason adding them together didn't work
    print "Press 'Enter' to add in table"
    raw_input()
    mgi.add_table()
    print "Press 'Enter' to add in wall"
    raw_input()
    mgi.add_wall()
    print "Press 'Enter' to goto start position"
    raw_input()
    mgi.goto_start()
    print "Press 'Enter' to begin planning playback"
    raw_input()
    
    rospy.Subscriber('/object/position_local', PointStamped, mgi.goto_xyz, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    point_follower()
