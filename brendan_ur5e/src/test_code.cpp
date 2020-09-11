

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>


#include <stdlib.h>

#include <boost/filesystem.hpp>
#include <stdlib.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <array>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <moveit/move_group/capability_names.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include <robotiq_2f_gripper_control/Robotiq2FGripper_robot_output.h>
#include <std_msgs/Int8.h>
/*
class Gripper
{
  public:

  // Members  
  robotiq_2f_gripper_control::Robotiq2FGripper_robot_output command;

  // Publishers and Subscribers
  ros::Publisher gripper_command;
  ros::Subscriber gripper_instruction;

  // Functions
  Gripper(ros::NodeHandle nodeHandle)
  {
    this->gripper_command = nodeHandle.advertise<robotiq_2f_gripper_control::Robotiq2FGripper_robot_output>("Robotiq2FGripperRobotOutput",10);
    this->gripper_instruction = nodeHandle.subscribe("gripper_instruction", 1, &Gripper::gripper_instruction_callback, this);
  }
  void activate_gripper()
  {
    this->command.rACT = 1;
    this->command.rGTO = 1;
    this->command.rSP = 255;
    this->command.rFR = 150;
  }
  void gripper_open()
  {
    this->command.rPR = 0;
  }
  void gripper_close()
  {
    this->command.rPR = 255;
  }
  void gripper_instruction_callback(const std_msgs::Int8 msg)
  {
    if(msg.data == 0)
    {
      this->gripper_open();
      this->gripper_command.publish(this->command);
    }
    else if(msg.data == 1);
    {
      this->gripper_close();
      this->gripper_command.publish(this->command);
    }
  }

};

*/
int main(int argc, char** argv)
{

ros::init(argc, argv, "test_code");
ros::NodeHandle node_handle;
ros::AsyncSpinner spinner(1);
spinner.start();

static const std::string PLANNING_GROUP = "manipulator";

moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools visual_tools("wrist_3_joint");
visual_tools.deleteAllMarkers();

visual_tools.loadRemoteControl();

Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
text_pose.translation().z() = 1.75;
visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

visual_tools.trigger();

ROS_INFO_NAMED("test", "Reference frame: %s", move_group.getPlanningFrame().c_str());

ROS_INFO_NAMED("test", "End effector link: %s", move_group.getEndEffectorLink().c_str());

visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

move_group.setMaxVelocityScalingFactor(0.1);

std::vector<moveit_msgs::CollisionObject> collision_objects;
collision_objects.resize(6);

collision_objects[0].header.frame_id = move_group.getPlanningFrame();
collision_objects[0].id = "box1";

collision_objects[0].primitives.resize(1);
collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
collision_objects[0].primitives[0].dimensions.resize(3);
collision_objects[0].primitives[0].dimensions[0] = 0.005;
collision_objects[0].primitives[0].dimensions[1] = 0.2;
collision_objects[0].primitives[0].dimensions[2] = 0.1;
collision_objects[0].primitive_poses.resize(1);
collision_objects[0].primitive_poses[0].position.x = -0.0975;
collision_objects[0].primitive_poses[0].position.y = 0.5;
collision_objects[0].primitive_poses[0].position.z = 0.05;

collision_objects[1].header.frame_id = move_group.getPlanningFrame();
collision_objects[1].id = "box2";

collision_objects[1].primitives.resize(1);
collision_objects[1].primitives[0].type = collision_objects[0].primitives[0].BOX;
collision_objects[1].primitives[0].dimensions.resize(3);
collision_objects[1].primitives[0].dimensions[0] = 0.005;
collision_objects[1].primitives[0].dimensions[1] = 0.2;
collision_objects[1].primitives[0].dimensions[2] = 0.1;
collision_objects[1].primitive_poses.resize(1);
collision_objects[1].primitive_poses[0].position.x = 0.0975;
collision_objects[1].primitive_poses[0].position.y = 0.5;
collision_objects[1].primitive_poses[0].position.z = 0.05;

collision_objects[2].header.frame_id = move_group.getPlanningFrame();
collision_objects[2].id = "box3";

collision_objects[2].primitives.resize(1);
collision_objects[2].primitives[0].type = collision_objects[0].primitives[0].BOX;
collision_objects[2].primitives[0].dimensions.resize(3);
collision_objects[2].primitives[0].dimensions[0] = 0.19;
collision_objects[2].primitives[0].dimensions[1] = 0.005;
collision_objects[2].primitives[0].dimensions[2] = 0.1;
collision_objects[2].primitive_poses.resize(1);
collision_objects[2].primitive_poses[0].position.x = 0.0;
collision_objects[2].primitive_poses[0].position.y = 0.5975;
collision_objects[2].primitive_poses[0].position.z = 0.05;

collision_objects[3].header.frame_id = move_group.getPlanningFrame();
collision_objects[3].id = "box4";

collision_objects[3].primitives.resize(1);
collision_objects[3].primitives[0].type = collision_objects[0].primitives[0].BOX;
collision_objects[3].primitives[0].dimensions.resize(3);
collision_objects[3].primitives[0].dimensions[0] = 0.19;
collision_objects[3].primitives[0].dimensions[1] = 0.005;
collision_objects[3].primitives[0].dimensions[2] = 0.1;
collision_objects[3].primitive_poses.resize(1);
collision_objects[3].primitive_poses[0].position.x = 0.0;
collision_objects[3].primitive_poses[0].position.y = 0.4025;
collision_objects[3].primitive_poses[0].position.z = 0.05;

collision_objects[4].header.frame_id = move_group.getPlanningFrame();
collision_objects[4].id = "table";

collision_objects[4].primitives.resize(1);
collision_objects[4].primitives[0].type = collision_objects[0].primitives[0].BOX;
collision_objects[4].primitives[0].dimensions.resize(3);
collision_objects[4].primitives[0].dimensions[0] = 3.0;
collision_objects[4].primitives[0].dimensions[1] = 3.0;
collision_objects[4].primitives[0].dimensions[2] = 0.1;
collision_objects[4].primitive_poses.resize(1);
collision_objects[4].primitive_poses[0].position.x = 0.0;
collision_objects[4].primitive_poses[0].position.y = 0.0;
collision_objects[4].primitive_poses[0].position.z = 0.0;

collision_objects[5].header.frame_id = move_group.getPlanningFrame();
collision_objects[5].id = "wall";

collision_objects[5].primitives.resize(1);
collision_objects[5].primitives[0].type = collision_objects[0].primitives[0].BOX;
collision_objects[5].primitives[0].dimensions.resize(3);
collision_objects[5].primitives[0].dimensions[0] = 3.0;
collision_objects[5].primitives[0].dimensions[1] = 0.1;
collision_objects[5].primitives[0].dimensions[2] = 3.0;
collision_objects[5].primitive_poses.resize(1);
collision_objects[5].primitive_poses[0].position.x = 0.0;
collision_objects[5].primitive_poses[0].position.y = -0.1;
collision_objects[5].primitive_poses[0].position.z = 0.0;

collision_objects[0].operation = collision_objects[0].ADD;
collision_objects[1].operation = collision_objects[1].ADD;
collision_objects[2].operation = collision_objects[2].ADD;
collision_objects[3].operation = collision_objects[3].ADD;
collision_objects[4].operation = collision_objects[4].ADD;
collision_objects[5].operation = collision_objects[5].ADD;

/*
Gripper gripper(nh);

gripper.activate_gripper();
gripper.gripper_command.publish(gripper.command);
ros::Duration(1.5).sleep();
*/
//this->planning_scene_ptr->applyCollisionObjects(collision_objects);
/*
shape_msgs::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[0] = 0.195;
primitive.dimensions[1] = 0.005;
primitive.dimensions[2] = 0.1;

geometry_msgs::Pose box_pose;
box_pose.orientation.w = 1.0;
box_pose.position.x = -0.1;
box_pose.position.y = 0.6;
box_pose.position.z = 0.1;

collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);

collision_object.operation = collision_object.ADD;

std::vector<moveit_msgs::CollisionObject> collision_objects;
collision_objects.push_back(collision_object);

ROS_INFO_NAMED("test", "Add an object into the world");
planing_scene_interface.addCollisionObjects(collision_objects);

//ROS_INFO_NAMED("test", "Add an object into the world");
//planning_scene_interface.addCollisionObjects(collision_objects);

visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
visual_tools.trigger();

visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window once the collision object appears");
*/


while(1) {
geometry_msgs::Pose target_pose1;
target_pose1.position.x = 0.0;
target_pose1.position.y = 0.480;
target_pose1.position.z = 0.500;
target_pose1.orientation.x = 0.0;
target_pose1.orientation.y = 1.0;
target_pose1.orientation.z = 0.0;
target_pose1.orientation.w = 0.0;
move_group.setPoseTarget(target_pose1);

moveit::planning_interface::MoveGroupInterface::Plan my_plan;

bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

ROS_INFO_NAMED("test", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

ROS_INFO_NAMED("test", "Visualizing plan 1 as trajectory line");
visual_tools.publishAxisLabeled(target_pose1, "pose1");
visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


move_group.move();

geometry_msgs::Pose target_pose2;
target_pose2.position.x = 0.0;
target_pose2.position.y = 0.550;
target_pose2.position.z = 0.150;
target_pose2.orientation.x = 0.0;
target_pose2.orientation.y = 1.0;
target_pose2.orientation.z = 0.0;
target_pose2.orientation.w = 0.0;
move_group.setPoseTarget(target_pose2);

//moveit::planning_interface::MoveGroupInterface::Plan my_plan;

success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

ROS_INFO_NAMED("test", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

ROS_INFO_NAMED("test", "Visualizing plan 1 as trajectory line");
visual_tools.publishAxisLabeled(target_pose2, "pose2");
visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


move_group.move();

/*
close gripper
*/
/*
gripper.gripper_close();
gripper.gripper_command.publish(gripper.command);
ros::Duration(1.5).sleep();
*/
move_group.setPoseTarget(target_pose1);

//moveit::planning_interface::MoveGroupInterface::Plan my_plan;

success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

ROS_INFO_NAMED("test", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

ROS_INFO_NAMED("test", "Visualizing plan 1 as trajectory line");
visual_tools.publishAxisLabeled(target_pose1, "pose1");
visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


move_group.move();


geometry_msgs::Pose target_pose3;
target_pose3.position.x = 0.0;
target_pose3.position.y = 0.880;
target_pose3.position.z = 0.315;
target_pose3.orientation.x = 0.0;
target_pose3.orientation.y = 0.7;
target_pose3.orientation.z = 0.7;
target_pose3.orientation.w = 0.0;
move_group.setPoseTarget(target_pose3);

//moveit::planning_interface::MoveGroupInterface::Plan my_plan;

success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

ROS_INFO_NAMED("test", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

ROS_INFO_NAMED("test", "Visualizing plan 1 as trajectory line");
visual_tools.publishAxisLabeled(target_pose3, "pose3");
visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
visual_tools.trigger();
visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");


move_group.move();

/*
open gripper
*/
/*
gripper.gripper_open();
gripper.gripper_command.publish(gripper.command);
ros::Duration(1.5).sleep();
*/
}
}
