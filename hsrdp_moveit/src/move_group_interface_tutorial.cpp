#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;  
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ROS_INFO("START move_group_interface_tutorial");

  sleep(5.0);
  
  
  moveit::planning_interface::MoveGroup group("main_group");
  moveit::planning_interface::MoveGroup group_gripper("gripper");
  group.setEndEffector("grip");
  
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;  

  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Active joints (main_group): %s, %s, %s, %s, %s, %s", group.getActiveJoints()[0].c_str(), group.getActiveJoints()[1].c_str(),group.getActiveJoints()[2].c_str(), group.getActiveJoints()[3].c_str(), group.getActiveJoints()[4].c_str(), group.getActiveJoints()[5].c_str());
  ROS_INFO("Active joints (gripper): %s", group_gripper.getActiveJoints()[0].c_str());
  
  ROS_INFO("Planning frame: %s", group.getPlanningFrame().c_str());  
  ROS_INFO("End Effector: %s", group.getEndEffectorLink().c_str());

//Motion 1--------------------------------------------------------------  
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0.2;
  target_pose1.position.x = 0.44473;
  target_pose1.position.y = 0.25665;
  target_pose1.position.z = 0.14;
  group.setPoseTarget(target_pose1);

  moveit::planning_interface::MoveGroup::Plan plan_1;
  bool success = group.plan(plan_1);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");    
  sleep(5.0);

  group.move();
  sleep(7.0);
//----------------------------------------------------------------------
  
//Motion 2--------------------------------------------------------------
  group_gripper.setJointValueTarget("gripper_left", -0.14);
  
  moveit::planning_interface::MoveGroup::Plan plan_2;
  success = group.plan(plan_2);

  ROS_INFO("Visualizing plan 2 (joint target) %s",success?"":"FAILED");    
  sleep(5.0);

  group.move();
  sleep(15.0);
//----------------------------------------------------------------------

  ros::shutdown();
  return 0;
}
