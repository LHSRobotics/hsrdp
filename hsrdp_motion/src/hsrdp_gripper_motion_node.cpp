#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hsrdp_gripper_motion_node");
  ros::NodeHandle node_handle;

  // ros::spinOnce();

  sleep(5.0);

  moveit::planning_interface::MoveGroup group("gripper");

  // group.setEndEffector("grip");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Publisher display_publisher =
    node_handle.advertise<moveit_msgs::DisplayTrajectory>(
      "/move_group/display_planned_path",
      1,
      true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  moveit::planning_interface::MoveGroup::Plan my_plan;

  ROS_INFO("Active joints (gripper): %s",
           group.getActiveJoints()[0].c_str());

  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(
    group.getCurrentState()->getRobotModel()->getJointModelGroup(
      group.getName()), group_variable_values);

  ROS_WARN("group var val: %f", group_variable_values[0]);
  group_variable_values[0] = 0.01;
  group.setJointValueTarget(group_variable_values);

  group.asyncMove();

  /* Sleep to give Rviz time to visualize the plan. */
  // sleep(5.0);

  // ----------------------------------------------------------------------
  while (ros::ok())
  {
    sleep(1.0);
    ros::spinOnce();
  }
  ros::shutdown();
  return 0;
}
