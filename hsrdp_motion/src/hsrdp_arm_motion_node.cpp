#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hsrdp_arm_motion_node");
  ros::NodeHandle node_handle;

  // ros::spinOnce();

  ROS_INFO("START hsrdp_arm_motion_node");

  sleep(10.0);

  moveit::planning_interface::MoveGroup group("main_group");

  // moveit::planning_interface::MoveGroup group_gripper("gripper");
  group.setEndEffector("grip");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ros::Publisher display_publisher =
    node_handle.advertise<moveit_msgs::DisplayTrajectory>(
      "/move_group/display_planned_path",
      1,
      true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  ROS_INFO("Active joints (main_group): %s, %s, %s, %s, %s, %s",
           group.getActiveJoints()[0].c_str(),
           group.getActiveJoints()[1].c_str(),
           group.getActiveJoints()[2].c_str(),
           group.getActiveJoints()[3].c_str(),
           group.getActiveJoints()[4].c_str(),
           group.getActiveJoints()[5].c_str());

  ROS_INFO("Planning frame: %s",
           group.getPlanningFrame().c_str());
  ROS_INFO("End Effector: %s",
           group.getEndEffectorLink().c_str());

  // Pose defs:       -0.49392; 0.4931; 0.50726; 0.50556 = down+rollLeft
  // 0.41824; -0.14418; 0.41441 bottom right hand corner
  geometry_msgs::Quaternion downLeft;
  downLeft.x = -0.5;
  downLeft.y = 0.5;
  downLeft.z = 0.5;
  downLeft.w = 0.5;

  float tableheight = 0.1;

  geometry_msgs::Pose pos0;
  pos0.orientation = downLeft;
  pos0.position.x  = 0.3;
  pos0.position.y  = -0.15;
  pos0.position.z  = tableheight;


  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose = pos0;
  waypoints.push_back(target_pose);
  target_pose.position.x += 0.05;
  target_pose.position.y += 0.05;
  waypoints.push_back(target_pose);
  target_pose.position.x += 0.05;
  target_pose.position.y += -0.05;
  waypoints.push_back(target_pose);
  target_pose.position.x += 0.05;
  target_pose.position.y += 0.05;
  waypoints.push_back(target_pose);
  target_pose.position.x += 0.05;
  target_pose.position.y += -0.05;
  waypoints.push_back(target_pose);
  target_pose.position.x += -0.05;
  target_pose.position.y += 0.05;
  target_pose.position.z += 0.02;
  waypoints.push_back(target_pose);
  target_pose.position.x += -0.05;
  target_pose.position.y += -0.05;
  target_pose.position.z += 0.02;
  waypoints.push_back(target_pose);
  target_pose.position.x += -0.05;
  target_pose.position.y += 0.05;
  target_pose.position.z += 0.02;
  waypoints.push_back(target_pose);
  target_pose.position.x += -0.05;
  target_pose.position.y += -0.05;
  target_pose.position.z += 0.02;
  waypoints.push_back(target_pose);

  ROS_INFO("fin waypoints");
  moveit_msgs::RobotTrajectory trajectory;
  double proportion_acheived = group.computeCartesianPath(waypoints,
                                               0.01, // eef_step
                                               0.0,  // jump_threshold
                                               trajectory,
                                               true);//colision_Avoid

  ROS_INFO("Visualizing plan (cartesian path) (%.2f%% acheived)",
           proportion_acheived * 100.0);

  /* Sleep to give Rviz time to visualize the plan. */
  // sleep(15.0);
  // ROS_INFO("fin sleep 15.0");
  moveit::planning_interface::MoveGroup::Plan plan;
  plan.trajectory_ = trajectory;
  group.asyncExecute(plan);
  // Motion 1--------------------------------------------------------------
  // moveit::planning_interface::MoveGroup::Plan plan;
  // group.setPoseTarget(pos0);
  // sleep(1.0);
  // group.asyncMove();
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
