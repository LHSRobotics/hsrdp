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

//Pose defs:       -0.49392; 0.4931; 0.50726; 0.50556 = down+rollLeft  
//0.41824; -0.14418; 0.41441 bottom right hand corner
  geometry_msgs::Quaternion downLeft;  
  downLeft.x = -0.5;
  downLeft.y = 0.5;
  downLeft.z = 0.5;
  downLeft.w = 0.5;
  
  float tableheight = 0.1;
  
  geometry_msgs::Pose pos0;
  pos0.orientation = downLeft;
  pos0.position.x = 0.3;
  pos0.position.y = -0.15;
  pos0.position.z = tableheight;
  
  geometry_msgs::Pose pos1;
  pos1.orientation = downLeft;
  pos1.position.x = 0.3;
  pos1.position.y = -0.05;
  pos1.position.z = tableheight;
  
  geometry_msgs::Pose pos2;
  pos2.orientation = downLeft;
  pos2.position.x = 0.3;
  pos2.position.y = 0.05;
  pos2.position.z = tableheight;
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  geometry_msgs::Pose pos3;
  pos3.orientation = downLeft;
  pos3.position.x = 0.4;
  pos3.position.y = -0.15;
  pos3.position.z = tableheight;
  
  geometry_msgs::Pose pos4;
  pos4.orientation = downLeft;
  pos4.position.x = 0.4;
  pos4.position.y = -0.05;
  pos4.position.z = tableheight;
  
  geometry_msgs::Pose pos5;
  pos5.orientation = downLeft;
  pos5.position.x = 0.4;
  pos5.position.y = 0.05;
  pos5.position.z = tableheight;
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  geometry_msgs::Pose pos6;
  pos6.orientation = downLeft;
  pos6.position.x = 0.5;
  pos6.position.y = -0.15;
  pos6.position.z = tableheight;
  
  geometry_msgs::Pose pos7;
  pos7.orientation = downLeft;
  pos7.position.x = 0.5;
  pos7.position.y = -0.05;
  pos7.position.z = tableheight;
  
  geometry_msgs::Pose pos8;
  pos8.orientation = downLeft;
  pos8.position.x = 0.5;
  pos8.position.y = 0.05;
  pos8.position.z = tableheight;


//Motion 1--------------------------------------------------------------
  moveit::planning_interface::MoveGroup::Plan plan;
  group.setPoseTarget(pos0);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",group.plan(plan)?"pos0":"FAILED");
  sleep(1);    
  group.move();
  sleep(3.5);
  
  group.setPoseTarget(pos1);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",group.plan(plan)?"pos1":"FAILED");
  sleep(1);    
  group.move();
  sleep(2.5);
  
  group.setPoseTarget(pos2);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",group.plan(plan)?"pos2":"FAILED");
  sleep(1);    
  group.move();
  sleep(2.5);
  
  group.setPoseTarget(pos3);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",group.plan(plan)?"pos3":"FAILED");
  sleep(1);    
  group.move();
  sleep(2.5);
  
  group.setPoseTarget(pos4);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",group.plan(plan)?"pos4":"FAILED");
  sleep(1);    
  group.move();
  sleep(2.5);
  
  group.setPoseTarget(pos5);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",group.plan(plan)?"pos5":"FAILED");
  sleep(1);    
  group.move();
  sleep(2.5);
  
  group.setPoseTarget(pos6);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",group.plan(plan)?"pos6":"FAILED");
  sleep(1);    
  group.move();
  sleep(2.5);
  
  group.setPoseTarget(pos7);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",group.plan(plan)?"pos7":"FAILED");
  sleep(1);
  group.move();
  sleep(2.5);
  
  group.setPoseTarget(pos8);
  ROS_INFO("Visualizing plan 1 (pose goal) %s",group.plan(plan)?"pos8":"FAILED");
  sleep(1);    
  group.move();
  sleep(2.5);

  //group.move();
  sleep(4.0);
//----------------------------------------------------------------------
  

//Motion 2--------------------------------------------------------------
  std::vector<double> group_gripper_variable_values;
  group_gripper.getCurrentState()->copyJointGroupPositions(group_gripper.getCurrentState()->getRobotModel()->getJointModelGroup(group_gripper.getName()), group_gripper_variable_values);
  group_gripper_variable_values[0] = 0.01;
  group_gripper.setJointValueTarget(group_gripper_variable_values);
  
  moveit::planning_interface::MoveGroup::Plan plan_2;

  ROS_INFO("Visualizing plan 2 (joint target) %s",group_gripper.plan(plan_2)?"":"FAILED");    
  sleep(1.0);

  //group_gripper.move();
  sleep(3.0);
//----------------------------------------------------------------------





  ros::shutdown();
  return 0;
}
