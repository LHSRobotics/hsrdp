#include <ros/ros.h>
#include <moveit/controller_manager/controller_manager.h>
#include <sensor_msgs/JointState.h>
#include "std_msgs/Int16MultiArray.h"
#include <pluginlib/class_list_macros.h>
#include <map>

namespace hsrdp_moveit_controller_manager
{

class ArmControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
std_msgs::Int16MultiArray encoder_targets;

  ArmControllerHandle(const std::string &name) : moveit_controller_manager::MoveItControllerHandle(name)
  {
  	 
  }
  
  double getPositionFromJointName(std::basic_string<char> name, std::vector<std::string> names, trajectory_msgs::JointTrajectoryPoint point)
  {
    int index;
    for(int joint_i=0; joint_i<sizeof(point.positions)/sizeof(double); joint_i++)
    {
      if(names[joint_i] == name) return point.positions[joint_i];
      ROS_INFO("position from name has been called :P");
    }

    return -1;
  }
  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory &t)
  {
    // do whatever is needed to actually execute this trajectory
/*
    ros::Publisher arm_targets_pub = nh_.advertise<std_msgs::Int16MultiArray>("arm_encoder_targets", 1000);

    for(int p=0; p < sizeof(t->joint_trajectory.points)/sizeof(trajectory_msgs::JointTrajectoryPoint); p++)
    {
    trajectory_msgs::JointTrajectoryPoint point = t->joint_trajectory.points[p];
    std::vector<std::string> j_n = t->joint_trajectory.joint_names;
    
    //gripper
    encoder_targets.data[0] = (short) (
                             getPositionFromJointName("gripper", j_n, point)
                             );

    //w1=(r-p)/k
    encoder_targets.data[1] = (short) (
                             (getPositionFromJointName("wrist_gripper_connection_pitch", j_n, point)
                             - getPositionFromJointName("wrist_gripper_connection_roll", j_n, point))
                             / 0.001294162
                             );
    //w2=(-r-p)/k
    encoder_targets.data[2] = (short) (
                             ((-getPositionFromJointName("wrist_gripper_connection_pitch", j_n, point))
                             - getPositionFromJointName("wrist_gripper_connection_roll", j_n, point))
                             / 0.001294162
                             );
    //wrist
    //THIS IS WRONG
    encoder_targets.data[3] = (short) (
                              (getPositionFromJointName("wrist", j_n, point) / -0.00179193)
                              );
                              
    //elbow
    encoder_targets.data[4] = (short) (
                              (getPositionFromJointName("elbow", j_n, point) / 0.001194503)
                              );

    //shoulder_joint
    encoder_targets.data[5] = (short) (
                              (getPositionFromJointName("shoulder_joint", j_n, point) / -0.000597252)
                              );
    
    //shoulder_updown
    encoder_targets.data[6] = (short) (
                              (getPositionFromJointName("shoulder_updown", j_n, point) / -0.0002667)
                              );   
                              
    arm_targets_pub.publish(encoder_targets); 
   */
   ROS_INFO("SEND TRAJ RUN");
   
   
    
    return true;
  }
  
  virtual bool cancelExecution()
  {   
    // do whatever is needed to cancel execution 
    return true;
  }
  
  virtual bool waitForExecution(const ros::Duration &)
  {
    // wait for the current execution to finish
    return true;
  }
  
  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus()
  {
    return moveit_controller_manager::ExecutionStatus(moveit_controller_manager::ExecutionStatus::SUCCEEDED);
  }
};


class HsrdpMoveItControllerManager : public moveit_controller_manager::MoveItControllerManager
{
public:

  HsrdpMoveItControllerManager()
  {
  }
  
  virtual ~HsrdpMoveItControllerManager()
  {
  }

  virtual moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &name)
  {
    return moveit_controller_manager::MoveItControllerHandlePtr(new ArmControllerHandle(name));
  }
  
  /*
   * Get the list of controller names.
   */
  virtual void getControllersList(std::vector<std::string> &names)
  {
    names.resize(1);
    names[0] = "arm_controller";
  }

  /*
   * This plugin assumes that all controllers are already active -- and if they are not, well, it has no way to deal with it anyways!
   */
  virtual void getActiveControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Controller must be loaded to be active, see comment above about active controllers...
   */
  virtual void getLoadedControllers(std::vector<std::string> &names)
  {
    getControllersList(names);
  }

  /*
   * Get the list of joints that a controller can control.
   */
  virtual void getControllerJoints(const std::string &name, std::vector<std::string> &joints)
  {
    joints.clear();
    if (name == "arm_controller")
    {
      // declare which joints this controller actuates
      joints.push_back("shoulder_updown");
      joints.push_back("shoulder_joint");
      joints.push_back("elbow, wrist");
      joints.push_back("wrist_gripper_connection_roll");
    }
  }

  /*
   * Controllers are all active and default.
   */
  virtual moveit_controller_manager::MoveItControllerManager::ControllerState getControllerState(const std::string &name)
  {
    moveit_controller_manager::MoveItControllerManager::ControllerState state;
    state.active_ = true;
    state.default_ = true;
    return state;
  }

  /* Cannot switch our controllers */
  virtual bool switchControllers(const std::vector<std::string> &activate, const std::vector<std::string> &deactivate) { return false; }

protected:

  ros::NodeHandle node_handle_;
  std::map<std::string, moveit_controller_manager::MoveItControllerHandlePtr> controllers_;
};

} // end namespace

PLUGINLIB_EXPORT_CLASS(hsrdp_moveit_controller_manager::HsrdpMoveItControllerManager,
                       moveit_controller_manager::MoveItControllerManager);
