#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/Int16.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

#include <iomanip>
#include <ctime>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <controller_manager/controller_manager.h>

//#include <realtime_tools/realtime_publisher.h>


/**
 * A intermediary node that translates the encoder counts from the umi arm
 * into joint states, and likewise target joint states to target joint counts
 */
 
sensor_msgs::JointState joint_state;
bool is_state_to_publish = false;;

class Hsrdp : public hardware_interface::RobotHW//, public hardware_interface::HardwareInterface
{
public:
  Hsrdp()
  {
    // connect and register the joint state interface
    hardware_interface::JointStateHandle sh_shoulder_joint("shoulder_joint", &pos[0], &vel[0], &eff[0]);
    jnt_state_interface.registerHandle(sh_shoulder_joint);

    hardware_interface::JointStateHandle sh_wrist_gripper_connection_pitch("wrist_gripper_connection_pitch", &pos[1], &vel[1], &eff[1]);
    jnt_state_interface.registerHandle(sh_wrist_gripper_connection_pitch);

    hardware_interface::JointStateHandle sh_wrist_gripper_connection_roll("wrist_gripper_connection_roll", &pos[2], &vel[2], &eff[2]);
    jnt_state_interface.registerHandle(sh_wrist_gripper_connection_roll);

    hardware_interface::JointStateHandle sh_shoulder_updown("shoulder_updown", &pos[3], &vel[3], &eff[3]);
    jnt_state_interface.registerHandle(sh_shoulder_updown);

    hardware_interface::JointStateHandle sh_elbow("elbow", &pos[4], &vel[4], &eff[4]);
    jnt_state_interface.registerHandle(sh_elbow);

    hardware_interface::JointStateHandle sh_wrist("wrist", &pos[5], &vel[5], &eff[5]);
    jnt_state_interface.registerHandle(sh_wrist);

    hardware_interface::JointStateHandle sh_gripper("gripper", &pos[6], &vel[6], &eff[6]);
    jnt_state_interface.registerHandle(sh_gripper);
    
    registerInterface(&jnt_state_interface);
    
    
    hardware_interface::JointHandle ph_shoulder_joint(jnt_state_interface.getHandle("shoulder_joint"), &cmd[0]);
    jnt_pos_interface.registerHandle(ph_shoulder_joint);

    hardware_interface::JointHandle ph_wrist_gripper_connection_pitch(jnt_state_interface.getHandle("wrist_gripper_connection_pitch"), &cmd[1]);
    jnt_pos_interface.registerHandle(ph_wrist_gripper_connection_pitch);

    hardware_interface::JointHandle ph_wrist_gripper_connection_roll(jnt_state_interface.getHandle("wrist_gripper_connection_roll"), &cmd[2]);
    jnt_pos_interface.registerHandle(ph_wrist_gripper_connection_roll);

    hardware_interface::JointHandle ph_shoulder_updown(jnt_state_interface.getHandle("shoulder_updown"), &cmd[3]);
    jnt_pos_interface.registerHandle(ph_shoulder_updown);

    hardware_interface::JointHandle ph_elbow(jnt_state_interface.getHandle("elbow"), &cmd[4]);
    jnt_pos_interface.registerHandle(ph_elbow);

    hardware_interface::JointHandle ph_wrist(jnt_state_interface.getHandle("wrist"), &cmd[5]);
    jnt_pos_interface.registerHandle(ph_wrist);

    hardware_interface::JointHandle ph_gripper(jnt_state_interface.getHandle("gripper"), &cmd[6]);
    jnt_pos_interface.registerHandle(ph_gripper);
    
    registerInterface(&jnt_pos_interface);
  }
  

  void publishTarget(ros::NodeHandle n)//sensor_msgs::JointState& cmd, ros::Publisher& pub)
  {
    //gripper
    encoder_targets.data[0] = (short) (
                             cmd[6]
                             );

    //w1=(r-p)/k
    encoder_targets.data[1] = (short) (
                             (cmd[1]
                             - cmd[2])
                             / 0.001294162
                             );
    //w2=(-r-p)/k
    encoder_targets.data[2] = (short) (
                             ((-cmd[1])
                             - cmd[2])
                             / 0.001294162
                             );
    //wrist
    /*THIS IS WRONG*/
    encoder_targets.data[3] = (short) (
                              (cmd[5] / -0.00179193)
                              );
                              
    //elbow
    encoder_targets.data[4] = (short) (
                              (cmd[4] / 0.001194503)
                              );

    //shoulder_joint
    encoder_targets.data[5] = (short) (
                              (cmd[0] / -0.000597252)
                              );
    
    //shoulder_updown
    encoder_targets.data[6] = (short) (
                              (cmd[3] / -0.0002667)
                              );
                              
    n.advertise<std_msgs::Int16MultiArray>("arm_encoder_targets", 1000).publish(encoder_targets);
    //arm_targets_pub.publish(encoder_targets);
                              
    /*if (realtime_pub->trylock())
    {
      realtime_pub->msg_.a_field = "hallo";
      realtime_pub->msg_.header.stamp = ros::Time::now();
      realtime_pub->unlockAndPublish();
    }*/
  }
private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  double cmd[7];
  double pos[7];
  double vel[7];
  double eff[7];
  std_msgs::Int16MultiArray encoder_targets;
  //ros::Publisher arm_targets_pub = n.advertise<std_msgs::Int16MultiArray>("arm_encoder_targets", 1000);
};


void encoderProcess(const std_msgs::Int16MultiArray& msg)
{
  ROS_INFO("I heard: [%d, %d, %d, %d, %d, %d, %d]", msg.data[0],msg.data[1],msg.data[2],
  msg.data[3],msg.data[4],msg.data[5],msg.data[6]);

  //update joint_state
  joint_state.header.stamp = ros::Time::now();
  joint_state.name.resize(7);
  joint_state.position.resize(7);

  joint_state.name[0] ="shoulder_joint";
  joint_state.position[0] = float(msg.data[5]) * -0.000597252;

  //0.001294162
  joint_state.name[1] ="wrist_gripper_connection_pitch";
  joint_state.position[1] = 
  0-((((float(msg.data[1]) + float(msg.data[2])) * 0.001294162 ) / 2)); 

  joint_state.name[2] ="wrist_gripper_connection_roll";
  joint_state.position[2] = 
  ((float(msg.data[1]) - float(msg.data[2])) * 0.001294162 ) / 2; 

  joint_state.name[3] ="shoulder_updown";
  joint_state.position[3] = float(msg.data[6]) * -0.0002667;

  joint_state.name[4] ="elbow";
  joint_state.position[4] = float(msg.data[4]) * 0.001194503;

  /*
  WRONG
  */
  joint_state.name[5] ="wrist";
  joint_state.position[5] = float(msg.data[3]) * -0.00179193;

  joint_state.name[6] ="gripper";
  joint_state.position[6] = msg.data[0];
  
  is_state_to_publish = true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "umi_bridge");

  ros::NodeHandle n;

  ros::Subscriber sub =  n.subscribe("arm_encoders", 1000, encoderProcess);

  ros::Publisher joint_pub =  n.advertise<sensor_msgs::JointState>("joint_states", 1);
  //ros::Publisher arm_targets_pub = n.advertise<std_msgs::Int16MultiArray>("arm_encoder_targets", 1000);

/*
  //not sure where this would go...
  realtime_pub = new 
    realtime_tools::RealtimePublisher<std_msgs::Int16MultiArray>(&n, "arm_encoder_targets", 1000);  
*/

  static ros::Time currentTick;
  static ros::Time previousTick;
  static ros::Duration deltaT;

  tf::TransformBroadcaster broadcaster;
  const double degree = M_PI/180;
  
  Hsrdp myHsrdp;
  
  controller_manager::ControllerManager cm(&myHsrdp, n);

  ros::Rate loop_rate(10);

  // message declarations
  geometry_msgs::TransformStamped odom_trans;
  
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "axis";


    int count = 0;
    while (ros::ok())
    {
      // update transform
      odom_trans.header.stamp = ros::Time::now();
      odom_trans.transform.translation.x = 0;
      odom_trans.transform.translation.y = 0;
      odom_trans.transform.translation.z = 0;
      odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(M_PI/2);

      //send the joint state and transform
      if(is_state_to_publish)
      {
        joint_pub.publish(joint_state);
        is_state_to_publish = false;
      }
      previousTick = currentTick;
      currentTick = ros::Time::now();
      deltaT = currentTick - previousTick;
      
      ros::Time* cTp = &currentTick;
      ros::Duration* dTp = &deltaT;
      
      cm.update(currentTick, deltaT);
      myHsrdp.publishTarget(n);
      
      broadcaster.sendTransform(odom_trans);

      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }


  return 0;
}
