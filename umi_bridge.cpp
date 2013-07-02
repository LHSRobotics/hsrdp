#include "ros/ros.h"
#include "std_msgs/Int16MultiArray.h"
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

/**
 * A intermediary node that translates the encoder counts from the umi arm
 * into joint states, and likewise target joint states to target joint counts
 */
 
sensor_msgs::JointState joint_state;
bool is_state_to_publish = false;

void encoderProcess(const std_msgs::Int16MultiArray& msg)
{
  ROS_INFO("I heard: [%d, %d, %d, %d, %d, %d, %d]", msg.data[0],msg.data[1],msg.data[2],
	msg.data[3],msg.data[4],msg.data[5],msg.data[6]);
	
	   //update joint_state
      joint_state.header.stamp = ros::Time::now();
      joint_state.name.resize(8);
      joint_state.position.resize(8);
      joint_state.name[0] ="shoulder";
      joint_state.position[0] = msg.data[5];
      joint_state.name[1] ="wrist_pitch";
      joint_state.position[1] = msg.data[1];
      joint_state.name[2] ="wrist_roll";
      joint_state.position[2] = msg.data[2];
      joint_state.name[3] ="shoulder_updown";
      joint_state.position[4] = msg.data[6] ;
      joint_state.name[4] ="elbow";
      joint_state.position[4] = msg.data[4];
      joint_state.name[5] ="wrist_yaw";
      joint_state.position[6] = msg.data[3];
	  joint_state.name[7] ="gripper";
      joint_state.position[7] = msg.data[0];
      
      is_state_to_publish = true;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "umi_bridge");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("arm_encoders", 1000, encoderProcess);

  ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  ros::Publisher arm_targets_pub = n.advertise<std_msgs::Int16MultiArray>("arm_encoder_targets", 1000);

  tf::TransformBroadcaster broadcaster;
  const double degree = M_PI/180;

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
      
      broadcaster.sendTransform(odom_trans);


      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }


  return 0;
}
