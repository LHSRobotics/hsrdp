#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <control_msgs/GripperCommandAction.h>
#include <std_msgs/Int16MultiArray.h>


class GripperCommandAction
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  control_msgs::GripperCommandFeedback feedback_;
  control_msgs::GripperCommandResult result_;
  std_msgs::Int16MultiArray encoder_targets;
  ros::Publisher gripper_target_pub;
  
public:

  GripperCommandAction(std::string name) :
    as_(nh_, name, boost::bind(&GripperCommandAction::executeCB, this, _1), false),
    action_name_(name)
    {
      as_.start();
      
      gripper_target_pub = nh_.advertise<std_msgs::Int16MultiArray>("arm_encoder_targets", 1000);  
    
    }

  ~GripperCommandAction(void)
  {
    
  }
  
  void executeCB(const control_msgs::GripperCommandActionGoalConstPtr &goal)
  {
    ROS_INFO("EXCB");
    ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
    //bool success = true;

    size_t size = 7;
    std::vector<short> temp(size);
    std_msgs::Int16MultiArray encoder_targets;// = {layout: {dim: [], data_offset: 0}, data: [0, 0, 0, 0, 0, 0, -375]};
      //gripper
      temp[0] = (short) (
                         0
                         );
      
      ROS_INFO("TARGET conv FIN");    
      
      ROS_INFO("data = temp and publish to encoder_target [START]");
      encoder_targets.data = temp;
      gripper_target_pub.publish(encoder_targets);
                             
      ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
      
      ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
      ROS_INFO("Published!");
      if(ros::ok())
      {ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        
        as_.setSucceeded(result_);
      }
    
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "gripperServer");
  GripperCommandAction gripperServer(ros::this_node::getName());
  
  
  ROS_INFO("MAIN RUN");
  ros::spin();

  return 0;
}
