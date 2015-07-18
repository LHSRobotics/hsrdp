#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <control_msgs/GripperCommandAction.h>
#include <std_msgs/Int16.h>

#include <math.h>

class GripperCommandAction {
protected:

  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange
  // error may occur.
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;
  std::string action_name_;
  // create messages that are used to published feedback/result
  control_msgs::GripperCommandFeedback feedback_;
  control_msgs::GripperCommandResult   result_;
  std_msgs::Int16 gripper_encoder_target;
  ros::Publisher  gripper_target_pub;

public:

  GripperCommandAction(std::string name) :
    as_(nh_, name, boost::bind(&GripperCommandAction::executeCB, this, _1),
        false),
    action_name_(name)
  {
    as_.start();
    gripper_target_pub = nh_.advertise<std_msgs::Int16>("gripper_encoder_target",
                                                        1000);
  }

  ~GripperCommandAction(void)
  {}

  void executeCB(const control_msgs::GripperCommandGoalConstPtr& goal)
  {
    ROS_INFO("EXCB");
    ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");

    // = {layout: {dim: [], data_offset: 0}, data: [0, 0, 0, 0, 0, 0, -375]};

    // float pivot_to_tip;

    float sep = goal->command.position / 1000.f;

    // 0.0107x^2 + 58.4 x - sep = 0

    // (-58.4f +sqrt(pow(58.4f, 2) + 4.0f * 0.0107f * sep))/0.0214f
    // 10.7*10^-6*x^2 +0.0584x -sep = 0

    float tempF = 100000 *
                  (-0.0584f +
                   sqrt(pow(0.0584f, 2) + 4.0f * 0.0000701f * sep)) / 0.0000214f;
    int16_t temp = (int16_t)tempF;

    ROS_INFO("data = temp and publish to encoder_target [START] || dataF = %f",
             tempF);
    gripper_encoder_target.data = temp;

    gripper_target_pub.publish(gripper_encoder_target);

    ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
    ROS_INFO("Published!");

    if (ros::ok())
    {
      ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
      ROS_INFO("%s: Succeeded",     action_name_.c_str());
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "gripperServer");
  GripperCommandAction gripperServer(ros::this_node::getName());

  ROS_INFO("gripperServer RUN");
  ros::spin();

  return 0;
}
