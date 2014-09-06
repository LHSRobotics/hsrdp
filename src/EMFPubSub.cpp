#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
#include <moveit_msgs/MoveGroupActionResult.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::Int16MultiArray>("/arm_encoder_targets", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/move_group/result", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const moveit_msgs::MoveGroupActionResult& input)
  {
    ROS_INFO("CB START");
    size_t size = 7;
    std::vector<short> temp(size);
    std_msgs::Int16MultiArray encoder_targets;// = {layout: {dim: [], data_offset: 0}, data: [0, 0, 0, 0, 0, 0, -375]};
    ROS_INFO("1");
    trajectory_msgs::JointTrajectoryPoint point = input.result.planned_trajectory.joint_trajectory.points[1];
    ROS_INFO("2");
    
    ros::Publisher arm_targets_pub;
    
    ROS_INFO("TARGET LOAD START");
   /* encoder_targets.data[0] = (short)(0);
    encoder_targets.data[1] = (short)(0);
    encoder_targets.data[2] = (short)(0);
    encoder_targets.data[3] = (short)(0);
    encoder_targets.data[4] = (short)(0);
    encoder_targets.data[5] = (short)(0);
    encoder_targets.data[6] = (short)(0);*/
    ROS_INFO("TARGET LOAD FIN");    
    
    ROS_INFO("traj fin -- encoder pub start");
    encoder_targets.data = temp;
    pub_.publish(encoder_targets);
    ROS_INFO("CB FIN");
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "EMFPubSub");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
