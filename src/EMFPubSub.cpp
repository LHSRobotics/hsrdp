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
    int waypointCount = input.result.planned_trajectory.joint_trajectory.points.size();
    ROS_INFO("Waypoint Count: %d", waypointCount);
    ros::Publisher arm_targets_pub;
    //Assuming joint order:  joint_names: ['shoulder_updown', 'shoulder_joint', 'elbow', 'wrist', 'wrist_gripper_connection_roll', 'wrist_gripper_connection_pitch']
   
    for(int p = 1; p < waypointCount; p++)
    {
      trajectory_msgs::JointTrajectoryPoint point = input.result.planned_trajectory.joint_trajectory.points[p];
      ROS_INFO("TARGET LOAD START || WAYPOINT: %d", p);
           
      //gripper
      temp[0] = (short) (
                         0
                         );

      //w1=(r-p)/k
      temp[1] = (short) (
                               (point.positions[5]
                               - point.positions[4])
                               / 0.001294162
                               );
      //w2=(-r-p)/k
      temp[2] = (short) (
                               ((-point.positions[5])
                               - point.positions[4])
                               / 0.001294162
                               );
      //wrist
      /*THIS IS WRONG*/
      temp[3] = (short) (
                                (point.positions[3] / -0.00179193)
                                );
                                
      //elbow
      temp[4] = (short) (
                                (point.positions[2] / 0.001194503)
                                );

      //shoulder_joint
      temp[5] = (short) (
                                (point.positions[1] / -0.000597252)
                                );
      
      //shoulder_updown
      temp[6] = (short) (
                                (point.positions[0] / -0.0002667)
                                );     

      
      ROS_INFO("TARGET LOAD FIN");    
      
      ROS_INFO("data = temp and publish to encoder_targets [START]");
      encoder_targets.data = temp;
      pub_.publish(encoder_targets);
      ROS_INFO("CB FIN");
    }
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
