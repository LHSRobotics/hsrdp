#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Int16MultiArray.h>


class FollowJointTrajectoryAction
{
protected:
  ros::NodeHandle nh_;
  // NodeHandle instance must be created before this line. Otherwise strange error may occur.
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_; 
  std::string action_name_;
  // create messages that are used to published feedback/result
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult result_;
  std_msgs::Int16MultiArray encoder_targets;
  ros::Publisher arm_targets_pub;
  
public:

  FollowJointTrajectoryAction(std::string name) :
    as_(nh_, name, boost::bind(&FollowJointTrajectoryAction::executeCB, this, _1), false),
    action_name_(name)
    {
      as_.start();
      
      arm_targets_pub = nh_.advertise<std_msgs::Int16MultiArray>("arm_encoder_targets", 1000);  
    
    }

  ~FollowJointTrajectoryAction(void)
  {
    
  }

  double getPositionFromJointName(std::basic_string<char> name, std::vector<std::string> names, trajectory_msgs::JointTrajectoryPoint point)
  {
    int index;
    for(int joint_i=0; joint_i<7; joint_i++)
    {
      if(names[joint_i] == name) return point.positions[joint_i];
      ROS_INFO("%s: position from name has been called :P >>> %s", action_name_.c_str(), name.c_str());
    }
    ROS_INFO("%s: getPositionFromJointName couldn't find name in string array!", action_name_.c_str());
    return -1;
  }
  
  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    ROS_INFO("EXCB");
    ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
    //bool success = true;

    std::vector<std::string> j_n = goal->trajectory.joint_names;
    size_t size = 7;
    std::vector<short> temp(size);
    std_msgs::Int16MultiArray encoder_targets;// = {layout: {dim: [], data_offset: 0}, data: [0, 0, 0, 0, 0, 0, -375]};
    ROS_INFO("1");
    int waypointCount = goal->trajectory.points.size();
    ROS_INFO("Waypoint Count: %d", waypointCount);
    //Assuming joint order:  joint_names: ['shoulder_updown', 'shoulder_joint', 'elbow', 'wrist', 'wrist_gripper_connection_roll', 'wrist_gripper_connection_pitch']
   
    for(int p = 1; p < waypointCount; p++)
    {
      trajectory_msgs::JointTrajectoryPoint point = goal->trajectory.points[p];
      ROS_INFO("TARGET conv START || WAYPOINT: %d", p);
           
      //gripper
      temp[0] = (short) (
                         0
                         );
      ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
      
      //w1=(r-p)/k
      temp[1] = (short) (
                               (getPositionFromJointName("wrist_gripper_connection_pitch", j_n, point)
                               - getPositionFromJointName("wrist_gripper_connection_roll", j_n, point))
                               / 0.001294162
                               );
                               ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
      //w2=(-r-p)/k
      temp[2] = (short) (
                               ((-getPositionFromJointName("wrist_gripper_connection_pitch", j_n, point))
                               - getPositionFromJointName("wrist_gripper_connection_roll", j_n, point))
                               / 0.001294162
                               );
      //wrist
      /*THIS IS WRONG*/
      temp[3] = (short) (
                                (getPositionFromJointName("wrist", j_n, point) / -0.00179193)
                                );
                                
      //elbow
      temp[4] = (short) (
                                (getPositionFromJointName("elbow", j_n, point) / 0.001194503)
                                );

      //shoulder_joint
      temp[5] = (short) (
                                (getPositionFromJointName("shoulder_joint", j_n, point) / -0.000597252)
                                );
      
      //shoulder_updown
      temp[6] = (short) (
                                (getPositionFromJointName("shoulder_updown", j_n, point) / -0.0002667)
                                );     

      
      ROS_INFO("TARGET conv FIN");    
      
      ROS_INFO("data = temp and publish to encoder_targets [START]");
      encoder_targets.data = temp;
      arm_targets_pub.publish(encoder_targets);
                             
      ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
      ROS_INFO("Published!");

      // check that preempt has not been requested by the client
      
      if(as_.isPreemptRequested() || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());
        // set the action state to preempted
        as_.setPreempted();
        //success = false;
        break;
      }
      }
      // publish the feedback
      //as_.publishFeedback(feedback_);
      ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
      if(ros::ok())
      {ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
       //result_.sequence = feedback_.sequence;
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        result_.error_code = 0;//succesful error code (I think)
        as_.setSucceeded(result_);
      }
    
  }

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectoryServer");
  FollowJointTrajectoryAction trajectoryServer(ros::this_node::getName());
  
  
  ROS_INFO("MAIN RUN");
  ros::spin();

  return 0;
}
