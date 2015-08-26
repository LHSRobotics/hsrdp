#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include "hsrdp_action/PublishNextWaypoint.h"

#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <std_msgs/Int16MultiArray.h>

#include <vector>
#include <queue>

class FollowJointTrajectoryAction {
protected:

  ros::NodeHandle nh_;

  // NodeHandle instance must be created before this line. Otherwise strange
  // error may occur.
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> as_;
  std::string action_name_;

  // create messages that are used to published feedback/result
  control_msgs::FollowJointTrajectoryFeedback feedback_;
  control_msgs::FollowJointTrajectoryResult   result_;
  std_msgs::Int16MultiArray encoder_targets;
  ros::Publisher arm_targets_pub;
  ros::ServiceServer NextWaypointService;

  // std::vector<trajectory_msgs::JointTrajectoryPoint> JointTrajectoryBuffer;
  std::queue<trajectory_msgs::JointTrajectoryPoint> JointTrajectoryBuffer;
  std::vector<std::string> j_n;

public:

  FollowJointTrajectoryAction(std::string name) :
    as_(nh_, name, boost::bind(&FollowJointTrajectoryAction::executeCB,
                               this,
                               _1), false),
    action_name_(name)
  {
    as_.start();

    arm_targets_pub = nh_.advertise<std_msgs::Int16MultiArray>(
      "arm_encoder_targets",
      1000);

    NextWaypointService = nh_.advertiseService("PublishNextWaypoint",
                                               &FollowJointTrajectoryAction::positionPublish,
                                               this);
  }

  ~FollowJointTrajectoryAction(void)
  {}

  double getPositionFromJointName(std::basic_string<char>               name,
                                  std::vector<std::string>              names,
                                  trajectory_msgs::JointTrajectoryPoint point)
  {
    int index;

    for (int joint_i = 0; joint_i < 7; joint_i++)
    {
      if (names[joint_i] == name) return point.positions[joint_i];

      ROS_INFO("%s: position from name has been called :P >>> %s",
               action_name_.c_str(),
               name.c_str());
    }
    ROS_INFO("%s: getPositionFromJointName couldn't find name in string array!",
             action_name_.c_str());
    return -1;
  }

  // Publish on messages for position control
  bool positionPublish(hsrdp_action::PublishNextWaypoint::Request & req,
                       hsrdp_action::PublishNextWaypoint::Response& res)
  {
    if (JointTrajectoryBuffer.size() > 0) {
      trajectory_msgs::JointTrajectoryPoint currentPoint =
        JointTrajectoryBuffer.front();
      size_t size = 6;
      std::vector<short> temp(size);
      std_msgs::Int16MultiArray encoder_targets;

      JointTrajectoryBuffer.pop();

      // w1=(r-p)/k
      temp[0] = (short)(
        (getPositionFromJointName("wrist_gripper_connection_roll", j_n,
                                  currentPoint)
         - getPositionFromJointName("wrist_gripper_connection_pitch", j_n,
                                    currentPoint))
        / 0.001294162
        );

      // w2=(-r-p)/k
      temp[1] = (short)(
        (-(getPositionFromJointName("wrist_gripper_connection_roll", j_n,
                                    currentPoint))
         - getPositionFromJointName("wrist_gripper_connection_pitch", j_n,
                                    currentPoint))
        / 0.001294162
        );

      // elbow
      temp[3] = (short)(
        (getPositionFromJointName("elbow", j_n, currentPoint) / 0.001194503)
        );

      // wrist
      // THIS IS PROBABLY WRONG
      temp[2] = (short)(
        (getPositionFromJointName("wrist", j_n, currentPoint) / -0.00179193)
        );

      // shoulder_joint
      temp[4] = (short)(
        (getPositionFromJointName("shoulder_joint", j_n,
                                  currentPoint) / -0.000597252)
        );

      // shoulder_updown
      temp[5] = (short)(
        (getPositionFromJointName("shoulder_updown", j_n,
                                  currentPoint) / -0.0002667)
        );

      encoder_targets.data = temp;
      arm_targets_pub.publish(encoder_targets);
      ROS_INFO("Published!");
    }
    res.WaypointsRemaining = JointTrajectoryBuffer.size();
    return true;
  }

  void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr& goal)
  {
    ROS_INFO("EXCB");
    ROS_INFO("isActive() returns %s", as_.isActive() ? "true" : "false");

    // bool success = true;

    j_n = goal->trajectory.joint_names;

    // size_t size                  = 6;
    // std::vector<short> temp(size);
    // std_msgs::Int16MultiArray encoder_targets;

    int waypointCount = goal->trajectory.points.size();
    ROS_INFO("Waypoint Count: %d", waypointCount);

    // Assuming joint order:  joint_names: ['shoulder_updown', 'shoulder_joint',
    // 'elbow', 'wrist', 'wrist_gripper_connection_roll',
    // 'wrist_gripper_connection_pitch']

    for (int p = 1; p < waypointCount; p++)
    {
      trajectory_msgs::JointTrajectoryPoint point = goal->trajectory.points[p];

      ROS_INFO("PUSH WAYPOINT %d TO BUFFER...", p);
      JointTrajectoryBuffer.push(point);

      ROS_INFO("isActive() returns %s",
               as_.isActive() ? "true" : "false");
      ROS_INFO("ENCODER TARGET CONVERSTION BEGIN FOR WAYPOINT: %d", p);

      // ROS_INFO("data = temp and publish to encoder_targets [START]");
      // encoder_targets.data = temp;
      // arm_targets_pub.publish(encoder_targets);
      //
      // ROS_INFO("Published!");

      // check that preempt has not been requested by the client
      if (as_.isPreemptRequested()) // || !ros::ok())
      {
        ROS_INFO("%s: Preempted", action_name_.c_str());

        // set the action state to preempted
        as_.setPreempted();
        break;
      }

      if (!ros::ok())
      {
        ROS_INFO("%s: Aborted", action_name_.c_str());

        // set the action state to aborted
        as_.setAborted();
        break;
      }
    }

    // publish the feedback
    // as_.publishFeedback(feedback_);
    // ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");

    // For testing purposes just set to Succeeded without bothering to check!!!
    if (ros::ok())
    {
      // ROS_INFO("isActive() >>> %s", as_.isActive() ? "true" : "false");
      ROS_INFO("%s: Succeeded", action_name_.c_str());

      // set the action state to succeeded
      result_.error_code = 0; // succesful error code (I think)
      as_.setSucceeded(result_);
    }
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "trajectoryServer");
  FollowJointTrajectoryAction trajectoryServer(ros::this_node::getName());


  ROS_INFO("MAIN RUN");
  ros::spin();

  return 0;
}
