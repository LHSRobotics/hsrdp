#include "ros/ros.h"
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

#include <vector>
#include <iterator>
#include <algorithm>
#include <map>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

std_msgs::Int16 gripper_encoder_target;
ros::Publisher gripper_target_pub;
ros::Publisher player_pub;
int16_t OPENNED = 0;
int16_t CLOSED = 350;


std::vector<int> board_occ (9);
std::vector<int> x_tray_occ (5, 1);
std::vector<int> o_tray_occ (5, 1);

std::vector<int> board (9);
std::vector<int> x_tray (5, 1);
std::vector<int> o_tray (5, 1);
char bot_symbol = 'o';
bool inGame = false;
int sum_board;
std_msgs::Int8 player;//first or second
bool buttonPressed = false;

float tableheight = 0.1;
  geometry_msgs::Quaternion downLeft;  
  geometry_msgs::Pose pos0;
  geometry_msgs::Pose pos1;
  geometry_msgs::Pose pos2;
  geometry_msgs::Pose pos3;
  geometry_msgs::Pose pos4;
  geometry_msgs::Pose pos5;
  geometry_msgs::Pose pos6;
  geometry_msgs::Pose pos7;
  geometry_msgs::Pose pos8;
  geometry_msgs::Pose idlePose;
  geometry_msgs::Pose xpos0;
  geometry_msgs::Pose xpos1;
  geometry_msgs::Pose xpos2;
  geometry_msgs::Pose xpos3;
  geometry_msgs::Pose xpos4;
  geometry_msgs::Pose opos0;
  geometry_msgs::Pose opos1;
  geometry_msgs::Pose opos2;
  geometry_msgs::Pose opos3;
  geometry_msgs::Pose opos4;
  
std::vector<geometry_msgs::Pose> board_poses;
std::vector<geometry_msgs::Pose> x_tray_poses;
std::vector<geometry_msgs::Pose> o_tray_poses;

void setup()
{
  player.data = 0;
  
  downLeft.x = -0.5;
  downLeft.y = 0.5;
  downLeft.z = 0.5;
  downLeft.w = 0.5;
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //~~~~~~~BOARD POSES~~~~~~~~~~~~~
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  pos0.orientation = downLeft;
  pos0.position.x = 0.3;
  pos0.position.y = -0.15;
  pos0.position.z = tableheight;

  pos1.orientation = downLeft;
  pos1.position.x = 0.3;
  pos1.position.y = -0.05;
  pos1.position.z = tableheight;

  pos2.orientation = downLeft;
  pos2.position.x = 0.3;
  pos2.position.y = 0.05;
  pos2.position.z = tableheight;
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  pos3.orientation = downLeft;
  pos3.position.x = 0.4;
  pos3.position.y = -0.15;
  pos3.position.z = tableheight;

  pos4.orientation = downLeft;
  pos4.position.x = 0.4;
  pos4.position.y = -0.05;
  pos4.position.z = tableheight;

  pos5.orientation = downLeft;
  pos5.position.x = 0.4;
  pos5.position.y = 0.05;
  pos5.position.z = tableheight;
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  pos6.orientation = downLeft;
  pos6.position.x = 0.5;
  pos6.position.y = -0.15;
  pos6.position.z = tableheight;

  pos7.orientation = downLeft;
  pos7.position.x = 0.5;
  pos7.position.y = -0.05;
  pos7.position.z = tableheight;

  pos8.orientation = downLeft;
  pos8.position.x = 0.5;
  pos8.position.y = 0.05;
  pos8.position.z = tableheight;

  idlePose = pos0;
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //~~~~~~~~TRAY POSES~~~~~~~~~~~~~
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
  xpos0.orientation = downLeft;
  xpos0.position.x = 0.8;
  xpos0.position.y = 0.0;
  xpos0.position.z = tableheight;
  
  xpos1.orientation = downLeft;
  xpos1.position.x = 0.8;
  xpos1.position.y = 0.0;
  xpos1.position.z = tableheight;
  
  xpos2.orientation = downLeft;
  xpos2.position.x = 0.8;
  xpos2.position.y = 0.0;
  xpos2.position.z = tableheight;
  
  xpos3.orientation = downLeft;
  xpos3.position.x = 0.8;
  xpos3.position.y = 0.0;
  xpos3.position.z = tableheight;
  
  xpos4.orientation = downLeft;
  xpos4.position.x = 0.8;
  xpos4.position.y = 0.0;
  xpos4.position.z = tableheight;
  
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  opos0.orientation = downLeft;
  opos0.position.x = 0.8;
  opos0.position.y = 0.0;
  opos0.position.z = tableheight;
  
  opos1.orientation = downLeft;
  opos1.position.x = 0.8;
  opos1.position.y = 0.0;
  opos1.position.z = tableheight;
  
  opos2.orientation = downLeft;
  opos2.position.x = 0.8;
  opos2.position.y = 0.0;
  opos2.position.z = tableheight;

  opos3.orientation = downLeft;
  opos3.position.x = 0.8;
  opos3.position.y = 0.0;
  opos3.position.z = tableheight;
  
  opos4.orientation = downLeft;
  opos4.position.x = 0.8;
  opos4.position.y = 0.0;
  opos4.position.z = tableheight;
  //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  
// = {{0,pos0},{1,pos1},{2,pos2},{3,pos3},{4,pos4},{5,pos5},{6,pos6},{7,pos7},{8,pos8}};

  board_poses.push_back(pos0);// = { pos0, pos1, pos2 , pos3, pos4, pos5, pos6, pos7, pos8 };
  board_poses.push_back(pos1);
  board_poses.push_back(pos2);
  board_poses.push_back(pos3);
  board_poses.push_back(pos4);
  board_poses.push_back(pos5);
  board_poses.push_back(pos6);
  board_poses.push_back(pos7);
  board_poses.push_back(pos8);
  //x_tray_poses = { xpos0, xpos1, xpos2 , xpos4, xpos3};
  x_tray_poses.push_back(pos0);
  x_tray_poses.push_back(pos1);
  x_tray_poses.push_back(pos2);
  x_tray_poses.push_back(pos3);
  x_tray_poses.push_back(pos4);
  //o_tray_poses = { opos0, opos1, opos2, opos3, opos4};
  o_tray_poses.push_back(pos0);
  o_tray_poses.push_back(pos1);
  o_tray_poses.push_back(pos2);
  o_tray_poses.push_back(pos3);
  o_tray_poses.push_back(pos4);
  //..
  // = {{0,xpos0},{1,xpos1},{2,xpos2},{3,xpos3},{4,xpos4}};
}

void moveToPose(geometry_msgs::Pose pos, int moveTime, moveit::planning_interface::MoveGroup& group)
{
  ROS_INFO("...Moving to pose...");
  
  moveit::planning_interface::MoveGroup::Plan arm_plan;
  ROS_INFO("......Arm plan...");
  group.setPoseTarget(pos);
  ROS_INFO("......setPoseTarget...");
  group.plan(arm_plan);
  ROS_INFO("......test...");
  //ROS_INFO("Visualizing arm_plan (pose goal) %s",group.plan(arm_plan)?"PLANNED":"FAILED");
  //sleep(1.2);    
  group.move();
  sleep(moveTime);
}

void pickup(moveit::planning_interface::MoveGroup& group)
{
  ROS_INFO("...Picking Up...");

  moveit::planning_interface::MoveGroup::Plan z_plan;
  gripper_encoder_target.data = OPENNED;
  gripper_target_pub.publish(gripper_encoder_target);
  sleep(0.5);
  
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group_variable_values[0] -= 0.1;
  group.setJointValueTarget(group_variable_values);
  ROS_INFO("Visualizing z_plan (joint target) %s",group.plan(z_plan)?"PLANNED":"FAILED");
  sleep(0.5);
  group.move();
  sleep(1);
  
  gripper_encoder_target.data = CLOSED;
  gripper_target_pub.publish(gripper_encoder_target);
  sleep(0.5);
  
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group_variable_values[0] += 0.1;
  group.setJointValueTarget(group_variable_values);
  ROS_INFO("Visualizing z_plan (joint target) %s",group.plan(z_plan)?"PLANNED":"FAILED");
  sleep(0.5);
  group.move();
  sleep(1);
}

void place(moveit::planning_interface::MoveGroup& group)
{
  moveit::planning_interface::MoveGroup::Plan z_plan;
  gripper_encoder_target.data = CLOSED;
  gripper_target_pub.publish(gripper_encoder_target);
  sleep(0.5);
  
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group_variable_values[0] -= 0.1;
  group.setJointValueTarget(group_variable_values);
  ROS_INFO("Visualizing z_plan (joint target) %s",group.plan(z_plan)?"PLANNED":"FAILED");
  sleep(0.5);
  group.move();
  sleep(1);
  
  gripper_encoder_target.data = OPENNED;
  gripper_target_pub.publish(gripper_encoder_target);
  sleep(0.5);
  
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group_variable_values[0] += 0.1;
  group.setJointValueTarget(group_variable_values);
  ROS_INFO("Visualizing z_plan (joint target) %s",group.plan(z_plan)?"PLANNED":"FAILED");
  sleep(0.5);
  group.move();
  sleep(1);
}

void getBlockToPlace(moveit::planning_interface::MoveGroup& group)
{
  ROS_INFO("...Getting Block...");
  switch(bot_symbol)            
  {
    case 'x':
      moveToPose(x_tray_poses[*(std::find(x_tray.begin(), x_tray.end(), 1))], 2, group);
      pickup(group);
      break;
    case 'o':
      moveToPose(o_tray_poses[*(std::find(o_tray.begin(), o_tray.end(), 1))], 2, group);
      pickup(group);
      break;
  }
}

void placeBlock(int pos, moveit::planning_interface::MoveGroup& group)
{
  ROS_INFO("Placing block at %d ...", pos);
  getBlockToPlace(group);
  moveToPose(board_poses[pos], 3, group);
  place(group);
  moveToPose(idlePose, 2, group);
}

void returnBlock(int index, char tray, moveit::planning_interface::MoveGroup& group)
{
  ROS_INFO("Returning Block To %d in %c ...", index, tray);
  moveToPose(board_poses[index], 3, group);
  pickup(group);
  switch(tray)            
  {
    case 'x':
      moveToPose(x_tray_poses[*(std::find(x_tray.begin(), x_tray.end(), 1))], 2, group);
      place(group);
      break;
    case 'o':
      moveToPose(o_tray_poses[*(std::find(o_tray.begin(), o_tray.end(), 1))], 2, group);
      place(group);
      break;
  }
}

void tidy(moveit::planning_interface::MoveGroup& group)
{
  ROS_INFO("Tidying...");
  for(int i = 0; i < 9; i++) returnBlock(i, board[i], group);
}

int sumOfIntVector(std::vector<int> vec)
{
  int sum = 0;
  //for (int n : vec) sum += n;
  for(std::vector<int>::iterator j=vec.begin();j!=vec.end();++j)
    sum += *j;  
  return sum;
}

int win(std::vector<int> board)
{
    unsigned wins[8][3] = {{0,1,2},{3,4,5},{6,7,8},{0,3,6},{1,4,7},{2,5,8},{0,4,8},{2,4,6}};
    int i;
    for(i = 0; i < 8; ++i) {
        if(board[wins[i][0]] != 0 &&
           board[wins[i][0]] == board[wins[i][1]] &&
           board[wins[i][0]] == board[wins[i][2]])
           return board[wins[i][2]];
    }
    return 0;
}

int minimax(std::vector<int> board, int player)
{
    int winner = win(board);
    if(winner != 0) return winner*player;

    int move = -1;
    int score = -2;
    int i;
    for(i = 0; i < 9; ++i) {
        if(board[i] == 0) {
            board[i] = player;
            int thisScore = -minimax(board, player*-1);
            if(thisScore > score) {
                score = thisScore;
                move = i;
            }
            board[i] = 0;
        }
    }
    if(move == -1) return 0;
    return score;
}

void botMove(std::vector<int> board, moveit::planning_interface::MoveGroup& group)
{
    int move = -1;
    int score = -2;
    int i;
    for(i = 0; i < 9; ++i)
    {
        if(board[i] == 0)
        {
            board[i] = 1;
            int tempScore = -minimax(board, -1);
            board[i] = 0;
            if(tempScore > score)
            {
                score = tempScore;
                move = i;
            }
        }
    }
    ROS_INFO("Bot move: %d", move);
    placeBlock(move, group);
}

void playerMove()
{
 sleep(1);
 ROS_INFO("Waiting for player");
}

void cb_Board(const std_msgs::Int8MultiArray& msg)
{
  ROS_INFO("I heard: \n %d | %d | %d \n =========== \n %d | %d | %d \n =========== \n %d | %d | %d \n",
  msg.data[0],msg.data[1],msg.data[2],
  msg.data[3],msg.data[4],msg.data[5],
  msg.data[6],msg.data[7],msg.data[8]);
 
  for(int i = 0; i < 9; i++)
  { //board_occ.push_back(msg.data[i]);
    if(msg.data[i] == board_occ
  if(PREV_board_msg.data[i] == 0)
    {
      if(sumOfPrevBoard() - player == -2){ 
        return 1;
      }
      else{ 
        return -1;
      }
      nh.logerror("Something went wrong with knowing who's turn it just was!!!");
    }
    else if(PREV_board_msg.data[i] != 0) return PREV_board_msg.data[i];
    nh.logerror("The data in PREV_board_msg was neither 0 or non-0");
  }
  
  
  if(!inGame && sumOfIntVector(board) == -1)
  {
    inGame = true;
    player.data = 1;
    player_pub.publish(player);
  }
}

void cb_xTray(const std_msgs::Int8MultiArray& msg)
{
  ROS_INFO("I heard x_tray changed");
  for(int i = 0; i < 5; i++) x_tray.push_back(msg.data[i]);
}

void cb_oTray(const std_msgs::Int8MultiArray& msg)
{
  ROS_INFO("I heard o_tray changed");
  for(int i = 0; i < 5; i++) o_tray.push_back(msg.data[i]);
}

void cb_Button(const std_msgs::Bool& msg)
{
  ROS_INFO("I heard button changed");
  buttonPressed = (bool)msg.data;
  if(buttonPressed && !inGame && sumOfIntVector(board)==0)
  {
    inGame = true;
    player.data = 2;
    player_pub.publish(player);
  }//start game where player goes second
}

//[HUMAN]:-1
//[ ]:0
//[BOT]:1

int main(int argc, char **argv)
{

  ros::init(argc, argv, "noughts_and_crosses");
  ros::NodeHandle nh;  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  ros::Subscriber sub_board_state = nh.subscribe("/board_state", 1000, cb_Board);
  ros::Subscriber sub_x_tray_state = nh.subscribe("/x_tray_state", 1000, cb_xTray);
  ros::Subscriber sub_o_tray_state = nh.subscribe("/o_tray_state", 1000, cb_oTray);
  ros::Subscriber sub_button_state = nh.subscribe("/button_state", 1000, cb_Button);
  
  gripper_target_pub = nh.advertise<std_msgs::Int16>("/gripper_controller/gripper_encoder_target", 1, true);
  player_pub = nh.advertise<std_msgs::Int8>("/player_var", 1, true);
  
  moveit::planning_interface::MoveGroup group("main_group");
  group.setEndEffector("grip");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  ROS_INFO("Active joints (main_group): %s, %s, %s, %s, %s, %s", group.getActiveJoints()[0].c_str(), group.getActiveJoints()[1].c_str(),group.getActiveJoints()[2].c_str(), group.getActiveJoints()[3].c_str(), group.getActiveJoints()[4].c_str(), group.getActiveJoints()[5].c_str());
  
  ROS_INFO("Planning frame: %s", group.getPlanningFrame().c_str());  
  ROS_INFO("End Effector: %s", group.getEndEffectorLink().c_str());
  
  
  std::vector<double> group_variable_values;
  
  setup();
  moveToPose(pos8, 3, group);
  
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group_variable_values[0] -= 0.1;
  group.setJointValueTarget(group_variable_values);
  ROS_INFO("Visualizing z_plan (joint target) %s",group.plan(z_plan)?"PLANNED":"FAILED");
  sleep(0.5);
  group.move();
  sleep(1);
  
  moveToPose(pos8, 3, group);
  
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group_variable_values[0] -= 0.1;
  group.setJointValueTarget(group_variable_values);
  ROS_INFO("Visualizing z_plan (joint target) %s",group.plan(z_plan)?"PLANNED":"FAILED");
  sleep(0.5);
  group.move();
  sleep(1);
  
  moveToPose(pos8, 3, group);
  
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group_variable_values[0] -= 0.1;
  group.setJointValueTarget(group_variable_values);
  ROS_INFO("Visualizing z_plan (joint target) %s",group.plan(z_plan)?"PLANNED":"FAILED");
  sleep(0.5);
  group.move();
  sleep(1);
  
  moveToPose(pos8, 3, group);
  
  std::vector<double> group_variable_values;
  group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group_variable_values[0] -= 0.1;
  group.setJointValueTarget(group_variable_values);
  ROS_INFO("Visualizing z_plan (joint target) %s",group.plan(z_plan)?"PLANNED":"FAILED");
  sleep(0.5);
  group.move();
  sleep(1);


  }
  return 0;
}



//2
