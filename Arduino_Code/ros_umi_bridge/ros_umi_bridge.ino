/*  Custom UMI RTX 100 controller (part of the HSRDP project http://hack.rs/p/HSRDP)
 *  Author: @mentarus
 *  Last updated: 04 September 2013
 *  Prereqs: Built ROS arudino libs as per http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup
 */


//#define UMI_DEBUG 1 // toggle, uncomment line to enable
#define NO_PORTB_PINCHANGES // to indicate that port b will not be used for pin change interrupts (faster)
#define NO_PORTJ_PINCHANGES // to indicate that port J will not be used for pin change interrupts (faster)
//#define NO_PORTK_PINCHANGES // to indicate that port d will not be used for pin change interrupts (faster)


#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "Streaming.h"
#include "PinChangeInt.h"
#include "Encoder.h"
#include "MotorController.h"
#include "FastPID.h"


Encoder encoders[7] =
{
  Encoder(A8,23),//gripper
  Encoder(A9,25),//wrist1
  Encoder(A10,27),//wrist2
  Encoder(A11,29),//wrist yaw
  Encoder(A12,31),//elbow
  Encoder(A13,33),//shoulder
  Encoder(A14,35)//Z-axis
  };

MotorController motors[7] =
{
  MotorController(2,3),//gripper
  MotorController(4,5),//wrist1
  MotorController(6,7),//wrist2
  MotorController(8,9),//wrist yaw
  MotorController(10,11),//elbow
  MotorController(12,46),//shoulder
  MotorController(44,45, true)//Z-axis, this requires a PWM + direction(digital) control as opposed to two directional PWMs
  };
  
  //Motor driver enable lines for all but Z axis
  //int motorEnables[6] ={22,24,26,28,30,32};

unsigned long curent_time = millis();
unsigned long joint_timers[7] = {curent_time,curent_time,curent_time,curent_time,curent_time,curent_time,curent_time};
int encoder_tick_time[7]={0,0,0,0,0,0,0}; //number of milliseconds between encoder ticks, velocities derived on the host 

FastPID pids[7] =
{
  FastPID(1,0,0),//gripper
  FastPID(1,0,0),//wrist1
  FastPID(1,0,0),//wrist2
  FastPID(1,0,0),//wrist yaw
  FastPID(1,0,0),//elbow
  FastPID(1,0,0),//shoulder
  FastPID(1,0,0),//Zaxis
};

int positions[7];
int16_t targets[7];

unsigned int ranges[7]={1326,1344,1381,2301,4907,5295,3559}; // Ranges of motion for the corresponding joints


boolean led_on;
int led = 13;
long publisher_timer;

//The callback function that receives listens to new postition targets
void targetsCb( const std_msgs::Int16MultiArray& target_msg)
{
  for (int i = 0; i < 7; i++){
    targets[i] = target_msg.data[i];
  }
}

//Set up the ros node and publisher
std_msgs::Int16MultiArray joints_msg;
std_msgs::Int16MultiArray encoder_ticks_msg;
//std_msgs::Int16MultiArray pid_out_msg;

ros::Publisher pub_joints_state("arm_encoders", &joints_msg);
ros::Publisher pub_encoder_ticks_state("arm_encoder_tick_times", &encoder_ticks_msg);
//ros::Publisher pub_joints_state("arm_pid_outputs", &pid_out_msg);
ros::Subscriber<std_msgs::Int16MultiArray> sub("arm_encoder_targets", targetsCb );

ros::NodeHandle nh;

void setup()
{

  #ifdef UMI_DEBUG
  Serial.begin (57600);
  Serial << "Start" << endl;
  #endif

  //This is where we connect our encoders, corresponds to encoders[7]
  PCintPort::attachInterrupt(A8, doEncoderGripper, CHANGE);
  PCintPort::attachInterrupt(A9, doEncoderWrist1, CHANGE);
  PCintPort::attachInterrupt(A10, doEncoderWrist2, CHANGE);
  PCintPort::attachInterrupt(A11, doEncoderWristYaw, CHANGE);
  PCintPort::attachInterrupt(A12, doEncoderElbow, CHANGE);
  PCintPort::attachInterrupt(A13, doEncoderShoulder, CHANGE);
  PCintPort::attachInterrupt(A14, doEncoderZ, CHANGE);

  //Handle calibration on powerup
  //calibrateJoints();
  
  //Set the init position to -375 (ref RTX_Inside.pdf)
  encoders[6].setPosition(-375);

  //TODO: Figure why assigning an integer array pointer to targets didn't work, ugly "fix" for now
  targets[0] = 0;
  targets[1] = 0;
  targets[2] = 0;
  targets[3] = 0;
  targets[4] = 0;
  targets[5] = 0;
  targets[6] = -375;

  nh.initNode();
  nh.advertise(pub_joints_state);
  nh.advertise(pub_encoder_ticks_state);
  nh.subscribe(sub);


  pinMode(13, OUTPUT);
  
  //Motor driver enable lines
  for (int8_t i = 0; i < 6; i++){
    //pinMode(motorEnables[i], OUTPUT);
    //;digitalWrite(motorEnables[i], LOW); //Disable by default
  }

}

void loop()
{
  if (millis() > publisher_timer)
  {
    publisher_timer = millis() + 200; //publish at 5 hz

    //Heartbeat
    if(led_on)
    {
      digitalWrite(led, HIGH);
      led_on = 0;
    }
    else
    {
      digitalWrite(led, LOW);
      led_on = 1;
    }

    joints_msg.data_length = 7;
    joints_msg.data = readEncoders();
    pub_joints_state.publish(&joints_msg);

    joints_msg.data_length = 7;
    joints_msg.data = encoder_tick_time;
    pub_encoder_ticks_state.publish(&encoder_ticks_msg);
    updatePIDs();
    

  }
#ifndef UMI_DEBUG
  nh.spinOnce();
#endif
}

//Updates the encoder values and drives the motors as necessary
void processJoint(int8_t j)
{
  boolean velocity_dir = true; //Need to differentiate the speeds direction to get the velocity
  int old_enc_value = encoders[j].getPosition();
  
  encoders[j].update();
  
  unsigned long new_time  = millis();
  if ( encoders[j].getPosition() - old_enc_value < 0){
    encoder_tick_time[j] = (int)(new_time - joint_timers[j]);
  }else{
     encoder_tick_time[j] = -(int)(new_time - joint_timers[j]);
  }
  
  joint_timers[j] = new_time;
}

void doEncoderGripper()
{
  processJoint(0);
}

void doEncoderWrist1()
{
  processJoint(1);
}

void doEncoderWrist2()
{
  processJoint(2);
}

void doEncoderWristYaw()
{
  processJoint(3);
}

void doEncoderElbow()
{
  processJoint(4);
}

void doEncoderShoulder()
{
  processJoint(5);
}

void doEncoderZ()
{
  processJoint(6);
}


int* readEncoders()
{
#ifdef UMI_DEBUG
  //Serial << "encoder readings: ";
#endif
  for(int i=0; i < 7; i++)
  {
    positions[i] = encoders[i].getPosition();
    #ifdef UMI_DEBUG
    //Serial << i << ":" << encoders[0].getPosition() << " " << endl;
    #endif
  }

  return positions;
}


void updatePIDs(){
  
  for(int i=0; i < 7; i++){
    if(targets[i] != encoders[i].getPosition()){
      
      //TODO:Enable controller 
      digitalWrite(22, HIGH);
      
      //Old way of feeding the PID controller
      //int output = pids[i].update(targets[i], encoders[i].getPosition());
      int output = pids[i].update(/*TODO target speed*/, encoder_tick_time[i]);
      motors[i].setThrottle(-output);
      
      #ifdef UMI_DEBUG
//        Serial << "PID joint["<< i << "] target: " << targets[i] << " current: " << encoders[i].getPosition() << " output:"<< output << endl;
      #endif

    }else{
      motors[i].setThrottle(0);
      digitalWrite(22, LOW);
      //TODO:Disable controller
    }
  }
}

void calibrateJoints(){
  #ifdef UMI_DEBUG
    Serial << "Performing calibration" << endl;
  #endif
    for (int8_t i = 3; i < 4; i++){
      int current_sample;
      int previous_sample = encoders[i].getPosition();
      motors[i].setThrottle(127);
      
      int sample_period = 50;
      
      delay(sample_period);
      current_sample = encoders[i].getPosition();
      
      Serial << "First previous:" << previous_sample <<  "    Current:" << current_sample << endl;
      
      if (previous_sample != current_sample){
        #ifdef UMI_DEBUG
         // Serial << "Previous:" << previous_sample <<  "    Current:" << current_sample << endl;
        #endif
        while(previous_sample != current_sample){
          delay(sample_period);
          previous_sample = current_sample;
          current_sample = encoders[i].getPosition();
        }
      }
      
        
      motors[i].setThrottle(0);
      delay(sample_period);
      
      #ifdef UMI_DEBUG
        Serial << "Last Previous:" << current_sample <<  "    Current:" << encoders[i].getPosition() << endl;
      #endif

      encoders[i].setPosition(-ranges[i]);
      #ifdef UMI_DEBUG
        Serial << "Post calibration position:" << encoders[i].getPosition() << endl;
      #endif
    }
      #ifdef UMI_DEBUG
        while(true){
          delay(255);
        }
      #endif

}
