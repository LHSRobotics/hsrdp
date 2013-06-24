//#define DEBUG 1 //Debug toggle, uncomment line to enable

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "Streaming.h"
#include "Encoder.h"
#include "MotorController.h"


Encoder encoders[7] = {
  Encoder(22,23),//gripper
  Encoder(24,25),//wrist1
  Encoder(26,27),//wrist2
  Encoder(28,29),//wrist yaw
  Encoder(30,31),//elbow
  Encoder(32,33),//shoulder
  Encoder(34,35)//Z-axis
};

MotorController motors[7] = {
  MotorController(2,3),//gripper
  MotorController(4,5),//wrist1
  MotorController(6,7),//wrist2
  MotorController(8,9),//wrist yaw
  MotorController(10,11),//elbow
  MotorController(12,12),//shoulder
  MotorController(44,45)//Z-axis TODO(this is requires a custom motor controler to talk to the zaxis motor) 
};

int positions[7];
int* targets;

boolean led_on;
int led = 13;
long publisher_timer;

//The callback function that receives listens to new postition targets
void targetsCb( const std_msgs::Int16MultiArray& target_msg){
  targets = target_msg.data;
}

//Set up the ros node and publisher
std_msgs::Int16MultiArray joints_msg;
ros::Publisher pub_joints_state("arm_encoders", &joints_msg);
ros::Subscriber<std_msgs::Int16MultiArray> sub("arm_encoders_targets", targetsCb );

ros::NodeHandle nh;

void setup() {
  
    //Set the init position to -375 (ref RTX_Inside.pdf)
    encoders[6].setPosition(-375);  
    
    int defaultTargets[] = {0,0,0,0,0,-375,0};
    targets = defaultTargets;
    nh.initNode();
    nh.advertise(pub_joints_state);
    nh.subscribe(sub);
  
    //This is where we connect our encoders, corresponds to encoders[7]
    attachInterrupt(22, doEncoderGripper, CHANGE);
    attachInterrupt(24, doEncoderWrist1, CHANGE);
    attachInterrupt(26, doEncoderWrist2, CHANGE); 
    attachInterrupt(28, doEncoderWristYaw, CHANGE);
    attachInterrupt(30, doEncoderElbow, CHANGE);
    attachInterrupt(32, doEncoderShoulder, CHANGE);
    attachInterrupt(34, doEncoderZ, CHANGE);

    pinMode(13, OUTPUT);
       
    #ifdef DEBUG
    Serial.begin (57600);
    Serial << "Start" << endl;
    #endif
} 

void loop(){
  if (millis() > publisher_timer) {
    publisher_timer = millis() + 20; //publish at 50 hz
      
      //Heartbeat
      if(led_on){
       digitalWrite(led, HIGH);
       led_on = 0;
      }else{
       digitalWrite(led, LOW);
       led_on = 1;
      }
      joints_msg.data_length = 7;
      joints_msg.data = readEncoders();
      pub_joints_state.publish(&joints_msg);
      
    }
    nh.spinOnce();
}

//Updates the encoder values and drives the motors as necessary
void processJoint(int8_t j)
{
    encoders[j].update();
    motors[j].setThrottle(encoders[j].getPosition()-targets[j]);
}

void doEncoderGripper(){
    processJoint(0);
}    

void doEncoderWrist1(){
    processJoint(1);
}    

void doEncoderWrist2(){
    processJoint(2);
}    

void doEncoderWristYaw(){
    processJoint(3);
}

void doEncoderElbow(){
    processJoint(4);
}

void doEncoderShoulder(){
    processJoint(5);
}

void doEncoderZ(){
    processJoint(6);
}


int* readEncoders(){
    #ifdef DEBUG
    Serial << "encoder readings: ";
    #endif
    for(int i=0; i < 7; i++)
    {
       positions[i] = encoders[i].getPosition();
       #ifdef DEBUG
       Serial << i << ":" << encoders[0].getPosition() << " ";
       #endif
    }
       #ifdef DEBUG
       Serial << endl;
       #endif
    return positions;
}

