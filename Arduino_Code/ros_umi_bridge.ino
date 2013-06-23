//#define DEBUG 1 //Debug toggle, uncomment line to enable

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "Streaming.h"
#include "Encoder.h"
#include "MotorController.h"

//Set up the ros node and publisher
std_msgs::Int16MultiArray joints_msg;
ros::Publisher pub_joints_state("arm_encoders", &joints_msg);
ros::NodeHandle nh;


Encoder encoders[7] = {
  Encoder(22,23),//gripper, not used yet as not enough interrupt pins, will need to think of something
  Encoder(24,25),//wrist1
  Encoder(26,27),//wrist2
  Encoder(28,29),//wrist yaw
  Encoder(30,31),//elbow
  Encoder(32,33),//shoulder
  Encoder(34,35)//Z-axis
};

MotorController motors[7] = {
  MotorController(2,3),//gripper, not used yet as not enough interrupt pins, will need to think of something
  MotorController(4,5),//wrist1
  MotorController(6,7),//wrist2
  MotorController(8,9),//wrist yaw
  MotorController(10,11),//elbow
  MotorController(12,12),//shoulder
  MotorController(44,45)//Z-axis TODO(this is requires a custom motor controler to talk to the zaxis motor) 
};

int positions[7];
int targets[7] ={0,0,0,0,0,-375,0};

boolean led_on;
int led = 13;
long publisher_timer;


void setup() {
  
    //Set the init position to -3554 (RTX inside p19)
    encoders[6].setPosition(-375);  
  
    nh.initNode();
    nh.advertise(pub_joints_state);
  
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
    publisher_timer = millis() + 20; //publish at 50 hz second
      
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

