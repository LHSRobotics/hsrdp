//#define DEBUG 1 //Debug toggle

#include "Encoder.h"
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#include "Streaming.h"

//Set up the ros node and publisher
std_msgs::Int16MultiArray joints_msg;
ros::Publisher pub_joints_state("arm_encoders", &joints_msg);
ros::NodeHandle nh;


Encoder encoders[7] = {
  Encoder(1,2),//gripper, not used yet as not enough interrupt pins, will need to think of something
  Encoder(2,4),//wrist1
  Encoder(3,6),//wrist2
  Encoder(18,22),//wrist yaw
  Encoder(19,23),//elbow
  Encoder(20,24),//shoulder
  Encoder(21,25)//Z-axis
};
int positions[7];
boolean led_on;
int led = 13;
long publisher_timer;


void setup() {
  
    //Set the init position to -3554 (RTX inside p19)
    encoders[5].setPosition(-3554);  
  
    nh.initNode();
    nh.advertise(pub_joints_state);
  

    attachInterrupt(2, doEncoderWrist1, CHANGE);
    attachInterrupt(3, doEncoderWrist2, CHANGE); 
    /*attachInterrupt(18, doEncoderWristYaw, CHANGE);
    attachInterrupt(19, doEncoderElbow, CHANGE);
    attachInterrupt(20, doEncoderShoulder, CHANGE);
    attachInterrupt(21, doEncoderZ, CHANGE);
    */
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

void doEncoderWrist1(){
    encoders[0].update();
}    

void doEncoderWrist2(){
    encoders[1].update();
}    

void doEncoderWristYaw(){
    encoders[2].update();
}

void doEncoderElbow(){
    encoders[3].update();
}

void doEncoderShoulder(){
    encoders[4].update();
}

void doEncoderZ(){
    encoders[5].update();
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

