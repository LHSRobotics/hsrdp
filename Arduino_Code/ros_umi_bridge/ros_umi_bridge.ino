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


Encoder encoders[7] = {
  Encoder(A8,23),//gripper
  Encoder(A9,25),//wrist1
  Encoder(A10,27),//wrist2
  Encoder(A11,29),//wrist yaw
  Encoder(A12,31),//elbow
  Encoder(A13,33),//shoulder
  Encoder(A14,35)//Z-axis
};

MotorController motors[7] = {
  MotorController(2,3),//gripper
  MotorController(4,5),//wrist1
  MotorController(6,7),//wrist2
  MotorController(8,9),//wrist yaw
  MotorController(10,11),//elbow
  MotorController(12,46),//shoulder
  MotorController(44,45, true)//Z-axis TODO(this is requires a custom motor controler to talk to the zaxis motor) 
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
ros::Subscriber<std_msgs::Int16MultiArray> sub("arm_encoder_targets", targetsCb );

ros::NodeHandle nh;

void setup() {
  
    //Set the init position to -375 (ref RTX_Inside.pdf)
    encoders[6].setPosition(-375);  
    
    int defaultTargets[] = {0,0,0,0,0,-375,0};
    targets = defaultTargets;
    nh.initNode();
    nh.advertise(pub_joints_state);
    nh.subscribe(sub);
     
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
    //attachInterrupt(0, doEncoderShoulder, CHANGE);
    PCintPort::attachInterrupt(A14, doEncoderZ, CHANGE);
    
    pinMode(13, OUTPUT);
       

} 

void loop(){
  if (millis() > publisher_timer) {
    publisher_timer = millis() + 200; //publish at 5 hz
    
        //Heartbeat
    if(led_on){
     digitalWrite(led, HIGH);
     led_on = 0;
    }else{
     digitalWrite(led, LOW);
     led_on = 1;
    }
    for(int i=0; i < 7; i++)
    {
       moveJoint(i);
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
    moveJoint(j);
    
}
void moveJoint(int8_t j)
{
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
    #ifdef UMI_DEBUG
    Serial << "encoder readings: ";
    #endif
    for(int i=0; i < 7; i++)
    {
       positions[i] = encoders[i].getPosition();
       #ifdef UMI_DEBUG
       Serial << i << ":" << encoders[0].getPosition() << " ";
       #endif
    }
       #ifdef UMI_DEBUG
       Serial << endl;
       #endif
    return positions;
}

