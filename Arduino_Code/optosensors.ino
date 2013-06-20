//##### Opto-sensors UMI ######

//Conection Pins
int connectionPins[14]={10,11,12,13,14,15,16,17,18,19,20,21,22,23};
// 10,11 zed
// 12,13 shoulder
// 14,15 elbow
// 16,17 wrist yaw
// 18,19 wrist1
// 20,21 wrist2
// 22,23 wripper

//CurrentState in counts 
//{zed, shoulder, elbow, wrist yaw, wrist1, wrist2, wripper}
int currentState[7] ={-375,0,0,0,0,0,0};

// Record of opto-sensors position 
//matrix of 2 steps by 7 joints (x2 opto-sensors each joint)
boolean optoRecord[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//actual lecture of the opto-sensors
boolean optoActual[14]={0,0,0,0,0,0,0,0,0,0,0,0,0,0};

void jointRead(){
   for (int i = 0; i>13; i++){
     optoActual[i] = digitalRead(connectionPins[i+10]);
   }
}

void jointUpdate(){
  for (int i = 0; i>13; i+2){
    //search for movement
   if(optoActual[i] != optoRecord[i] 
     && optoRecord[i] == 1){
       if(optoRecord[i+1]==0){
         //it moves torward B
         currentState[i/2] -= 1;
       }else if(optoRecord[i+1]==1){
         //it moves torward A
          currentState[i/2] += 1;
       }
     }else if(optoActual[i] != optoRecord[i] 
     && optoRecord[i] == 0){
       if(optoRecord[i+1]==1){
         //it moves torward B
          currentState[i/2] -= 1;
       }else if(optoRecord[i+1]==0){
         //it moves torward A
         currentState[i/2] += 1;
       }
     }else if (optoActual[i+1] != optoRecord[i+1] 
     && optoRecord[i+1] == 1){
       if(optoRecord[i]==1){
         //it moves torward B
          currentState[i/2] -= 1;
       }else if(optoRecord[i]==0){
         //it moves torward A
         currentState[i/2] += 1;
       }
     }else if (optoActual[i+1] != optoRecord[i+1] 
     && optoRecord[i+1] == 0){
       if(optoRecord[i]==0){
         //it moves torward B
          currentState[i/2] -= 1;
       }else if(optoRecord[i]==1){
         //it moves torward A
         currentState[i/2] += 1;
       }
     }
   }
}

