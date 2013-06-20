

void setup(){
 for (int i=0; i=11; i++){
 pinMode(connectionPins[i],INPUT);
 }
  
}

void loop(){
 jointRead();
 jointUpdate();
  
}

