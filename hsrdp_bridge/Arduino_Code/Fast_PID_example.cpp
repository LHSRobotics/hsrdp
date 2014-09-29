// Do not remove the include below
#include "FastPID.h"

FastPID myController(1,0,0);

//The setup function is called once at startup of the sketch
void setup()
{
	  //output over serial
		Serial.begin(9600);
        Serial.println("Starting");
}


void loop()
{
	  // Run PID calculations once every PID timer timeout
    pinMode(A0, INPUT);
    int input = analogRead(A0);
    int output = myController.update(300, input);

    Serial.print(input);
    Serial.print(" -> ");
    Serial.print(output);
    Serial.println("");
    delay(1000);
}
