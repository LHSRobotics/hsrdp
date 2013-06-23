#ifndef __MOTORCONTROLLER_H__
#define __MOTORCONTROLLER_H__

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"

#else

#include "WProgram.h"

#endif

class MotorController {
  /*  
  Motor Controller for H-bridges.
  Constructor takes the + and the - motor PWM pins respectively
  
    */

public:

    // constructor : sets PWM pins as outputs

    MotorController( int8_t PinA, int8_t PinB) : pin_a ( PinA), pin_b( PinB ) {
        // set pin a and b to be output 
        pinMode(pin_a, OUTPUT); 
        pinMode(pin_b, OUTPUT); 
    };

 
    // returns current throttle level

    int getThrottle () { return throttle; };

    // set the position value
    void setThrottle ( const int p) {
       throttle = constrain(p, -254, 254);
       if(p > 0)
       {
          analogWrite(pin_b, 0);
          analogWrite(pin_a, throttle);
          
       }else if(p < 0){
          analogWrite(pin_b, abs(throttle));
          analogWrite(pin_a, 0);
       }else{
         analogWrite(pin_a, 0);
         analogWrite(pin_b, 0);
       }
     
    };

private:

    int throttle;

    int8_t pin_a;

    int8_t pin_b;
};

#endif // __MOTORCONTROLLER_H__
