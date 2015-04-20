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

    // constructor : sets PWM pins as outputs, 
    // 3rd param changes the second pin to a direction pin, so that the PWM is only used on the first param pin

    MotorController( int8_t PinA, int8_t PinB, boolean one_pwm = false) : pin_a ( PinA), pin_b( PinB ) {
        // set pin a and b to be output 
        pinMode(pin_a, OUTPUT); 
        pinMode(pin_b, OUTPUT); 
        one_pwm_type = one_pwm;
    };

 
    // returns current throttle level

    int getThrottle () { return throttle; };

    // set the position value
    void setThrottle ( const int throttle_raw) {
       throttle = constrain(throttle_raw, -254, 254);
       if(throttle_raw > 0)
       {
         if(one_pwm_type)
         {
           analogWrite(pin_a,abs(throttle));
           digitalWrite(pin_b, HIGH);
         } else
         {
          analogWrite(pin_b, 0);
          analogWrite(pin_a, abs(throttle));
         }
          
       }else if(throttle_raw < 0){
         if(one_pwm_type)
         {
           analogWrite(pin_a,abs(throttle));
           digitalWrite(pin_b, LOW);
         }else{ 
          analogWrite(pin_b, abs(throttle));
          analogWrite(pin_a, 0);
         }
       }else{
         analogWrite(pin_a, 0);
         analogWrite(pin_b, 0);
       }
     
    };

private:

    int throttle;

    int8_t pin_a;

    int8_t pin_b;
    
    boolean one_pwm_type;
};

#endif // __MOTORCONTROLLER_H__

