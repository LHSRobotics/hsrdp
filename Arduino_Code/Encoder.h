#ifndef __ENCODER_H__
#define __ENCODER_H__

#include "Arduino.h"

class Encoder {
  /*  
    wraps encoder setup and update functions in a class

    !!! NOTE : user must call the encoders update method from an
    interrupt function himself! i.e. user must attach an interrupt to the
    encoder pin A and call the encoder update method from within the 
    interrupt

    uses Arduino pullups on A & B channel outputs
    turning on the pullups saves having to hook up resistors 
    to the A & B channel outputs 
    */

public:

    // constructor : sets pins as inputs and turns on pullup resistors

    Encoder( int8_t PinA, int8_t PinB) : pin_a ( PinA), pin_b( PinB ) {
        // set pin a and b to be input 
        pinMode(pin_a, INPUT); 
        pinMode(pin_b, INPUT); 
        // and turn on pullup resistors
        digitalWrite(pin_a, HIGH);    
        digitalWrite(pin_b, HIGH);
        position = 0;        
    };

    // call this from your interrupt function

    void update () {
        if (digitalRead(pin_a)) digitalRead(pin_b) ? position++ : position--;
        else digitalRead(pin_b) ? position-- : position++;
    };

    // returns current position

    int getPosition () { return position; };

    // set the position value

    void setPosition ( const int p) { position = p; };

private:

    int position;

    int8_t pin_a;

    int8_t pin_b;
};

#endif // __ENCODER_H__
