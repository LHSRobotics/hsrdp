// Do not remove the include below
#include "pid_example.h"


/*! \brief P, I and D parameter values
 *
 * The K_P, K_I and K_D values (P, I and D gains)
 * need to be modified to adapt to the application at hand
 */
//! \xrefitem todo "Todo" "Todo list"
#define K_P     1.00
//! \xrefitem todo "Todo" "Todo list"
#define K_I     0.00
//! \xrefitem todo "Todo" "Todo list"
#define K_D     0.00

/*! \brief Flags for status information
 */
struct GLOBAL_FLAGS {
  //! True when PID control loop should run one time
  uint8_t pidTimer:1;
  uint8_t dummy:7;
} gFlags = {0, 0};

//! Parameters for regulator
struct PID_DATA pidData;
int16_t referenceValue, measurementValue, inputValue;


/*! \brief Sampling Time Interval
 *
 * Specify the desired PID sample time interval
 * With a 8-bit counter (255 cylces to overflow), the time interval value is calculated as follows:
 * TIME_INTERVAL = ( desired interval [sec] ) * ( frequency [Hz]/64 ) / 2*255
 * In default arduino phase-correct pwm mode the TOV is triggerd only at bottom so double the cycle
 * default arduino setup in wiring.c is prescaler=64
 */TODO: check why 1000 gives 1.5 second interval, expect much longer
//! \xrefitem todo "Todo" "Todo list"
#define TIME_INTERVAL   1000

/*! \brief Timer interrupt to control the sampling interval
 */
ISR(TIMER1_OVF_vect)
{
  
  static uint16_t i = 0;
  if(i < TIME_INTERVAL)
    i++;
  else{
    gFlags.pidTimer = TRUE;
    i = 0;
  }
}



/*! \brief Init of PID controller demo
 */
void Init(void)
{

 Serial.println("starting");
  pid_Init(K_P * SCALING_FACTOR, K_I * SCALING_FACTOR , K_D * SCALING_FACTOR , &pidData);


//From atmel:
//  // Set up timer, enable timer/counte 0 overflow interrupt
//  TCCR2A = (1<<CS00);
//  TIMSK2 = (1<<TOIE2);
//  TCNT2 = 0;
  // From:
  noInterrupts();           // disable all interrupts
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();
}

/*! \brief Read reference value.
 *
 * This function must return the reference value.
 * May be constant or varying
 */
int16_t Get_Reference(void)
{
  return 8;
}

/*! \brief Read system process value
 *
 * This function must return the measured data
 */
int16_t Get_Measurement(void)
{
  return 4;
}

/*! \brief Set control input to system
 *
 * Set the output from the controller as input
 * to system.
 */
void Set_Input(int16_t inputValue)
{
	Serial.print("Output set:");
	Serial.println(inputValue);
}






//The setup function is called once at startup of the sketch
void setup()
{
	  //output over serial
		Serial.begin(9600);

// Add your initialization code here
	  Init();



}

// The loop function is called in an endless loop
void loop()
{
	  // Run PID calculations once every PID timer timeout
	    if(gFlags.pidTimer)
	    {
	      referenceValue = Get_Reference();
	      measurementValue = Get_Measurement();

	      inputValue = pid_Controller(referenceValue, measurementValue, &pidData);

	      Set_Input(inputValue);

	      gFlags.pidTimer = FALSE;
	    }


}
