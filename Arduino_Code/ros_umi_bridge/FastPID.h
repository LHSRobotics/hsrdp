#ifndef __FASTPID_H__
#define __FASTPID_H__

#include "Arduino.h"


// Maximum value of variables

#define MAX_INT         0x7fff
#define MAX_LONG        0x7fffffffL
#define MAX_I_TERM      (MAX_LONG / 2)
#define SCALING_FACTOR 32


// Boolean values

#define FALSE           0
#define TRUE            1


class FastPID
{
    /*
      wraps PID setup and update functions in a class

      !!! NOTE : user must call the PID update method from an
      interrupt function himself
      */

private:

    //! Last process value, used to find derivative of process value.
    int16_t lastProcessValue;

    //! Summation of errors, used for integrate calculations
    int32_t sumError;

    //! The Proportional tuning constant, multiplied with SCALING_FACTOR
    int16_t P_Factor;

    //! The Integral tuning constant, multiplied with SCALING_FACTOR
    int16_t I_Factor;

    //! The Derivative tuning constant, multiplied with SCALING_FACTOR
    int16_t D_Factor;

    //! Maximum allowed error, avoid overflow
    int16_t maxError;

    //! Maximum allowed sumerror, avoid overflow
    int32_t maxSumError;

     //! Scaling factor
    int8_t ScalingFactor;

public:

    /*! \brief PID controller constructor.
     *  Initialise the variables used by the PID algorithm.
     *  \param p_factor  Proportional term.
     *  \param i_factor  Integral term.
     *  \param d_factor  Derivate term.
     */

    FastPID( int16_t p_factor, int16_t i_factor, int16_t d_factor)
    {


        // Start values for PID controller
        sumError = 0;
        lastProcessValue = 0;

        // Tuning constants for PID loop
        P_Factor = p_factor * SCALING_FACTOR;
        I_Factor = i_factor * SCALING_FACTOR;
        D_Factor = d_factor * SCALING_FACTOR;

        // Limits to avoid overflow
        maxError = MAX_INT / (P_Factor + 1);
        maxSumError = MAX_I_TERM / (I_Factor + 1);
    };

    /*! \brief PID control algorithm.
     *  Calculates output from setpoint, process value and PID status.
     *  \param setPoint  Desired value.
     *  \param processValue  Measured value.
     */

    int16_t update(int16_t setPoint, int16_t processValue)

    {

        int16_t error, p_term, d_term;
        int32_t i_term, ret, temp;

        error = setPoint - processValue;

        // Calculate Pterm and limit error overflow
        if (error > maxError)
        {
            p_term = MAX_INT;
        }

        else if (error < -maxError)
        {
            p_term = -MAX_INT;
        }

        else
        {
            p_term = P_Factor * error;
        }

        // Calculate Iterm and limit integral runaway

        temp = sumError + error;

        if(temp > maxSumError)
        {
            i_term = MAX_I_TERM;
            sumError = maxSumError;
        }

        else if(temp < -maxSumError)
        {
            i_term = -MAX_I_TERM;
            sumError = -maxSumError;
        }

        else
        {
            sumError = temp;
            i_term = I_Factor * sumError;
        }

        // Calculate Dterm
        d_term = D_Factor * (lastProcessValue - processValue);

        lastProcessValue = processValue;
        ret = (p_term + i_term + d_term) / SCALING_FACTOR;

        if(ret > MAX_INT)
        {
            ret = MAX_INT;
        }

        else if(ret < -MAX_INT)
        {
            ret = -MAX_INT;
        }

        return((int16_t)ret);
    }


    /*! \brief Resets the integrator.
     *  Calling this function will reset the integrator in the PID regulator.
     */

    void Reset_Integrator()
    {
        sumError = 0;
    }

};

#endif // e__FASTPID_H__
