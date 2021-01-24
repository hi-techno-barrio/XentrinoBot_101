#ifndef ENCODER_LIB_H
#define ENCODER_LIB_H

#include "Arduino.h"
#include "../common/foc_utils.h"
#include "../common/time_utils.h"
#include "../common/base_classes/Sensor.h"

// sign funciton
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )
// utility defines
#define _2_SQRT3 1.15470053838
#define _1_SQRT3 0.57735026919
#define _SQRT3_2 0.86602540378
#define _SQRT2 1.41421356237
#define _120_D2R 2.09439510239
#define _PI_2 1.57079632679
#define _2PI 6.28318530718
#define _3PI_2 4.71238898038

/**
 *  Quadrature mode configuration structure
 */


class Encoder
{
 public:
    /**
    Encoder class constructor
    @param encA  encoder B pin
    @param encB  encoder B pin
    @param ppr  impulses per rotation  (cpr=ppr*4)
    @param index index pin number (optional input)
    */
    // !<  Enable quadrature mode CPR = 4xPPR OFF 
    // !<  Disable quadrature mode / CPR = PPR
    
    enum Quadrature{ON,OFF };
    Encoder(int encA, int encB , float ppr );
  
    /** encoder initialise pins */
    void init();
    /**
     *  function enabling hardware interrupts for the encoder channels with provided callback functions
     *  if callback is not provided then the interrupt is not enabled
     * 
     * @param doA pointer to the A channel interrupt handler function
     * @param doB pointer to the B channel interrupt handler function
     * @param doIndex pointer to the Index channel interrupt handler function
     * 
     */
    void enableInterrupts(void (*doA)() = nullptr, void(*doB)() = nullptr);
    
    /* Default Quadrature and Pull ups! */
    void Encoder::defaultEnc();
    //  Encoder interrupt callback functions
    /** A channel callback function */
    void call_intA();
    /** B channel callback function */
    void call_intB();
    
    
    // pins A and B
    int pinA; //!< encoder hardware pin A
    int pinB; //!< encoder hardware pin B
  
    // Encoder configuration
    Pullup pullup; //!< Configuration parameter internal or external pullups
    float cpr;//!< encoder cpr number

    // Abstract functions of the Sensor class implementation
    /** get current angle (rad) */
    float getAngle() override;
    /**  get current angular velocity (rad/s) */
    float getVelocity() override;
    /** 
     *  set current angle as zero angle 
     * return the angle [rad] difference
     */
    float initRelativeZero() override;
    /**
     * set index angle as zero angle
     * return the angle [rad] difference
     */
    float initAbsoluteZero() override;
    /**
     *  returns 0 if it has no index 
     * 0 - encoder without index
     * 1 - encoder with index 
     */
    int hasAbsoluteZero() override;
    /**
     * returns 0 if it does need search for absolute zero
     * 0 - encoder without index 
     * 1 - ecoder with index
     */
    int needsAbsoluteZeroSearch() override;

  private:
    Quadrature var_quadrature_;         // !< Configuration parameter enable or disable quadrature mode
    volatile long pulse_counter;   // !< current pulse counter
    volatile long pulse_timestamp; // !< last impulse timestamp in us
    volatile int A_active;         // !< current active states of A channel
    volatile int B_active;         // !< current active states of B channel
  
    // velocity calculation variables
    float prev_Th, pulse_per_second;
    volatile long prev_pulse_counter, prev_timestamp_us;
};


#endif
