#ifndef PID_v1_h
#define PID_v1_h

class PID
{
  public:

    //Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

    //commonly used functions **************************************************************************
    PID(volatile float*, volatile float*, volatile float*,        // * constructor.  links the PID to the Input, Output, and
        volatile float, volatile float, volatile float, int);     //   Setpoint.  Initial tuning parameters are also set here

    void SetMode(bool);               // * sets PID to either Manual (0) or Auto (non-0)

    bool Compute();                       // * performs the PID calculation.  it should be
    //   called every time loop() cycles. ON/OFF and
    //   calculation frequency can be set using SetMode
    //   SetSampleTime respectively

    void SetOutputLimits(float, float); // * clamps the output to a specific range. 0-255 by default, but
    //   it's likely the user will want to change this depending on
    //   the application



    //available but not commonly used functions ********************************************************
    void SetTunings(float, float,       // * While most users will set the tunings once in the
                    float);         	    //   constructor, this function gives the user the option
    //   of changing tunings during runtime for Adaptive control

    void SetControllerDirection(int);	  // * Sets the Direction, or "Action" of the controller. DIRECT
    //   means the output will increase when error is positive. REVERSE
    //   means the opposite.  it's very unlikely that this will be needed
    //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which
    //   the PID calculation is performed.  default is 100



    //Display functions ****************************************************************
    float GetKp();						  // These functions query the pid for interal values.
    float GetKi();						  //  they were created mainly for the pid front-end,
    float GetKd();						  // where it's important to know what is actually
    int GetMode();						  //  inside the PID.
    int GetDirection();					  //

  private:
    void Initialize();

    float dispKp;				// * we'll hold on to the tuning parameters in user-entered
    float dispKi;				//   format for display purposes
    float dispKd;				//

    volatile float kp;                  // * (P)roportional Tuning Parameter
    volatile float ki;                  // * (I)ntegral Tuning Parameter
    volatile float kd;                  // * (D)erivative Tuning Parameter

    int controllerDirection;

    volatile float *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    volatile float *myOutput;             //   This creates a hard link between the variables and the
    volatile float *mySetpoint;           //   PID, freeing the user from having to constantly tell us
    //   what these values are.  with pointers we'll just know.
    volatile float input;
    volatile float error;
    volatile float dInput;
    volatile float output;

    volatile float outputSum, lastInput;

    unsigned long SampleTime;
    volatile float outMin, outMax;
    volatile bool inAuto;
};
#endif

