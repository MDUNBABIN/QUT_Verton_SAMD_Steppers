/**
 * Verton Spin Pod Controller (SPC)  Stepper Driver Interface (SDI) : Verton_SPC_PCB_SDI_V1.h
 * - Header file for pin assignments and functions
 * ------------------------------------------------------------------------------------------
 * 
 * Written By: Matthew Dunbabin and Riki Lamont
 * Queensland University of Technology (QUT) 2020
 * 
 * DISCLAIMER: This software is for research and development purposes only for evaluating 
 * and debugging the Spin Pod Controller PCB Revision 1.0 functionality. This includes 
 * general operation, low-level gyro and stepper motor control, low-level emergency stop 
 * management, and gyro/beam spin control functions. No guarentees or responsibilities are 
 * given for its use beyond research evaluation at QUT on the laboratory-based prototype 
 * gyroscopically controlled beam.
 */

#pragma once

#include <Arduino.h>

#include <wiring_private.h>                  // pinPeripheral() function

#include <AccelStepper.h>
#include <MultiStepper.h>
#include <PString.h>

#define  QUT_MODE                           0
#define  ID                                 1 

/* Verbose parameters for debugging */
#define  VERBOSE_SAFETY                    0
#define  VERBOSE_CONTROLLER                0
#define  VERBOSE_ROLL_STEPPERS             0

#define  DEBUG_COMS                         SerialUSB

#define  encoder0PinA                       A0    // PA02
#define  encoder0PinB                       A1    // PB08
#define  STEPPER0_A_HALL                    6
#define  STEPPER0_B_HALL                    7

#define  encoder1PinA                       A2    // PB09
#define  encoder1PinB                       A3    // PA04
#define  STEPPER1_A_HALL                    0
#define  STEPPER1_B_HALL                    1 

#define  STEPPER0_EN_PIN                    13      
#define  STEPPER0_PULSE_PIN                 5 
#define  STEPPER0_DIR_PIN                   30
#define  STEPPER0_ERROR_PIN                 A4
             
#define  STEPPER1_EN_PIN                    31
#define  STEPPER1_PULSE_PIN                 10  
#define  STEPPER1_DIR_PIN                   38
#define  STEPPER1_ERROR_PIN                 A5
          
#define  SP_SERIAL_BAUD_RATE                115200          // (baud)
#define  DEBUG_SERIAL_BAUD                  115200          // (baud) Appears to have no effect for USB configuration
#define  INCLINOMETER_SERIAL_BAUD           9600            // (baud)
#define  DT_CONTROL_MS                      20              // (ms)
#define  DT_SERIAL_MS                       250             // (ms)
#define  DT_WATCHDOG_TIMEOUT_MS             2000             // (ms)
#define  DT_INCLINOMETER_UPDATE_MS          500             // (ms)

#define  MOTOR_MIN_COMMAND                  -100.0
#define  MOTOR_MAX_COMMAND                  100.0


#define  ANGLE_MAX_COMMAND                  90.0
#define  ANGLE_MIN_COMMAND                  -90.0

#define  STEPPER_MAX_SPEED                  3000            // (steps per second)
#define  CONTROLLER_MAX_SPEED               2500.0          // (steps per second)
#define  CONTROLLER_MIN_SPEED               -CONTROLLER_MAX_SPEED 

const int ROLL_LIMIT_STBD_MIN_PIN = STEPPER0_A_HALL;
const int ROLL_LIMIT_STBD_MAX_PIN = STEPPER0_B_HALL;
const int ROLL_LIMIT_PORT_MIN_PIN = STEPPER1_B_HALL;
const int ROLL_LIMIT_PORT_MAX_PIN = STEPPER1_A_HALL;

enum { PORT_GIMBAL = 0, STBD_GIMBAL = 1 };

/*
 * Communication parameters
 */
#define   MAX_SERIAL_PACKET_LENGTH      120

struct SERIAL_MESSAGE{
     char  msg[ MAX_SERIAL_PACKET_LENGTH ];
     uint8_t   syncCnt;
     uint8_t   pktLen;
     uint8_t   nextByte;
} ;

// Stepper parameters
struct STEPPER_PARAMS{
     int   errorStatus;
     int   enabled;
     float rollDeg;
     float rollRateDegPerSec;
     long  stepCount;
     float inclinRollDeg;
     int   limitSwitchMin;
     int   limitSwitchMax;
     float rollDegDmd;
     float rollRateDegPerSecDmd;
     float rollDmd;
     float timeSinceLastUpdate_sec;
     long  numMsgsRecieved = 0;
} ;

// Stepper parameters
struct CONTROLLER_PARAM {
     double kp = 2.0;
     double ki = 0.0;
     double kd = 0.0;
     double rateLimit = 700.0;            // Steps per second
     unsigned long previousTime = 0; 
     double cumulativeError = 0.0;
     double lastError = 0.0;
     float  input;
     float  setPoint;
     double lastOutput = 0.0;
     double maxRateDegPerSec;
} ;


enum {CALIBRATION_IDLE = 0,
      CALIBRATION_FIND_MIN_LIMIT = 1,
      CALIBRATION_FIND_MAX_LIMIT = 2,
      CALIBRATION_CALCULATE_OFFSETS = 3,
      CALIBRATION_MOVE_TO_ZERO_ANGLE = 4,
      CALIBRATION_COMPLETED = 5,
      CALIBRATION_ERROR = 6};


// Gimbal roll angle calibration parameters
struct CALIBRATION_PARAMS {
     int    mode_port = CALIBRATION_IDLE;
     int    mode_stbd = CALIBRATION_IDLE;
     double timeSinceStart_sec = 0.0;
     long   stepCountStart_port;
     long   stepCountMinLimit_port;
     long   stepCountMaxLimit_port;
     long   stepCountStart_stbd;
     long   stepCountMinLimit_stbd;
     long   stepCountMaxLimit_stbd;
     long   stepOffset_port = 0;
     long   stepOffset_stbd = 0;
};


extern AccelStepper stepper0, stepper1; 

extern STEPPER_PARAMS rollStepperStbd, rollStepperPort;

extern CONTROLLER_PARAM rollControllerStbd, rollControllerPort;

extern CALIBRATION_PARAMS gimbal_calib;

// State machine variables
enum {CONTROL_MODE_IDLE = 0, 
      CONTROL_MODE_DRIVE_SPEED = 1, 
      CONTROL_MODE_DRIVE_ANGLE = 2, 
      CONTROL_MODE_MANUAL = 3, 
      CONTROL_MODE_CALIBRATE_GIMBAL = 4};

extern uint8_t  control_mode;
enum {DISABLE = 0, ENABLE = 1};
enum {notInitialised = 0, Initialised = 1};

extern float MotorCmd0, MotorCmd1;

enum {GIMBAL_ERROR_NONE = 0,
      GIMBAL_ERROR_MIN_LIMIT = 1,
      GIMBAL_ERROR_MAX_LIMIT = 2,
      GIMBAL_ERROR_INCLINOMETER_TIMEOUT = 3};

// timer and debug varibles
extern unsigned long prevMillisControl, prevMillisSerial;
extern float    timeSinceLastWatchdogMsg_sec ;
extern int      loopCnt,verbose_mode ;

// Status message variables
extern char      statusBuffer[80];
extern PString   statusPkt;
extern SERIAL_MESSAGE  spcMsg;
extern SERIAL_MESSAGE  usbMsg;

//void   SERCOM0_Handler( void );
float  calculate_roll_angle( long encoderPos );
int    reset_roll_angle_to_inclinometer( int id, float offset );
double computePID( struct CONTROLLER_PARAM *controller );
void   zeroPID( struct CONTROLLER_PARAM *controller );
int    drive_starboard_stepper( float stepSpeed );
int    drive_port_stepper( float stepSpeed );
void   disable_starboard_stepper( void );
void   disable_port_stepper( void );
float  map_float( float x, float in_min, float in_max, float out_min, float out_max);

//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler( void );

/* 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */
// Configures the TC to generate output events at the sample frequency.
// Configures the TC in Frequency Generation mode, with an event output once
// each time the audio sample frequency period expires.
 void tcConfigure( void );

// Function that is used to check if TC5 is done syncing
// returns true when it is done syncing
bool tcIsSyncing( void );
//This function enables TC5 and waits for it to be ready
void tcStartCounter( void );
//Reset TC5 
void tcReset( void );
//disable TC5
void tcDisable( void );
