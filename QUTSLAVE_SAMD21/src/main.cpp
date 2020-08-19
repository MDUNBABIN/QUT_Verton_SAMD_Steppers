/**
 * Verton Spin Pod Controller (SPC) Stepper Driver Interface (SDI) : Main.cpp
 * - Laboratory Prototype Board Controller (PBC)
 * ---------------------------------------------------------------------------
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




// TODO
//  - verify the checksum on the inclinometers
//  - set CHECK NMEA_CHECKSUM to 1
//  - Set parameters in EEPROM
//  - Set what constitutes a limit switch when using inclinometers




#include <Arduino.h>
#include "Verton_SPC_PBC_SDI_V1.h"
#include "Verton_SDI_IO.h"
#include "Verton_SDI_COMS.h"
#include "Verton_Inclinometer.h"

AccelStepper stepper0 = AccelStepper( AccelStepper::DRIVER, STEPPER0_PULSE_PIN, STEPPER0_DIR_PIN );      // Starboard gimbal
AccelStepper stepper1 = AccelStepper( AccelStepper::DRIVER, STEPPER1_PULSE_PIN, STEPPER1_DIR_PIN );      // Port gimbal

STEPPER_PARAMS rollStepperStbd, rollStepperPort;

CONTROLLER_PARAM rollControllerStbd, rollControllerPort;

CALIBRATION_PARAMS gimbal_calib;

uint8_t control_mode;
float   MotorCmd0 = 0.0;
float   MotorCmd0Last = 0.0;
float   MotorCmd1 = 0.0;
float   MotorCmd1Last = 0.0;

unsigned long prevMillisControl,prevMillisSerial = 0;
unsigned long prevMillisInclinometer;
float timeSinceLastWatchdogMsg_sec = 0.0;
int   loopCnt = 0;
int   verbose_mode = 0;

SERIAL_MESSAGE  spcMsg;
SERIAL_MESSAGE  usbMsg;
char     statusBuffer[80];
PString  statusPkt = PString( statusBuffer, sizeof( statusBuffer ) );



/**
 * Initialisation Routine
 */
void setup() {
     /**
      * Assign the interrupt pins to inputs for the Roll motors
      */
     roll_motor_init_steppers();

     /* Assign pins 8 & 9 SERCOM Alternate functionality used for Serial Port0 */
     pinPeripheral( 9, PIO_SERCOM_ALT );
     pinPeripheral( 8, PIO_SERCOM_ALT );
  
     /* Set the serial communciation baud rate */
     SPC_COMS.begin( SP_SERIAL_BAUD_RATE );
     STARBOARD_COMS.begin( INCLINOMETER_SERIAL_BAUD );
     PORT_COMS.begin( INCLINOMETER_SERIAL_BAUD );
     DEBUG_COMS.begin( DEBUG_SERIAL_BAUD );

     /*
      * Set up the starboard stepper motor controller (stepper0)
      */ 
     stepper0.setMinPulseWidth( 3 );
     stepper0.setEnablePin( STEPPER0_EN_PIN );
     if ( QUT_MODE ) {
          stepper0.setPinsInverted( false, true, true );
     } else {
          stepper0.setPinsInverted( false, true, false );         
     }
     pinMode( STEPPER0_ERROR_PIN, INPUT ); 
     // pinMode( STEPPER0_EN_PIN, OUTPUT );
     // digitalWrite( STEPPER0_EN_PIN,HIGH);
     stepper0.setMaxSpeed( STEPPER_MAX_SPEED );   
     stepper0.setAcceleration( 1000 );

     /*
      * Set up the port stepper motor controller (stepper1)
      */
     stepper1.setMinPulseWidth( 3 );
     stepper1.setEnablePin( STEPPER1_EN_PIN );
     if ( 0 ) { // QUT MODE
          stepper1.setPinsInverted( false, true , true );
     } else {
         stepper1.setPinsInverted( false, true , false );
     }
     pinMode( STEPPER1_ERROR_PIN, INPUT );
     // pinMode( STEPPER1_EN_PIN, OUTPUT );
     // digitalWrite( STEPPER1_EN_PIN,HIGH );
     stepper1.setMaxSpeed( STEPPER_MAX_SPEED ); 
     stepper1.setAcceleration( 1000 );

     /*
      * Enable the stepper motor controllers
      */
     stepper0.enableOutputs();
     stepper1.enableOutputs();

     /*
      * initialise all the timing variables, and set watchdog to an invalid update
      */
     prevMillisSerial = millis();
     prevMillisControl = millis();
     prevMillisInclinometer = millis();
     timeSinceLastWatchdogMsg_sec = 1.0 * DT_WATCHDOG_TIMEOUT_MS / 1000.0 + 1.0;

     DEBUG_COMS.println( "Verton SpinPod Stepper Motor Drive" );
     tcConfigure();             // configure the timer to run at <sampleRate> Hertz
     tcStartCounter();          // starts the timer
     /*
      * add a small delay to ensure the unit does not start writing to any serial ports
      */
     delay( 100 );
     DEBUG_COMS.println( " Initialisation Complete" );
}



/**
 * Main control loop. Reads the serial port for incoming command messages. Runs the gimbal angle control
 * logic at a control rate of DT_CONTROL_MS, and periodically sends a status message to the serial
 * port at DT_SERIAL_MS.
 */
void loop() {

     /* 
      * read the incoming serial packets from the Spin Pod Controller 
      */
     if ( read_spc_interface() == 1 ) {
          if ( process_spc_packet() == 1 ) {}
     }    

     /* 
      * read the incoming serial packets from the USB debug port 
      */
     if ( read_usb_interface() == 1 ) {
          if ( process_usb_packet() == 1 ) {}
     }    

     /*
      * read the incoming serial packets from the port inclinometer
      */
     if ( inclinometer_port_read_angle_message() == 1 ) {
          if ( inclinometer_port_process_message() == 1 ) {}
     }

     /*
      * read the incoming serial packets from the starboard inclinometer
      */
     if ( inclinometer_starboard_read_angle_message() == 1 ) {
          if ( inclinometer_starboard_process_message() == 1 ) {}
     }

     /**
      * Check the roll limits against the maximum allowable angle
      */
     inclinometer_check_roll_limits();
     
     /*
      * Sample rate controlled process for the stepper motors
      */
     if ( ( millis() - prevMillisControl ) >= DT_CONTROL_MS ) {
          /* update the control timer */
          prevMillisControl = millis();

          /* update the watchdog timeout in seconds */
          timeSinceLastWatchdogMsg_sec += 1.0 * ( DT_CONTROL_MS ) / 1000.0;
          roll_motor_read_steppers();
          
          /* get the current gimbal angles */
          rollStepperStbd.rollDeg = calculate_roll_angle( rollStepperStbd.stepCount );
          if ( 0 ) { // QUT MODE
               rollStepperPort.rollDeg = -calculate_roll_angle( rollStepperPort.stepCount );
          } else {
               rollStepperPort.rollDeg = calculate_roll_angle( rollStepperPort.stepCount );
          }

          /*  control logic state machine */
          switch ( control_mode) {
               case CONTROL_MODE_IDLE:
                    // Disable the starboard stepper and zero the controller parameters
                    zeroPID( &rollControllerStbd );
                    disable_starboard_stepper();
                    // Disable the port stepper and zero the controller parameters
                    zeroPID( &rollControllerPort );
                    disable_port_stepper();
                    if ( VERBOSE_SAFETY )  DEBUG_COMS.println( "IDLE" ); 
                    break;
               
               case CONTROL_MODE_DRIVE_SPEED:
                    /* Check that the watchdog is being tickled */
                    if ( timeSinceLastWatchdogMsg_sec > (1.0 * DT_WATCHDOG_TIMEOUT_MS / 1000.0 ) ) {
                         if ( VERBOSE_SAFETY )  DEBUG_COMS.println( "Serial Watchdog Timeout" ); 
                         rollStepperStbd.rollRateDegPerSecDmd = 0.0;
                         disable_starboard_stepper();
                         rollStepperPort.rollRateDegPerSecDmd = 0.0;
                         disable_port_stepper();
                         control_mode = CONTROL_MODE_IDLE;
                         break;
                    }
                    /* 
                     * Drive stepper motor 0 (Starboard) 
                     */
                    if ( !rollStepperStbd.enabled ) {
                         rollStepperStbd.enabled = ENABLE;
                    }
                    MotorCmd0 = map_float( rollStepperStbd.rollRateDegPerSecDmd, -100.0, 100.0, -STEPPER_MAX_SPEED, STEPPER_MAX_SPEED );
                    if ( (rollStepperStbd.errorStatus = drive_starboard_stepper( MotorCmd0 )) != GIMBAL_ERROR_NONE ) {
                         MotorCmd0 = 0.0;
                    }
                    /* 
                     * Drive stepper motor 1 (Port) 
                     */
                    if ( !rollStepperPort.enabled ) {
                         rollStepperPort.enabled = ENABLE;
                    }
                    MotorCmd1 = map_float( rollStepperPort.rollRateDegPerSecDmd, -100.0, 100.0, -STEPPER_MAX_SPEED, STEPPER_MAX_SPEED );
                    if ( (rollStepperPort.errorStatus = drive_port_stepper( MotorCmd1 )) != GIMBAL_ERROR_NONE ) {
                         MotorCmd1 = 0.0;
                    }
                    break;   

               case CONTROL_MODE_DRIVE_ANGLE:
                    /* 
                     * Check that the watchdog is being tickled 
                     */
                    if ( timeSinceLastWatchdogMsg_sec > (1.0 * DT_WATCHDOG_TIMEOUT_MS / 1000.0 ) ) {
                         if ( VERBOSE_SAFETY )  DEBUG_COMS.println( "Serial Watchdog Timeout" ); 
                         rollStepperStbd.rollRateDegPerSecDmd = 0.0;
                         disable_starboard_stepper();
                         rollStepperPort.rollRateDegPerSecDmd = 0.0;
                         disable_port_stepper();
                         control_mode = CONTROL_MODE_IDLE;
                         break;
                    }                    
                    /* 
                     * Drive stepper motor 0 (Starboard) 
                     */
                    if ( !rollStepperStbd.enabled ) {
                         rollStepperStbd.enabled = ENABLE;
                    }
                    rollControllerStbd.input = rollStepperStbd.rollDeg; 
                    rollControllerStbd.setPoint = rollStepperStbd.rollDmd ;
                    rollControllerStbd.maxRateDegPerSec = rollStepperStbd.rollRateDegPerSecDmd;
                    MotorCmd0 = computePID( &rollControllerStbd );
                    if ( (rollStepperStbd.errorStatus = drive_starboard_stepper( MotorCmd0 )) != GIMBAL_ERROR_NONE ) {
                         MotorCmd0 = 0.0;
                    }
                    /* 
                     * Drive stepper motor 1 (Port) 
                     */
                    if ( !rollStepperPort.enabled ) {
                         rollStepperPort.enabled = ENABLE;
                    }
                    rollControllerPort.input = rollStepperPort.rollDeg; 
                    rollControllerPort.setPoint = rollStepperPort.rollDmd ;
                    rollControllerPort.maxRateDegPerSec = rollStepperPort.rollRateDegPerSecDmd;
                    MotorCmd1 = computePID( &rollControllerPort );
                    if ( (rollStepperPort.errorStatus = drive_port_stepper( MotorCmd1 )) != GIMBAL_ERROR_NONE ) {
                         MotorCmd1 = 0.0;
                    }                    
                    break;

               case CONTROL_MODE_CALIBRATE_GIMBAL:
                    // CHECK THAT MOTORS HAVE STOPPED
                    if ( 0 ) { DEBUG_COMS.print( "Calib_state = " ); DEBUG_COMS.println( gimbal_calib.mode_port ); }
                    /* Check that the calibraiton isn't taking too long, if so abort */
                    gimbal_calib.timeSinceStart_sec += (1.0 * DT_CONTROL_MS / 1000.0);
                    if ( gimbal_calib.timeSinceStart_sec > 90.0 ) {
                         if ( gimbal_calib.mode_port != CALIBRATION_COMPLETED ) {
                              gimbal_calib.mode_port = CALIBRATION_ERROR;
                         }
                         if ( gimbal_calib.mode_stbd != CALIBRATION_COMPLETED ) {
                              gimbal_calib.mode_port = CALIBRATION_ERROR;
                         }
                         control_mode = CONTROL_MODE_IDLE;
                         break;
                    }
                    /* Implement the gimbal calibration state machine for the port gimbal */
                    switch ( gimbal_calib.mode_port ) {
                         case CALIBRATION_IDLE:
                              //gimbal_calib.stepCountStart_port = rollStepperPort.stepCount;
                              //gimbal_calib.mode_port = CALIBRATION_FIND_MIN_LIMIT;
                              //if ( 1 ) DEBUG_COMS.println( "Port - going to calib state: CALIBRATION_FIND_MIN_LIMIT");
                              break;
                         case CALIBRATION_FIND_MIN_LIMIT:
                              if ( !rollStepperPort.enabled ) { rollStepperPort.enabled = ENABLE; }
                              rollControllerPort.input = rollStepperPort.rollDeg; 
                              rollControllerPort.setPoint = rollStepperPort.rollDeg - 4.0;
                              rollControllerPort.maxRateDegPerSec = 4.0;
                              MotorCmd1 = computePID( &rollControllerPort );
                              if ( 0 ) { DEBUG_COMS.print( " MotorCmd1 = "); DEBUG_COMS.println( MotorCmd1 ); }
                              rollStepperPort.errorStatus = drive_port_stepper( MotorCmd1 );
                              if ( rollStepperPort.errorStatus == GIMBAL_ERROR_MIN_LIMIT ) {
                                   gimbal_calib.stepCountMinLimit_port = rollStepperPort.stepCount;
                                   // gimbal_calib.mode_port = CALIBRATION_FIND_MAX_LIMIT;
                                   if ( 1 ) {
                                        DEBUG_COMS.print( "Port - Start_cnt = "); DEBUG_COMS.println( gimbal_calib.stepCountStart_port );
                                        DEBUG_COMS.print( "Port - Min Limit Step Cnt = "); DEBUG_COMS.println( gimbal_calib.stepCountMinLimit_port );
                                   }

                                   // Hard code the variables for a half calibration
                                   rollStepperPort.stepCount = 228583;
                                   gimbal_calib.mode_port = CALIBRATION_MOVE_TO_ZERO_ANGLE;
                                   if ( 1 ) DEBUG_COMS.println( "Port - going to calib state: CALIBRATION_MOVE_TO_ZERO_ANGLE");
                                   // if ( 1 ) DEBUG_COMS.println( "Port - going to calib state: CALIBRATION_FIND_MAX_LIMIT");
                              }
                              break;
                         case CALIBRATION_FIND_MAX_LIMIT:
                              if ( !rollStepperPort.enabled ) { rollStepperPort.enabled = ENABLE; }
                              rollControllerPort.input = rollStepperPort.rollDeg; 
                              rollControllerPort.setPoint = rollStepperPort.rollDeg + 4.0;
                              rollControllerPort.maxRateDegPerSec = 4.0;
                              MotorCmd1 = computePID( &rollControllerPort );
                              rollStepperPort.errorStatus = drive_port_stepper( MotorCmd1 );
                              if ( rollStepperPort.errorStatus == GIMBAL_ERROR_MAX_LIMIT ) {
                                   gimbal_calib.stepCountMaxLimit_port = rollStepperPort.stepCount;
                                   gimbal_calib.mode_port = CALIBRATION_CALCULATE_OFFSETS;
                                   if ( 1 ) DEBUG_COMS.println( "Port - going to calib state: CALIBRATION_CALCULATE_OFFSETS");
                              }
                              break;
                         case CALIBRATION_CALCULATE_OFFSETS:
                              gimbal_calib.stepOffset_port = (gimbal_calib.stepCountMaxLimit_port + gimbal_calib.stepCountMinLimit_port ) / 2;
                              DEBUG_COMS.print( "Min Step Count = " );
                              DEBUG_COMS.println( gimbal_calib.stepCountMinLimit_port );
                              DEBUG_COMS.print( "Max Step Count = " );
                              DEBUG_COMS.println( gimbal_calib.stepCountMaxLimit_port );
                              DEBUG_COMS.print( "Zero Angle Step Count = " );
                              DEBUG_COMS.println( gimbal_calib.stepOffset_port );
                              gimbal_calib.mode_port = CALIBRATION_MOVE_TO_ZERO_ANGLE;
                              if ( 1 ) DEBUG_COMS.println( "Port - going to calib state: CALIBRATION_MOVE_TO_ZERO_ANGLE");
                              break;
                         case CALIBRATION_MOVE_TO_ZERO_ANGLE:
                              if ( !rollStepperPort.enabled ) { rollStepperPort.enabled = ENABLE; }
                              rollControllerPort.input = rollStepperPort.rollDeg; 
                              rollControllerPort.setPoint = 0.0;
                              rollControllerPort.maxRateDegPerSec = 4.0;
                              MotorCmd1 = computePID( &rollControllerPort );
                              rollStepperPort.errorStatus = drive_port_stepper( MotorCmd1 );
                              if ( fabs(rollStepperPort.rollDeg) < 1.0 ) {
                                   gimbal_calib.mode_port = CALIBRATION_COMPLETED;
                                   if ( 1 ) DEBUG_COMS.println( "Port - going to calib state: CALIBRATION_COMPLETED");
                              }
                              break;
                         case CALIBRATION_COMPLETED:
                              // if ( 1 ) DEBUG_COMS.println( "Port - CALIBRATION_COMPLETED");
                              break;
                    }

                    /* Implement the gimbal calibration state machine for the starboard gimbal */
                    switch ( gimbal_calib.mode_stbd ) {
                         case CALIBRATION_IDLE:
                              //gimbal_calib.stepCountStart_stbd = rollStepperStbd.stepCount;
                              //gimbal_calib.mode_stbd = CALIBRATION_FIND_MIN_LIMIT;
                              //if ( 1 ) DEBUG_COMS.println( "Stbd - going to calib state: CALIBRATION_FIND_MIN_LIMIT");
                              break;
                         case CALIBRATION_FIND_MIN_LIMIT:
                              if ( !rollStepperStbd.enabled ) { rollStepperStbd.enabled = ENABLE; }
                              rollControllerStbd.input = rollStepperStbd.rollDeg; 
                              rollControllerStbd.setPoint = rollStepperPort.rollDeg - 4.0;
                              rollControllerStbd.maxRateDegPerSec = 4.0;
                              MotorCmd0 = computePID( &rollControllerStbd );
                              if ( 0 ) { DEBUG_COMS.print( " MotorCmd0 = "); DEBUG_COMS.println( MotorCmd0 ); }
                              rollStepperStbd.errorStatus = drive_starboard_stepper( MotorCmd0 );
                              if ( rollStepperStbd.errorStatus == GIMBAL_ERROR_MIN_LIMIT ) {
                                   gimbal_calib.stepCountMinLimit_stbd = rollStepperStbd.stepCount;
                                   // gimbal_calib.mode_port = CALIBRATION_FIND_MAX_LIMIT;
                                   if ( 0 ) {
                                        DEBUG_COMS.print( "Stbd - Start_cnt = "); DEBUG_COMS.println( gimbal_calib.stepCountStart_stbd );
                                        DEBUG_COMS.print( "Stbd - Min Limit Step Cnt = "); DEBUG_COMS.println( gimbal_calib.stepCountMinLimit_stbd );
                                   }

                                   // Hard code the variables for a half calibration
                                   rollStepperStbd.stepCount = -200000;
                                   gimbal_calib.mode_stbd = CALIBRATION_MOVE_TO_ZERO_ANGLE;
                                   if ( 1 ) DEBUG_COMS.println( "Stbd - going to calib state: CALIBRATION_MOVE_TO_ZERO_ANGLE");
                                   // if ( 1 ) DEBUG_COMS.println( "Port - going to calib state: CALIBRATION_FIND_MAX_LIMIT");
                              }
                              break;
                         case CALIBRATION_FIND_MAX_LIMIT:
                              gimbal_calib.mode_stbd = CALIBRATION_CALCULATE_OFFSETS;
                              break;
                         case CALIBRATION_CALCULATE_OFFSETS:
                              gimbal_calib.mode_stbd = CALIBRATION_COMPLETED;
                              break;
                         case CALIBRATION_MOVE_TO_ZERO_ANGLE:
                              if ( !rollStepperStbd.enabled ) { rollStepperStbd.enabled = ENABLE; }
                              rollControllerStbd.input = rollStepperStbd.rollDeg; 
                              rollControllerStbd.setPoint = 0.0;
                              rollControllerStbd.maxRateDegPerSec = 4.0;
                              MotorCmd0 = computePID( &rollControllerStbd );
                              rollStepperStbd.errorStatus = drive_starboard_stepper( MotorCmd0 );
                              if ( fabs(rollStepperStbd.rollDeg) < 1.0 ) {
                                   gimbal_calib.mode_stbd = CALIBRATION_COMPLETED;
                                   if ( 1 ) DEBUG_COMS.println( "Stbd - going to calib state: CALIBRATION_COMPLETED");
                              }
                              break;
                         case CALIBRATION_COMPLETED:
                              // if ( 1 ) DEBUG_COMS.println( "Stbd - CALIBRATION_COMPLETED");
                              break;
                    }
                    if ( (gimbal_calib.mode_port == CALIBRATION_COMPLETED ) && (gimbal_calib.mode_stbd == CALIBRATION_COMPLETED) ) {
                         gimbal_calib.mode_port = CALIBRATION_IDLE;
                         gimbal_calib.mode_stbd = CALIBRATION_IDLE;
                         control_mode = CONTROL_MODE_IDLE;
                         if ( 1 ) { DEBUG_COMS.println( "*** CALIBRATION COMPLETED ***" ); }
                    }
                    break;

               default:
                    break;
          }
     }

     /*
      * Rate controlled inclinometer angle read
      */
     if ( ( millis() - prevMillisInclinometer ) >= DT_INCLINOMETER_UPDATE_MS ) {
          /* update the rate and last message recieved timers */
          prevMillisInclinometer = millis();
          rollStepperStbd.timeSinceLastUpdate_sec += (1.0 * DT_INCLINOMETER_UPDATE_MS / 1000.0);
          rollStepperPort.timeSinceLastUpdate_sec += (1.0 * DT_INCLINOMETER_UPDATE_MS / 1000.0);
          /*
           * Check that the inclinometers are communicating
           */
          if ( rollStepperPort.timeSinceLastUpdate_sec > 2.0 ) { rollStepperPort.errorStatus = GIMBAL_ERROR_INCLINOMETER_TIMEOUT; }
          if ( rollStepperStbd.timeSinceLastUpdate_sec > 2.0 ) { rollStepperStbd.errorStatus = GIMBAL_ERROR_INCLINOMETER_TIMEOUT; }
          /*
           * Just after startup reset the stepper motor encoders to the measured inclinometer angle
           */
          if ( rollStepperPort.numMsgsRecieved == 4 ) {
               reset_roll_angle_to_inclinometer( PORT_GIMBAL, rollStepperPort.inclinRollDeg );
          } else {
               // PERHAPS MONITOR ANGLE DIVERGENCE BTW ENCODER AND INCLINOMETER AND RESET IF > THRESHOLD
          }
          if ( rollStepperStbd.numMsgsRecieved == 4 ) {
               reset_roll_angle_to_inclinometer( STBD_GIMBAL, rollStepperStbd.inclinRollDeg );
          } else {
               // PERHAPS MONITOR ANGLE DIVERGENCE BTW ENCODER AND INCLINOMETER AND RESET IF > THRESHOLD
          }
          /* 
           * Send the angle request packet to the inclinometer
           */
          inclinometer_send_angle_request( PORT_GIMBAL );
          inclinometer_send_angle_request( STBD_GIMBAL );
     }

     /*
      * rate controlled status update
      */
     if ( ( millis() - prevMillisSerial ) >= DT_SERIAL_MS ) {
          /* update the rate timer */
          prevMillisSerial = millis();
          /*
           * Send the status packet (encoder and limit switch) via serial
           */
          send_status_packet();
          /*
           * Debug mode
           */
          send_debug_stepper_packet();
    }
}


/**
 * Initiliase the PID controller parameters
 */
void initPID( void )
{
     // Starboard gimbal
     rollControllerStbd.kp = 10.0;
     rollControllerStbd.ki = 0.0;
     rollControllerStbd.kd = 0.0;
     // Port gimbal
     rollControllerPort.kp = 10.0;
     rollControllerPort.ki = 0.0;
     rollControllerPort.kd = 0.0;
}


/**
 * Zero the gimbal angle PID control variables
 */
void zeroPID( struct CONTROLLER_PARAM *controller ) {
     controller->previousTime = notInitialised;
     controller->cumulativeError = 0.0;
     controller->lastError = 0.0;
     controller->input = 0.0;
     controller->setPoint =  0.0;
     controller->lastOutput = 0.0;
     return;
}


/**
 * Compute the gimbal stepper motor speed based on a PID controller
 */
double computePID( struct CONTROLLER_PARAM *controller ) {     
     // compute the time between controller udpates
     unsigned long currentTime = millis();
     if ( controller->previousTime == notInitialised ) {
          controller->previousTime = currentTime - DT_CONTROL_MS;
     }
     double elapsedTimeSec = (double)(currentTime - controller->previousTime) / 1000.0;
        
     // compute proportional error, scaling by 100 to get better resolution
     double error = ( controller->setPoint - controller->input ) * 100.0;     
     // compute the integral error
     controller->cumulativeError += (error * elapsedTimeSec);
     // MDD - should deal with integral windup

     // compute the derivative error
     double rateError = (error - controller->lastError) / elapsedTimeSec;
     
     // calculate the desired output and clip to operational limits
     double out = (controller->kp * error) + (controller->ki * controller->cumulativeError) + (controller->kd * rateError);
     if ( out > ( controller->lastOutput + controller->rateLimit ) ) out = ( controller->lastOutput + controller->rateLimit );
     if ( out < ( controller->lastOutput - controller->rateLimit ) ) out = ( controller->lastOutput - controller->rateLimit );

     double maxRollRateStepsPerSec = (controller->maxRateDegPerSec * 175.8);

     if ( maxRollRateStepsPerSec > CONTROLLER_MAX_SPEED ) { maxRollRateStepsPerSec = CONTROLLER_MAX_SPEED; }
     if ( out > maxRollRateStepsPerSec ) out = maxRollRateStepsPerSec; 
     if ( out < -maxRollRateStepsPerSec ) out = -maxRollRateStepsPerSec; 
     // if ( out > CONTROLLER_MAX_SPEED) out = CONTROLLER_MAX_SPEED; 
     //if ( out < CONTROLLER_MIN_SPEED) out = CONTROLLER_MIN_SPEED; 

     // Stash current error and timing values for next controller step
     controller->lastError = error;
     controller->previousTime = currentTime;
     controller->lastOutput = out;

     if ( 0 ) {     // VERBOSE_ROLL_STEPPERS && (control_mode == CONTROL_MODE_DRIVE_ANGLE)) {
          DEBUG_COMS.print( "SetPoint: " );
          DEBUG_COMS.print( controller->setPoint );
          DEBUG_COMS.print( ", Input: " );
          DEBUG_COMS.print( controller->input );
          DEBUG_COMS.print( ", current time: " );
          DEBUG_COMS.print( currentTime );
          DEBUG_COMS.print( ", elapsed time: " );
          DEBUG_COMS.print( elapsedTimeSec );               
          DEBUG_COMS.print( ", error:  " );
          DEBUG_COMS.print( error );               
          DEBUG_COMS.print( ", cumError: " );
          DEBUG_COMS.print(controller->cumulativeError ); 
          DEBUG_COMS.print( ", rate error: " );
          DEBUG_COMS.print( rateError );               
          DEBUG_COMS.print( ", out:  " );
          DEBUG_COMS.print( out );
          DEBUG_COMS.print( ", last error:  " );
          DEBUG_COMS.print( controller->lastError );               
          DEBUG_COMS.print( ", PrevTime:  " );
          DEBUG_COMS.println( controller->previousTime ); 
     }
 
     return out;
}




/**
 * Drive the starboard stepper motor (motor0)
 */
int drive_starboard_stepper( float stepSpeed ) {
     float speedOut = 0.0;
     int   status = GIMBAL_ERROR_NONE;

     /* Check the limit switches */
     if ( ( rollStepperStbd.limitSwitchMin == 1 ) && ( stepSpeed < 0.0 ) ) {
          speedOut = 0.0;
          status = GIMBAL_ERROR_MIN_LIMIT;
          if ( VERBOSE_SAFETY ) DEBUG_COMS.println( "Stbd Min Limit" );
     } else if ( ( rollStepperStbd.limitSwitchMax == 1 ) && ( stepSpeed > 0.0 ) ) {
          speedOut = 0.0;
          status = GIMBAL_ERROR_MAX_LIMIT;
          if ( VERBOSE_SAFETY ) DEBUG_COMS.println( "Stbd Max Limit" ); 
     } else {
          speedOut = stepSpeed;
          if ( VERBOSE_SAFETY ) DEBUG_COMS.println( "Stbd Run" );
     }
     /* only update the stepper speed if different from previous setpoint */
     if ( stepper0.speed() != speedOut ) {
          stepper0.setSpeed( speedOut );
     }
     //stepper0.runSpeed(); --RL only call from timed interupt
     return status;
}


/**
 * Drive the port stepper motor (motor1)
 */
int drive_port_stepper( float stepSpeed ) {
     float speedOut = 0.0;
     int   status = GIMBAL_ERROR_NONE;

     /* Check the limit switches */
     if ( ( rollStepperPort.limitSwitchMin == 1 ) && ( stepSpeed < 0.0 ) ) {
          speedOut = 0.0;
          status = GIMBAL_ERROR_MIN_LIMIT;
          if ( VERBOSE_SAFETY ) DEBUG_COMS.println( "Port Min Limit" );
     } else if ( ( rollStepperPort.limitSwitchMax == 1 ) && ( stepSpeed > 0.0 ) ) {
          speedOut = 0.0;
          status = GIMBAL_ERROR_MAX_LIMIT;
          if ( VERBOSE_SAFETY ) DEBUG_COMS.println( "Port Max Limit" ); 
     } else {
          speedOut = stepSpeed;
          if ( VERBOSE_SAFETY ) DEBUG_COMS.println( "Port Run" );
     }
     /* only update the stepper speed if different from previous setpoint */
     if ( stepper1.speed() != speedOut ) {
          stepper1.setSpeed( speedOut );
     }
     //stepper1.runSpeed(); --RL only call from timed interupt
     return status;
}


/**
 * Disable the starboard stepper motor by writing 0 to the speed, then set the structure to disabled
 */
void disable_starboard_stepper( void )
{
     MotorCmd0 = 0.0;
     stepper0.setSpeed( MotorCmd0 );
     //stepper0.runSpeed(); --RL only call from timed interupt
     rollStepperStbd.enabled = DISABLE;
}


/**
 * Disable the port stepper motor by writing 0 to the speed, then set the structure to disabled
 */
void disable_port_stepper( void )
{
     MotorCmd1 = 0.0;
     stepper1.setSpeed( MotorCmd1 );
     //stepper1.runSpeed(); --RL only call from timed interupt
     rollStepperPort.enabled = DISABLE;
}


/**
 * Maps the input variable between in_min and in_max to a range between
 * out_min and out_max using floating points and returning a float
 */
float map_float( float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


/**
 * Estimate the roll angle of the gimbal based on the current motor encoder value and
 * the gear ratio of the motor gearbox and the gimble ring and pinnion gear. 
 * NOTE: These parameters are for the QUT stepper motors and gearbox setup
 * @PARAM encoder count
 * @RETURN gimbal angle in Degrees (relatively to initial starting point encoderPos = 0)
 */
float calculate_roll_angle( long encoderPos ) {
     float  gearRatio1 = 50.0;          // Stepper motor gear box ratio - Lab model = 50.0
     float  gearRatio2 = 5.00;          // Ring gear ration - Lab model = 6.33;
     float  encResolution = 4000.0;     // counts per revoluation of motor - Lab model = 4000.0

     float  valA = ( 360.0 / ( gearRatio1 * gearRatio2 * encResolution ) );
     float  angleTmp = valA * (float)(encoderPos);
     return (float) angleTmp;
}


/**
 * Gimbal angle encoder reset calculation function
 * to the inclinometer
 * @PARAM id = PORT or STBD ecoder
 * @PARAM offset angle in degrees
 * @RETURN 0 if not changed, 1 if reset encoder
 */
int reset_roll_angle_to_inclinometer( int id, float offset ) {
     float  gearRatio1 = 50.0;          // Stepper motor gear box ratio - Lab model = 50.0
     float  gearRatio2 = 5.00;          // Ring gear ration - Lab model = 6.33;
     float  encResolution = 4000.0;     // counts per revoluation of motor - Lab model = 4000.0
     // Calculate the equivelant encoder value for the angle offset
     float  valB = 360.0 / ( gearRatio1 * gearRatio2 * encResolution );
     float  encoderValFloat = offset / valB;

     if ( id == PORT_GIMBAL ) {
          rollStepperPort.stepCount = (long)(floor( encoderValFloat ));
          return 1;
     }
     if ( id == STBD_GIMBAL ) {
          rollStepperStbd.stepCount = (long)(floor( encoderValFloat ));
          return 1;
     }
     return 0;
};


/**
 * This function gets called by the interrupt at <sampleRate> Hertz
 */
void TC5_Handler( void ) 
{
     stepper0.runSpeed();
     stepper1.runSpeed();
     TC5->COUNT16.INTFLAG.bit.MC0 = 1;   // Writing a 1 to INTFLAG.bit.MC0 clears the interrupt so that it will run again
}




/** 
 *  TIMER SPECIFIC FUNCTIONS FOLLOW
 *  you shouldn't change these unless you know what you're doing
 */

/**
 * Configures the TC to generate output events at the sample frequency.
 * Configures the TC in Frequency Generation mode, with an event output once
 * each time the audio sample frequency period expires.
 */
void tcConfigure( void )
{
     // Enable GCLK for TCC2 and TC5 (timer counter input clock)
     GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
     while ( GCLK->STATUS.bit.SYNCBUSY );

     tcReset();                               //reset TC5

     // Set Timer counter Mode to 16 bits
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
     // Set TC5 mode as match frequency
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
     // set prescaler and enable TC5
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE; //you can use different prescaler divisons here like TC_CTRLA_PRESCALER_DIV1 to get different ranges of frequencies
     // set TC5 timer counter based off of the system clock and the user defined sample rate or waveform

// MDD - THIS NEXT LINE LOOKS HARDCODED - IS IT MEANT TO BE? ALSO IF SAMPLERATE IS USED, THE COMMENTS SHOULD HAVE UNITS (e.g. s, us, ns)
     TC5->COUNT16.CC[0].reg = (uint16_t) 15;  // <clock_cycle_counts> - 1  @48mhz to wait between timed interupts 
     while ( tcIsSyncing() ); // wait until TC5 is done syncing. Blocking Call
 
     // Configure interrupt request
     NVIC_DisableIRQ( TC5_IRQn );
     NVIC_ClearPendingIRQ( TC5_IRQn );
     NVIC_SetPriority( TC5_IRQn, 0 );
     NVIC_EnableIRQ( TC5_IRQn );

     // Enable the TC5 interrupt request
     TC5->COUNT16.INTENSET.bit.MC0 = 1;
     while ( tcIsSyncing() );                    // wait until TC5 is done syncing. BLocking call 
} 


/**
 * Function that is used to check if TC5 is done syncing
 * returns true when it is done syncing
 */
bool tcIsSyncing()
{
     // MDD - this could be a blocking function???
     return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}


/**
 * This function enables TC5 and waits for it to be ready
 */
void tcStartCounter()
{
     TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; // set the CTRLA register
     while ( tcIsSyncing() );                   // wait until snyc'd
}


/**
 * Reset TC5
 */ 
void tcReset()
{
     TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
     while ( tcIsSyncing() );
     while ( TC5->COUNT16.CTRLA.bit.SWRST );
}


/**
 * Disable TC5
 */
void tcDisable()
{
     TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
     while ( tcIsSyncing() );
}
