/**
 * Verton Spin Pod Controller (SPC) Stepper Driver Interface (SDI) : Verton_SDI_IO.cpp
 * - Laboratory Prototype Board Controller (PBC)
 * ------------------------------------------------------------------------------------
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

#include "Verton_SDI_IO.h"
#include <Arduino.h>

/**
 * Initialise the port and starboard roll stepper motors
 * stepper0 = Starboard gimbal
 * stepper1 = Port gimbal
 */
int  roll_motor_init_steppers(void) {
     /* limit switches */
     pinMode( ROLL_LIMIT_PORT_MIN_PIN, INPUT );
     pinMode( ROLL_LIMIT_PORT_MAX_PIN, INPUT );
     pinMode( ROLL_LIMIT_STBD_MIN_PIN, INPUT );
     pinMode( ROLL_LIMIT_STBD_MAX_PIN, INPUT );
     /* read the limit switches */
     if ( QUT_MODE ) {
          rollStepperPort.limitSwitchMin = digitalRead( ROLL_LIMIT_PORT_MIN_PIN );
          rollStepperPort.limitSwitchMax = digitalRead( ROLL_LIMIT_PORT_MAX_PIN );
          rollStepperStbd.limitSwitchMin = digitalRead( ROLL_LIMIT_STBD_MIN_PIN );
          rollStepperStbd.limitSwitchMin = digitalRead( ROLL_LIMIT_STBD_MAX_PIN );
     }
     /* encoder - Starboard gimbal */
     pinMode( encoder0PinA, INPUT ); 
     pinMode( encoder0PinB, INPUT ); 
     attachInterrupt( digitalPinToInterrupt( encoder0PinA ), procEncoder0A, CHANGE );
     attachInterrupt( digitalPinToInterrupt( encoder0PinB ), procEncoder0B, CHANGE );
     /* encoder - Port gimbal */
     pinMode( encoder1PinA, INPUT ); 
     pinMode( encoder1PinB, INPUT );
     attachInterrupt( digitalPinToInterrupt( encoder1PinA ), procEncoder1A, CHANGE );
     attachInterrupt( digitalPinToInterrupt( encoder1PinB ), procEncoder1B, CHANGE );              
     
     /* init the gimbal error state */
     rollStepperPort.errorStatus = GIMBAL_ERROR_NONE;
     rollStepperStbd.errorStatus = GIMBAL_ERROR_NONE;
     return 0;
}


/**
 * Read the port and starboard roll stepper motor variables
 */
int  roll_motor_read_steppers( void ) {
     roll_motor_port_stepper();
     roll_motor_stbd_stepper();
     return 0;
}


/**
 * Read the port stepper motor variables
 */
int roll_motor_port_stepper(void) {
     if ( QUT_MODE ) {
          rollStepperPort.limitSwitchMin = digitalRead( ROLL_LIMIT_PORT_MIN_PIN );
          rollStepperPort.limitSwitchMax = digitalRead( ROLL_LIMIT_PORT_MAX_PIN );
     }
     return 0;     
}


/**
 * Read the port stepper motor variables
 */
int roll_motor_stbd_stepper(void) {
     if ( QUT_MODE ) {
          rollStepperStbd.limitSwitchMin = digitalRead( ROLL_LIMIT_STBD_MIN_PIN );
          rollStepperStbd.limitSwitchMax = digitalRead( ROLL_LIMIT_STBD_MAX_PIN );
     }
     return 0;     
}


/**
 * The encoderA ISR - Counts the pulses and determines direction of starboard gimbal encoder channel A
 *   #define  encoder0PinA                       A0    //PA02
 */
void procEncoder0A(void) { 
     /*
      * Look for a low-to-high on channel A  
      */
     if ( REG_PORT_IN0 & PORT_PA02) { 
          /*
           * Now check channel B to see which way encoder is turning
           */
          if ( !(REG_PORT_IN1 & PORT_PB08) ) {  
               rollStepperStbd.stepCount = rollStepperStbd.stepCount + 1;         // Clockwise pulse, so increment counter
          } else {
               rollStepperStbd.stepCount = rollStepperStbd.stepCount - 1;         // Counter-clockwise pulse, so decrement counter
          }
     } else {
          /*
           * Must be a high-to-low edge on channel A, 
           * check channel B to see which way the encoder is turning
           */
          if ( REG_PORT_IN1 & PORT_PB08 ) {   
               rollStepperStbd.stepCount = rollStepperStbd.stepCount + 1;          // Clockwise pulse, so increment counter
          } else {
               rollStepperStbd.stepCount = rollStepperStbd.stepCount - 1;          // Counter-clockwise pulse, so decrement counter
          }
     }
} 


/**
 * The encoderB ISR - Counts the pulses and determines direction of starboard gimbal encoder channel B
 *   #define  encoder0PinB                       A1    //PB08
 */
void procEncoder0B(void) { 
     /*
      * Look for a low-to-high on channel B  
      */
     if ( REG_PORT_IN1 & PORT_PB08 ) {   
          /*
           * Now check channel A to see which way encoder is turning
           */
          if ( REG_PORT_IN0 & PORT_PA02 ) {  
               rollStepperStbd.stepCount = rollStepperStbd.stepCount + 1;         // Clockwise pulse, so increment counter
          } else {
               rollStepperStbd.stepCount = rollStepperStbd.stepCount - 1;         // Counter-clockwise pulse, so decrement counter
          }
     } else {
         /*
           * Must be a high-to-low edge on channel B, 
           * check channel A to see which way the encoder is turning
           */ 
          if ( !(REG_PORT_IN0 & PORT_PA02) ) {   
               rollStepperStbd.stepCount = rollStepperStbd.stepCount + 1;          // Clockwise pulse, so increment counter
          } else {
               rollStepperStbd.stepCount = rollStepperStbd.stepCount - 1;          // Counter-clockwise pulse, so decrement counter
          }
     }
} 


/**
 * The encoderA ISR - Counts the pulses and determines direction of port gimbal encoder channel A
 *   #define  encoder1PinA                       A2    //PB09
 */
void procEncoder1A(void) { 
     /*
      * Look for a low-to-high on channel A  
      */
     if ( REG_PORT_IN1 & PORT_PB09 ) { 
          /*
           * Now check channel B to see which way encoder is turning
           */
          if ( !(REG_PORT_IN0 & PORT_PA04) ){  
               rollStepperPort.stepCount = rollStepperPort.stepCount + 1;         // Clockwise pulse, so increment counter
          } else {
               rollStepperPort.stepCount = rollStepperPort.stepCount - 1;         // Counter-clockwise pulse, so decrement counter
          }
     } else {
          /*
           * Must be a high-to-low edge on channel A, 
           * check channel B to see which way the encoder is turning
           */
          if ( REG_PORT_IN0 & PORT_PA04 ) {   
               rollStepperPort.stepCount = rollStepperPort.stepCount + 1;          // Clockwise pulse, so increment counter
          } else {
               rollStepperPort.stepCount = rollStepperPort.stepCount - 1;          // Counter-clockwise pulse, so decrement counter
          }
     }
} 


/**
 * The encoderB ISR - Counts the pulses and determines direction of port gimbal encoder channel B
 *   #define  encoder1PinB                       A3    //PA04
 */
void procEncoder1B(void) { 
     /*
      * Look for a low-to-high on channel B  
      */
    // if ( digitalRead( encoder1PinB ) == HIGH )
     if (REG_PORT_IN0 & PORT_PA04) {   
          /*
           * Now check channel A to see which way encoder is turning
           */
          //if ( digitalRead( encoder1PinA ) == HIGH ) 
          if (REG_PORT_IN1 & PORT_PB09){  
               rollStepperPort.stepCount = rollStepperPort.stepCount + 1;         // Clockwise pulse, so increment counter
          } else {
               rollStepperPort.stepCount = rollStepperPort.stepCount - 1;         // Counter-clockwise pulse, so decrement counter
          }
     } else {
         /*
           * Must be a high-to-low edge on channel B, 
           * check channel A to see which way the encoder is turning
           */ 
          if ( !(REG_PORT_IN1 & PORT_PB09) ) {   
               rollStepperPort.stepCount = rollStepperPort.stepCount + 1;          // Clockwise pulse, so increment counter
          } else {
               rollStepperPort.stepCount = rollStepperPort.stepCount - 1;          // Counter-clockwise pulse, so decrement counter
          }
     }
} 
