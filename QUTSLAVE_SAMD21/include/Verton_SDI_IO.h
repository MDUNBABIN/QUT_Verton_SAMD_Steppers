/**
 * Verton Spin Pod Controller (SPC)  Stepper Driver Interface (SDI) : Verton_SDI_IO.h
 * - Header file for pin assignments and functions
 * ----------------------------------------------------------------------------------
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

#include "Verton_SPC_PBC_SDI_V1.h"

#pragma once
#ifndef Verton_SDI_IO_h
#define Verton_SDI_IO_h
int  roll_motor_init_steppers(void);

int  roll_motor_read_steppers(void);

int roll_motor_port_stepper(void);
int roll_motor_stbd_stepper(void);

void procEncoder0A(void);
void procEncoder0B(void); 
void procEncoder1A(void);
void procEncoder1B(void);
#endif