/**
 * Verton Spin Pod Controller (SPC)  Stepper Driver Interface (SDI) : Verton_SDI_COMMS.h
 * - Header file for pin assignments and functions
 * -------------------------------------------------------------------------------------
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

#define  USE_NMEA_CHECKSUM_VALIDATION    1

int read_spc_interface( void );

int process_spc_packet( void );

int read_usb_interface( void );

int process_usb_packet( void );

int send_status_packet( void );

int send_debug_stepper_packet( void );

int getNextDataValue( char *outPkt, char *inPkt, int n );

int validate_packet_checksum( char *msg, int pktLen );

int fromHex( char a );
