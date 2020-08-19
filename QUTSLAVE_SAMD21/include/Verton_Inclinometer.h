/**
 * Verton Spin Pod Controller (SPC)  Inclinometer request and read functions : Verton_Inclinometer.h
 * -------------------------------------------------------------------------------------------------
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

#define INCLINOMETER_ANGLE_LIMIT_SWITCH   60.0      // (Deg)
#define MAX_INCLINOMETER_MESSAGE_SIZE     20


typedef struct {
    byte      nextByte;
    uint8_t   pktLen;
    uint8_t   numBytesToRead;
    byte      msg[ MAX_INCLINOMETER_MESSAGE_SIZE ];  
} INCLINOMETER_MESSAGE;

extern INCLINOMETER_MESSAGE inclinPortMsg;
extern INCLINOMETER_MESSAGE inclinStbdMsg;

int inclinometer_send_angle_request( int val );

int inclinometer_port_read_angle_message( );
int inclinometer_starboard_read_angle_message( );

int inclinometer_port_process_message();
int inclinometer_starboard_process_message();
float inclinometer_extract_angle( byte incomingBytes[] );
int inclinometer_check_roll_limits();