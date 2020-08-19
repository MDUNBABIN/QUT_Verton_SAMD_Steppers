/**
 * Verton Spin Pod Controller (SPC)  Inclinometer request and read functions : Verton_Inclinometer.cpp
 * ---------------------------------------------------------------------------------------------------
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
#include "Verton_Inclinometer.h"


INCLINOMETER_MESSAGE inclinPortMsg;
INCLINOMETER_MESSAGE inclinStbdMsg;

/**
 * Send the polled tilt angle read request to the inclinometer on side "val"
 */
int inclinometer_send_angle_request( int val ) {
    char outPkt[] = {0x68,0x04,0x00,0x04,0x08};

    if ( val == PORT_GIMBAL ) {
        for (int i = 0; i<5; i++ ) {
            PORT_COMS.write( outPkt[i] );
        }
        if ( 0 ) DEBUG_COMS.println( "Inclin: Send port measure request" );
    }
    if ( val == STBD_GIMBAL ) {
        for (int i = 0; i<5; i++ ) {
            STARBOARD_COMS.write( outPkt[i] );
        }
        if ( 0 ) DEBUG_COMS.println( "Inclin: Send starboard measure request" );
    }
    return 0;
}


/**
 * Read the Port Inclinometer packet
 */
int inclinometer_port_read_angle_message( ) {
    while ( PORT_COMS.available() > 0 ) {
        // Read the incoming byte
        inclinPortMsg.nextByte = PORT_COMS.read();
        if ( 0 ) { DEBUG_COMS.print( inclinPortMsg.nextByte, HEX ); DEBUG_COMS.print( " " ); }
        if ( inclinPortMsg.nextByte == 0x68 ) {
            inclinPortMsg.pktLen = 0;
        }
        inclinPortMsg.msg[inclinPortMsg.pktLen] = inclinPortMsg.nextByte;
        inclinPortMsg.pktLen++;
        // Read the packet length if the second byte
        if ( inclinPortMsg.pktLen == 2 ) {
            inclinPortMsg.numBytesToRead = inclinPortMsg.msg[1];
        }
        // If read the number of packet length bytes + header then return 1 to allow processing
        if ( inclinPortMsg.pktLen == (inclinPortMsg.numBytesToRead + 1) ){
            // TODO - Verify the checksum
        if ( 0 ) {
            for ( int i = 0; i < inclinPortMsg.numBytesToRead; i++ ) { 
                DEBUG_COMS.print( inclinPortMsg.msg[i], HEX );
                DEBUG_COMS.print( " " ); 
            }
            DEBUG_COMS.println( "" );
        }
            return 1;
        }
        // Check that there won't be a buffer overflow
        if ( inclinPortMsg.pktLen >= MAX_INCLINOMETER_MESSAGE_SIZE - 1 ) {
            inclinPortMsg.pktLen = 0;   
        }
    }
    return 0;
}


/**
 * Read the Starboard Inclinometer packet
 */
int inclinometer_starboard_read_angle_message( ) {
    while ( STARBOARD_COMS.available() > 0 ) {
        // Read the incoming byte
        inclinStbdMsg.nextByte = STARBOARD_COMS.read();
        if ( 0 ) { DEBUG_COMS.print( inclinStbdMsg.nextByte, HEX ); DEBUG_COMS.print( " " ); }
        if ( inclinStbdMsg.nextByte == 0x68 ) {
            inclinStbdMsg.pktLen = 0;
        }
        inclinStbdMsg.msg[inclinStbdMsg.pktLen] = inclinStbdMsg.nextByte;
        inclinStbdMsg.pktLen++;
        // Read the packet length if the second byte
        if ( inclinStbdMsg.pktLen == 2 ) {
            inclinStbdMsg.numBytesToRead = inclinStbdMsg.msg[1];
        }
        // If read the number of packet length bytes + header then return 1 to allow processing
        if ( inclinStbdMsg.pktLen == (inclinStbdMsg.numBytesToRead + 1) ){
            // TODO - Verify the checksum
            return 1;
        }
        // Check that there won't be a buffer overflow
        if ( inclinStbdMsg.pktLen >= MAX_INCLINOMETER_MESSAGE_SIZE - 1 ) {
            inclinStbdMsg.pktLen = 0;   
        }
    }
    return 0;
}


/**
 * Process the port inclinometer serial message
 */
int inclinometer_port_process_message() {
    rollStepperPort.inclinRollDeg = inclinometer_extract_angle( inclinPortMsg.msg );
    // Apply roll angle based limit switches
    // Reset the timer
    rollStepperPort.timeSinceLastUpdate_sec = 0.0;
    rollStepperPort.numMsgsRecieved += 1;
    return 0;
}


int inclinometer_check_roll_limits() {
    rollStepperPort.limitSwitchMin = (int)((rollStepperPort.inclinRollDeg < -INCLINOMETER_ANGLE_LIMIT_SWITCH) || (rollStepperPort.rollDeg < -INCLINOMETER_ANGLE_LIMIT_SWITCH));
    rollStepperPort.limitSwitchMax = (int)((rollStepperPort.inclinRollDeg > INCLINOMETER_ANGLE_LIMIT_SWITCH) || (rollStepperPort.rollDeg > INCLINOMETER_ANGLE_LIMIT_SWITCH));
    rollStepperStbd.limitSwitchMin = (int)((rollStepperStbd.inclinRollDeg < -INCLINOMETER_ANGLE_LIMIT_SWITCH) || (rollStepperStbd.rollDeg < -INCLINOMETER_ANGLE_LIMIT_SWITCH));
    rollStepperStbd.limitSwitchMax = (int)((rollStepperStbd.inclinRollDeg > INCLINOMETER_ANGLE_LIMIT_SWITCH) || (rollStepperStbd.rollDeg > INCLINOMETER_ANGLE_LIMIT_SWITCH));
    return 0;
}


/**
 * Process the starboard inclinometer serial message
 */
int inclinometer_starboard_process_message() {
    rollStepperStbd.inclinRollDeg = inclinometer_extract_angle( inclinStbdMsg.msg );
    // Apply roll angle based limit switches
    // Reset the timer
    rollStepperStbd.timeSinceLastUpdate_sec = 0.0;
    rollStepperStbd.numMsgsRecieved += 1;
    return 0;
}


/**
 * Extract the angle from the recieved serial message 
 * */
float inclinometer_extract_angle( byte incomingBytes[] ) {
    // Process the sign of the number from the top four bits from byte[4]
    int signAngle = ((incomingBytes[4] & 0xF0) / 16);
    // Extract the integer and decimal components from bytes [4], [5] and [6]
    int intDig1 = (int)(incomingBytes[4] & 0x0F);
    int intDig2 = (int)((incomingBytes[5] & 0xF0) / 16);
    int intDig3 = (int)(incomingBytes[5] & 0x0F);
    int decDig1 = (int)((incomingBytes[6] & 0xF0) / 16);
    int decDig2 = (int)(incomingBytes[6] & 0x0F);
    if ( 0 ) {
        for (int i = 0; i < 11; i++ ) {
            DEBUG_COMS.print( incomingBytes[i], HEX );
            DEBUG_COMS.print( " " );
        }
        DEBUG_COMS.println();
        DEBUG_COMS.print( "      Sign: " ); DEBUG_COMS.print( signAngle );
        DEBUG_COMS.print( " : I1: "); DEBUG_COMS.print( intDig1 );
        DEBUG_COMS.print( " : I2: "); DEBUG_COMS.print( intDig2 );
        DEBUG_COMS.print( " : I3: "); DEBUG_COMS.print( intDig3 );
        DEBUG_COMS.print( " : D1: "); DEBUG_COMS.print( decDig1 );
        DEBUG_COMS.print( " : D2: "); DEBUG_COMS.println( decDig2 );
    }

    int intByteValue = (100 * intDig1) + (10 * intDig2) + (1 * intDig3);
    int decByteValue = (10 * decDig1) + (1 * decDig2);
    // Calculate the floating point value of the angle components
    float angle = (1.0 * intByteValue) + (1.0 * decByteValue / 100.0);
    // Apply the sign of the angle measurement
    if ( signAngle == 1) { angle = angle * -1.0; }
    return angle;
}
