/**
 * Verton Spin Pod Controller (SPC) Stepper Driver Interface (SDI) : Verton_SDI_COMS.cpp
 * - Communication functions
 * --------------------------------------------------------------------------------------
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

#include "Verton_SDI_COMS.h"

/**
 * Read the incoming user interface serial packets in NMEA format and 
 * validate the checksum for correct packet transmission
 */
int read_spc_interface()
{    
     while ( SPC_COMS.available() > 0 ) {
          spcMsg.nextByte = SPC_COMS.read();

          if ( spcMsg.nextByte == '$' ) {
               spcMsg.syncCnt = 0;
               spcMsg.pktLen = 0;
          }
          spcMsg.msg[spcMsg.pktLen] = spcMsg.nextByte;
          spcMsg.pktLen++;
          if ( spcMsg.pktLen >= MAX_SERIAL_PACKET_LENGTH - 2 ) {
               spcMsg.syncCnt = 0;
               spcMsg.pktLen = 0;   
          }
          if ( (spcMsg.nextByte == '\n') && (spcMsg.syncCnt == 1) ) {
               if ( 0 ) {
                    for ( int k=0; k<spcMsg.pktLen; k++ )
                         DEBUG_COMS.write( spcMsg.msg[k] );
               }
               /* Check the incoming packet for a valid checksum */
               if ( USE_NMEA_CHECKSUM_VALIDATION ) {
                    if ( validate_packet_checksum( spcMsg.msg, spcMsg.pktLen ) == 0 ) {
                         spcMsg.syncCnt = 0;
                         spcMsg.pktLen = 0;
                         return 0;
                    }
               }
               return 1;
          } else {
               spcMsg.syncCnt = 0;
          }
          if ( spcMsg.nextByte == '\r' ) {
               spcMsg.syncCnt = 1;
          }    
    } 
    return 0; 
}


/**
 * Process the incoming serial packets from the spin pod
 * controller and process if of a valid type.
 * Packet strcutures are defined as:
 *   "$GIME" = enable the stepper motors
 *   "$GIMR" = reset the stepper encoders
 *   "$GIMD" = enter the debug mode on the serial port
 *   "$GIMI" = put the steppers into the idle state
 *   "$GIMC" = calibrate the gimbal angle limits
 *   "$GIMA" = set the gimbals to a specified angle
 *   "$GIMS" = set the steppers to a specified speed
 */
int process_spc_packet()
{
     char      nmeaGIMI[] = "$GIMI";
     char      nmeaGIME[] = "$GIME";
     char      nmeaGIMR[] = "$GIMR";
     char      nmeaGIMD[] = "$GIMD";     
     char      nmeaGIMC[] = "$GIMC";     
     char      nmeaGIMS[] = "$GIMS";
     char      nmeaGIMA[] = "$GIMA";
     int       i = 0;
     char      nmeaHdr[8];
     char      pkt[12];

     /* read the nmea header */
     while ( ( spcMsg.msg[i] != ',' ) && ( spcMsg.msg[i] != '*' ) && ( i < 7 ) ) {
          nmeaHdr[i] = spcMsg.msg[i];
          i++;
     }
     nmeaHdr[i++] = '\0';
     if ( 0 ) DEBUG_COMS.println( nmeaHdr );
          
     /*
      * Set idle mode message: $GIMI,<BEAM_ID>*CS
      */
     if ( strcmp ( nmeaGIMI, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, spcMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          control_mode = CONTROL_MODE_IDLE;
          return 1;
     }

     /*
      * Reset the encoders message: $GIMR,<BEAM_ID>*CS
      */
     if ( strcmp ( nmeaGIMR, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, spcMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          rollStepperStbd.stepCount = 0;
          rollStepperPort.stepCount = 0;
          return 1;
     }

     /*
      * enter debug mode and set verbose mode $GIMD,<BEAM_ID>*CS
      */
     if ( strcmp ( nmeaGIMD, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, spcMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          if ( verbose_mode ) {
               verbose_mode = 0;
               SPC_COMS.println( "debug mode turned off" );
          } else {
               verbose_mode = 1;
               SPC_COMS.println( "debug mode turned on" );
          }
          return 1;
     }

     /*
      * Enable the gimbals message: $GIME,<BEAM_ID>,<gimbal_num>*CS
      *  - gimbal_num: 1 = port, 2 = starboard, 3 = both
      */
     if ( strcmp ( nmeaGIME, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, spcMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          return 1;
     }

     /*
      * Calibrate the limit switches for the gimbals: $GIMC,<BEAM_ID>,<gimbal_num>,<speed>*CS
      *  - gimbal_num: 1 = port, 2 = starboard, 3 = both
      *  - speed in deg/sec
      */
     if ( strcmp ( nmeaGIMC, nmeaHdr ) == 0 ) {
          // Check message is for this Beam, or special case "0" where put the beam to idle state
          i = getNextDataValue( pkt, spcMsg.msg, i );
          if ( atoi( pkt) == 0 ) {
               control_mode = CONTROL_MODE_IDLE;
               return 0;
          }
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          control_mode = CONTROL_MODE_CALIBRATE_GIMBAL;
          gimbal_calib.timeSinceStart_sec = 0.0;
          gimbal_calib.mode_port = CALIBRATION_FIND_MIN_LIMIT;
          gimbal_calib.mode_stbd = CALIBRATION_FIND_MIN_LIMIT;
          if ( 1 ) DEBUG_COMS.println( "CALIB: Starting Calibration Routine...");
          return 1;
     }

     /*
      * Set the gimbals to a roll rate (speed): $GIMS,<BEAM_ID>,<speed_port>,<speed_stbd>*CS
      */
     if ( strcmp ( nmeaGIMS, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, spcMsg.msg, i );          
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          // Read the demanded speeds
          i = getNextDataValue( pkt, spcMsg.msg, i );
          float rPortDmd = atof( pkt );
          i = getNextDataValue( pkt, spcMsg.msg, i );
          float rStbdDmd = atof( pkt );
          // Check the inputs are within allowable limits
          rollStepperPort.rollRateDegPerSecDmd = 0.0;
          rollStepperStbd.rollRateDegPerSecDmd = 0.0;
          if ( (rPortDmd < MOTOR_MIN_COMMAND) || (rPortDmd > MOTOR_MAX_COMMAND) ) { return 0; }
          if ( (rStbdDmd < MOTOR_MIN_COMMAND) || (rStbdDmd > MOTOR_MAX_COMMAND) ) { return 0; }
          // All good, so set the updated angle values
          rollStepperStbd.rollRateDegPerSecDmd = rStbdDmd;
          rollStepperPort.rollRateDegPerSecDmd = rPortDmd;
          control_mode = CONTROL_MODE_DRIVE_SPEED;
          timeSinceLastWatchdogMsg_sec = 0.0;
          if ( VERBOSE_ROLL_STEPPERS ) {    
               DEBUG_COMS.print( "SPEED Control: " );
               DEBUG_COMS.print( "Stepper0_cmd_Speed = " );
               DEBUG_COMS.print( rollStepperStbd.rollRateDegPerSecDmd );
               DEBUG_COMS.print( " : Stepper1_cmd_Speed = " );
               DEBUG_COMS.println( rollStepperPort.rollRateDegPerSecDmd );
          }
          return 1;
     }

     /*
      * Set the gimbals to a Desired Angle (angle): $GIMA,<BEAM_ID>,<anlge_port_deg>,<angle_stbd_deg>,<speed_port>,<speed_stbd>*CS
      */
     if ( strcmp ( nmeaGIMA, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, spcMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          // Read the demanded angles
          i = getNextDataValue( pkt, spcMsg.msg, i );
          float aPortDmd = atof( pkt );
          if ( 0 ) { DEBUG_COMS.print( "aPortDmd: " ); DEBUG_COMS.println( aPortDmd ); }
          i = getNextDataValue( pkt, spcMsg.msg, i );
          float aStbdDmd = atof( pkt );
          if ( 0 ) { DEBUG_COMS.print( "aStbdDmd: " ); DEBUG_COMS.println( aStbdDmd ); }
          // Read the demanded speeds
          i = getNextDataValue( pkt, spcMsg.msg, i );
          float rPortDmd = atof( pkt );
          if ( 0 ) { DEBUG_COMS.print( "rPortDmd: " ); DEBUG_COMS.println( rPortDmd ); }
          i = getNextDataValue( pkt, spcMsg.msg, i );
          float rStbdDmd = atof( pkt );
          if ( 0 ) { DEBUG_COMS.print( "rStbdDmd: " ); DEBUG_COMS.println( rStbdDmd ); }
          // Check the inputs are within allowable limits
          if ( (aPortDmd < ANGLE_MIN_COMMAND) || (aPortDmd > ANGLE_MAX_COMMAND) ) { return 0; }
          if ( (aStbdDmd < ANGLE_MIN_COMMAND) || (aStbdDmd > ANGLE_MAX_COMMAND) ) { return 0; }
          if ( (rPortDmd < MOTOR_MIN_COMMAND) || (rPortDmd > MOTOR_MAX_COMMAND) ) { return 0; }
          if ( (rStbdDmd < MOTOR_MIN_COMMAND) || (rStbdDmd > MOTOR_MAX_COMMAND) ) { return 0; }
          // All good, so set the updated angle values
          rollStepperStbd.rollDmd = aStbdDmd;
          rollStepperStbd.rollRateDegPerSecDmd = rStbdDmd;
          rollStepperPort.rollDmd = aPortDmd;
          rollStepperPort.rollRateDegPerSecDmd = rPortDmd;
          control_mode = CONTROL_MODE_DRIVE_ANGLE;
          timeSinceLastWatchdogMsg_sec = 0.0;
          if ( VERBOSE_ROLL_STEPPERS ) {
               DEBUG_COMS.print( "ANGLE Control: " );
               DEBUG_COMS.print( "Stepper0_ANGLE = " );
               DEBUG_COMS.print( rollStepperStbd.rollDmd );
               DEBUG_COMS.print( " : Stepper1_Angle = " );
               DEBUG_COMS.println( rollStepperPort.rollDmd );
          }
          return 1;
     }

     return 0;     
}


/*
 * Send a status message with avalid message: 
 * $ROLLS,<ID>,P,<ERROR>,<ANGLE_DEG>,<MIN_LIMIT_SWITCH>,<MAX_LIMIT_SWITCH>,S,<ERROR>,<ANGLE_DEG>,<MIN_LIMIT_SWITCH>,<MAX_LIMIT_SWITCH>*CS
 */
int send_status_packet( void ) {
     char crc = 0;
     
     statusPkt.begin();
     // Header
     statusPkt.print( "$ROLLS," );
     statusPkt.print( ID );
     statusPkt.print( ",M," );
     statusPkt.print( control_mode );
     // Tether parameters
     statusPkt.print( ",P," );
     statusPkt.print( rollStepperPort.errorStatus );
     statusPkt.print( "," );     
     statusPkt.print( rollStepperPort.rollDeg, 2 );
     statusPkt.print( "," );
     statusPkt.print( rollStepperPort.inclinRollDeg, 2 );
     statusPkt.print( "," );      
     statusPkt.print( rollStepperPort.limitSwitchMin );
     statusPkt.print( "," );      
     statusPkt.print( rollStepperPort.limitSwitchMax);
     
     statusPkt.print( ",S," );
     statusPkt.print( rollStepperStbd.errorStatus );
     statusPkt.print( "," );     
     statusPkt.print( rollStepperStbd.rollDeg, 2 );
     statusPkt.print( "," );
     statusPkt.print( rollStepperStbd.inclinRollDeg, 2 );
     statusPkt.print( "," );      
     statusPkt.print( rollStepperStbd.limitSwitchMin );
     statusPkt.print( "," );      
     statusPkt.print( rollStepperStbd.limitSwitchMax);
     statusPkt.print( "," );
     /* Calculate and print the CRC */
     for ( unsigned int k=1; k<statusPkt.length(); k++ )
          crc^=statusPkt[k];
     
     /* Append the CRC and \r\n to the end of the packet */
     statusPkt.print( "*" );
     if ( crc < 16 ) { statusPkt.print( "0" ); }
     statusPkt.print( crc, HEX );
     statusPkt.print( "\r\n" );     
     /* Write the entire NMEA packet to the serial port */
     SPC_COMS.print( statusPkt );
     if ( 1 ) DEBUG_COMS.print( statusPkt );
     return 0;
}


/**
 * Send a debug packet with stepper values
 */
int send_debug_stepper_packet()
{
     if ( VERBOSE_ROLL_STEPPERS && (control_mode == CONTROL_MODE_DRIVE_SPEED) ) {
          DEBUG_COMS.print( "[" );
          DEBUG_COMS.print( loopCnt++ );
          DEBUG_COMS.print( "] : " );
          DEBUG_COMS.print( control_mode );               
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.print( rollStepperStbd.stepCount );               
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.print(rollStepperPort.stepCount ); 
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.print( rollStepperStbd.rollRateDegPerSecDmd );               
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.print( rollStepperPort.rollRateDegPerSecDmd );
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.print( MotorCmd0 );               
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.println( MotorCmd1 );               
     }
     if ( VERBOSE_ROLL_STEPPERS && (control_mode == CONTROL_MODE_DRIVE_ANGLE) ) {
          DEBUG_COMS.print( "[" );
          DEBUG_COMS.print( loopCnt++ );
          DEBUG_COMS.print( "] : " );
          DEBUG_COMS.print( control_mode );               
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.print( rollStepperStbd.stepCount );               
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.print(rollStepperPort.stepCount ); 
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.print( rollStepperStbd.rollDmd );               
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.print( rollStepperPort.rollDmd );
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.print( MotorCmd0 );               
          DEBUG_COMS.print( " : " );
          DEBUG_COMS.println( MotorCmd1 );               
     }
     return 0;
}


/**
 * Read the next value in the incoming packet
 */
int getNextDataValue( char *outPkt, char *inPkt, int n )
{
    int  k = 0;
    
    while ( (inPkt[k+n] != ',') && (inPkt[k+n] != '*') && (inPkt[k+n] != '\r') ) {
        outPkt[k] = inPkt[k+n];
        k++;
    }
    outPkt[k++] = '\0';
    return (n + k);
}



/**
 * Check if the incoming packet checksum is correct for a complete valid packet,
 * 1=valid, 0=not valid
 */
int validate_packet_checksum( char *msg, int pktLen ) {
     char crc = 0;
     int  i = 1;
     char nmeaCSchk;

     /* calculate the checksum as read */
     while ( ( msg[i] != '*' ) && ( i <= pktLen ) ) {
          crc^=msg[i];
          i++;
     }

     char gotSum[2];
     gotSum[0] = msg[i+1];
     gotSum[1] = msg[i+2];
     
     nmeaCSchk = (16 * fromHex(gotSum[0])) + fromHex(gotSum[1]);
     
     // Check that the checksums match up
     if ( nmeaCSchk == crc ) {
          return 1;
     } else{
          return 0;
     }
}



/**
 * Convert the hex byte to an int
 */
int fromHex( char a )
{
     if ( a >= 'A' && a <= 'F' )
          return a - 'A' + 10;
     else if ( a >= 'a' && a <= 'f' )
          return a - 'a' + 10;
     else
          return a - '0';
}


/**
 * Read the incoming user serial packets from the USB port in NMEA 
 * format and validate the checksum for correct packet transmission
 */
int read_usb_interface()
{    
     while ( DEBUG_COMS.available() > 0 ) {
          usbMsg.nextByte = DEBUG_COMS.read();

          if ( usbMsg.nextByte == '$' ) {
               usbMsg.syncCnt = 0;
               usbMsg.pktLen = 0;
          }
          usbMsg.msg[usbMsg.pktLen] = usbMsg.nextByte;
          usbMsg.pktLen++;
          if ( usbMsg.pktLen >= MAX_SERIAL_PACKET_LENGTH - 2 ) {
               usbMsg.syncCnt = 0;
               usbMsg.pktLen = 0;   
          }
          if ( (usbMsg.nextByte == '\n') && (usbMsg.syncCnt == 1) ) {
               if ( 0 ) {
                    for ( int k=0; k<usbMsg.pktLen; k++ )
                         DEBUG_COMS.write( usbMsg.msg[k] );
               }
               /* Check the incoming packet for a valid checksum */
               if ( USE_NMEA_CHECKSUM_VALIDATION ) {
                    if ( validate_packet_checksum( usbMsg.msg, usbMsg.pktLen ) == 0 ) {
                         usbMsg.syncCnt = 0;
                         usbMsg.pktLen = 0;
                         return 0;
                    }
               }
               return 1;
          } else {
               usbMsg.syncCnt = 0;
          }
          if ( usbMsg.nextByte == '\r' ) {
               usbMsg.syncCnt = 1;
          }    
    } 
    return 0; 
}


/**
 * Process the incoming usb serial packet and process if of a valid type.
 * Packet strcutures are defined as:
 *   "$GIME" = enable the stepper motors
 *   "$GIMR" = reset the stepper encoders
 *   "$GIMD" = enter the debug mode on the serial port
 *   "$GIMI" = put the steppers into the idle state
 *   "$GIMC" = calibrate the gimbal angle limits
 *   "$GIMA" = set the gimbals to a specified angle
 *   "$GIMS" = set the steppers to a specified speed
 */
int process_usb_packet()
{
     char      nmeaGIMI[] = "$GIMI";
     char      nmeaGIME[] = "$GIME";
     char      nmeaGIMR[] = "$GIMR";
     char      nmeaGIMD[] = "$GIMD";     
     char      nmeaGIMC[] = "$GIMC";     
     char      nmeaGIMS[] = "$GIMS";
     char      nmeaGIMA[]  = "$GIMA";
     int       i = 0;
     char      nmeaHdr[8];
     char      pkt[12];

     /* read the nmea header */
     while ( ( usbMsg.msg[i] != ',' ) && ( usbMsg.msg[i] != '*' ) && ( i < 7 ) ) {
          nmeaHdr[i] = usbMsg.msg[i];
          i++;
     }
     nmeaHdr[i++] = '\0';
     if ( 0 ) DEBUG_COMS.println( nmeaHdr );
          
     /*
      * Set idle mode message: $GIMI,<BEAM_ID>*CS
      */
     if ( strcmp ( nmeaGIMI, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, usbMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          control_mode = CONTROL_MODE_IDLE;
          return 1;
     }

     /*
      * Reset the encoders message: $GIMR,<BEAM_ID>*CS
      */
     if ( strcmp ( nmeaGIMR, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, usbMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          rollStepperStbd.stepCount = 0;
          rollStepperPort.stepCount = 0;
          return 1;
     }

     /*
      * enter debug mode and set verbose mode $GIMD,<BEAM_ID>*CS
      */
     if ( strcmp ( nmeaGIMD, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, usbMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          if ( verbose_mode ) {
               verbose_mode = 0;
               SPC_COMS.println( "debug mode turned off" );
          } else {
               verbose_mode = 1;
               SPC_COMS.println( "debug mode turned on" );
          }
          return 1;
     }

     /*
      * Enable the gimbals message: $GIME,<BEAM_ID>,<gimbal_num>*CS
      *  - gimbal_num: 1 = port, 2 = starboard, 3 = both
      */
     if ( strcmp ( nmeaGIME, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, usbMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          return 1;
     }

     /*
      * Calibrate the limit switches for the gimbals: $GIMC,<BEAM_ID>,<gimbal_num>,<speed>*CS
      *  - gimbal_num: 1 = port, 2 = starboard, 3 = both
      *  - speed in deg/sec
      */
     if ( strcmp ( nmeaGIMC, nmeaHdr ) == 0 ) {
          // Check message is for this Beam, or special case "0" where put the beam to idle state
          i = getNextDataValue( pkt, usbMsg.msg, i );
          if ( atoi( pkt) == 0 ) {
               control_mode = CONTROL_MODE_IDLE;
               return 0;
          }
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          control_mode = CONTROL_MODE_CALIBRATE_GIMBAL;
          gimbal_calib.timeSinceStart_sec = 0.0;
          gimbal_calib.mode_port = CALIBRATION_FIND_MIN_LIMIT;
          gimbal_calib.mode_stbd = CALIBRATION_FIND_MIN_LIMIT;
          if ( 1 ) DEBUG_COMS.println( "CALIB: Starting Calibration Routine...");
          return 1;
     }

     /*
      * Set the gimbals to a roll rate (speed): $GIMS,<BEAM_ID>,<speed_port>,<speed_stbd>*CS
      */
     if ( strcmp ( nmeaGIMS, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, usbMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          // Read the demanded speeds
          i = getNextDataValue( pkt, usbMsg.msg, i );
          float rPortDmd = atof( pkt );
          i = getNextDataValue( pkt, usbMsg.msg, i );
          float rStbdDmd = atof( pkt );
          // Check the inputs are within allowable limits
          rollStepperPort.rollRateDegPerSecDmd = 0.0;
          rollStepperStbd.rollRateDegPerSecDmd = 0.0;
          if ( (rPortDmd < MOTOR_MIN_COMMAND) || (rPortDmd > MOTOR_MAX_COMMAND) ) { return 0; }
          if ( (rStbdDmd < MOTOR_MIN_COMMAND) || (rStbdDmd > MOTOR_MAX_COMMAND) ) { return 0; }
          // All good, so set the updated angle values
          rollStepperStbd.rollRateDegPerSecDmd = rStbdDmd;
          rollStepperPort.rollRateDegPerSecDmd = rPortDmd;
          control_mode = CONTROL_MODE_DRIVE_SPEED;
          timeSinceLastWatchdogMsg_sec = 0.0;
          if ( VERBOSE_ROLL_STEPPERS ) {    
               DEBUG_COMS.print( "SPEED Control: " );
               DEBUG_COMS.print( "Stepper0_cmd_Speed = " );
               DEBUG_COMS.print( rollStepperStbd.rollRateDegPerSecDmd );
               DEBUG_COMS.print( " : Stepper1_cmd_Speed = " );
               DEBUG_COMS.println( rollStepperPort.rollRateDegPerSecDmd );
          }
          return 1;
     }

     /*
      * Set the gimbals to a Desired Angle (angle): $GIMA,<BEAM_ID>,<anlge_port_deg>,<angle_stbd_deg>,<speed_port>,<speed_stbd>*CS
      */
     if ( strcmp ( nmeaGIMA, nmeaHdr ) == 0 ) {
          // Check message is for this Beam
          i = getNextDataValue( pkt, usbMsg.msg, i );
          if ( atoi( pkt ) != ID ) {
               return 0;
          }
          // Read the demanded angles
          i = getNextDataValue( pkt, usbMsg.msg, i );
          float aPortDmd = atof( pkt );
          if ( 0 ) { DEBUG_COMS.print( "aPortDmd: " ); DEBUG_COMS.println( aPortDmd ); }
          i = getNextDataValue( pkt, usbMsg.msg, i );
          float aStbdDmd = atof( pkt );
          if ( 0 ) { DEBUG_COMS.print( "aStbdDmd: " ); DEBUG_COMS.println( aStbdDmd ); }
          // Read the demanded speeds
          i = getNextDataValue( pkt, usbMsg.msg, i );
          float rPortDmd = atof( pkt );
          if ( 0 ) { DEBUG_COMS.print( "rPortDmd: " ); DEBUG_COMS.println( rPortDmd ); }
          i = getNextDataValue( pkt, usbMsg.msg, i );
          float rStbdDmd = atof( pkt );
          if ( 0 ) { DEBUG_COMS.print( "rStbdDmd: " ); DEBUG_COMS.println( rStbdDmd ); }
          // Check the inputs are within allowable limits
          if ( (aPortDmd < ANGLE_MIN_COMMAND) || (aPortDmd > ANGLE_MAX_COMMAND) ) { return 0; }
          if ( (aStbdDmd < ANGLE_MIN_COMMAND) || (aStbdDmd > ANGLE_MAX_COMMAND) ) { return 0; }
          if ( (rPortDmd < MOTOR_MIN_COMMAND) || (rPortDmd > MOTOR_MAX_COMMAND) ) { return 0; }
          if ( (rStbdDmd < MOTOR_MIN_COMMAND) || (rStbdDmd > MOTOR_MAX_COMMAND) ) { return 0; }
          // All good, so set the updated angle values
          rollStepperStbd.rollDmd = aStbdDmd;
          rollStepperStbd.rollRateDegPerSecDmd = rStbdDmd;
          rollStepperPort.rollDmd = aPortDmd;
          rollStepperPort.rollRateDegPerSecDmd = rPortDmd;
          control_mode = CONTROL_MODE_DRIVE_ANGLE;
          timeSinceLastWatchdogMsg_sec = 0.0;
          if ( VERBOSE_ROLL_STEPPERS ) {
               DEBUG_COMS.print( "ANGLE Control: " );
               DEBUG_COMS.print( "Stepper0_ANGLE = " );
               DEBUG_COMS.print( rollStepperStbd.rollDmd );
               DEBUG_COMS.print( " : Stepper1_Angle = " );
               DEBUG_COMS.println( rollStepperPort.rollDmd );
          }
          return 1;
     }

     return 0;     
}

