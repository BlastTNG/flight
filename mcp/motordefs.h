/*
 * motordefs.h: Contains any functions or structures used by both
 *              copleycommand.c and pivotcommand.c
 *
 */

#ifndef MOTORDEFS_H
#define MOTORDEFS_H

#include <stdarg.h>  /* ANSI C variable arguments (va_list, va_start, va_end) */

//#define MOTORS_VERBOSE /*** Motors output more information that is only 
//                           interesting to those working on motor code ***/

enum CheckType {resp, comm, both};

struct MotorInfoStruct {
  int fd; // File descriptor.                          
  int open; // 0 is closed, 1 is open                                         
  int init; // 0 has not yet been initialized                                 
            // 1 has been initialized with no errors                          
            // 2 initialization was attempted but failed                     

  short unsigned int err;  // Gives the communication error status:                          
            // 1st Bit set: select call timed out
            // 2nd Bit set: sending command failed. 
            // 3rd Bit set: controller response is incorrect
            // 4th Bit set: Controller returns Error message.                  
            // triggers configure_react                                       
            // 3 running configure_react failed.    
            // 0 No errors                                                    
  unsigned int err_count; // How many communication errors have we had since 
                 // command was executed properly?
  int disabled; // Is the motor controller disabled                           
                // 0 if no                                                      
                // 1 in yes                                                     
                // 2 if we aren't sure (because we have just started mcp)      
  int bdrate; // Baud rate with which we are communicating with the controller
  int closing; // 1 if in the process of closing down.     
  int reset; // 1 to reset the drive.     
  int writeset; // 1 if write access has been set.
                // 0 if it has not yet been set
                // 2 if there is an error while sending the 
                // set write access command.
                // ***Only needed for AMC controllers.  
                // For copley controllers set to 1 because we
                // should always be able to set variables.   
  char motorstr[6]; // A character array containing the name of the motor.               
  unsigned int verbose; // Used to tell the motor threads how much chatter they should output.
                        // set to CommandData.verbose_[motor] 
};

// Used in the serial threads to decide whether output should be printed.
#define MC_NOT_VERBOSE 0
#define MC_VERBOSE 1
#define MC_EXTRA_VERBOSE 2

#endif
