/*
 * motordefs.h: Contains any functions or structures used by both
 *              copleycommand.c and pivotcommand.c
 *
 */

#ifndef MOTORDEFS_H
#define MOTORDEFS_H

#include <stdarg.h>  /* ANSI C variable arguments (va_list, va_start, va_end) */

enum CheckType {resp, comm, both};

enum MotorType {pivot, rw, elev};

struct MotorInfoStruct {
  int fd; // File descriptor.                          
  int open; // 0 is closed, 1 is open                                         
  int loop; // 1 ->there is a current loop running                            
            // 0 ->there is no current loop running                           
            // -1 -> there is no current loop running                         
            //       because of some sort of error.                           
  int init; // 0 has not yet been initialized                                 
            // 1 has been initialized with no errors                          
            // -1 initialization was attempted but failed                     

  int err;  // Gives the communication error status:                          
            // 1 communication error sending some command.                    
            // triggers configure_react                                       
            // 3 running configure_react failed.    
            // --> TODO: later this condition will trigger                    
            //           a power cycle of the controller                      
            // 0 No errors                                                    
  int disabled; // Is the motor controller disabled                           
  int bdrate; // Baud rate with which we are communicating with the controller
  // 0 if no                                                      
  // 1 in yes                                                     
  // 10 if we aren't sure (because we have just started mcp)      
  int closing; // 1 if in the process of closing down.     
  int writeset; // 1 if write access has been set.
                // 0 if it has not yet been set
                // -1 if there is an error while sending the 
                // set write access command.
                // ***Only needed for AMC controllers.  
                // For copley controllers set to 1 because we
                // should always be able to set variables.   
  char motorstr[6]; // A character array containing the name of the motor.               
};

#endif
