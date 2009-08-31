/*
 * copleycommand.h: Header file for commands to be sent to 
 * the Copley Accelnet EtherCAT motor controllers (flywheel 
 * and elevation drive) on BLAST.
 *
 */

#ifndef COPLEYCOMMAND_H
#define COPLEYCOMMAND_H

#include <stdarg.h>  /* ANSI C variable arguments (va_list, va_start, va_end) */
#include "motordefs.h"

struct CopleyInfoStruct {
  int fd; // File descriptor for the reaction wheel.                          
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
  // 0 if no                                                      
  // 1 in yes                                                     
  // 10 if we aren't sure (because we have just started mcp)      
  int closing; // 1 if in the process of closing down.                        
};

struct CopleyInfoStruct *get_motor_pointer(enum MotorType motor);
void open_copley(char *address, enum MotorType motor);
void close_copley(enum MotorType motor);
void setopts_copley(int bdrate,enum MotorType motor);
void send_copleycmd(char cmd[], enum MotorType motor);
void configure_copley(enum MotorType motor);
int check_copleyready(enum CheckType check, enum MotorType motor);
void check_resp(enum MotorType motor);
int ping_copley(enum MotorType motor);
int checkCopleyResp(enum MotorType motor);
int enableCopley(enum MotorType motor);
int disableCopley(enum MotorType motor);
extern struct CopleyInfoStruct reactinfo; /* declared in reactcommand.c        
                                          *                                   
                                          */
extern struct CopleyInfoStruct elevinfo;
#endif
