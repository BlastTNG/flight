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

#define RW_ENC_CTS 2097152.0 // Reaction Wheel Encoder Counts per revolution
#define ELEV_ENC_CTS 524288.0 // Elevation Drive Encoder Counts per revolution

struct MotorInfoStruct *get_motor_pointer(enum MotorType motor);
void MotorStrOut(char *str,enum MotorType motor);
void copyouts(char *in, char *out);
void open_copley(char *address, enum MotorType motor);
void close_copley(enum MotorType motor);
void setopts_copley(int bdrate,enum MotorType motor);
void send_copleycmd(char cmd[], enum MotorType motor);
void configure_copley(enum MotorType motor);
int check_copleyready(enum CheckType check, enum MotorType motor);
void check_resp(enum MotorType motor);
int ping_copley(enum MotorType motor);
int checkCopleyResp(enum MotorType motor);
int readCopleyResp(enum MotorType motor);
int enableCopley(enum MotorType motor);
int disableCopley(enum MotorType motor);
long int getCopleyVel(enum MotorType motor);
long int getCopleyPos(enum MotorType motor);
int read_line(char *outs,enum MotorType motor);

extern struct MotorInfoStruct reactinfo; /* declared in copleycommand.c        
                                          *                                   
                                          */
extern struct MotorInfoStruct elevinfo;
#endif
