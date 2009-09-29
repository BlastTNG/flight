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

//struct MotorInfoStruct *get_motor_pointer(enum MotorType motor);
void MotorStrOut(char *str,struct MotorInfoStruct* copleyinfo);
void copyouts(char *in, char *out);
void open_copley(char *address, struct MotorInfoStruct* copleyinfo);
void close_copley(struct MotorInfoStruct* copleyinfo);
void setopts_copley(int bdrate,struct MotorInfoStruct* copleyinfo);
void send_copleycmd(char cmd[], struct MotorInfoStruct* copleyinfo);
void configure_copley(struct MotorInfoStruct* copleyinfo);
int check_copleyready(enum CheckType check, struct MotorInfoStruct* copleyinfo);
void check_resp(struct MotorInfoStruct* copleyinfo);
int ping_copley(struct MotorInfoStruct* copleyinfo);
int checkCopleyResp(struct MotorInfoStruct* copleyinfo);
int readCopleyResp(struct MotorInfoStruct* copleyinfo);
int enableCopley(struct MotorInfoStruct* copleyinfo);
int disableCopley(struct MotorInfoStruct* copleyinfo);
long int getCopleyVel(struct MotorInfoStruct* copleyinfo);
long int getCopleyPos(struct MotorInfoStruct* copleyinfo);
int read_line(char *outs,struct MotorInfoStruct* copleyinfo);

extern struct MotorInfoStruct reactinfo; /* declared in copleycommand.c        
                                          *                                   
                                          */
extern struct MotorInfoStruct elevinfo;
#endif
