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

//#define DEBUG_COPLEY

#define RW_ENC_CTS 2097152.0 // Reaction Wheel Encoder Counts per revolution
#define ELEV_ENC_CTS 524288.0 // Elevation Drive Encoder Counts per revolution

#define COPLEY_ERR_TIMEOUT 2 // Number of consecutive serious errors before the
                              // thread attempts to reset the controller.

// Indices for various Copley Controller Run time variables
#define COP_IND_TEMP "0x20"
#define COP_IND_CURRENT "0x0c"
#define COP_IND_STATUS "0xa0"
#define COP_IND_FAULTREG "0xa4"
#define COP_IND_VEL "0x18"
#define COP_IND_POS "0x32"

// Error masks.  Tells the controller which bits set in the copleyinfo->err
// variable should count towards triggering a reset.
#define COP_ERR_MASK 0x001e // Bit #2 (send command failed) implying that the 
                              // serial hub not powered.

//  Copley Error codes
# define COP_ERR_2MUCH_DATA 1
# define COP_ERR_UNKNOWN_CMD 3
# define COP_ERR_NOT_ENOUGH_DATA 4
# define COP_ERR_TOO_MUCH_DATA 5
# define COP_ERR_UNKNOWN_ID 9
# define COP_ERR_OUT_RANGE 10
# define COP_ERR_READ_ONLY 11
# define COP_ERR_CAN_FAIL 32
# define COP_ERR_PARSE 33


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
//int readCopleyResp(struct MotorInfoStruct* copleyinfo); // Not used for anything.  I think...
int enableCopley(struct MotorInfoStruct* copleyinfo);
int disableCopley(struct MotorInfoStruct* copleyinfo);
long int queryCopleyInd(char ind[],struct MotorInfoStruct* copleyinfo);
int readCopleyResp(char *outs, int *l, struct MotorInfoStruct* copleyinfo);
void resetCopley(char *address, struct MotorInfoStruct* copleyinfo);
void restartCopley(char *address, struct MotorInfoStruct* copleyinfo);

extern struct MotorInfoStruct reactinfo; /* declared in copleycommand.c        
                                          *                                   
                                          */
extern struct MotorInfoStruct elevinfo;

/* Defined in motors.c*/
void bprintfverb(buos_t l, unsigned short int verb_level_req, unsigned short int verb_level_comp, const char* fmt, ...);
#endif
