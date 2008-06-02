/*
 * pivotcommand.h: Header file for commands to be sent to 
 * the pivot motor controller on SPIDER.
 *
 */
#define PIV_ACCEL 40000 // units: encoder ticks per second squared

#ifndef PIVOTCOMMAND_H
#define PIVOTCOMMAND_H

#include <stdarg.h>  /* ANSI C variable arguments (va_list, va_start, va_end) */

#include "motordefs.h"

struct PivotInfoStruct {
  int fd; // File descriptor for the pivot.
  int open; // 0 is closed, 1 is open
  int loop; // 1 ->there is a velocity loop running
            // 0 ->there is no velocity loop running
            // -1 -> there is no velocity loop running 
            //       because of some sort of error. 
  int init; // 0 has not yet been initialized
            // 1 has been initialized with no errors
            // -1 initialization was attempted but failed
  int err;  // Gives the communication error status:
            // 3 running configure_pivot failed.  
            // --> TODO: later this condition will trigger
            //           a power cycle of the controller
            // 0 No errors
  int ldir; // Loop direction:
            // 1 -> forward (i.e. /1P0)
            //-1 -> backward (i.e. /1D0)
  int closing; // 1 if in the process of closing down.
};

// TODO lmf: Leave this here for now but if I decide to use the same
// structure for the reaction wheel then I should move this definition 
// to motors.cpp
struct MotorCommandStruct {
  char *c;  // pointer to the command string
  int nc;   // character length of command string 
  char *r;  // pointer to the response string
  int nr;   // length of the response string
  int stat; // Status of the command:
            // 0 has not been sent
            // 1 has been sent successfully 
            // -1 could not be sent
};

#define QUER_COMPOS "/1?0\r\n"
#define QUER_VEL    "/1?2\r\n"
#define QUER_POS    "/1?8\r\n"
#define QUER_STAT   "/1Q\r\n"

// Pivot encoder counts per degree = 500 (encoder) * 353 (gearbox) * 32.768 ticks/s
//                                 /360 degrees
#define COUNTS_PER_DEGREE 16065.422
#define COUNTS_PER_ETURN 176500

#define PIVOT_ACCEL 50000
#define PIVOT_MIN_VEL 11

//enum CheckType {resp, comm, both}; //Now defined in motordefs.h ...

void open_pivot(char *address);

void close_pivot();

void configure_pivot();

void run_command(char cmd[],char tag[]);

int check_ready(enum CheckType check);

void send_pivotcmd(char cmd[]);

void check_resp();
int check_sresp(int statcheck1, int statcheck2);

int checkstatus(char *respstr, int statcheck1, int statcheck2);

extern struct PivotInfoStruct pivotinfo;

unsigned long int getquery(char *cmd);

void start_loop(double v);

void change_piv_vel(double v);

#endif
