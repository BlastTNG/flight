/*
 * Reactcommand.h: Header file for commands to be sent to 
 * the reaction wheel motor controller on SPIDER.
 *
 */

#ifndef REACTCOMMAND_H
#define REACTCOMMAND_H

#include <stdarg.h>  /* ANSI C variable arguments (va_list, va_start, va_end) */
#include "motordefs.h"

// Maximum reaction wheel current
#define MAX_RWHEEL_CURRENT 5.0 // TODO: This is just until I get an idea of the
                               // spin gains.
                               // RW controller is rated to 17.5 Amp continuous
                               // current.

// This is the maximum continuous current for which the reaction wheel 
// controller is rated to. Note that we will *never* ask for this much 
// current, but this value is needed for conversion of the requested 
// current to serial format in setRWCurrent.
#define PEAK_RW_CURRENT 35.0 

// NOTE: The actual PEAK RW CURRENT IS 40 A, but that gives a lower than 
// expected current when used by setRWCur.  I have no idea why, maybe
// some kind of conversion from Peak to RMS?


//#define DISABLE_RW // Keeps the reaction wheel in disabled mode.
//#define DEBUG_RW

struct ReactInfoStruct {
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
  int writeset; // 1 if write access has been set.
                // 0 if it has not yet been set
                // -1 if there is an error while sending the 
                // set write access command.
};

// Structures used by 

struct SerialCommandHeadStruct {
  char sof; /* Start of frame (should always be set to A5h) */
  char address;
  char controlbyte;
  char index;
  char offset;
  char datawords;
  char crc[2];
};

struct DriveIPVStruct {
  int index;
  int param;
  long int value;
  int readwrite;  // If readwrite is 1 then assume this command is to be sent.
  int counter;
  int offset;
  long int nwords;  // Each word is 2 bytes (0-65536)
};

struct DriveIPVResStruct {
  int sof; // Set to one if correct otherwise 0
  int type;
  int counter;
  int stat; // Status
  int nwords; 
  int crchead[2];
  long int value;
};

extern struct ReactInfoStruct reactinfo; /* declared in reactcommand.c
				          * 
				          */
enum CmdorQuery {cmd, query};


// Function Declarations

void open_react(char *address);

void close_react();

void setopts_rw(int bdrate);

void MakeSCHeadStruct(struct DriveIPVStruct *ValuesSend,struct SerialCommandHeadStruct *MessSendHead, unsigned short *crctable, char *header,char **command, int *commsize,enum CmdorQuery type);

void crccheck(unsigned short data, unsigned short *accumulator, unsigned short *crctable);

unsigned short crchware(unsigned short data, unsigned short genpoly, unsigned short accum);

void configure_react();

int check_rwready(enum CheckType check);

int sendThisCommand(int index,int offset,int value,int nwords,enum CmdorQuery type);
int queryind(int index, int offset, int nwords);
int areWeDisabled();
int readRWResp(int seq, char *response, int *l);

int getRWResp(int seq, int *val, int *l);

int checkRWResp(int seq);

void checkRWStatus(int stat);

void setWriteAccess();
int disableRW();
int enableRW();
int setRWCurrent(double curtarget);
void startRWLoop();

#endif
