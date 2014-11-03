//===================================================================
// pivotcommand.h
//
// Modified from the reactcommand.h taken from the Spider
// mcp repository (version 1.8 last modified Tue Jun  3 03:09:08 2008)
//===================================================================

#ifndef AMCCOMMAND_H
#define AMCCOMMAND_H

#include <stdarg.h>  /* ANSI C variable arguments (va_list, va_start, va_end) */
#include "motordefs.h"

// Maximum reaction wheel current
#define MAX_AMC_CURRENT 10.0   // AMC controller is rated to 10 Amp continuous
                               // current.

// This is the maximum continuous current for which the reaction wheel 
// controller is rated to. Note that we will *never* ask for this much 
// current, but this value is needed for conversion of the requested 
// current to serial format in setRWCurrent.
#define PEAK_AMC_CURRENT 10.0 

// NOTE: The actual PEAK RW CURRENT IS 20 A, but that gives a lower than 
// expected current when used by setRWCur.  I have no idea why, maybe
// some kind of conversion from Peak to RMS?

#define PIV_RES_CTS 16384.0

#define AMC_ERR_TIMEOUT 5 // Number of consecutive serious errors before the
                             // thread attempts to reset the controller.

// Error masks.  Tells the controller which bits set in the amcinfo->err
// variable should count towards triggering a reset.
#define AMC_ERR_MASK 0x001e // Bit #2 (send command failed, implying that the 
                              // serial hub not powered.)
                            // Bit #3 (Controller response is incorrect)
                            // Bit #4 (Controller returns error)
                            // Bit #5 (Controller did not respond)

//#define DEBUG_AMC

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

enum CmdorQuery {cmd, query};

// Function Declarations

void open_amc(char *address, struct MotorInfoStruct* amcinfo);

void close_amc(struct MotorInfoStruct* amcinfo);

void setopts_amc(int bdrate, struct MotorInfoStruct* amcinfo);

void MakeSCHeadStruct(struct DriveIPVStruct *ValuesSend,struct SerialCommandHeadStruct *MessSendHead, unsigned short *crctable, char *header,char **command, int *commsize,enum CmdorQuery type, struct MotorInfoStruct* amcinfo);

void crccheck(unsigned short data, unsigned short *accumulator, unsigned short *crctable, struct MotorInfoStruct* amcinfo);

unsigned short crchware(unsigned short data, unsigned short genpoly, unsigned short accum);

void configure_amc(struct MotorInfoStruct* amcinfo);

int check_amcready(enum CheckType check, struct MotorInfoStruct* amcinfo, unsigned int waittime);

int send_amccmd(int index,int offset,int value,int nwords,enum CmdorQuery type, struct MotorInfoStruct* amcinfo);
int queryAMCInd(int index, int offset, int nwords, struct MotorInfoStruct* amcinfo);
int areWeDisabled(struct MotorInfoStruct* amcinfo);
int readAMCResp(int seq, unsigned char *outs, int *l, struct MotorInfoStruct* amcinfo);

int getAMCResp(int seq, int *val, int *l, struct MotorInfoStruct* amcinfo);

int checkAMCResp(int seq, struct MotorInfoStruct* amcinfo);

void checkAMCStatus(int stat, struct MotorInfoStruct* amcinfo);

void setWriteAccess(struct MotorInfoStruct* amcinfo);
int disableAMC(struct MotorInfoStruct* amcinfo);
int enableAMC(struct MotorInfoStruct* amcinfo);
int getAMCResolver(struct MotorInfoStruct* amcinfo);
int checkAMCAccess(struct MotorInfoStruct* amcinfo);
void resetAMC(char *address, struct MotorInfoStruct* amcinfo);
void restoreAMC(struct MotorInfoStruct* amcinfo);

/* Defined in motors.c*/
void bprintfverb(buos_t l, unsigned short int verb_level_req, unsigned short int verb_level_comp, const char* fmt, ...);
#endif
