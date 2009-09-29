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
#define MAX_AMC_CURRENT 5.0 // TODO: This is just until I get an idea of the
                               // spin gains.
                               // AMC controller is rated to 10 Amp continuous
                               // current.

// This is the maximum continuous current for which the reaction wheel 
// controller is rated to. Note that we will *never* ask for this much 
// current, but this value is needed for conversion of the requested 
// current to serial format in setRWCurrent.
#define PEAK_AMC_CURRENT 20.0 

// NOTE: The actual PEAK RW CURRENT IS 20 A, but that gives a lower than 
// expected current when used by setRWCur.  I have no idea why, maybe
// some kind of conversion from Peak to RMS?


#define DISABLE_AMC // Keeps the reaction wheel in disabled mode.
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

extern struct MotorInfoStruct pivotinfo; /* declared in reactcommand.c
				          * 
				          */
enum CmdorQuery {cmd, query};


// Function Declarations

void open_amc(char *address);

void close_amc();

void setopts_amc(int bdrate);

void MakeSCHeadStruct(struct DriveIPVStruct *ValuesSend,struct SerialCommandHeadStruct *MessSendHead, unsigned short *crctable, char *header,char **command, int *commsize,enum CmdorQuery type);

void crccheck(unsigned short data, unsigned short *accumulator, unsigned short *crctable);

unsigned short crchware(unsigned short data, unsigned short genpoly, unsigned short accum);

void configure_amc();

int check_amcready(enum CheckType check);

int sendThisCommand(int index,int offset,int value,int nwords,enum CmdorQuery type);
int queryind(int index, int offset, int nwords);
int areWeDisabled();
int readAMCResp(int seq, char *response, int *l);

int getAMCResp(int seq, int *val, int *l);

int checkAMCResp(int seq);

void checkAMCStatus(int stat);

void setWriteAccess();
int disableAMC();
int enableAMC();

#endif
