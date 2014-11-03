/*************************************************************************
 
 sync_comms.h -- function declarations and #define statements for 
                 sync box code. 		

*************************************************************************/

#ifndef SYNCBOX_H 
#define SYNCBOX_H

/* sync box info struct */
struct SyncInfo {
  int fd;                      // file descriptor
  int open;                    // 0 for closed, 1 for open
  int err;                     // 1 if a comms error occurred 
  int closing;                 // 1 if in process of closing
  int reset;                   // 1 if a reset is required
};

extern struct SyncInfo syncinfo;

/* sync box data struct written to by serial thread in sync_comms.c */
struct SyncData {
  unsigned int row_len;
  unsigned int num_rows;
  unsigned int free_run;
};

extern struct SyncData SyncBoxData;
                                         
/* function declarations */

/*************************************************************** 
 open_sync -- sets up a serial port for communication with the 
              sync box
 ***************************************************************/
 void open_sync(void);


/*************************************************************** 
 close_sync -- closes serial port for MODBUS communication
               with the charge controller
 ***************************************************************/
 void close_sync(void);

 
/*************************************************************** 
 query_sync -- queries sync box to determine its internal
               parameter values
 ***************************************************************/ 
 void query_sync(void);

/*************************************************************** 
 command_sync -- writes a parameter value to the sync box 

 arguments:

 write_param:    enum determining which parameter to write to
 value:          value to be written
 ***************************************************************/ 
void command_sync(enum SyncParam write_param, int value);

#endif  // SYNCBOX_H
