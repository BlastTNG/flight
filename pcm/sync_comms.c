/*************************************************************************** 

pcm: the Spider master control program

This software is copyright (C) 2002-2013 University of Toronto

This file is part of pcm.

pcm is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
at your option) any later version.
 
pcm is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with pcm; if not, write to the Free Software Foundation, Inc.,
59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

**************************************************************************/


/*************************************************************************
 
 sync_comms.c -- pcm code to read data from MCE Sync Box over RS-232.

*************************************************************************/

#include <stdio.h>           
#include <stdlib.h>
#include <math.h>         
#include <string.h>
#include <sys/time.h>        // time structures for select()
#include <pthread.h>         // POSIX threads
#include <unistd.h>          // POSIX symbolic constants
#include <termios.h>         // POSIX terminal control definitions
#include <fcntl.h>           // file control definitions

#include "command_struct.h"
#include "mcp.h" 
#include "sync_comms.h"      // sync box comms function declarations 

#define SYNC_DEV "/dev/ttySI0"
#define MAX_BYTES 10000
//#define SYNCBOX_VERBOSE

static pthread_t synccomm_id; // thread ID
struct SyncInfo syncinfo;     // device status info -- see sync_comms.h
struct SyncData SyncBoxData;  // data from device -- see sync_comms.h
static void* syncComm(void* arg);

void nameThread(const char*);	// in mcp.c
extern short int InCharge;    // in tx.c

/* create sync box serial thread */
void startSync()
{  
  //bprintf(info, "startSync: creating sync box serial thread");
  pthread_create(&synccomm_id, NULL, syncComm, NULL );
}

/* end sync box serial thread */
void endSync()  // declare in mcp.c along with startChrgCtrl
{
  int i = 0;
  syncinfo.closing = 1; // tells the serial thread to shut down

  while ((syncinfo.open == 1) && (i++ < 100)) {
    usleep(10000);
  }
}

/* thread routine: continously poll sync box for data */

void* syncComm(void* arg)
{
  int n_reconn=0; // loop constants

  /* initialize values in sync box info struct */
  syncinfo.open = 0;
  syncinfo.err = 0;
  syncinfo.closing = 0;
  syncinfo.reset = 0;

  nameThread("SynCom");

  /* check whether this is the ICC */
  while (!InCharge) {
    usleep(20000);
  }

  /* main loop */
  while (1) {
    if (syncinfo.closing == 1) {
      close_sync();
      usleep(10000); 
    } else if (syncinfo.reset == 1) {
      /* initialize values in sync box info struct */
      if (syncinfo.open) {
        /* if there's an error, reset connection */
        close_sync();
        if (n_reconn == 0) {
          bprintf(info,"Error occurred: attempting to re-open serial port.");
          n_reconn = 1;
        } 
      }
      
      syncinfo.open = 0;
      syncinfo.err = 0;
      syncinfo.closing = 0;
      syncinfo.reset = 0;
      
      while (syncinfo.open == 0) {
        open_sync();
        
        if (syncinfo.open == 0) {
          if (n_reconn == 10) { 
            bprintf(err,"Failed to re-open sync box after 10 attempts.");
          }          
          n_reconn++;         
          sleep(1);
        }
      }
      
      if (n_reconn>9) { // error had been reported
        bprintf(info, "Opened sync box on attempt %i.",  n_reconn);
        syncinfo.reset = syncinfo.err = 0;       
      }
    } else {                       // query the sync box      
      
      /* only query if not commanding */
      if (CommandData.sync_box.cmd) { 
        CommandData.sync_box.cmd = 0;
        command_sync(CommandData.sync_box.write_param, 
                     CommandData.sync_box.param_value);
        usleep(10000);
      } else {
        query_sync();
        
        /* check to see if read values are equal to the intended values
         * if not, command them to be on the next cycle */
        if (SyncBoxData.row_len != CommandData.sync_box.rl_value) {
          CommandData.sync_box.cmd = 1;
          CommandData.sync_box.write_param = sync_rl;
          CommandData.sync_box.param_value = CommandData.sync_box.rl_value;
        } else if (SyncBoxData.num_rows != CommandData.sync_box.nr_value) {
          CommandData.sync_box.cmd = 1;
          CommandData.sync_box.write_param = sync_nr;
          CommandData.sync_box.param_value = CommandData.sync_box.nr_value;
        } else if (SyncBoxData.free_run != CommandData.sync_box.fr_value) {
          CommandData.sync_box.cmd = 1;
          CommandData.sync_box.write_param = sync_fr;
          CommandData.sync_box.param_value = CommandData.sync_box.fr_value;
        }
          
        usleep(10000);
      }

      /* error handling for communication failures or corrupted data */
      /* TODO: have some condition under which err is set: */
      //syncinfo.err = 1;
      
      if (syncinfo.err == 1) {
        syncinfo.reset = 1;
        /* put sync box data values into obvious error state */
        SyncBoxData.row_len = 0;
        SyncBoxData.num_rows = 0;
        SyncBoxData.free_run = 0;
        continue; // go back up to top of infinite loop
      } else if ((SyncBoxData.row_len!=0)&&(n_reconn>0)) {
        bprintf(info, "Re-established communication with sync box");
        #ifdef SYNCBOX_VERBOSE
        bprintf(info, "Re-established communication with sync box");
        #endif
        n_reconn = 0; // managed this query cycle w/o errors; 
                      // reset for the next one 
      }
    
      usleep(10000); 
    }
  }
  return NULL;
}


/* open serial port */
void open_sync(void)
{  
  struct termios settings;
    
  if ( (syncinfo.fd = open(SYNC_DEV, O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0 ) {
    /* port failed to open */
#ifdef SYNCBOX_VERBOSE
    berror(err, "Sync box port %s failed to open", SYNC_DEV); 
#endif 
    syncinfo.open = 0;
    return;
  } else {
    fcntl(syncinfo.fd, F_SETFL, 0);
    syncinfo.open = 1;
  }
	
  /* initialize settings by getting terminal attributes */
  if (tcgetattr(syncinfo.fd, &settings)) {
    berror(err, "Failed to get terminal attributes");
    syncinfo.open = 0;
    return;
  }
  
  /* sync box only works at 9600 baud */
  if (cfsetispeed(&settings, B9600)) { // set input baud rate
    berror(err,"Error setting input baud rate");
    syncinfo.open = 0;
    return; 
  }

  if (cfsetospeed(&settings, B9600)) { // set output baud rate
    berror(err, "Error setting output baud rate");
    syncinfo.open = 0;
    return;
  }

  /* before explicitly setting certain flags, put things in most basic
   * terminal state: */ 
  cfmakeraw(&settings);
  
  /* clear character size; set no parity bits; set one stop bit */
  settings.c_cflag &= ~(CSTOPB | CSIZE | PARENB);

  /* set 8 data bits; set local port; enable receiver */
  settings.c_cflag |= (CS8 | CLOCAL | CREAD);

  /* disable all software flow control */
  settings.c_iflag &= ~(IXON | IXOFF | IXANY | IGNCR);
  settings.c_iflag |= ICRNL;
  
  /* disable output processing (raw output) */
  settings.c_oflag &= ~OPOST;

  /* disable input processing (raw input) */
  settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  if ( tcsetattr(syncinfo.fd, TCSANOW, &settings) ) {
    berror(err, "Failed to set terminal attributes");
    syncinfo.open = 0;
    return;
  }
}

/* close the serial port */
void close_sync(void)
{
  #ifdef SYNCBOX_VERBOSE
  bprintf(info, "close_sync: closing serial port.");
  #endif

  if (syncinfo.open == 0) {
    #ifdef SYNCBOX_VERBOSE
    bprintf(info, "close_sync: port is already closed!");
    #endif
  } else {
    #ifdef SYNCBOX_VERBOSE
    bprintf(info, "close_sync: closing serial port.");
    #endif
    if (syncinfo.fd >= 0) {
      close(syncinfo.fd);
    }

    syncinfo.open = 0;
    #ifdef SYNCBOX_VERBOSE
    bprintf(info, "close_sync: connection to port is now closed.");
    #endif
  }    
}

/* query the sync box parameters */
void query_sync(void)
{
  int write_num;
  int read_num;
  int rxchar = -1;
  int bytes_received = 0;
  int count=0;

  char data[MAX_BYTES];
  
  struct timeval tv;

  unsigned short found_params = 0;
  unsigned short data_avail = 0;
  
  fd_set rfds;

  /* flush the input and output streams*/
  tcflush(syncinfo.fd, TCIOFLUSH);

  if ( (write_num = write(syncinfo.fd, "?\r", 2)) < 0 ){
    #ifdef SYNCBOX_VERBOSE
    berror(err, "failed to write ? command to port");
    #endif
  }

  tv.tv_sec = 1.0;
  tv.tv_usec = 0;

  FD_ZERO(&rfds);
  FD_SET(syncinfo.fd, &rfds);

  data_avail = select(FD_SETSIZE, &rfds, NULL, NULL, &tv);
  
  if (!data_avail) {  // select() returns zero if the timeout is reached
    bytes_received = 0;
    #ifdef SYNCBOX_VERBOSE
    bprintf(err, "Communication with sync box timed out.");
    #endif
  }

  FD_ZERO(&rfds);
  FD_SET(syncinfo.fd, &rfds);

  while (data_avail) {
    if(select(FD_SETSIZE, &rfds, NULL, NULL, &tv)) {
      read_num = read(syncinfo.fd, &rxchar, 1);

      if (read_num <= 0) {
          bytes_received = 0;
          data_avail = 0; // break out of while loop
          #ifdef SYNCBOX_VERBOSE
          berror(err, "read failed");
          #endif
      }   else {
          rxchar = rxchar & 0xFF;  
          data[bytes_received++] = rxchar;
      }

      if (bytes_received >= MAX_BYTES) {
          bytes_received = 0;
          data_avail = 0;
          #ifdef SYNCBOX_VERBOSE
          bprintf(err, "response from sync box exceeded max bytes");
          #endif
      }
    } else {
      data_avail = 0;
    }
  }

  data[bytes_received] = '\0';

  while(!found_params && (count < bytes_received)) {
    found_params =  sscanf(data+count, 
                    "FRun_Count = %d\nRow_Len = %d\nNum_Row = %d\n", 
                    &(SyncBoxData.free_run), &(SyncBoxData.row_len), 
                    &(SyncBoxData.num_rows) );
    count++;
  }

  if (!found_params) {
    #ifdef SYNCBOX_VERBOSE
    bprintf(err, "failed to find parameters in data read from sync box");
    #endif
    syncinfo.err = 1;
  } 
}

/* write a parameter value to the sync box */
void command_sync(enum SyncParam write_param, int value) 
{
  int write_num;
  int n_val;

  char cmd[9];

  switch (write_param) {
    case sync_fr:
      n_val = sprintf(cmd, "fr %d\r", value);
      break;
    case sync_nr:
      n_val = sprintf(cmd, "nr %d\r", value);
      break;
    case sync_rl:
      n_val = sprintf(cmd, "rl %d\r", value);
      break;
    default:
      #ifdef SYNCBOX_VERBOSE
      bprintf(err, "invalid parameter type specified");
      #endif
      return;
  }
  
  #ifdef SYNCBOX_VERBOSE
  bprintf(info, "n_val = %d, cmd written = %s\n", n_val, cmd);
  #endif

  if ( (write_num = write(syncinfo.fd, cmd, n_val)) < 0) { 
    #ifdef SYNCBOX_VERBOSE
    berror(err, "failed to write cmd %s to port", cmd); 
    #endif
    syncinfo.err = 1;
  }
  
  tcflush(syncinfo.fd, TCIOFLUSH);
}
