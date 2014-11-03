/* mcp: the BLAST master control program
 * 
 * This software is copyright (C) 2003-2006 University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stddef.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include "blast.h"
#include "pointing_struct.h"
#include "mcp.h"

#include <ctype.h>

#define MAGCOM "/dev/ttySI14"
extern short int InCharge; /* tx.c */

#define BDRATE B9600

void nameThread(const char*); /* mcp.c */

struct MagInfoStruct {
  int open; // 0 is closed, 1 is open
  int init; // 0 has not yet been initialized
  // 1 has been initialized with no errors
  // 2 initialization was attempted but failed
} maginfo;

void Magnetometer()
{
  nameThread("Mag");
  int attempts = 0;
  /* initialize values in maginfo structure */
  maginfo.init = 0;
  ACSData.mag_index = 0;
  
  while(!InCharge) {
    usleep(20000);
  }
  int fd;
  struct termios term;
  
  //try to open the port
  maginfo.open = 0;
  while (maginfo.open == 0) {
    if ((fd = open(MAGCOM, O_RDWR | O_NOCTTY)) < 0) {
      attempts++;
      usleep(20000);
    } else {
      maginfo.open = 1;
      if (attempts>=10) bprintf(info,"open on attempt #%i",attempts);
    }
  }
  
  //Set options
  tcgetattr(fd,&term);
  cfsetispeed(&term,BDRATE);
  cfsetospeed(&term,BDRATE);
  
  /*Control Modes*/
  term.c_cflag |= (CLOCAL | CREAD); //local connection, no modem control; enable receiving characters   
  term.c_cflag &= ~PARENB;   // No Parity
  term.c_cflag &= ~CSTOPB;   // 1 Stop Bit 
  term.c_cflag &= ~CSIZE;    // Mask the character size bits
  term.c_cflag |= CS8;       // 8 data bits
  term.c_cflag &= ~CRTSCTS;  //flow control off (no RTS/CTS)
  /*Local Modes*/
  term.c_lflag =0;
  //term.c_cc[VTIME]=0;
  //term.c_cc[VMIN]=0;
  //term.c_lflag |= ICANON;    // enable canonical (line-based) input
  term.c_lflag &= ~ICANON;    // disable canonical (line-based) input
  /*Input Modes*/
  //term.c_iflag = 0;
  //term.c_iflag = ICRNL;      // map CR to NL on input
  /*Output Modes*/
  term.c_oflag = 0;
  
  cfmakeraw(&term);
  
  /*Activate settings for the port*/
  tcsetattr(fd,TCSANOW,&term);
  
  fd_set input;
  struct timeval read_timeout;
  int res;
  
  char cmd[] = "*99C\r";
  int l = strlen(cmd);
  int m;
  m = write(fd,cmd,l);
  if (m<0) bprintf(err,"failed to send command.");
  char buf;
  char buffer[255];
  int n=0;
  int o=0;
  char x2[2],x3[2],x4[2],x5[2];
  char y2[2],y3[2],y4[2],y5[2];
  char z2[2],z3[2],z4[2],z5[2];
  char xsn,ysn,zsn;
  int error_reported = 0;
  int first_attempt = 1;

  x2[1] = x3[1] = x4[1] = x5[1] = 
  y2[1] = y3[1] = y4[1] = y5[1] = 
  z2[1] = z3[1] = z4[1] = z5[1] = 0;
  
  while(1) {
    read_timeout.tv_sec = 10;
    read_timeout.tv_usec = 0;
    o=0;
    usleep(10000);
    FD_ZERO(&input);
    FD_SET(fd, &input);
    res = select(fd+1,&input,NULL,NULL,&read_timeout);
    if (res==0) {
      if (first_attempt) {
        first_attempt = 0;
      } else if (!error_reported) {
        bprintf(info,"No response.  Attempting to reconnect.\n");
        error_reported = 1;
      }
      m = write(fd,cmd,l);
      if (m<0) bprintf(err,"failed to send command.");
    } else {
      do {
        n = read(fd,&buf,1);
        if (n < 0)
          berror(warning, "Read error");
        else
          buffer[o] = buf;
        o++;
      } while (buf != '\r');
    }
    
    //bprintf(info, "nZero: %d", nZero);
    
    if (o==28) {
      first_attempt = 0;
      if (error_reported) {
        bprintf(info, "Connected");
        error_reported = 0;
      }
      xsn=buffer[0]; //sign/space
      x2[0]=buffer[2]; //number/space
      x3[0]=buffer[4]; //number
      x4[0]=buffer[5]; //number
      x5[0]=buffer[6]; //number
      ysn=buffer[9]; //sign/space
      y2[0]=buffer[11]; //number/space
      y3[0]=buffer[13]; //number
      y4[0]=buffer[14]; //number
      y5[0]=buffer[15]; //number
      zsn=buffer[18]; //sign/space
      z2[0]=buffer[20]; //number/space
      z3[0]=buffer[22]; //number
      z4[0]=buffer[23]; //number
      z5[0]=buffer[24]; //number
            
      int x = 1000*(atoi(x2))+100*(atoi(x3))+10*(atoi(x4))+atoi(x5);
      int y = 1000*(atoi(y2))+100*(atoi(y3))+10*(atoi(y4))+atoi(y5);
      int z = 1000*(atoi(z2))+100*(atoi(z3))+10*(atoi(z4))+atoi(z5);
      if(xsn=='-') x*=-1;
      if(ysn=='-') y*=-1;
      if(zsn=='-') z*=-1;
      ACSData.mag_x = x; // atomic on single core
      ACSData.mag_y = y; 
      ACSData.mag_z = z;
      ACSData.mag_index++;
    }
  }
  close(fd);
  return;
  
}
