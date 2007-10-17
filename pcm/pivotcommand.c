#include <sys/select.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>

#include "blast.h"

/* EZ Servo status bit masks (Copied from mcp/actuators.c) */
#define EZ_ERROR  0x0F
#define EZ_READY  0x20
#define EZ_STATUS 0x40

/* EZ Servo Error error numbers */
#define EZ_ERR_OK      0 /* No error */
#define EZ_ERR_INIT    1 /* Initialisation error */
#define EZ_ERR_BADCMD  2 /* Bad command */
#define EZ_ERR_BADOP   3 /* Bad operand */
#define EZ_ERR_COMM    5 /* Communications error */
#define EZ_ERR_NOINIT  7 /* Not initialised */
#define EZ_ERR_OVER    9 /* overload error */
#define EZ_ERR_NOMOVE 11 /* Move Not allowed */
#define EZ_ERR_BUSY   15 /* Command overflow */

/* Define command strings to return requested position,
   encoder position, and slew velocity*/
#define QUER_COMPOS "/1?0\r\n"
#define QUER_VEL    "/1?2\r\n"
#define QUER_POS    "/1?8\r\n"
#define QUER_STAT   "/1Q\r\n"


extern struct PivotInfoStruct pivotinfo; /* Contains the status info and file
                                          * descriptor
                                          */

/*
 * Open_pivot: opens a connection to the address given which 
 * is hopefully to the pivot controller.  Also sets up the 
 * connection, and tests the baud rate.  
 *
 */



void open_pivot(char *address)
{
  char a[256];
  strcpy(a, address);
  
  pivotinfo.fd = open(address, O_RDWR | O_NOCTTY | O_NDELAY);
  if (fdpiv==-1)
  {
   /*
    * Could not open the port.
    */
  
    berror(err,"open_port: Unable to open %s - ",a);
    pivotinfo.open=0;
  }
  else
  {
    fcntl(fdpiv, F_SETFL, 0);
    pivotinfo.open=1
  }  
  pivotinfo.init=0;
}

int setopts(int bdrate)
{
  struct termios options;
  /*
   * Get the current options for the port...
   */
  tcgetattr(fdpiv, &options);
  
  /*
   * Set the baud rate to bdrate.  Default is B9600.
   */

  switch( bdrate)
  {
  case 9600:
    cfsetispeed(&options, B9600);               //input speed
    cfsetospeed(&options, B9600);               //output speed
    bprintf(info,"setopts:Setting baud rate to 9600\n");
    break;
  case 19200:
    cfsetispeed(&options, B19200);               //input speed
    cfsetospeed(&options, B19200);              //output speed
    bprintf(info,"setopts:Setting baud rate to 19200\n");
    break;
  case 38400:
    cfsetispeed(&options, B38400);               //input speed
    cfsetospeed(&options, B38400);               //output speed
    bprintf(info,"setopts:Setting baud rate to 38400\n");
    break;
  default:
    bprintf(info,"Invalid baud rate %i. Using the default 9600.\n",bdrate);
    cfsetispeed(&options, B9600);               //input speed
    cfsetospeed(&options, B9600);               //output speed
    break;
  }

   /*
    * Enable the receiver and set local mode...
    */

  
  options.c_cflag |= (CLOCAL | CREAD);

  // Setting the Parity
  options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;  // 1 Stop Bit 
    options.c_cflag &= ~CSIZE;  // Mask the character size bits
  options.c_cflag |= CS8;       // Select 8 data bits

  // Disable Hardware Flow Control
  options.c_cflag &= ~CRTSCTS;

  /* Enter Local Options */
  options.c_lflag = 0;
  options.c_lflag |= ICANON;                   //enable canonical (line-based) mode
  
  /* Enter Input Options */
  options.c_iflag = 0;
  //  options.c_iflag = ICRNL;                     //map '\r' to '\n' on input

  /* Enter Output Options */
  options.c_oflag = 0;

 
  /*
   * Set the new options for the port...
   */
  
   tcsetattr(fdpiv, TCSANOW, &options);
}

void close_pivot()
{
  close(fdpiv);
}
