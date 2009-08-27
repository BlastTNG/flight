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
#include <math.h>

#include "blast.h"
#include "copleycommand.h"
#include "motordefs.h"

struct CopleyInfoStruct reactinfo;  
struct CopleyInfoStruct elevinfo; /* These file descriptors contain the status
                                     information for each motor and the file descriptor
				   */

void open_copley(char *address, enum MotorType motor)
{
  char a[256];
  strcpy(a, address);

  static struct CopleyInfoStruct* copleyinfo;  
  switch( motor)
    { 
    case rw:
      copleyinfo = &reactinfo;
      break;
    case elev:
      copleyinfo = &elevinfo;
      break;
    default:
      bprintf(err,"CopelyComm open_copley: Invalid motor type.  Motor cannot be opened");
      return;
      break;
    }

  copleyinfo->fd = open(address, O_RDWR | O_NOCTTY | O_NDELAY);
  if (copleyinfo->fd==-1)
    {
      /*
       * Could not open the port.
       */

      copleyinfo->open=0;
    }
  else
    {
      fcntl(copleyinfo->fd, F_SETFL, 0);
      copleyinfo->open=1;
    }
  copleyinfo->init=0;
}

void close_copley(enum MotorType motor)
{
  int n;

  static struct CopleyInfoStruct* copleyinfo;  
  switch( motor)
    { 
    case rw:
      copleyinfo = &reactinfo;
      break;
    case elev:
      copleyinfo = &elevinfo;
      break;
    default:
      bprintf(err,"CopelyComm open_copley: Invalid motor type.  Motor cannot be opened");
      return;
      break;
    }

  copleyinfo->closing=1; // Tells copleyComm that the motor communcations are closing.                                                                        
  usleep(500000); // Wait half a second to let reactComm close the current loop.                                                                           
  bprintf(info,"copleyComm: Closing connection to Copley controller.");

  /*  TODO-LMF:  Write these functions!
 n = disableRW();
 if(n>0)
   {
     checkRWStatus(n);
   }
 else
   {
     bprintf(err,"reactComm close_react: Disabling RW controller failed.");
   }
  */
   close(copleyinfo->fd); 
}
