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

#if 0
void open_copley(char *address)
{
  char a[256];
  strcpy(a, address);

  reactinfo.fd = open(address, O_RDWR | O_NOCTTY | O_NDELAY);
  if (reactinfo.fd==-1)
    {
      /*
       * Could not open the port.
       */

      reactinfo.open=0;
    }
  else
    {
      fcntl(reactinfo.fd, F_SETFL, 0);
      reactinfo.open=1;
    }
  reactinfo.init=0;
}
#endif
