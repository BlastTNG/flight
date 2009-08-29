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
struct CopleyInfoStruct *get_motor_pointer(enum MotorType motor)
{
  switch( motor)
    {
    case rw:
      return &reactinfo;
      break;
    case elev:
      return &elevinfo;
      break;
    default: // Technically this can't happen
      bprintf(err,"CopelyComm open_copley: Invalid motor type.  Motor cannot be opened");
      return NULL;
      break;
    }
}

void open_copley(char *address, enum MotorType motor)
{
  char a[256];
  strcpy(a, address);

  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor);

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
  //  int n;

  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.

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

void setopts_copley(int bdrate,enum MotorType motor)
{
  struct termios options;
  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.
  /*                                                                          
   * Get the current options for the port...                                  
   */
  tcgetattr(copleyinfo->fd, &options);

  /*                                                                          
   * Set the baud rate to bdrate.  Default is B115200.                        
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
    case 115200:
      cfsetispeed(&options, B115200);               //input speed             
      cfsetospeed(&options, B115200);               //output speed            
      bprintf(info,"setopts:Setting baud rate to 115200\n");
      break;
    default:
      bprintf(info,"Invalid baud rate %i. Using the default 115200.\n",bdrate\
	      );
      cfsetispeed(&options, B115200);               //input speed             
      cfsetospeed(&options, B115200);               //output speed            
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
  options.c_lflag &= ~ICANON;                   //disable canonical (line-based) mode                                        
                                                // We want raw character mode                                                

  /* Enter Input Options */
  options.c_iflag = 0;
  //  options.c_iflag = ICRNL;                     //map '\r' to '\n' on input                                               

  /* Enter Output Options */
  options.c_oflag = 0;


  /*                                                                                                                         
   * Set the new options for the port...                                                                                     
   */

  tcsetattr(copleyinfo->fd, TCSANOW, &options);
}


void configure_copley(enum MotorType motor)
{
  int n,m;
  bprintf(info,"reactComm configure_react: Testing a 115200 baud rate...\n");
  setopts_copley(115200,rw);
  //  n = areWeDisabled();

}
