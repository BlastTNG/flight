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

#define COPLEYCOM_MUS_WAIT 100000 // wait time after a command is given                                                         
                                  // TODO: optimize this wait time, is it even necessary?        
#define SELECT_COP_MUS_OUT 200000 // time out for reading from the Copley controller  
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
  int n;

  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.

  copleyinfo->closing=1; // Tells copleyComm that the motor communcations are closing.                                                                        
  usleep(500000); // Wait half a second to let mcp close the current loop.                                                                           
  bprintf(info,"copleyComm: Closing connection to Copley controller.");

 n = disableCopley(rw);
 if(n==0)
   {
     //     checkCopleyStatus(n);
   }
 else
   {
     bprintf(err,"copleyComm close_copley: Disabling Copley controller failed.");
   }
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

void send_copleycmd(char cmd[], enum MotorType motor)
{
  int l = strlen(cmd);
  int n;
  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.
#ifdef DEBUG_COPLEY
  bprintf(info,"send_copleycmd: cmd = %s",cmd);
#endif // DEBUG_COPLEY                                                                                                          
  n = write(copleyinfo->fd,cmd,l);
  if (n < 0)
    bprintf(err,"send_copleycmd: failed.");
  usleep(COPLEYCOM_MUS_WAIT);
}

void configure_copley(enum MotorType motor)
{
  int n,m,i;
  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.
  setopts_copley(9600,motor);
  bprintf(info, "copleyComm: Setting Baud rate to 9600");
   n = ping_copley(motor);
  if(n > 0)
    {
      bprintf(info,"copleyComm configure_copley: Controller responds to a 9600 baud rate.");
      bprintf(info,"copleyComm configure_copley: Attempting to set baud rate to 115200");
      m = check_copleyready(comm,motor);
      if(m >= 0)
	{
	  send_copleycmd("s r0x90 115200\r",motor);
	}  
      m = check_copleyready(resp,motor);
      if(m >= 0)
	{
	  bprintf(info,"Hey it responded!  Wicked!");
          i=checkCopleyResp(motor);
	}
      setopts_copley(115200,motor);
      //Check to make sure that the motor responds to the new 115200 baud rate.
      i = ping_copley(motor);
      if(i > 0)
	{
	  bprintf(info, "copleyComm configure_copley: Controller now responds to a 115200 baud rate.");
          copleyinfo->init=1;
          copleyinfo->err=0;
          return;
	}
      else
	{
	  bprintf(err, "copleyComm configure_copley: Controller does not respond to a 115200 baud rate!");
          copleyinfo->init=0;
          copleyinfo->err=3;
	  return;
	}

  //mark
    }  
  else
    {
      bprintf(info, "copleyComm configure_copley: Controller does not respond to a 9600 baud rate.");
      bprintf(info, "copleyComm: Setting Baud rate to 115200");
      setopts_copley(115200,motor);
      i = ping_copley(motor);
      if(i > 0)
	{
	  bprintf(info, "copleyComm configure_copley: Controller now responds to a 115200 baud rate.");
          copleyinfo->init=1;
          copleyinfo->err=0;
          return;
	}
      else
	{
	  bprintf(err, "copleyComm configure_copley: Controller does not respond to a 115200 baud rate!");
          copleyinfo->init=0;
          copleyinfo->err=3;
	  return;
	}
    }
  //TODO-LMF: write some error handling in case configuring the controller fails.
}

int check_copleyready(enum CheckType check, enum MotorType motor)
{
  int n=0;
  int m;
  int max_fd;
  fd_set         input;
  fd_set         output;
  struct timeval timeout;
  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.
  max_fd=copleyinfo->fd+1;
  timeout.tv_sec=0;
  timeout.tv_usec=SELECT_COP_MUS_OUT;
  FD_ZERO(&input);
  FD_ZERO(&output);

  switch(check){
  case resp:
    FD_SET(copleyinfo->fd, &input);
    n = select(max_fd, &input, NULL, NULL, &timeout);
    break;
  case comm:
    FD_SET(copleyinfo->fd, &output);
    n = select(max_fd, NULL, &output, NULL, &timeout);
    break;
  case both:
    FD_SET(copleyinfo->fd, &input);
    FD_SET(copleyinfo->fd, &output);
    n = select(max_fd, &input, &output, NULL, &timeout);
    break;
  default:
    berror(err, "reactComm check_rwready: CheckType is in valid.");
    return -3;
    return -3;
    break;
  }
  /* Was there an error? */
  if (n < 0)
    {
      bprintf(err,"reactComm: Select command failed!");
      return -2;
    }
  else if (n==0)
    {
#ifdef DEBUG_RW
      bprintf(warning,"reactComm: Select call timed out.");
#endif
      return -1;
    }
  else
    {
      // Sets a 2 bit integer m.                                                                                            
      // m=1 ready to be written into                                                                                       
      // m=2 there is something to be read                                                                                  
      // m=3 ready to recieve a command and something to be read.                                                           
      m=0;
      if (check==resp || check== both)
	{
          if (FD_ISSET(copleyinfo->fd, &input))
            m+=2;
	}
      if (check==comm || check==both)
	{
          if (FD_ISSET(copleyinfo->fd, &output))
            m+=1;
	}
      if(n==0) 
	{
	  berror(fatal,"copleyComm checkready: Serial Poll error!");
	}
	//      bprintf(info, "reactComm: checksum returns %i.",m);                                                           
	return m;
    }
}


// Can we communciate with the controller?
// reads the status variable, but only looks to be sure it is getting the correct format
// in the response.
//
//  Returns 1 if the ping was sucessful.
int ping_copley(enum MotorType motor)
{
  char outs[255];
  int n;
  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.
  n = check_copleyready(comm,motor);
  if(n >= 0)
    {
      //      bprintf(info,"copleyComm configure_copley: Ready to send command!");
      send_copleycmd("g r0xa0\r",motor);
    }  
  else
    {
      berror(err,"copleyComm ping_copley: Serial port is not ready to command.");
      return -5;
    }
  n = check_copleyready(resp,motor);
  if(n >= 0)
    {
      n = read(copleyinfo->fd,outs,254);
            bprintf(info,"copleyComm ping_copley: Controller response= %s\n",outs);
            bprintf(info,"copleyComm ping_copley: First character= %c\n",outs[0]);
      if(outs[0]== 'v' || outs[0]== 'e')
	{
	  return 1;
	}
      else
	{
          bprintf(warning,"copleyComm ping_copley: The controller response is incorrect.");
          bprintf(info,"copleyComm ping_copley: Controller response= %s\n",outs);
	  return 0;
	}
    }
  else
    {
      berror(err,"copleyComm ping_copley: Select failed.");
      return -1;
    }

}

// Check the controller response after a command.
// If the response from the controller is "ok" (i.e. no errors) return 0.
int checkCopleyResp(enum MotorType motor)
{
  char outs[255];
  int n,m,i,errcode;
  static struct CopleyInfoStruct* copleyinfo;  
  char* ptr;
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.
  n = read(copleyinfo->fd,outs,254);
  if(n<0)
    {
      return n;
    }
  bprintf(info,"copleyComm checkCopleyResp: Controller response= %s\n",outs);
  // Did the controller respond ok?
  if(outs[0]=='o' && outs[1]=='k')
    {
      return 0;
    }
  else  // This should mean it returned some kind of error code.
    {
      if(outs[0]=='e')
	{
          m=1;
	  i=2;
	  errcode=0;
          ptr=outs;
          ptr++;
	  ptr++;
	  while(m)
	    {
              if(*ptr=='\0'|| i==254 )
		{
		  m=0;
		}
	      else
		{
		  errcode*=10+atoi(ptr);
		}
	      i++;
	    }
	  bprintf(warning,"copleyComm checkCopleyResp: Controller returned error message %i",errcode);
	  return errcode;
	}
      else // Controller responded with garbage
	{
	  return -10;
	}
    }
}

int enableCopley(enum MotorType motor)
{
  int n;
  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.
  //  int count=0;
  send_copleycmd("s r0x24 2\r",motor);
  n = check_copleyready(resp,motor);
  if (n < 0)
    {
      berror(err,"copleyComm disableCopley: Communication error.");
      return -1;
    }
  n=checkCopleyResp(motor);
  return n;
}
int disableCopley(enum MotorType motor)
{
  int n;
  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.
  //  int count=0;
  send_copleycmd("s r0x24 0\r",motor);
  n = check_copleyready(resp,motor);
  if (n < 0)
    {
      berror(err,"copleyComm disableCopley: Communication error.");
      return -1;
    }
  n=checkCopleyResp(motor);
  return n;
}

long int getCopleyVel(enum MotorType motor)
{
  int n;
  char outs[255];
  char* ptr;
  long int vel;
  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.
  n = check_copleyready(comm,motor);
  if(n >= 0)
    {
      //      bprintf(info,"copleyComm configure_copley: Ready to send command!");
      send_copleycmd("g r0x18\r",motor);
    }  
  else
    {
      berror(err,"copleyComm getCopleyVel: Serial port is not ready to command.");
      return -5;
    }
  n = check_copleyready(resp,motor);
  if(n >= 0)
    {
      n = read(copleyinfo->fd,outs,254);
      //            bprintf(info,"copleyComm getCopleyVel: Controller response= %s\n",outs);
      //            bprintf(info,"copleyComm getCopleyVel: First character= %c\n",outs[0]);
      if(outs[0]== 'v')
	{
          ptr=outs;
          ptr++;
	  ptr++;
          vel=atoi(ptr);
	  //	  bprintf(warning,"copleyComm getCopleyVel: Velocity= %ld",vel);
	  return vel;
	}
      else
	{
	  if(outs[0]== 'e')
	    {
	      bprintf(warning,"copleyComm getCopleyVel: Controller returned error.");
              return -1;  // TODO-LMF parse this error!
	    }
	  else
	    {
	      bprintf(warning,"copleyComm getCopleyVel: The controller response is incorrect.");
	      bprintf(info,"copleyComm getCopleyVel: Controller response= %s\n",outs);
	      return 0;
	    }
	}
    }
  else
    {
      berror(err,"copleyComm getCopleyVel: Select failed.");
      return -1;
    }

}

long int getCopleyPos(enum MotorType motor)
{
  int n;
  char outs[255];
  char* ptr;
  long int pos;
  static struct CopleyInfoStruct* copleyinfo;  
  copleyinfo = get_motor_pointer(motor); // Point to the right motor info struct.
  n = check_copleyready(comm,motor);
  if(n >= 0)
    {
      //      bprintf(info,"copleyComm configure_copley: Ready to send command!");
      send_copleycmd("g r0x32\r",motor);
    }  
  else
    {
      berror(err,"copleyComm getCopleyPos: Serial port is not ready to command.");
      return -5;
    }
  n = check_copleyready(resp,motor);
  if(n >= 0)
    {
      n = read(copleyinfo->fd,outs,254);
      //            bprintf(info,"copleyComm getCopleyPos: Controller response= %s\n",outs);
      //            bprintf(info,"copleyComm getCopleyPos: First character= %c\n",outs[0]);
      if(outs[0]== 'v')
	{
          ptr=outs;
          ptr++;
	  ptr++;
          pos=atoi(ptr);
	  //	  bprintf(warning,"copleyComm getCopleyPos: Position= %ld",pos);
	  return pos;
	}
      else
	{
	  if(outs[0]== 'e')
	    {
	      bprintf(warning,"copleyComm getCopleyPos: Controller returned error.");
              return -1;  // TODO-LMF parse this error!
	    }
	  else
	    {
	      bprintf(warning,"copleyComm getCopleyPos: The controller response is incorrect.");
	      bprintf(info,"copleyComm getCopleyPos: Controller response= %s\n",outs);
	      return 0;
	    }
	}
    }
  else
    {
      berror(err,"copleyComm getCopleyPos: Select failed.");
      return -1;
    }

}
