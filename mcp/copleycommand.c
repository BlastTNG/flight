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

#define COPLEYCOM_MUS_WAIT 12000 // wait time after a command is given                                                         
                                  // TODO: optimize this wait time, is it even necessary?        
#define SELECT_COP_MUS_OUT 200000 // time out for reading from the Copley controller  
struct MotorInfoStruct reactinfo;  
struct MotorInfoStruct elevinfo; /* These file descriptors contain the status
                                     information for each motor and the file descriptor
				   */

void copyouts(char *in, char *out) {
  int i;

  for (i=0; i<strlen(in); i++) {
    out[i] = (in[i]=='\r' ? ' | ' : in[i]);
  }
  out[i] = '\0';
}

void open_copley(char *address, struct MotorInfoStruct* copleyinfo)
{
  char a[256];
  strncpy(a, address,254);

  //  struct MotorInfoStruct* copleyinfo;  
  //  copleyinfo = get_motor_pointer(copleyinfo;

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

void close_copley(struct MotorInfoStruct* copleyinfo)
{
  int n;

  usleep(1000000); // Wait a second to let mcp close the current loop.                                                                           
  bprintf(info,"%sComm: Closing connection to Copley controller.",copleyinfo->motorstr);
  n = disableCopley(copleyinfo);

  if(n!=0) {
    bprintf(err,"%sComm close_copley: Disabling Copley controller failed.",copleyinfo->motorstr);
  }

  bprintf(info,"%sComm close_copley: Closing serial port.",copleyinfo->motorstr);
  close(copleyinfo->fd); 
}

void setopts_copley(int bdrate,struct MotorInfoStruct* copleyinfo)
{
  struct termios options;
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
      bprintf(info,"%s setopts_copley:Setting baud rate to 9600\n",copleyinfo->motorstr);
      break;
    case 19200:
      cfsetispeed(&options, B19200);               //input speed              
      cfsetospeed(&options, B19200);              //output speed              
      bprintf(info,"%sComm setopts_copley:Setting baud rate to 19200\n",copleyinfo->motorstr);
      break;
    case 38400:
      cfsetispeed(&options, B38400);               //input speed              
      cfsetospeed(&options, B38400);               //output speed             
      bprintf(info,"%sComm setopts_copley:Setting baud rate to 38400\n",copleyinfo->motorstr);
      break;
    case 115200:
      cfsetispeed(&options, B115200);               //input speed             
      cfsetospeed(&options, B115200);               //output speed            
      bprintf(info,"%sComm setopts_copley:Setting baud rate to 115200\n",copleyinfo->motorstr);
      break;
    default:
      bprintf(info,"%sComm setopts_copley: Invalid baud rate %i. Using the default 115200.\n",copleyinfo->motorstr,bdrate);
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

void send_copleycmd(char cmd[], struct MotorInfoStruct* copleyinfo)
{
  int l = strlen(cmd);
  int n;

#ifdef DEBUG_COPLEY
  bprintf(info,"%sComm send_copleycmd: cmd = %s",copleyinfo->motorstr,cmd);
#endif // DEBUG_COPLEY

  n = write(copleyinfo->fd,cmd,l);
  if (n < 0)
    bprintf(err,"%sComm send_copleycmd: failed.",copleyinfo->motorstr);

  switch( copleyinfo->bdrate) {
  case 9600:
    usleep((12*COPLEYCOM_MUS_WAIT));
    break;
  case 38400:
    //usleep((3*COPLEYCOM_MUS_WAIT));
    break;
  case 115200:
    usleep(COPLEYCOM_MUS_WAIT);
    break;
  default:
    usleep((12*COPLEYCOM_MUS_WAIT));
    break;
  }
}

void configure_copley(struct MotorInfoStruct* copleyinfo)
{
  int n,m,i;
  setopts_copley(9600,copleyinfo);
  copleyinfo->bdrate=9600;
  bprintf(info, "%sComm configure_copley: Try communicating at 9600 baud rate",copleyinfo->motorstr);
  n = ping_copley(copleyinfo);
  if(n > 0)
    {
      bprintf(info,"%sComm configure_copley: Controller responds to a 9600 baud rate.",copleyinfo->motorstr);
      //
      bprintf(info,"%sComm configure_copley: Attempting to set baud rate to 38400",copleyinfo->motorstr);
      m = check_copleyready(comm,copleyinfo);
      if(m >= 0)
	{
	  send_copleycmd("s r0x90 38400\r",copleyinfo);
	}  
      m = check_copleyready(resp,copleyinfo);
      if(m >= 0)
	{
	  bprintf(info,"%sComm configure_copley: Hey it responded!  Wicked!",copleyinfo->motorstr);
          i=checkCopleyResp(copleyinfo);
	}
      setopts_copley(38400,copleyinfo);
      copleyinfo->bdrate=38400;

// Check to make sure that there aren't leftover bytes to be read from the serial port.
//      clearCopleyPort(copleyinfo);

      //Check to make sure that the motor responds to the new 38400 baud rate.
      i = ping_copley(copleyinfo);
      if(i > 0)
	{
	  bprintf(info, "%sComm configure_copley: Controller now responds to a 38400 baud rate.",copleyinfo->motorstr);
          copleyinfo->init=1;
          copleyinfo->err=0;
          return;
	}
      else
	{
	  bprintf(err, "%sComm configure_copley: Controller does not respond to a 38400 baud rate!",copleyinfo->motorstr);
          copleyinfo->init=0;
          copleyinfo->err=3;
	  return;
	}

  //mark
    }  
  else
    {
      bprintf(info, "%sComm configure_copley: Controller does not respond to a 9600 baud rate.",copleyinfo->motorstr);
      bprintf(info, "%sComm configure_copley: Setting Baud rate to 38400",copleyinfo->motorstr);
      setopts_copley(38400,copleyinfo);
      setopts_copley(38400,copleyinfo);
      copleyinfo->bdrate=38400;
      i = ping_copley(copleyinfo);
      if(i > 0)
	{
	  bprintf(info, "%sComm configure_copley: Controller now responds to a 115200 baud rate.",copleyinfo->motorstr);
          copleyinfo->init=1;
          copleyinfo->err=0;
          return;
	}
      else
	{
	  bprintf(info, "%sComm configure_copley: Setting Baud rate to 115200",copleyinfo->motorstr);
	  setopts_copley(115200,copleyinfo);
          copleyinfo->bdrate=38400;
          i = ping_copley(copleyinfo);
          if(i > 0)
	    {
	  bprintf(info, "%sComm configure_copley: Controller now responds to a 115200 baud rate.",copleyinfo->motorstr);
          copleyinfo->init=1;
          copleyinfo->err=0;
          return;
	    }
	  else
	    {
	      bprintf(err, "%sComm configure_copley: Controller does not respond to a 115200 baud rate!",copleyinfo->motorstr);
	      copleyinfo->init=0;
	      copleyinfo->err=3;
	    }
	  return;
	}
    }
  //TODO-LMF: write some error handling in case configuring the controller fails.
}

int check_copleyready(enum CheckType check, struct MotorInfoStruct* copleyinfo)
{
  int n=0;
  int m;
  int max_fd;
  fd_set         input;
  fd_set         output;
  struct timeval timeout;
  max_fd=copleyinfo->fd+1;
  timeout.tv_sec=0;
  timeout.tv_usec=SELECT_COP_MUS_OUT;
  FD_ZERO(&input);
  FD_ZERO(&output);
  //  bprintf(info,"%sComm check_copleyready: File descriptor is %i",copleyinfo->motorstr,copleyinfo->fd);
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
    berror(err, "%sComm check_rwready: CheckType is in valid.",copleyinfo->motorstr);
    return -3;
    return -3;
    break;
  }
  /* Was there an error? */
  if (n < 0) {
    bprintf(err,"%sComm: Select command failed!",copleyinfo->motorstr);
    return -2;
  } else if (n==0) {
    bprintf(warning,"%sComm: Select call timed out.",copleyinfo->motorstr);
    return -1;
  } else {
    // Sets a 2 bit integer m.                                                                                            
    // m=1 ready to be written into                                                                                       
    // m=2 there is something to be read                                                                                  
    // m=3 ready to recieve a command and something to be read.                                                           
    m=0;
    if (check==resp || check== both) {      
      if (FD_ISSET(copleyinfo->fd, &input))
	m|=2;
    }
    if (check==comm || check==both) {
      if (FD_ISSET(copleyinfo->fd, &output))
	m|=1;
    }

    return m;
  }
}


// Can we communciate with the controller?
// reads the status variable, but only looks to be sure it is getting the correct format
// in the response.
//
//  Returns 1 if the ping was sucessful.
int ping_copley(struct MotorInfoStruct* copleyinfo)
{
  char outs[255],outs_noCR[255];
  int n;
  n = check_copleyready(comm,copleyinfo);
  if(n >= 0)
    {
      //      bprintf(info,"copleyComm configure_copley: Ready to send command!");
      send_copleycmd("g r0xa0\r",copleyinfo);
    }  
  else
    {
      berror(err,"%sComm ping_copley: Serial port is not ready to command.",copleyinfo->motorstr);
      return -5;
    }
  n = read_line(outs,copleyinfo);
  if(n >= 0){
    copyouts(outs, outs_noCR);  
    bprintf(info,"%sComm ping_copley: Controller response= %s\n",copleyinfo->motorstr,outs_noCR);
    bprintf(info,"%sComm ping_copley: First character= %c\n",copleyinfo->motorstr,outs_noCR[0]);
    if(outs[0]== 'v' || outs[0]== 'e')
      {
	return 1;
      }
    else
      {
	bprintf(warning,"%sComm ping_copley: The controller response is incorrect.",copleyinfo->motorstr);
	bprintf(info,"%sComm ping_copley: Controller response= %s\n",copleyinfo->motorstr,outs_noCR);
	return 0;
      }
    }
  else if (n==-1){
    berror(err,"%sComm ping_copley: Select failed.",copleyinfo->motorstr);
    return -1;
  }
  else {
    outs[10]='\0';
    bprintf(warning,"%sComm ping_copley: Controller responded with garbage. outs=%s",copleyinfo->motorstr,outs);
    return 0;
  }

}

// Check the controller response after a command.
// If the response from the controller is "ok" (i.e. no errors) return 0.
int checkCopleyResp(struct MotorInfoStruct* copleyinfo)
{
  char outs[255],outs_noCR[255];
  int n,m,i,errcode;
  char* ptr;
  n = read_line(outs,copleyinfo);
  copyouts(outs, outs_noCR);
  if(n<0)
    {
      return n;
    }
  bprintf(info,"%sComm checkCopleyResp: Controller response= %s\n",copleyinfo->motorstr,outs_noCR);
  // Did the controller respond ok?
  if(outs_noCR[0]=='o' && outs_noCR[1]=='k')
    {
      return 0;
    }
  else  // This should mean it returned some kind of error code.
    {
      if(outs_noCR[0]=='e')
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
	  bprintf(warning,"%sComm checkCopleyResp: Controller returned error message %i",copleyinfo->motorstr,errcode);
	  return errcode;
	}
      else // Controller responded with garbage
	{
	  return -10;
	}
    }
}

// Read from the controller.
// Print the controller output
int readCopleyResp(struct MotorInfoStruct* copleyinfo)
{
  char outs[255],outs2[255];
  int n;
  n = read_line(outs,copleyinfo);
  if(n<0)
    {
      berror(warning,"%sComm readCopleyResp: Reading controller response failed!",copleyinfo->motorstr);
      return n;
    }
  copyouts(outs, outs2);
  bprintf(info,"%sComm readCopleyResp: Controller response= %s\n",copleyinfo->motorstr,outs2);
  return n;
}

int enableCopley(struct MotorInfoStruct* copleyinfo)
{
  int n;
  //  int count=0;
  send_copleycmd("s r0x24 2\r",copleyinfo);
  n = check_copleyready(resp,copleyinfo);
  if (n < 0)
    {
      berror(err,"%sComm enableCopley: Communication error.",copleyinfo->motorstr);
      return -1;
    }
  n=checkCopleyResp(copleyinfo);
  return n;
}

int disableCopley(struct MotorInfoStruct* copleyinfo)
{
  int n=0;
  bprintf(info,"%sComm disableCopley: Attempting to disable Copley motor controller.",copleyinfo->motorstr);
  //  int count=0;
  send_copleycmd("s r0x24 0\r",copleyinfo);
  n = check_copleyready(resp,copleyinfo);
  if (n < 0){
    berror(err,"%sComm disableCopley: Communication error.",copleyinfo->motorstr);
    return -1;
  }
  else {
    bprintf(info,"%sComm disableCopley: Copley controller disabled.",copleyinfo->motorstr);
  }
  n=checkCopleyResp(copleyinfo);
  return n;
}

long int getCopleyVel(struct MotorInfoStruct* copleyinfo)
{
  int n;
  char outs[255];
  char outs_noCR[255];
  long int vel;
  if(copleyinfo->closing==1) 
    {
      //      bprintf(info,"%sComm getCopleyVel: We are closing so I'm returning 42",copleyinfo->motorstr);
      return 42;// Don't query the serial port if we are 
                // closing the connection to the controller.
    }
  n = check_copleyready(comm,copleyinfo);

  if(n >= 0)
    {
      //      bprintf(info,"copleyComm configure_copley: Ready to send command!");
      send_copleycmd("g r0x18\r",copleyinfo);
    }  
  else
    {
      berror(err,"%sComm getCopleyVel: Serial port is not ready to command.",copleyinfo->motorstr);
      return -5;
    }

  n = read_line(outs,copleyinfo);
  if(n >= 0)
    {
      //      bprintf(info,"getCopleyVel n=%i",n);
      //      n = read(copleyinfo->fd,outs,254);
      outs[n] = '\0';
                  copyouts(outs, outs_noCR);
		  //                  bprintf(info,"%sComm getCopleyVel: Controller response= %s\n",copleyinfo->motorstr,outs_noCR);
		  //             bprintf(info,"copleyComm getCopleyVel: First character= %c\n",outs[0]);
      if(outs[0]== 'v')	{
        vel=atoi(outs+2);
	//	  bprintf(warning,"copleyComm getCopleyVel: Velocity= %ld",vel);
	return vel;
	}
      else {
	if(outs[0]== 'e')
	  {
	    bprintf(warning,"%sComm getCopleyVel: Controller returned error.",copleyinfo->motorstr);
	    return -1;  // TODO-LMF parse this error!
	  }
	else
	  {
	    bprintf(warning,"%sComm getCopleyVel: The controller response is incorrect.",copleyinfo->motorstr);
	    //bprintf(info,"copleyComm getCopleyVel: Controller response= %s\n",outs);
	    return 0;
	  }
      }
    } else {
    berror(err,"%sComm getCopleyVel: No useful response.",copleyinfo->motorstr);
    return -1;
  }

}

long int getCopleyPos(struct MotorInfoStruct* copleyinfo)
{
  int n;
  char outs[255];
  long int pos;
  if (copleyinfo->closing==1) return 42;// Don't query the serial port if we are 
                                       // closing the connection to the controller.

  n = check_copleyready(comm,copleyinfo);
  if(n >= 0) {
    //      bprintf(info,"copleyComm configure_copley: Ready to send command!");
    send_copleycmd("g r0x32\r",copleyinfo);
  } else {
    berror(err,"%sComm getCopleyPos: Serial port is not ready to command.",copleyinfo->motorstr);
    return -5;
  }
  n = read_line(outs,copleyinfo);
  if(n >= 0) {
    outs[n] = '\0';
    //     bprintf(info,"%sComm getCopleyPos: Controller response= %s\n",copleyinfo->motorstr,outs);
    //            bprintf(info,"copleyComm getCopleyPos: First character= %c\n",outs[0]);
    if(outs[0]== 'v') {
      pos=atoi(outs+2);
      //	  bprintf(warning,"copleyComm getCopleyPos: Position= %ld",pos);
      return pos;
    } else if (outs[0]== 'e') {
      bprintf(warning,"%sComm getCopleyPos: Controller returned error.",copleyinfo->motorstr);
      return -1;  // TODO-LMF parse this error!
    } else {
      bprintf(warning,"%sComm getCopleyPos: The controller response is incorrect.",copleyinfo->motorstr);
      //bprintf(info,"copleyComm getCopleyPos: Controller response= %s\n",outs);
      return 0;
    }
  } else {
    berror(err,"%sComm getCopleyPos: No useful response.",copleyinfo->motorstr);
    return -1;
  }
}

int read_line(char *outs,struct MotorInfoStruct* copleyinfo)
{
  int n;
  int i=0;
  int j=0;
  int done=0;
  int timeout=0;
  int timeoutlim=3;
  while(done==0){
    if(check_copleyready(resp,copleyinfo) >= 0){
      n = read(copleyinfo->fd,outs,1);
      //      bprintf(info,"Sweetness and light! n=%i, outs[i]=%c",n,*outs);
      if((*outs)=='\r'|| (*outs)=='\n' || i>=254) done=1;
      outs+=n;
      i+=n;
      timeout=0;
    } else {
      if(timeout==timeoutlim){ // If there is no data after two tries return an.
	if(j==timeoutlim) {// The controller never responded.
	  bprintf(err,"%sComm read_line: The controller did not respond.",copleyinfo->motorstr);
	  return -1;
	} else {
 	  bprintf(err,"%sComm read_line: Did not find the appropriate response end character.",copleyinfo->motorstr);
	  return -2; // For some reason the controller never found the end character.
	             // which means the response was probably garbage. 

	}
      }
      timeout++;
      usleep(2000);
    }
    j++;
  }
  return i;
}
