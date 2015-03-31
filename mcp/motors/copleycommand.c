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
#include "command_struct.h"

#define COPLEYCOM_MUS_WAIT 12000 // wait time after a command is given                                                         

#define SELECT_COP_MUS_OUT 200000 // time out for reading from the Copley controller  
struct MotorInfoStruct reactinfo;  
struct MotorInfoStruct elevinfo; /* These file descriptors contain the status
                                     information for each motor and the file descriptor
				   */

void copyouts(char *in, char *out) {
  int i;

  for (i=0; i<strnlen(in,254); i++) {
    out[i] = (in[i]=='\r' ? '|' : in[i]);
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

  bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm: Closing connection to Copley controller.",copleyinfo->motorstr);
  if (copleyinfo->open==0) {
    bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm: Controller is already closed!",copleyinfo->motorstr);
  } else {

    n = disableCopley(copleyinfo);

    if(n!=0) {
      bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm close_copley: Disabling Copley controller failed.",copleyinfo->motorstr);
    }
    
    bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm close_copley: Closing serial port.",copleyinfo->motorstr);
    if(copleyinfo->fd >=0) {
      close(copleyinfo->fd); 
    }
  }
  copleyinfo->init=0;
  copleyinfo->open=0;
  bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm close_copley: Connection to motor serial port is closed.",copleyinfo->motorstr);
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
      bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%s setopts_copley:Setting baud rate to 9600\n",copleyinfo->motorstr);
      break;
    case 19200:
      cfsetispeed(&options, B19200);               //input speed              
      cfsetospeed(&options, B19200);              //output speed              
      bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm setopts_copley:Setting baud rate to 19200\n",copleyinfo->motorstr);
      break;
    case 38400:
      cfsetispeed(&options, B38400);               //input speed              
      cfsetospeed(&options, B38400);               //output speed             
      bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm setopts_copley:Setting baud rate to 38400\n",copleyinfo->motorstr);
      break;
    case 115200:
      cfsetispeed(&options, B115200);               //input speed             
      cfsetospeed(&options, B115200);               //output speed            
      bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm setopts_copley:Setting baud rate to 115200\n",copleyinfo->motorstr);
      break;
    default:
      bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm setopts_copley: Invalid baud rate %i. Using the default 115200.\n",copleyinfo->motorstr,bdrate);
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
  int l = strnlen(cmd,254);
  int n;

  bprintfverb(info,copleyinfo->verbose,MC_EXTRA_VERBOSE,"%sComm send_copleycmd: cmd = %s",copleyinfo->motorstr,cmd);

  n = write(copleyinfo->fd,cmd,l);
  if (n < 0) {
  bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm send_copleycmd: failed.",copleyinfo->motorstr);
  copleyinfo->err |= 0x0002;
  } else {
    copleyinfo->err &= ~0x0002;
  }
}

void configure_copley(struct MotorInfoStruct* copleyinfo)
{
  int n,m,i=0;

  do {
    setopts_copley(9600,copleyinfo);
    copleyinfo->bdrate=9600;
    bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm configure_copley: Try communicating at 9600 baud rate",copleyinfo->motorstr);
    n = ping_copley(copleyinfo);
  } while ((n<=0) && (++i < 10));

  if(n > 0) {
    bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm configure_copley: Controller responds to a 9600 baud rate.",copleyinfo->motorstr);
      //
    bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm configure_copley: Attempting to set baud rate to 38400",copleyinfo->motorstr);
    m = check_copleyready(comm,copleyinfo);
    if (m >= 0) {
      send_copleycmd("s r0x90 38400\r",copleyinfo);
    }  
    m = check_copleyready(resp,copleyinfo);
    if (m >= 0) {
      bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm configure_copley: Hey it responded!  Wicked!",copleyinfo->motorstr);
      i=checkCopleyResp(copleyinfo);
    }
    setopts_copley(38400,copleyinfo);
    copleyinfo->bdrate=38400;

// Check to make sure that there aren't leftover bytes to be read from the serial port.
//      clearCopleyPort(copleyinfo);

      //Check to make sure that the motor responds to the new 38400 baud rate.
    i = ping_copley(copleyinfo);
    if (i > 0) {
      bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm configure_copley: Controller now responds to a 38400 baud rate.",copleyinfo->motorstr);
      copleyinfo->init=1;
      copleyinfo->err=0;
      return;
    } else {
      bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm configure_copley: Controller does not respond to a 38400 baud rate!",copleyinfo->motorstr);
      copleyinfo->init=2;
      return;
    }
    
    //mark
  } else {
    bprintfverb(info,copleyinfo->verbose,MC_VERBOSE, "%sComm configure_copley: Controller does not respond to a 9600 baud rate.",copleyinfo->motorstr);
    bprintfverb(info,copleyinfo->verbose,MC_VERBOSE, "%sComm configure_copley: Setting Baud rate to 38400",copleyinfo->motorstr);
    setopts_copley(38400,copleyinfo);
    copleyinfo->bdrate=38400;
    i = ping_copley(copleyinfo);
    if(i > 0) {
      bprintfverb(info,copleyinfo->verbose,MC_VERBOSE, "%sComm configure_copley: Controller now responds to a 38400 baud rate.",copleyinfo->motorstr);
      copleyinfo->init=1;
      copleyinfo->err=0;
      return;
    } else {
      bprintfverb(info,copleyinfo->verbose,MC_VERBOSE, "%sComm configure_copley: Setting Baud rate to 115200",copleyinfo->motorstr);
      setopts_copley(115200,copleyinfo);
      copleyinfo->bdrate=38400;
      i = ping_copley(copleyinfo);
      if (i > 0) {
	bprintfverb(info,copleyinfo->verbose,MC_VERBOSE, "%sComm configure_copley: Controller now responds to a 115200 baud rate.",copleyinfo->motorstr);
	copleyinfo->init=1;
	copleyinfo->err=0;
	return;
      } else {
	bprintfverb(err,copleyinfo->verbose,MC_VERBOSE, "%sComm configure_copley: Controller does not respond to a 115200 baud rate!",copleyinfo->motorstr);
	bprintfverb(err,copleyinfo->verbose,MC_VERBOSE, "%sComm configure_copley: Controller does not respond to any baud rate!",copleyinfo->motorstr);
	copleyinfo->init=2;
        copleyinfo->bdrate=1; //i.e. there is no meaningful baud rate
 
      }
      return;
    }
  }
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
    break;
  }
  /* Was there an error? */
  if (n < 0) {
    bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm: Select command failed!",copleyinfo->motorstr);
    copleyinfo->err |= 0x0001;
    return -2;
  } else if (n==0) {
    bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm: Select call timed out.",copleyinfo->motorstr);
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

    copleyinfo->err &= ~0x0001;
    return m;
  }
}

void flushCopley(struct MotorInfoStruct* copleyinfo) {
  int i,n,total=0;
  char outs[2];
  for (i=0; (i<65536) && (check_copleyready(resp,copleyinfo)>0); ++i) {
    n=read(copleyinfo->fd,outs,1);
    total+=n;
  }
  bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm flushCopley: read %i characters total.",copleyinfo->motorstr,total);
}

// Can we communciate with the controller?
// reads the status variable, but only looks to be sure it is getting the correct format
// in the response.
//
//  Returns 1 if the ping was sucessful.
int ping_copley(struct MotorInfoStruct* copleyinfo)
{
  char outs[255],outs_noCR[255];
  int n=0,l=0;
  memset(outs,'\0',255);

  //flushCopley(copleyinfo);

  n = check_copleyready(comm,copleyinfo);
  if(n >= 0) {
    //      bprintf(info,"copleyComm configure_copley: Ready to send command!");
    send_copleycmd("g r0xa0\r",copleyinfo);
  } else {
    bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm ping_copley: Serial port is not ready to command.",copleyinfo->motorstr);
    return -5;
  }

  // mark
  n = readCopleyResp(outs,&l,copleyinfo);
  if (n >= 0) {
    copyouts(outs, outs_noCR);  
    bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm ping_copley: Controller response= %s\n",copleyinfo->motorstr,outs_noCR);
    bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm ping_copley: First character= %c\n",copleyinfo->motorstr,outs_noCR[0]);
    if (outs[0]== 'v')  {
      copleyinfo->err &= ~0x0004;      
      return 1;
    } else if(outs[0]== 'e') {
      copleyinfo->err &= ~0x0004;      
      return 1;
    } else {
      bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm ping_copley: The controller response is incorrect.",copleyinfo->motorstr);
      bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm ping_copley: Controller response= %s\n",copleyinfo->motorstr,outs_noCR);
      copleyinfo->err |= 0x0004;      
      return 0;
    }
  } else if (n==-1) {
    bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm ping_copley: Select failed.",copleyinfo->motorstr);
    return -1;
  } else {
    outs[10]='\0';
    bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm ping_copley: Controller responded with garbage. outs=%s",copleyinfo->motorstr,outs);
    copleyinfo->err |= 0x0004;      
    return 0;
  }
  
}

// Check the controller response after a command.
// If the response from the controller is "ok" (i.e. no errors) return 0.
// Returns -10 if the controller returned garbage.
// Returns the error code if the contoller responded with an error message.
int checkCopleyResp(struct MotorInfoStruct* copleyinfo)
{
  char outs[255],outs_noCR[255]; 
  int n,m,i,errcode,l=0;
  char* ptr;
  memset(outs,'\0',255);
  n = readCopleyResp(outs,&l,copleyinfo);
  copyouts(outs, outs_noCR);
  if(n<0)
    {
      return n;
    }
  bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm checkCopleyResp: Controller response= %s\n",copleyinfo->motorstr,outs_noCR);
  // Did the controller respond ok?
  if(outs_noCR[0]=='o' && outs_noCR[1]=='k')
    {
      copleyinfo->err &= ~0x0012;      
      return 0;
    }
  else  // This should mean it returned some kind of error code.
    {
      if(outs_noCR[0]=='e')
	{
	  copleyinfo->err |= 0x0008;      
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
  bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm checkCopleyResp: Controller returned error message %i",copleyinfo->motorstr,errcode);
	  return errcode;
	}
      else // Controller responded with garbage
	{
	  copleyinfo->err &= ~0x0004;      
	  return -10;
	}
    }
}

// Read from the controller.
// Print the controller output

int enableCopley(struct MotorInfoStruct* copleyinfo)
{
  int n;
  bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm enableCopley: Attempting to enable Copley motor controller.",copleyinfo->motorstr);
  //  int count=0;
  send_copleycmd("s r0x24 2\r",copleyinfo);
  n = check_copleyready(resp,copleyinfo);
  if (n < 0)
    {
      bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm enableCopley: Communication error.",copleyinfo->motorstr);
      return -1;
    }
  n=checkCopleyResp(copleyinfo);
  if(n==0) {
    copleyinfo->disabled=0;
  }
  return n;
}

// Disable the Copley controller
// Returns:
// -1 Communication Error
//  0 Controller was disabled
int disableCopley(struct MotorInfoStruct* copleyinfo)
{
  int n=0;
  bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm disableCopley: Attempting to disable Copley motor controller.",copleyinfo->motorstr);
  //  int count=0;
  send_copleycmd("s r0x24 0\r",copleyinfo);
  n = check_copleyready(resp,copleyinfo);
  if (n < 0){
    bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm disableCopley: Communication error.",copleyinfo->motorstr);
    return -1;
  } 
  n=checkCopleyResp(copleyinfo);
  if(n==0) {
  bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm disableCopley: Copley controller disabled.",copleyinfo->motorstr);
    copleyinfo->disabled=1;  
  }
 
  return n;
}


long int queryCopleyInd(char ind[],struct MotorInfoStruct* copleyinfo)
{
  int n,m,l=0;
  char outs[255];
  char cmd[255];
  char outs_noCR[255];
  long int val;
  memset(outs,'\0',255);
  if (copleyinfo->closing==1) {
    //      bprintf(info,"%sComm queryCopleyInd: We are closing so I'm returning 42",copleyinfo->motorstr);
    return 42;// Don't query the serial port if we are 
    // closing the connection to the controller.
  }
  n = check_copleyready(comm,copleyinfo);
  
  if (n >= 0) {
    //      bprintf(info,"copleyComm configure_copley: Ready to send command!");
    m = strnlen(ind,254);
    if (m != 0 && m!= 254) {
      strncpy(cmd, "g r",3);
      strncpy(cmd+3, ind,m);
      strncpy(cmd+3+m, "\r\0",2);
      send_copleycmd(cmd,copleyinfo);
      //		berror(err,"%sComm queryCopleyInd: cmd= %s.",copleyinfo->motorstr,cmd);        
    } else {
      berror(err,"%sComm queryCopleyInd: Invalid index to query.",copleyinfo->motorstr);
      return -4;
    }
  } else {
    bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm queryCopleyInd: Serial port is not ready to command.",copleyinfo->motorstr);
    return -5;
  }

  n = readCopleyResp(outs,&l,copleyinfo);
  if (n == 0)
    {
      //      bprintf(info,"queryCopleyInd n=%i",n);
      //      n = read(copleyinfo->fd,outs,254);
      outs[l] = '\0';
      copyouts(outs, outs_noCR);
      //             bprintf(info,"copleyComm queryCopleyInd: First character= %c\n",outs[0]);
      if (outs[0]== 'v')	{
        val=atoi(outs+2);
	//	  bprintf(warning,"copleyComm queryCopleyInd: Value= %ld",vel);
	copleyinfo->err_count=0;
	return val;        
      }
      else {
	if(outs[0]== 'e')
	  {
	    copleyinfo->err |= 0x0008;
            if((copleyinfo->err_count)%500==1) {
	      bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm queryCopleyInd: Controller returned error for the %dth time succesively.",copleyinfo->motorstr,copleyinfo->err_count);
	      bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm queryCopleyInd: cmd= %s.",copleyinfo->motorstr,cmd);        
	      bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm queryCopleyInd: Controller response was = %s\n",copleyinfo->motorstr,outs_noCR);
	    }
	    return -1;  
	  }
	else
	  {
	    copleyinfo->err |= 0x0004;
            if((copleyinfo->err_count)%500==1) {
	      bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm queryCopleyInd: Controller response was incorrect for the %dth time succesively.",copleyinfo->motorstr,copleyinfo->err_count);
	      bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm queryCopleyInd: cmd= %s.",copleyinfo->motorstr,cmd);        
	      bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm queryCopleyInd: Controller response was = %s\n",copleyinfo->motorstr,outs_noCR);
	    }
	    return 0;
	  }
      }
    } else {
    bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm queryCopleyInd: No useful response.",copleyinfo->motorstr);
    return -1;
  }

}


int readCopleyResp(char *outs,int *l,struct MotorInfoStruct* copleyinfo)
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
	  bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm readCopleyResp: The controller did not respond.",copleyinfo->motorstr);
	  copleyinfo->err |= 0x0010;
	  return -1;
	} else {
	  bprintfverb(err,copleyinfo->verbose,MC_VERBOSE,"%sComm readCopleyResp: Did not find the appropriate response end character.",copleyinfo->motorstr);
          copleyinfo->err |= 0x0004;
	  return -2; // For some reason the controller never found the end character.
	             // which means the response was probably garbage. 

	}
      }
      timeout++;
      usleep(2000);
    }
    j++;
  }
  if(i==0)   bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm readCopleyResp: Read 0 characters!!!",copleyinfo->motorstr);
  *l=i;
  copleyinfo->err &= ~0x0004;
  copleyinfo->err &= ~0x0010;
  return 0;
}

void resetCopley(char *address, struct MotorInfoStruct* copleyinfo)
{
  copleyinfo->disabled=2;
  copleyinfo->init=2;

  //  int count = 10;
  close_copley(copleyinfo);
  //  while(copleyinfo->open==0 && count > 0) {
  open_copley(address,copleyinfo);
    //    count--;
    //  }

  if(copleyinfo->open==0) {
    bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE, "%sComm resetCopley: Failed to open serial port!",copleyinfo->motorstr);
    bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE, "%sComm resetCopley: Attempt to reset controller failed.",copleyinfo->motorstr);
    return;
  }

  //  count = 10;
  //  while(copleyinfo->init==0  && count > 0 ) {
    configure_copley(copleyinfo);
    //    count--;
    //  }
  if(copleyinfo->init!=1) {
    bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm resetCopley: Failed to configure the drive!",copleyinfo->motorstr);
    bprintfverb(warning,copleyinfo->verbose,MC_VERBOSE,"%sComm resetCopley: Attempt to reset controller failed.",copleyinfo->motorstr);
    copleyinfo->disabled=2;
    copleyinfo->init=2;
    return;
  } else {
    bprintfverb(info,copleyinfo->verbose,MC_VERBOSE,"%sComm resetCopley: Controller reset was successful!",copleyinfo->motorstr);
    copleyinfo->reset=0;
    copleyinfo->err_count=0;
    copleyinfo->disabled=2;
  }

}

