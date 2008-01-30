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
#include "pivotcommand.h"
#include "motordefs.h"

//#define DEBUG_PIV // prints out more information when reforming 
                  // checks of the pivot response.

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

#define SELECT_MUS_OUT 200000 // time out for pivot controller
                              // select poll
#define PIVCOM_MUS_WAIT 200000 // wait time after a command is given
                               // TODO: optimize this wait time
/*
 * Open_pivot: opens a connection to the address give which 
 * is hopefully to the pivot controller.  Also sets up the 
 * connection, and tests the baud rate.  
 *
 */


struct PivotInfoStruct pivotinfo; /* Contains the status info and file
                                          * descriptor
                                          */

void open_pivot(char *address)
{
  char a[256];
  strcpy(a, address);
  
  pivotinfo.fd = open(address, O_RDWR | O_NOCTTY | O_NDELAY);
  if (pivotinfo.fd==-1)
  {
   /*
    * Could not open the port.
    */
  
    pivotinfo.open=0;
  }
  else
  {
    fcntl(pivotinfo.fd, F_SETFL, 0);
    pivotinfo.open=1;
    bprintf(info,"Pivot: Controller File descriptor is %i",pivotinfo.fd);
  }  
  pivotinfo.init=0;
}

void setopts(int bdrate)
{
  struct termios options;
  /*
   * Get the current options for the port...
   */
  tcgetattr(pivotinfo.fd, &options);
  
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
 
   tcsetattr(pivotinfo.fd, TCSANOW, &options);
}

/* Tries to figure out the current baud rate setting.  If it is not already, sets 
 * it to the maximum baud rate 38400
 */
void configure_pivot(int fd)
{
  int n;
  char testcmd[]="/1Q\r\n";
  bprintf(info,"pivotComm: Testing a 38400 baud rate...\n");
  setopts(38400);
  send_pivotcmd(testcmd);
  n = check_ready(resp);
  if(n >= 0)
    {
      bprintf(info,"pivotComm: Pivot controller responds to a 38400 baud rate.");
      pivotinfo.err=0;
      check_resp();
      return;
    }
  else
    {
      bprintf(info,"pivotComm: No controller response for a baud rate of 38400.");
    }

  bprintf(info,"pivotComm: Testing a 9600 baud rate...\n");
  setopts(9600);
  n = check_ready(resp);
  send_pivotcmd(testcmd);
  n = check_ready(resp);
  if(n >= 0)
    {
      bprintf(info,"pivotComm: Pivot controller responds to a 9600 baud rate.");
      check_resp();
      bprintf(info,"pivotComm: Attempting to set the baud rate to 38400.");
      run_command("/1T\r\n","configure_pivot");
      run_command("/1ar5073R\r\n","configure_pivot");
      run_command("/1b38400R\r\n","configure_pivot");
    }
  else
    {
      bprintf(info,"pivotComm: No controller response for a baud rate of 9600.");
    }
  bprintf(info,"pivotComm: Testing a 19200 baud rate...\n");
  setopts(19200);
  n = check_ready(resp);
  send_pivotcmd(testcmd);
  n = check_ready(resp);
  if(n >= 0)
    {
      bprintf(info,"pivotComm: Pivot controller responds to a 19200 baud rate.");
      check_resp();
      bprintf(info,"pivotComm: Attempting to set the baud rate to 38400.");
      run_command("/1T\r\n","configure_pivot");
      run_command("/1ar5073R\r\n","configure_pivot");
      run_command("/1b38400R\r\n","configure_pivot");
    }
  else
    {
      bprintf(info,"pivotComm: No controller response for a baud rate of 19200.");
    }
  bprintf(info,"pivotComm: Testing a 38400 baud rate...\n");
  setopts(38400);
  n = check_ready(resp);
  send_pivotcmd(testcmd);
  n = check_ready(resp);
  if(n >= 0)
    {
      bprintf(info,"pivotComm: Pivot controller responds to a 38400 baud rate.");
      check_resp();
      pivotinfo.err=0;
    }
  else
    {
      bprintf(err,"pivotComm: Cannot communicate with the pivot controller at any baud rate.");
      pivotinfo.err=3; // Sets comm_error to level 3
                       // which should trigger a power cycle
    }
}

void run_command(char cmd[],char tag[])
{
  int n;
  n=check_ready(resp);
  if(n > 0)
    {
      check_resp();
    }
  send_pivotcmd(cmd);
  n=check_ready(resp);
  if(n > 0)
    {
      check_resp();
    } 
}

void send_pivotcmd(char cmd[])
{
  int l = strlen(cmd);
  int n;
#ifdef DEBUG_PIV
  bprintf(info,"pivotComm send_pivotcmd: cmd = %s",cmd);
#endif // DEBUG_PIV
  n = write(pivotinfo.fd,cmd,l);
  if (n < 0)
    bprintf(err,"pivotComm: send_pivotcmd failed.");
  usleep(PIVCOM_MUS_WAIT);
}

void check_resp()
{
  char outs[255];
  int n;
  n = read(pivotinfo.fd,outs,254);
    if (n < 0)
    {
      berror(err,"pivotComm: Error reading from the pivot.");
    }
  outs[n]='\0';
  n=checkstatus(outs,-1,-1);
}
int check_sresp(int statcheck1, int statcheck2)
{
  char outs[255];
  int n;
  n = read(pivotinfo.fd,outs,254);
    if (n < 0)
    {
      berror(err,"pivotComm: Error reading from the pivot.");
    }
  outs[253]='\0';
  n=checkstatus(outs,statcheck1, statcheck2);
  return n;
}


// Returns 1 if the status character matches the requested byte.
// Returns 0 if there was no match requested.
// Returns -1 if the status character does not match the requested response
int checkstatus(char *respstr, int statcheck1, int statcheck2)
{
  // 1st check that this string has the correct markers of a response from the controller.

  if ((respstr[1] != '/' || respstr[2] != '0') &&(respstr[0] != '/' || respstr[1] != '0') )
    {
      bputs(err,"pivotComm: Response string doesn't have a valid controller form");
      bprintf(err,"pivotComm: Response String= %s\n",respstr);
    }
  int stat= (int) respstr[3];

#ifdef DEBUG_PIV
  bprintf(err,"pivotComm checkstatus: Response String= %s\n",respstr);
#endif // DEBUG_PIV
  // Now check for errors 
  int errcheck=stat%16;
  int haserrors=0;
  switch (errcheck) {
  case EZ_ERR_INIT:
    bputs(err,"pivotComm checkstatus: initialisation error.");
    haserrors++;
    break;
  case EZ_ERR_BADCMD:
    bputs(err,"pivotComm checkstatus: bad command.");
    haserrors++;
    break;
  case EZ_ERR_BADOP:
    bputs(err,"pivotComm checkstatus: bad operand.");
    haserrors++;
    break;
  case EZ_ERR_COMM:
    bputs(err,"pivotComm checkstatus: communications error.");
    haserrors++;
    break;
  case EZ_ERR_NOINIT:
    bputs(err,"pivotComm checkstatus: not initialied.");
    haserrors++;
    break;
  case EZ_ERR_OVER:
    bputs(err,"pivotComm checkstatus: overload.");
    haserrors++;
    break;
  case EZ_ERR_NOMOVE:
    bputs(err,"pivotComm checkstatus: move not allowed.");
    haserrors++;
    break;
  case EZ_ERR_BUSY:
    bputs(err,"pivotComm checkstatus: command overflow.");
    haserrors++;
    break;
  }

#ifdef DEBUG_PIV
  int isready=stat & EZ_READY ;
  bprintf(info,"pivotComm: statcheck1 is %i, statcheck2 is %i, errcheck is %i",statcheck1,statcheck2,errcheck);
  bprintf(info,"checkstatus: isready = %i\n",isready);
#endif // DEBUG_PIV
  
  if((statcheck1 <0) && (statcheck2 <0)){
    return 0;
  }
  else
    {
      if(statcheck1==errcheck || statcheck2==errcheck)
	{
	  return 1;
	}
      else
	{
	  return -1;
	}
    }
}

int check_ready(enum CheckType check)
{
  int n=0;
  int m;
  int max_fd=pivotinfo.fd+1;
  fd_set         input;
  fd_set         output; 
  struct timeval timeout;
  timeout.tv_sec=0;
  timeout.tv_usec=SELECT_MUS_OUT;
  FD_ZERO(&input);
  FD_ZERO(&output);

  switch(check){
  case resp:
  FD_SET(pivotinfo.fd, &input);
  n = select(max_fd, &input, NULL, NULL, &timeout);
  break;
  case comm:
  FD_SET(pivotinfo.fd, &output);
  n = select(max_fd, NULL, &output, NULL, &timeout);
  break;
  case both:
  FD_SET(pivotinfo.fd, &input);
  FD_SET(pivotinfo.fd, &output);
  n = select(max_fd, &input, &output, NULL, &timeout);
  break;
  default:
    berror(err, "pivotComm: check_ready CheckType is in valid.");
    return -3;
    break;
  }
  /* Was there an error? */
  if (n < 0)
    {
      bprintf(err,"pivotComm: Select command failed!");
      return -2;
    }
  else if (n==0)
    {
#ifdef DEBUG_PIV
      bprintf(warning,"PivotComm: Select call timed out.");
#endif
      return -1;
    }
  else 
    {
      // Sets a 2 bit integer m.  
      // n=1 ready to be written into 
      // n=2 there is something to be read
      // n=3 ready to recieve a command and something to be read.
      m=0;
      if (check==resp || check== both)
	{
        if (FD_ISSET(pivotinfo.fd, &input))
	  m+=2;
	}
      if (check==comm || check==both)
	{
        if (FD_ISSET(pivotinfo.fd, &output))
	  m+=1;
	}
      if(n==0) {
	berror(fatal,"pivotComm checkready: Serial Poll error!");
      }
      //      bprintf(info, "pivotComm: checksum returns %i.",m);
      return m;
    }
}

unsigned long int getquery(char *cmd)
{
  int i,j,n;
  char outs[20], posstr[20], teststr[20];
  // clear outs & posstr
  for(i=0;i<20;i++) {
    outs[i]='\0';
    posstr[i]='\0';
  }
  if(cmd[3] != '0' && cmd[3] != '2' && cmd[3] != '8')
    {
      berror(err,"pivotComm:Query command is not recognized.");
    }
  send_pivotcmd(cmd);
  n = check_ready(resp);
  if (n < 0)
    {
      berror(err,"pivotComm getquery: Communication error.");
      return -1;
    }
  n = read(pivotinfo.fd, &outs, 19);
  j=0;
  outs[19]='\0';
  bprintf(info,"outs: %s",outs);
  for(i =0; i<n; i++)
    {
      if(outs[i] <= '9' && outs[i] >= '0')
	{
	  posstr[j]=outs[i];
	  j++;
        }
    }
  teststr[j]='\0';
  //  bprintf(info,"cmd: %s",cmd);
  unsigned long int pos = atol(posstr); 
  if(pos == 0)
    {
      bprintf(warning,"pivotComm getquery: Warning, value is either zero or there was an error reading the position from the controller.\n");
    }

  return pos;
  
}

void close_pivot()
{
  int n;
  char tmp[20];
  pivotinfo.closing=1; // Tells pivotComm that the pivot communcations are closing.
  bprintf(info,"pivotComm: Closing connection to pivot controller.");
  n = check_ready(resp);
  if(n >=0) {
    n = read(pivotinfo.fd, &tmp, 19);
#ifdef DEBUG_PIV
    bprintf(info,"pivotComm close_pivot: tmp= %s",tmp);
#endif
    }
  send_pivotcmd("/1T\r\n");
  n = check_ready(resp);
  if (n < 0)
    {
      berror(err,"pivotComm close_pivot: Communication error.");
      return;
    }
  check_resp();
  close(pivotinfo.fd);
}

void setvel(double vel)
{
  int l,n;
  l=20;
  int vint= (int) (vel*COUNTS_PER_DEGREE);
  // Check that the value of vel is within the accepted 
  // range 0-20000000.
  if(vint > 20000000 || vint <0)
    {
      bputs(err,"pivotComm: Requested velocity is outside the accepted range of 0-20000000 encoder ticks /sec.");
      bputs(err,"pivotComm: Leaving velocity unchanged.");
      return;
    }
  char cmd[l];
  char form[]="/1V%iR\r\n";
  n = sprintf(cmd,form,vel);
  //  printf("%s",cmd);

  send_pivotcmd(cmd);
  n = check_ready(resp);
  if (n < 0)
    {
      berror(err,"pivotComm setVel: Communication error.");
      return;
    }
  check_resp();
}

void start_loop(double v)
{
  bprintf(info,"pivotComm start_loop: v=%f",v);
 int n;
 char cmd[25];
 send_pivotcmd("/1Q\r\n");
 n = check_ready(resp);
 if (n < 0)
   {
     berror(err,"pivotComm start_loop: Communication error.");
     return;
   }

 n=check_sresp(EZ_ERR_OK,-1);
 if(n!=1) // If the response from the pivot was not OK, we might
         // for some reason still be in a loop.  Send a terminate
         // command and try again.
   {
     bprintf(info,"pivotComm start_loop: n=%i",n);
     send_pivotcmd("/1T"); // terminate
     n = check_ready(resp);
     if(n < 0)
       {
	 berror(err,"pivotComm start_loop: Communication error.");
	 return;
       }
     check_resp();
     send_pivotcmd("/1Q");
     n = check_ready(resp);
     if (n < 0)
       {
	 berror(err,"pivotComm start_loop: Communication error.");
	 return;
       }
     n=check_sresp(EZ_ERR_OK,-1);
     if(n!=1)
       {
	 berror(err,"pivotComm start_loop: The pivot controller is not responding correctly.  Cannot start the loop.");
         return;
       }
   }
 
char form[]="/1L%iV%i%s0R\r\n";
char dir[1];
int vc = (int) fabs(v * COUNTS_PER_DEGREE);
 static int firsttime=1;
  if(v >= 0)
    {
      strcpy(dir,"P"); // Forwards 
      pivotinfo.ldir=1;
    }
  else
    {
      strcpy(dir,"D"); // Backwards
      pivotinfo.ldir=-1;
    }
  if (vc<PIVOT_MIN_VEL)
    {
      vc=PIVOT_MIN_VEL;      
    }

n = sprintf(cmd,form,PIVOT_ACCEL,vc,dir);

if(firsttime)
  {
    bprintf(info,"pivotComm start_loop: Command is %s",cmd);
    firsttime=0;
  }
 send_pivotcmd(cmd); // Start the loop.
 n = check_ready(resp);
 if(n<0) {
   bputs(err,"pivotComm start_loop: Communication error.");
   return;
 }
 n = check_sresp(EZ_ERR_BUSY,EZ_ERR_OK);
 if (n==1)
   {
      pivotinfo.loop=1;
      bprintf(info,"pivotComm start_loop: Velocity loop has been started!");
   }
 else
   {
     berror(err,"pivotComm start_loop: Attempt to start velocity loop failed.");
   }

}

void change_piv_vel(double v)
{
  static int firsttime=1;
  static double vlast=0; // pivot velocity requested last 
                         // time start_loop or change_piv_vel was called
  int dlast=pivotinfo.ldir;   // loop direction the last time 
  int n;
  int dcur=pivotinfo.ldir; //unless we find that it should change later in this function...
  char dir[1];
  int vc= (int) fabs(v * COUNTS_PER_DEGREE);
  char cmd[25];
  char form1[]="/1L%iV%i%s0R\r\n";
  char form2[]="/1V%iR\r\n";
  // Check to make sure that the status character is as expected.
  // If a loop is running we expect to get a response of 15 (command overflow)
  // we'll also accept 0 (no errors).
  int i=0;
  int ilim=3;

#ifdef DEBUG_PIV
  bprintf(info,"pivotComm change_piv_vel: vlast=%f, v=%f",vlast,v);
#endif

  while(i<ilim)
    {
      send_pivotcmd("/1Q\r\n");
      n = check_ready(resp);
      if (n < 0)
        {
          berror(err,"pivotComm change_piv_vel: Communication error.");
          return;
        }
      n = check_sresp(EZ_ERR_BUSY,EZ_ERR_OK);
      if(n==1)
	{
	  i=ilim+10; // break out of the loop
	}
      else
	{
	  i++;
	}
    }
  if(i==ilim)
    {
      berror(err,"pivotComm change_piv_vel: The controller response to status queries is unsatisfactory.");
      return;
    }
  // Do we need to change direction?
  if (v*vlast <0)
    {
      i=0;
      while(i<ilim)
	{
	  // Send commands to turn around.
          send_pivotcmd("/1T\r\n"); // Stop any loop that is currently running.
          n = check_ready(resp);
          if (n < 0)
            {
              berror(err,"pivotComm change_piv_vel: Communication error.");
              return;
	    }
	  n=check_sresp(EZ_ERR_OK,-1);
	  if(n==1) 
	    {
	      pivotinfo.loop=0;
              i=ilim+10; // break out of the loop
	    }
          if(n==ilim)
	    {
	      berror(err, "pivotComm change_piv_vel: Cannot send loop terminate command");
	    }
	}                
      if(dlast ==-1)
	{
	  dcur=1;
	  strcpy(dir,"P"); // Forwards
          pivotinfo.ldir=1;
	}
      if(dlast ==1)
	{
	  dcur=-1;
	  strcpy(dir,"D"); // Backwards
          pivotinfo.ldir=-1;
	}
      if (vc<PIVOT_MIN_VEL)
        {
          vc=PIVOT_MIN_VEL;
        }
      n = sprintf(cmd,form1,PIVOT_ACCEL,vc,dir);
    }
  else
    {
      if (vc<PIVOT_MIN_VEL)
	{
	  vc=PIVOT_MIN_VEL;
	}
      n = sprintf(cmd,form2,vc);
    }
i=0;
while(i<=0) // While loop is to make sure that the pivot command actually is 
           // actually received and understood by the pivot controller.
 {
  send_pivotcmd(cmd);
      n = check_ready(resp);
      if (n < 0)
        {
          berror(err,"pivotComm change_piv_vel: Communication error.");
          return;
        }
      n = check_sresp(EZ_ERR_BUSY,EZ_ERR_OK);
       if(n==1)
	{
          pivotinfo.loop=1;
	  i=ilim+10; // break out of the loop
	}
      else
	{
	  i++;
	}
     if(i==ilim)
       {
          berror(err,"pivotComm change_piv_vel: Couldn't restart velocity loop.");
          return;
       }
 }//while
  firsttime=0;
  vlast=((double) vc) /((double)COUNTS_PER_DEGREE) *((double)dcur);
}
