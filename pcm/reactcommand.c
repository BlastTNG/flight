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
#include "reactcommand.h"
#include "motordefs.h"

#define CRC_POLY 0x1021

struct ReactInfoStruct reactinfo; /* Contains the status info and file
                                          * Descriptor
                                          */
#define SELECT_RMUS_OUT 200000 // time out for RW controller
/*
 * open_react: opens a connection to the address given which
 * is hopefully that of the RW controller.  Also sets up the
 * connection, and tests the baud rate.
 *
 */


//  Define RW status
# define RW_COMPLETE 1
# define RW_INCOMPLETE 2
# define RW_INVALID 4
# define RW_NOACCESS 6
# define RW_FRAMECRC 8


void open_react(char *address)
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


void setopts_rw(int bdrate)
{
  struct termios options;
  /*
   * Get the current options for the port...
   */
  tcgetattr(reactinfo.fd, &options);
  
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
      bprintf(info,"Invalid baud rate %i. Using the default 115200.\n",bdrate);
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
  
  tcsetattr(reactinfo.fd, TCSANOW, &options);
}


void MakeSCHeadStruct(struct DriveIPVStruct *ValuesSend,struct SerialCommandHeadStruct *MessSendHead, unsigned short *crctable, char *header,char **command, int *commsize,enum CmdorQuery type){
  int i;
  char *comm0,*commbegin;
  unsigned short  accumulator;
  char fouts[512];
  MessSendHead->sof='\xA5';
  MessSendHead->address='\x3F';
  if(type==cmd)
    {
      MessSendHead->controlbyte= (char)(2+(ValuesSend->counter)*4);
    }
  else
    {
      if(type!=query)
	{
	  berror(err,"reactComm MakeCmdStr: CmdorQuery type is not recognized.");
	}
      else
	{ 
	  MessSendHead->controlbyte= (char)(1+(ValuesSend->counter)*4);
	}
    }
  MessSendHead->index=(char) ValuesSend->index;
  MessSendHead->offset=(char) ValuesSend->offset;
  
  

  MessSendHead->datawords=ValuesSend->nwords;
  
  header[0]= MessSendHead->sof;
  header[1]= MessSendHead->address;
  header[2]= MessSendHead->controlbyte;
  header[3]= MessSendHead->index;
  header[4]= MessSendHead->offset;
  header[5]= MessSendHead->datawords;
  
  accumulator = 0;

  for(i=0; i<6; i++)
    {
      crccheck(((unsigned char) header[i]),&accumulator, crctable);
    }
  //  printf("\n");

  //  printf("\n Accumulator is %i\n",accumulator);

  MessSendHead->crc[1]=*((char*) &accumulator);
  MessSendHead->crc[0]=*(((char*) &accumulator)+1);

  header[6]=MessSendHead->crc[0];
  header[7]=MessSendHead->crc[1];

  if(type==cmd)
    {
      *commsize = 8+2*(ValuesSend->nwords)+2;
    }
  else
    {
      *commsize=8;
    }
  *command = (char *)malloc((*commsize)*sizeof(char));

  commbegin=*command;
  if(command == NULL)
    {
      bprintf(err,"ReactComm: Malloc allocation of the command string failed.");
      bprintf(err,"ReactComm:Aborting before we get a segmentation fault...");
      return;
    }


  //    printf("Here's what I'm assigning to command: ");
  for(i=0; i<8; i++)
    {
      **command=header[i];
      //      printf("%x,",(unsigned char) (**command));
      (*command)++;
    }  
  //  printf("\n");
#ifdef DEBUG_RW
  bprintf(info,"reactComm MakeSCHeadStruct: value we are sending %ld",ValuesSend->value);
#endif

  if(type==cmd)
    {
      comm0=*command;  // Remember where the beginning of the command array is...
      for(i=0;i<2*(ValuesSend->nwords);i++)
        {
          **command=*((char*) &(ValuesSend->value)+i);
          (*command)++;
        }
      (*command)=comm0;
      // Calculate the CRC for the data field only!   
      accumulator = 0;
      for(i=0; i<2*(ValuesSend->nwords); i++)
        {
          crccheck(((unsigned char) (**command)),&accumulator, crctable);
          (*command)++;
        }
      
      //  printf("\n Accumulator is %i\n",accumulator);
      MessSendHead->crc[1]=*((char*) &accumulator);
      MessSendHead->crc[0]=*(((char*) &accumulator)+1);
      **command=MessSendHead->crc[0];
      (*command)++;
      **command=MessSendHead->crc[1];
      /*   (*command)++; */
      
      /*    (**command)='\r';  */
      /*    (*command)++;  */
      /*    (**command)='\n';  */
      
      // For debugging purposes:  Print out the int value of every byte in the 
      // command.
      //  bprintf(info,"Command is:  ");
    }
  *command=commbegin;
for(i=0;i<*commsize;i++)
   {
     //     bprintf(info)
     sprintf(fouts+2*i,"%2x",((unsigned char) (*((*command)+i))));
   }
#ifdef DEBUG_RW
 bprintf(info,"reactComm: Command being sent: %s",fouts);
#endif // DEBUG_RW
}

void crccheck(unsigned short data, unsigned short *accumulator, unsigned short *crctable)
{
  *accumulator=(*accumulator << 8)^crctable[(*accumulator >> 8)^data];
}

unsigned short *mk_crctable(unsigned short poly, unsigned short (*crcfn)
			    (unsigned short, unsigned short, unsigned short))
{
  unsigned short *crctable;
  int i;
  if((crctable = (unsigned short *)malloc(256*sizeof(unsigned))) == NULL)
    {
      return NULL;
    }
  for(i=0; i < 256; i++)
    {
      crctable[i]=0;
      crctable[i] = (*crcfn)(i,poly,0);
    }
  return crctable;
}

unsigned short crchware(unsigned short data, unsigned short genpoly, unsigned short accum)
{
  static int i;
  data <<= 8;
  for(i = 8; i > 0; i--)
    {
      if((data ^ accum) & 0x8000)
	accum = (accum << 1 ) ^ genpoly;
      else
	accum <<=1;
      data <<=1;
    }
  return accum;
}

void configure_react()
{
  int n,m;
//  char testcmd[]="/1Q\r\n";
  bprintf(info,"reactComm configure_react: Testing a 115200 baud rate...\n");
  setopts_rw(115200);
  n = areWeDisabled();

  if(n >= 0)
    {
      bprintf(info,"reactComm: RW controller responds to a 115200 baud rate.");
      reactinfo.err=0;
      if(reactinfo.writeset!=1)
	{
	  setWriteAccess();
          m=disableRW(); // Make sure the RW is disabled
	}
      reactinfo.init=1;
      reactinfo.err=0;
      return;
    }
  else
    {
      bprintf(info,"reactComm: No controller response for a baud rate of 115200.");
    }

  bprintf(info,"reactComm configure_react: Testing a 9600 baud rate...\n");
  setopts_rw(9600);
  n = areWeDisabled();
  if(n >= 0)
    {
      bprintf(info,"reactComm: RW controller responds to a 9600 baud rate.");
      bprintf(info,"reactComm: Attempting to set the baud rate to 115200.");

      if(reactinfo.writeset!=1)
	{
	  setWriteAccess();
          m=disableRW(); // Make sure the RW is disabled
	}
      n=sendThisCommand(5,1,4,1,cmd); // Right now sending this command generates 
                                      // a response that we don't have write 
                                      // access to this index. I've email JR for
                                      // help.
                                      // TODO: Make sure this gets fixed.
   

      m=checkRWResp(n);
    }
  else
    {
      bprintf(info,"reactComm: No controller response for a baud rate of 9600.");
    }
  bprintf(info,"reactComm: Testing a 115200 baud rate...\n");
  setopts_rw(115200);
  n = areWeDisabled();
  if(n >= 0)
    {
      bprintf(info,"reactComm: RW controller responds to a 115200 baud rate.");
      if(reactinfo.writeset!=1)
	{
	  setWriteAccess();
          m=disableRW(); // Make sure the RW is disabled
	}
      reactinfo.err=0;
      reactinfo.init=1;
    }
  else
    {
      bprintf(err,"reactComm: Cannot communicate with the RW controller at any baud rate.");
      reactinfo.err=3; // Sets comm_error to level 3
                       // which should trigger a power cycle
    }

}

// LMF: Identical to the check_ready routine in pivotcontrol.c
// TODO If this doesn't change maybe I should make a file of 
// common motor commands.
 int check_rwready(enum CheckType check)
 {
   int n=0;
   int m;
   int max_fd=reactinfo.fd+1;
   fd_set         input;
   fd_set         output;
   struct timeval timeout;
   timeout.tv_sec=0;
   timeout.tv_usec=SELECT_RMUS_OUT;
   FD_ZERO(&input);
   FD_ZERO(&output);
 
   switch(check){
   case resp:
     FD_SET(reactinfo.fd, &input);
     n = select(max_fd, &input, NULL, NULL, &timeout);
     break;
   case comm:
     FD_SET(reactinfo.fd, &output);
     n = select(max_fd, NULL, &output, NULL, &timeout);
     break;
   case both:
     FD_SET(reactinfo.fd, &input);
     FD_SET(reactinfo.fd, &output);
     n = select(max_fd, &input, &output, NULL, &timeout);
     break;
   default:
     berror(err, "reactComm check_rwready: CheckType is in valid.");
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
       // n=1 ready to be written into
       // n=2 there is something to be read
       // n=3 ready to recieve a command and something to be read.
       m=0;
       if (check==resp || check== both)
         {
 	  if (FD_ISSET(reactinfo.fd, &input))
 	    m+=2;
         }
       if (check==comm || check==both)
         {
 	  if (FD_ISSET(reactinfo.fd, &output))
 	    m+=1;
         }
       if(n==0) {
         berror(fatal,"reactComm checkready: Serial Poll error!");
       }
       //      bprintf(info, "reactComm: checksum returns %i.",m);
       return m;
     }
 }
int sendThisCommand(int index,int offset,int value,int nwords,enum CmdorQuery type)
{
#ifdef DEBUG_RW
  //  bprintf(info,"reactComm sendThisCommand: index %"
#endif

  static int count=0;
  struct SerialCommandHeadStruct MessSendHead;
  struct DriveIPVStruct ValuesSend;
  unsigned short *crctable;
  char headersend[8];
  char *command;
  int n,l,i;

  if((crctable = mk_crctable((unsigned short)CRC_POLY,crchware)) == NULL)
 
    {
      printf("mk_crctable() memory allocation failed\n");
      exit(1);
    }
  ValuesSend.index=index;
  ValuesSend.offset=offset;
  ValuesSend.value=value;
  ValuesSend.readwrite=1;
  ValuesSend.nwords=nwords;
  count=(count+1)%16;
  ValuesSend.counter=count;
  MakeSCHeadStruct(&ValuesSend,&MessSendHead, crctable,headersend,&command,&l,type);
  // mark2
#ifdef DEBUG_RW
  char fouts[512];
  for(i=0;i<l;i++)
    {
      //     bprintf(info)
      sprintf(fouts+2*i,"%2x",((unsigned char) (*(command+i))));
    }
  bprintf(info,"reactComm: Command being sent: %s",fouts);
#endif // DEBUG_RW

  n = write(reactinfo.fd, command, l);
  if(n<0)
    {
      berror(err,"reactComm: Send command failed!");
      return -1;
    }
  return count;
}

int queryind(int index, int offset, int nwords)
{
#ifdef DEBUG_RW
  bprintf(info,"reactComm queryind: Querying index %d, offset %d, which has %d words.",index,offset,nwords);
#endif 
  int n,stat;
  int val=0;
  int l=0;
  int count=sendThisCommand(index,offset,0,nwords,query);
#ifdef DEBUG_RW
  bprintf(info,"reactComm queryind: Count returned by sendThisCommand is %d.",count);
#endif

  n = check_rwready(resp);  
  if(n >= 0)
    {
      stat=getRWResp(count,&val,&l);  
  
  if(stat == 1)
    {
#ifdef DEBUG_RW
      bprintf(info,"reactComm queryind: Query returns %d.  Sequence number is %d",val,count);
#endif

      return val;

    }
    }
      bprintf(err,"reactComm queryind: Error querying index.");
      return -1;
  // mark1
}

// Check to see whether the reaction wheel controller is enabled or
// disabled.  
// Returns:
// 0 -> We are enabled
// 1 -> We are disabled
// -1 -> Error, couldn't get a response from the RW
int areWeDisabled()
{
  int n= queryind(1,0,1);
  if(n<0)
    {
      bprintf(warning,"reactComm areWeDisabled: Couldn't send the query.");
      return -1;
    }
  else
    {
      if(n==1)
	{
#ifdef DEBUG_RW
	  bprintf(info,"reactComm areWeDisabled: We are disabled");
#endif
          reactinfo.disabled=1;
	}
      if(n==0)
        {
#ifdef DEBUG_RW
          bprintf(info,"reactComm areWeDisabled: We are enabled");
#endif
          reactinfo.disabled=0;
        }

      return n;      
    }
}

// Returns 0 if successful.
int readRWResp(int seq, char *response, int *l)
{
  int n,i;
  char outs[255],fouts[510];
  int rseq;
  for(i=0;i<255;i++)
    {
      outs[i]='\0';
      fouts[i]='\0';
    }
  n=check_rwready(resp);
  if(n<0)
    {
      berror(err,"reactComm readRWResp: Select failed.");
      return -1;
    }
  usleep(20000);
  n = read(reactinfo.fd, &outs, 254);
  if (n < 0)
    {
      berror(err,"reactComm readRWResp: Couldn't read from the device.");
      return -2;
    }
  // Is this the right sequence number?
  rseq=((int) outs[2])/4;
 for(i=0;i<255;i++)
   {
     sprintf((fouts+2*i),"%2x",((unsigned char) outs[i]));
   }
#ifdef DEBUG_RW
 bprintf(info,"reactComm readRWResp: Controller response: %s",fouts);
#endif // DEBUG_RW

#ifdef DEBUG_RW
 bprintf(info,"reactComm readRWResp: Our sequence number is %d, Response sequence number is %d",seq,rseq);
#endif // DEBUG_RW
  if(rseq != seq)
    {
      usleep(20000); // try agin
      n=check_rwready(resp);
      if(n<0)
	{
	  berror(err,"reactComm readRWResp: Select failed.");
	  return -3;
	}
      usleep(20000);
      n = read(reactinfo.fd, &outs, 254);
      if (n < 0)
	{
	  berror(err,"reactComm readRWResp: Couldn't read from the device.");
	  return -3;
	}
      rseq=((int) outs[2])/4;
      if(rseq != seq)
	{
	  berror(err,"reactComm readRWResp: After two tries no response from the controller with the correct sequence number rseq=%d, seq=%d",rseq, seq);
	  return -3;
	}
    }
 *l = 2*((int) outs[5])+8+2;
 int length= *l;

 for(i=0;i<length;i++)
   {
     *(response+i)= outs[i];
   }

 return 0;
}

// Gets the response corresponding to a particular control sequence number
// 
// Returns the status byte which can be parsed by CheckRWStatus
// val-> the value returned by the response.  If there is no return value
// returns 0
// l-> the length of *val.  If there is no return value *l = -1.
//
// In case of an error return:
// -1 if select fails
// -2 read failed
// -3 got a response from the device but the sequence number was wrong after two tries.
int getRWResp(int seq, int *val, int *l)
{
  int n,i;
  char response[256];
  int rseq, rl, rval,rstat;
  n=readRWResp(seq,response,l);
  if(n<0)
    {
      return n;
    }
#ifdef DEBUG_RW
  bprintf(info,"reactComm getRWResp: response[2]: %x, [3]: %x, [4]: %x",(unsigned char) response[2],(unsigned char) response[3],(unsigned char) response[4]);
#endif
  rseq=((int) response[2])/4;
  rval=0;
  rl=((int) response[5]); // Gives the number of words
  for(i=0;i<rl;i++)
    {
      rval+=((int)response[8+i]) <<(8*i);
      rval+=((int)response[8+i+1]) <<(8*i+1);
    }  
  *val=rval;

  rstat=((int) response[3]);

#ifdef DEBUG_RW
  bprintf(info,"reactComm getRWResp: Results status: %d , length %d, value %d",rstat,*l,*val);

  checkRWStatus(rstat);


#endif //DEBUG_RW


  // Returns the status byte
  return rstat;
  
}

// CheckRWResp:
// Same as getRWResponse except it only gets the status byte of the response.
// i.e. doesn't care about the data returned in the response
// 
// Returns the status byte which can be parsed by checkRWStatus
// val-> the value returned by the response.  If there is no return value
// returns 0
// l-> the length of *val.  If there is no return value *l = -1.
//
// In case of an error return:
// -1 if select fails
// -2 read failed
// -3 got a response from the device but the sequence number was wrong after two tries.
int checkRWResp(int seq)
{
  int n;
  int l=0;
  char response[256];
  int rval, rseq, rstat;
  n=readRWResp(seq,response,&l);
  if(n<0)
    {
      return n;
    }
  rseq=((int) response[2])/4;
  rval=0;

  rstat=((int) response[3]);

#ifdef DEBUG_RW
  checkRWStatus(rstat);
#endif //DEBUG_RW

  // Returns the status byte
  return rstat;
}

void checkRWStatus(int stat)
{
  switch(stat)
    {
    case RW_COMPLETE:
#ifdef DEBUG_RW
      bprintf(info,"reactcomm checkRWStatus: Command was completed.");
#endif
      break;
    case RW_INCOMPLETE:
      bprintf(warning,"reactcomm checkRWStatus: Command was not completed.");
    case RW_INVALID:
      bprintf(warning,"reactcomm checkRWStatus: Invalid Command.");
      break;
    case RW_NOACCESS:
      bprintf(warning,"reactcomm checkRWStatus: Do not have write access.");
      break;
    case RW_FRAMECRC:
      bprintf(warning,"reactcomm checkRWStatus: Frame or CRC error.");
      break;
    default:
      bprintf(warning,"reactcomm checkRWStatus: Invalid status byte.");
      break;
    }
}

void close_react()
{
  int n;
  //  char tmp[20];
  reactinfo.closing=1; // Tells reactComm that the pivot communcations are closing.
  usleep(500000); // Wait half a second to let reactComm close the current loop.
  bprintf(info,"reactComm: Closing connection to RW controller.");
#ifdef DEBUG_RW
  //    bprintf(info,"reactComm close_react: tmp= %s",tmp);
#endif

  n = disableRW();
  if(n>0)
    {
      checkRWStatus(n);
    }
  else
    {
      bprintf(err,"reactComm close_react: Disabling RW controller failed.");
    }
  close(reactinfo.fd);
}

void setWriteAccess()
{
  int n, count;
  count = sendThisCommand(7,0,15,1,cmd); // ???
  n = check_rwready(resp);
  if (n < 0)
    {
      berror(err,"reactComm setWriteAccess: Communication error.");
      reactinfo.writeset=-1;
      return;
    }
  n=checkRWResp(count);
  if(n>0)
    {
      checkRWStatus(n);
      if(n!=1) reactinfo.writeset=-1;

    }
}

int disableRW()
{
  int count,n;
  count = sendThisCommand(1,0,1,1,cmd); // Disable the reaction wheel controller.
  n = check_rwready(resp);
  if (n < 0)
    {
      berror(err,"reactComm disableRW: Communication error.");
      return -1;
    }  
  n=checkRWResp(count);
  if(n==1) 
    {
      reactinfo.disabled=1;
      //      bprintf(info,"reactComm disableRW: Reaction Wheel is now disabled.,");
    }
  return n;
}

int enableRW()
{
  int count,n;
#ifndef DISABLE_RW
  count = sendThisCommand(1,0,0,1,cmd); // Enable the reaction wheel controller.
  n = check_rwready(resp);
  if (n < 0)
    {
      berror(err,"reactComm disableRW: Communication error.");
      return -1;
    }  
  n=checkRWResp(count);
  if(n==1) 
    {
      reactinfo.disabled=0;
      //      bprintf(info,"reactComm enableRW: Reaction Wheel is enabled.,");
    }
#endif //n DISABLE_RW
#ifdef DISABLE_RW
  n=1;
#endif // DISABLE_RW
  return n;
}

int setRWCurrent(double curtarget)
{
  int count,n;
  int curreq;
 
  // Check safety limits.  
  double acurtarget= fabs((double)curtarget);
  if(acurtarget > MAX_RWHEEL_CURRENT)
    {
      curtarget=curtarget/curtarget*MAX_RWHEEL_CURRENT;
      bprintf(warning,"reactComm SetRWcurrent:  Requested current target is outside safety limits.");
      bprintf(warning,"reactComm SetRWcurrent:  Setting current to the maximum of %f.",curtarget);
    }

  curreq=(int) (curtarget/PEAK_RW_CURRENT*32767.0); // 
#ifdef DEBUG_RW
  bprintf(info,"reactComm setRWCurrent: curtarget= %f, curreq= %d",curtarget,curreq);
#endif // DEBUG_RW
  count = sendThisCommand(69,0,curreq,2,cmd); // Send current request
  n = check_rwready(resp);
  if (n < 0)
    {
      berror(err,"reactComm disableRW: Communication error.");
      reactinfo.err=1;
      return -1;
    }  
  n=checkRWResp(count);
  if(n<0) 
    {
      reactinfo.err=1;
      return -1;
    }
  return n;
}

void startRWLoop()
{
  if(reactinfo.err>0) return; // Don't attempt to start a current loop if we are in
                              // in an error state
  int count,n,m;
  // make sure we are disabled before proceeding...
  n=areWeDisabled();
  if(n==0)     
    {
      m=disableRW();
      if(m!=1)
	{
	  bprintf(err,"reactComm startRWLoop: Communication error, exiting.");
          reactinfo.err=1;
	  reactinfo.loop=-1;
	}
    }
  // Start the current loop.
  count=sendThisCommand(208,1,24,1,cmd);
  n = check_rwready(resp);
  if (n < 0)
    {
      berror(err,"reactComm disableRW: Communication error.");
      reactinfo.loop=-1;
      reactinfo.err=1;
      return;
    }  
  n=checkRWResp(count);
  if(n<0) 
    {
      reactinfo.err=1;
      reactinfo.loop=-1;
      return;
    }
  n = setRWCurrent(0.0);
  if(n!=1)
    { 
      bprintf(warning,"reactComm startRWLoop: Current could not be set.  Start loop failed.");
      return;
    }
  reactinfo.loop=1;
  m=enableRW();
  if(m!=1) bprintf(warning,"reactComm startRWLoop: Warning, RW Controller could not be enabled after loop was started.");

}
