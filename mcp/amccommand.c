//===================================================================
// amccommand.c
//
// Modified from the reactcommand.c taken from the Spider
// mcp repository (version 1.2 last modified Fri Aug 22 17:06:45 2008)
// Commands for serial communication with Advanced Motion Controls
// controllers.  
//
// Used to communicate with the pivot motor.
//===================================================================

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
#include "amccommand.h"
#include "motordefs.h"

#define CRC_POLY 0x1021

struct MotorInfoStruct pivotinfo; /* Contains the status info and file
                                          * Descriptor
                                          */
#define SELECT_RMUS_OUT 200000 // time out for AMC controller


//  Define AMC status
# define AMC_COMPLETE 1
# define AMC_INCOMPLETE 2
# define AMC_INVALID 4
# define AMC_NOACCESS 6
# define AMC_FRAMECRC 8


/*
 * open_amc: opens a connection to the address given which
 * is hopefully that of the AMC controller.  Also sets up the
 * connection, and tests the baud rate.
 *
 */
void open_amc(char *address)
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
      }
  pivotinfo.init=0;
}


void setopts_rw(int bdrate)
{
  struct termios options;
  /*
   * Get the current options for the port...
   */
  tcgetattr(pivotinfo.fd, &options);
  
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
  
  tcsetattr(pivotinfo.fd, TCSANOW, &options);
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
	  berror(err,"amcComm MakeCmdStr: CmdorQuery type is not recognized.");
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
      bprintf(err,"AmcComm: Malloc allocation of the command string failed.");
      bprintf(err,"AmcComm:Aborting before we get a segmentation fault...");
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
#ifdef DEBUG_AMC
  bprintf(info,"amcComm MakeSCHeadStruct: value we are sending %ld",ValuesSend->value);
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
#ifdef DEBUG_AMC
 bprintf(info,"amcComm: Command being sent: %s",fouts);
#endif // DEBUG_AMC
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

void configure_amc()
{
  int n,m;
//  char testcmd[]="/1Q\r\n";
  bprintf(info,"amcComm configure_amc: Testing a 115200 baud rate...\n");
  setopts_rw(115200);
  n = areWeDisabled();

  if(n >= 0)
    {
      bprintf(info,"amcComm: AMC controller responds to a 115200 baud rate.");
      pivotinfo.err=0;
      if(pivotinfo.writeset!=1)
	{
	  setWriteAccess();
          m=disableAMC(); // Make sure the AMC is disabled
	}
      pivotinfo.init=1;
      pivotinfo.err=0;
      return;
    }
  else
    {
      bprintf(info,"amcComm: No controller response for a baud rate of 115200.");
    }

  bprintf(info,"amcComm configure_amc: Testing a 9600 baud rate...\n");
  setopts_rw(9600);
  n = areWeDisabled();
  if(n >= 0)
    {
      bprintf(info,"amcComm: AMC controller responds to a 9600 baud rate.");
      bprintf(info,"amcComm: Attempting to set the baud rate to 115200.");

      if(pivotinfo.writeset!=1)
	{
	  setWriteAccess();
          m=disableAMC(); // Make sure the AMC is disabled
	}
      n=sendThisCommand(5,1,4,1,cmd); // Right now sending this command generates 
                                      // a response that we don't have write 
                                      // access to this index. I've email JR for
                                      // help.
                                      // TODO: Make sure this gets fixed.
   

      m=checkAMCResp(n);
    }
  else
    {
      bprintf(info,"amcComm: No controller response for a baud rate of 9600.");
    }
  bprintf(info,"amcComm: Testing a 115200 baud rate...\n");
  setopts_rw(115200);
  n = areWeDisabled();
  if(n >= 0)
    {
      bprintf(info,"amcComm: AMC controller responds to a 115200 baud rate.");
      if(pivotinfo.writeset!=1)
	{
	  setWriteAccess();
          m=disableAMC(); // Make sure the AMC is disabled
	}
      pivotinfo.err=0;
      pivotinfo.init=1;
    }
  else
    {
      bprintf(err,"amcComm: Cannot communicate with the AMC controller at any baud rate.");
      pivotinfo.err=3; // Sets comm_error to level 3
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
   int max_fd=pivotinfo.fd+1;
   fd_set         input;
   fd_set         output;
   struct timeval timeout;
   timeout.tv_sec=0;
   timeout.tv_usec=SELECT_RMUS_OUT;
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
     berror(err, "amcComm check_rwready: CheckType is in valid.");
     return -3;
     break;
   }
   /* Was there an error? */
   if (n < 0)
     {
       bprintf(err,"amcComm: Select command failed!");
       return -2;
     }
   else if (n==0)
     {
#ifdef DEBUG_AMC
       bprintf(warning,"amcComm: Select call timed out.");
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
         berror(fatal,"amcComm checkready: Serial Poll error!");
       }
       //      bprintf(info, "amcComm: checksum returns %i.",m);
       return m;
     }
 }
int sendThisCommand(int index,int offset,int value,int nwords,enum CmdorQuery type)
{
#ifdef DEBUG_AMC
  //  bprintf(info,"amcComm sendThisCommand: index %"
#endif

  static int count=0;
  struct SerialCommandHeadStruct MessSendHead;
  struct DriveIPVStruct ValuesSend;
  unsigned short *crctable;
  char headersend[8];
  char *command;
  int n,l;
#ifdef DEBUG_AMC
  int i;
#endif

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
#ifdef DEBUG_AMC
  char fouts[512];
  for(i=0;i<l;i++)
    {
      //     bprintf(info)
      sprintf(fouts+2*i,"%2x",((unsigned char) (*(command+i))));
    }
  bprintf(info,"amcComm: Command being sent: %s",fouts);
#endif // DEBUG_AMC

  n = write(pivotinfo.fd, command, l);
  if(n<0)
    {
      berror(err,"amcComm: Send command failed!");
      return -1;
    }
  return count;
}

int queryind(int index, int offset, int nwords)
{
#ifdef DEBUG_AMC
  bprintf(info,"amcComm queryind: Querying index %d, offset %d, which has %d words.",index,offset,nwords);
#endif 
  int n,stat;
  int val=0;
  int l=0;
  int count=sendThisCommand(index,offset,0,nwords,query);
#ifdef DEBUG_AMC
  bprintf(info,"amcComm queryind: Count returned by sendThisCommand is %d.",count);
#endif

  n = check_rwready(resp);  
  if(n >= 0)
    {
      stat=getAMCResp(count,&val,&l);  
  
  if(stat == 1)
    {
#ifdef DEBUG_AMC
      bprintf(info,"amcComm queryind: Query returns %d.  Sequence number is %d",val,count);
#endif

      return val;

    }
    }
      bprintf(err,"amcComm queryind: Error querying index.");
      return -1;
  // mark1
}

// Check to see whether the amcion wheel controller is enabled or
// disabled.  
// Returns:
// 0 -> We are enabled
// 1 -> We are disabled
// -1 -> Error, couldn't get a response from the AMC
int areWeDisabled()
{
  int n= queryind(1,0,1);
  if(n<0)
    {
      bprintf(warning,"amcComm areWeDisabled: Couldn't send the query.");
      return -1;
    }
  else
    {
      if(n==1)
	{
#ifdef DEBUG_AMC
	  bprintf(info,"amcComm areWeDisabled: We are disabled");
#endif
          pivotinfo.disabled=1;
	}
      if(n==0)
        {
#ifdef DEBUG_AMC
          bprintf(info,"amcComm areWeDisabled: We are enabled");
#endif
          pivotinfo.disabled=0;
        }

      return n;      
    }
}

// Returns 0 if successful.
int readAMCResp(int seq, char *response, int *l)
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
      berror(err,"amcComm readAMCResp: Select failed.");
      return -1;
    }
  usleep(20000);
  n = read(pivotinfo.fd, &outs, 254);
  if (n < 0)
    {
      berror(err,"amcComm readAMCResp: Couldn't read from the device.");
      return -2;
    }
  // Is this the right sequence number?
  rseq=((int) outs[2])/4;
 for(i=0;i<255;i++)
   {
     sprintf((fouts+2*i),"%2x",((unsigned char) outs[i]));
   }
#ifdef DEBUG_AMC
 bprintf(info,"amcComm readAMCResp: Controller response: %s",fouts);
#endif // DEBUG_AMC

#ifdef DEBUG_AMC
 bprintf(info,"amcComm readAMCResp: Our sequence number is %d, Response sequence number is %d",seq,rseq);
#endif // DEBUG_AMC
  if(rseq != seq)
    {
      usleep(20000); // try agin
      n=check_rwready(resp);
      if(n<0)
	{
	  berror(err,"amcComm readAMCResp: Select failed.");
	  return -3;
	}
      usleep(20000);
      n = read(pivotinfo.fd, &outs, 254);
      if (n < 0)
	{
	  berror(err,"amcComm readAMCResp: Couldn't read from the device.");
	  return -3;
	}
      rseq=((int) outs[2])/4;
      if(rseq != seq)
	{
	  berror(err,"amcComm readAMCResp: After two tries no response from the controller with the correct sequence number rseq=%d, seq=%d",rseq, seq);
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
// Returns the status byte which can be parsed by CheckAMCStatus
// val-> the value returned by the response.  If there is no return value
// returns 0
// l-> the length of *val.  If there is no return value *l = -1.
//
// In case of an error return:
// -1 if select fails
// -2 read failed
// -3 got a response from the device but the sequence number was wrong after two tries.
int getAMCResp(int seq, int *val, int *l)
{
  int n,i;
  char response[256];
  int rseq, rl, rval,rstat;
  n=readAMCResp(seq,response,l);
  if(n<0)
    {
      return n;
    }
#ifdef DEBUG_AMC
  bprintf(info,"amcComm getAMCResp: response[2]: %x, [3]: %x, [4]: %x",(unsigned char) response[2],(unsigned char) response[3],(unsigned char) response[4]);
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

#ifdef DEBUG_AMC
  bprintf(info,"amcComm getAMCResp: Results status: %d , length %d, value %d",rstat,*l,*val);

  checkAMCStatus(rstat);


#endif //DEBUG_AMC


  // Returns the status byte
  return rstat;
  
}

// CheckAMCResp:
// Same as getAMCResponse except it only gets the status byte of the response.
// i.e. doesn't care about the data returned in the response
// 
// Returns the status byte which can be parsed by checkAMCStatus
// val-> the value returned by the response.  If there is no return value
// returns 0
// l-> the length of *val.  If there is no return value *l = -1.
//
// In case of an error return:
// -1 if select fails
// -2 read failed
// -3 got a response from the device but the sequence number was wrong after two tries.
int checkAMCResp(int seq)
{
  int n;
  int l=0;
  char response[256];
  int rval, rseq, rstat;
  n=readAMCResp(seq,response,&l);
  if(n<0)
    {
      return n;
    }
  rseq=((int) response[2])/4;
  rval=0;

  rstat=((int) response[3]);

#ifdef DEBUG_AMC
  checkAMCStatus(rstat);
#endif //DEBUG_AMC

  // Returns the status byte
  return rstat;
}

void checkAMCStatus(int stat)
{
  switch(stat)
    {
    case AMC_COMPLETE:
#ifdef DEBUG_AMC
      bprintf(info,"amccomm checkAMCStatus: Command was completed.");
#endif
      break;
    case AMC_INCOMPLETE:
      bprintf(warning,"amccomm checkAMCStatus: Command was not completed.");
    case AMC_INVALID:
      bprintf(warning,"amccomm checkAMCStatus: Invalid Command.");
      break;
    case AMC_NOACCESS:
      bprintf(warning,"amccomm checkAMCStatus: Do not have write access.");
      break;
    case AMC_FRAMECRC:
      bprintf(warning,"amccomm checkAMCStatus: Frame or CRC error.");
      break;
    default:
      bprintf(warning,"amccomm checkAMCStatus: Invalid status byte.");
      break;
    }
}

void close_amc()
{
  int n;
  //  char tmp[20];
  pivotinfo.closing=1; // Tells amcComm that the pivot communcations are closing.
  usleep(500000); // Wait half a second to let amcComm close the current loop.
  bprintf(info,"amcComm: Closing connection to AMC controller.");
#ifdef DEBUG_AMC
  //    bprintf(info,"amcComm close_amc: tmp= %s",tmp);
#endif

  n = disableAMC();
  if(n>0)
    {
      checkAMCStatus(n);
    }
  else
    {
      bprintf(err,"amcComm close_amc: Disabling AMC controller failed.");
    }
  close(pivotinfo.fd);
}

void setWriteAccess()
{
  int n, count;
  count = sendThisCommand(7,0,15,1,cmd); // ???
  n = check_rwready(resp);
  if (n < 0)
    {
      berror(err,"amcComm setWriteAccess: Communication error.");
      pivotinfo.writeset=-1;
      return;
    }
  n=checkAMCResp(count);
  if(n>0)
    {
      checkAMCStatus(n);
      if(n!=1) pivotinfo.writeset=-1;

    }
}

int disableAMC()
{
  int count,n;
  count = sendThisCommand(1,0,1,1,cmd); // Disable the amcion wheel controller.
  n = check_rwready(resp);
  if (n < 0)
    {
      berror(err,"amcComm disableAMC: Communication error.");
      return -1;
    }  
  n=checkAMCResp(count);
  if(n==1) 
    {
      pivotinfo.disabled=1;
      //      bprintf(info,"amcComm disableAMC: Amcion Wheel is now disabled.,");
    }
  return n;
}

int enableAMC()
{
  int n;
#ifndef DISABLE_AMC
  int count;
  count = sendThisCommand(1,0,0,1,cmd); // Enable the AMC controller.
  n = check_rwready(resp);
  if (n < 0)
    {
      berror(err,"amcComm disableAMC: Communication error.");
      return -1;
    }  
  n=checkAMCResp(count);
  if(n==1) 
    {
      pivotinfo.disabled=0;
      //      bprintf(info,"amcComm enableAMC: AMC controller is enabled.,");
    }
#endif //n DISABLE_AMC
#ifdef DISABLE_AMC
  n=1;
#endif // DISABLE_AMC
  return n;
}

