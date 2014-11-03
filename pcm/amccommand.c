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
#include "command_struct.h" 

#define CRC_POLY 0x1021

#define SELECT_RMUS_OUT 200 // time out for AMC controller
                            // in milliseconds

//#define DEBUG_AMC

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
void open_amc(char *address, struct MotorInfoStruct* amcinfo)
{
  char a[256];
  strcpy(a, address);

  amcinfo->fd = open(address, O_RDWR | O_NOCTTY | O_NDELAY);
  if (amcinfo->fd==-1)
    {
      /*
      * Could not open the port.
      */

      amcinfo->open=0;
    }
  else
    {
      fcntl(amcinfo->fd, F_SETFL, 0);
      amcinfo->open=1;
      }
  amcinfo->init=0;
}

void close_amc(struct MotorInfoStruct* amcinfo)
{
    int n=0;

  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm close_amc: Closing connection to AMC controller.",amcinfo->motorstr);

  if (amcinfo->open==0) {
    bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm: Controller is already closed!",amcinfo->motorstr);
  } else {

    //    n = disableAMC(amcinfo);

    if (n>0) {
      checkAMCStatus(n,amcinfo);
    } else {
      bprintfverb(err,amcinfo->verbose,MC_VERBOSE,"%sComm close_amc: Disabling AMC controller failed.",amcinfo->motorstr);
    }
    
  }
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm close_amc: Closing serial port.",amcinfo->motorstr);

  close(amcinfo->fd);
  amcinfo->init=0;
  amcinfo->open=0;
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm close_amc: Connection to motor serial port is closed.",amcinfo->motorstr);
}

void setopts_amc(int bdrate, struct MotorInfoStruct* amcinfo)
{
  struct termios options;
  /*
  * Get the current options for the port...
  */
  tcgetattr(amcinfo->fd, &options);
  
  /*
  * Set the baud rate to bdrate.  Default is B115200.
  */

  switch( bdrate)
    {
    case 9600:
      cfsetispeed(&options, B9600);               //input speed
      cfsetospeed(&options, B9600);               //output speed
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm setopts_amc:Setting baud rate to 9600\n",amcinfo->motorstr);
      break;
    case 19200:
      cfsetispeed(&options, B19200);               //input speed
      cfsetospeed(&options, B19200);              //output speed
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm setopts_amc:Setting baud rate to 19200\n",amcinfo->motorstr);
      break;
    case 38400:
      cfsetispeed(&options, B38400);               //input speed
      cfsetospeed(&options, B38400);               //output speed
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm setopts_amc:Setting baud rate to 38400\n",amcinfo->motorstr);
      break;
    case 115200:
      cfsetispeed(&options, B115200);               //input speed
      cfsetospeed(&options, B115200);               //output speed
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm setopts_amc:Setting baud rate to 115200\n",amcinfo->motorstr);
      break;
    default:
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm setopts_amc: Invalid baud rate %i. Using the default 115200.\n",amcinfo->motorstr,bdrate);
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
  
  tcsetattr(amcinfo->fd, TCSANOW, &options);
}


void MakeSCHeadStruct(struct DriveIPVStruct *ValuesSend,struct SerialCommandHeadStruct *MessSendHead, unsigned short *crctable, char *header,char **command, int *commsize,enum CmdorQuery type, struct MotorInfoStruct* amcinfo){
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
    berror(err,"%sComm MakeCmdStr: CmdorQuery type is not recognized.",amcinfo->motorstr);
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
      crccheck(((unsigned char) header[i]),&accumulator, crctable, amcinfo);
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
  if(command == NULL) {
    bprintf(err,"AmcComm: Malloc allocation of the command string failed. Aborting.");
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
  bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm MakeSCHeadStruct: value we are sending %ld",amcinfo->motorstr,ValuesSend->value);
  
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
          crccheck(((unsigned char) (**command)),&accumulator, crctable,amcinfo);
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
  bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm MakeSCHeadStruct: Command being sent: %s",amcinfo->motorstr,fouts);
}

void crccheck(unsigned short data, unsigned short *accumulator, unsigned short *crctable, struct MotorInfoStruct* amcinfo)
{
  *accumulator=(*accumulator << 8)^crctable[(*accumulator >> 8)^data];
}

unsigned short *mk_crctable(unsigned short poly, unsigned short (*crcfn)
          (unsigned short, unsigned short, unsigned short), struct MotorInfoStruct* amcinfo)
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

//unsigned short crchware(unsigned short data, unsigned short genpoly, unsigned short accum, struct MotorInfoStruct* amcinfo)
unsigned short crchware(unsigned short data, unsigned short genpoly, unsigned short accum) {
  static int i;
  data <<= 8;
  for(i = 8; i > 0; i--) {
    if((data ^ accum) & 0x8000)
      accum = (accum << 1 ) ^ genpoly;
    else
      accum <<=1;
    data <<=1;
  }
  return accum;
}

void flushAMC(struct MotorInfoStruct* amcinfo) {
  int i,n,total=0;
  char outs[2];
  for (i=0; (i<65536) && (check_amcready(resp,amcinfo,0)>0); ++i) {
    n=read(amcinfo->fd,outs,1);
    total+=n;
  }
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm flushAMC: read %i characters total.",amcinfo->motorstr,total);
}

int send_amccmd(int index,int offset,int value,int nwords,enum CmdorQuery type, struct MotorInfoStruct* amcinfo)
{

  static int count=0;
  struct SerialCommandHeadStruct MessSendHead;
  struct DriveIPVStruct ValuesSend;
  unsigned short *crctable;
  char headersend[8];
  char *command;
  int n,l;
  int i;

  if((crctable = mk_crctable((unsigned short)CRC_POLY,crchware,amcinfo)) == NULL) {
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
  MakeSCHeadStruct(&ValuesSend,&MessSendHead, crctable,headersend,&command,&l,type,amcinfo);

  free(crctable);

  // mark2
  if (amcinfo->verbose >= MC_EXTRA_VERBOSE) {
    char fouts[512];
    for(i=0;i<l;i++) {
      sprintf(fouts+2*i,"%2x",((unsigned char) (*(command+i))));
    }
    //bprintf(info,"%sComm send_amccmd: Command being sent: %s",amcinfo->motorstr,fouts);
  }
  flushAMC(amcinfo);
  n = write(amcinfo->fd, command, l);
  free(command);
  if (n<0) {
    bprintfverb(err,amcinfo->verbose,MC_VERBOSE,"%sComm send_amccmd: Send command failed!",amcinfo->motorstr);
    amcinfo->err |= 0x0002;
    return -1;
  } else {
    amcinfo->err &= ~0x0002;
    return count;
  }
}

int queryAMCInd(int index, int offset, int nwords, struct MotorInfoStruct* amcinfo)
{
  int n,stat=0;
  int val=0;
  int l=0;
  int count=0;
  bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm queryAMCInd: Querying index %d, offset %d, which has %d words.",amcinfo->motorstr,index,offset,nwords);
  if(amcinfo->closing==1) {
    return 42;// Don't query the serial port if we are 
                // closing the connection to the controller.
  }

  n = check_amcready(comm,amcinfo,SELECT_RMUS_OUT);
  bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"PIVOT DEBUG TMP: check_amcready(comm,amcinfo) response is %i",n);
  
  if(n>=0) {
    if(nwords!= 0) {
      count=send_amccmd(index,offset,0,nwords,query,amcinfo);
    } else {
      berror(err,"%sComm queryAMCInd: Improper index (nwords=0).",amcinfo->motorstr);
      return -4;
    }
  } else {
    berror(err,"%sComm queryAMCInd: Serial port is not ready to command. n=%i",amcinfo->motorstr,n);
    amcinfo->err |=0x0002;
    return -5;
  }

  bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm queryAMCInd: Count returned by send_amccmd is %d.",amcinfo->motorstr,count);

  n = check_amcready(resp,amcinfo,SELECT_RMUS_OUT);  
  if(n >= 0) {
    stat=getAMCResp(count,&val,&l,amcinfo);
  } else {
    bprintfverb(warning,amcinfo->verbose,MC_VERBOSE,"%sComm queryAMCInd: Did not find a response.",amcinfo->motorstr);
    amcinfo->err |= 0x0010;
    return -2;
  }
  if(stat == 1) {
    bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm queryAMCInd: Query returns %d.  Sequence number is %d",amcinfo->motorstr,val,count);
    
    return val;
    
  } else {

    bprintfverb(err,amcinfo->verbose,MC_VERBOSE,"%sComm queryAMCInd: Error querying index: stat=%i",amcinfo->motorstr,stat);
    
    return -2;
  }
  // mark1
}

void configure_amc(struct MotorInfoStruct* amcinfo)
{
  int n;
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm configure_amc: Testing a 38400 baud rate...\n",amcinfo->motorstr);
  setopts_amc(38400,amcinfo);
  amcinfo->bdrate=38400;

  n = areWeDisabled(amcinfo);
  if(n >= 0)
    {
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm configure_amc: AMC controller responds to a 38400 baud rate.",amcinfo->motorstr);
      amcinfo->err=0;
      checkAMCAccess(amcinfo);
      if(amcinfo->writeset!=1)
  {
    setWriteAccess(amcinfo);
    checkAMCAccess(amcinfo);
  }
      amcinfo->init=1;
      amcinfo->err=0;
      return;
    }
  else
    {
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm configure_amc: No controller response for a baud rate of 38400.",amcinfo->motorstr);
    }

  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm configure_amc: Testing a 9600 baud rate...\n",amcinfo->motorstr);
  setopts_amc(9600,amcinfo);
  amcinfo->bdrate=9600;
  n = areWeDisabled(amcinfo);
  if(n >= 0)
    {
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm configure_amc: AMC controller responds to a 9600 baud rate.",amcinfo->motorstr);
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm configure_amc: Attempting to set the baud rate to 38400.",amcinfo->motorstr);

      checkAMCAccess(amcinfo);
      if(amcinfo->writeset!=1)
  {
    setWriteAccess(amcinfo);
    checkAMCAccess(amcinfo);
          disableAMC(amcinfo); // Make sure the AMC is disabled
  }
      n=send_amccmd(5,1,2,1,cmd,amcinfo); 

      checkAMCResp(n, amcinfo);
    }
  else
    {
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm configure_amc: No controller response for a baud rate of 9600.",amcinfo->motorstr);
    }
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm configure_amc: Testing a 38400 baud rate...\n",amcinfo->motorstr);
  setopts_amc(38400, amcinfo);
  amcinfo->bdrate=38400;
  n = areWeDisabled(amcinfo);
  if(n >= 0)
    {
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm configure_amc: AMC controller responds to a 38400 baud rate.",amcinfo->motorstr);
      checkAMCAccess(amcinfo);
      if(amcinfo->writeset!=1)
  {
    setWriteAccess(amcinfo);
    checkAMCAccess(amcinfo);
          disableAMC(amcinfo); // Make sure the AMC is disabled
  }
      amcinfo->err=0;
      amcinfo->init=1;
    }
  else
    {
      bprintfverb(warning,amcinfo->verbose,MC_VERBOSE,"%sComm configure_amc: Cannot communicate with the AMC controller at any baud rate.",amcinfo->motorstr);
      amcinfo->init=2; 
      amcinfo->bdrate=1;
    }
}

int check_amcready(enum CheckType check, struct MotorInfoStruct* amcinfo, unsigned int waittime)
{
  int n=0;
  int m;
  int max_fd=amcinfo->fd+1;
  fd_set         input;
  fd_set         output;
  struct timeval timeout;
  timeout.tv_sec=waittime/1000;
  timeout.tv_usec=(waittime-timeout.tv_sec)*1000;
  FD_ZERO(&input);
  FD_ZERO(&output);

  switch(check){
  case resp:
    FD_SET(amcinfo->fd, &input);
    n = select(max_fd, &input, NULL, NULL, &timeout);
    break;
  case comm:
    FD_SET(amcinfo->fd, &output);
    n = select(max_fd, NULL, &output, NULL, &timeout);
    break;
  case both:
    FD_SET(amcinfo->fd, &input);
    FD_SET(amcinfo->fd, &output);
    n = select(max_fd, &input, &output, NULL, &timeout);
    break;
  default:
    bprintfverb(warning,amcinfo->verbose,MC_VERBOSE,"%sComm check_amcready: CheckType is in valid.",amcinfo->motorstr);
    return -3;
    break;
  }
  /* Was there an error? */
  if (n < 0) {
    bprintfverb(err,amcinfo->verbose,MC_VERBOSE,"%sComm: Select command failed!",amcinfo->motorstr);
    amcinfo->err |= 0x0001;
    return -2;
  } else if (n==0) {
    bprintfverb(warning,amcinfo->verbose,MC_VERBOSE,"%sComm: Select call timed out.",amcinfo->motorstr);
    return -1;
  } else {
      // Sets a 2 bit integer m.
      // n=1 ready to be written into
      // n=2 there is something to be read
      // n=3 ready to recieve a command and something to be read.
    m=0;
    if (check==resp || check== both) {      
      if (FD_ISSET(amcinfo->fd, &input))
  m|=2;
    }
    if (check==comm || check==both) {
      if (FD_ISSET(amcinfo->fd, &output))
  m|=1;
    }
      //      bprintf(info, "%sComm: checksum returns %i.",m);
    amcinfo->err &= ~0x0001;
    return m;
  }
}

// Check to see whether the amcion wheel controller is enabled or
// disabled.  
// Returns:
// 0 -> We are enabled
// 1 -> We are disabled
// -1 -> Error, couldn't get a response from the AMC
int areWeDisabled(struct MotorInfoStruct* amcinfo)
{
  int n= queryAMCInd(1,0,1,amcinfo);
  if(n<0)
    {
      bprintfverb(err,amcinfo->verbose,MC_VERBOSE,"%sComm areWeDisabled: Couldn't send the query.",amcinfo->motorstr);
      return -1;
    }
  else
    {
      if(n==1)
  {
    bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm areWeDisabled: We are disabled",amcinfo->motorstr);
          amcinfo->disabled=1;
  }
      if(n==0)
        {
    bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm areWeDisabled: We are enabled",amcinfo->motorstr);
          amcinfo->disabled=0;
        }

      return n;      
    }
}

// Returns 0 if successful.
// outs should be 255 bytes
int readAMCResp(int seq, unsigned char *outs, int *l, struct MotorInfoStruct* amcinfo)
{
  int n,i=0,j=0;
  //  int fouts[510];
  int done=0;
  int firstchar_found=0;
  int timeout=0;
  int timeoutlim=3;
  int nchars=8 ; // The number of characters expected in a reply without data.
                // 8 bytes for the header.
                // Add 2*nwords (header bit 6) +2 bytes for a response
  unsigned int ndatawords=0;
  while(done==0) {
    if(check_amcready(resp,amcinfo,SELECT_RMUS_OUT) >= 0){ 
      n = read(amcinfo->fd,outs,1);
      bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"Sweetness and light! n=%i, outs[i]=%2x  %i",n,((unsigned char)*outs),((unsigned int)*outs));
      if(*outs == 0xa5) firstchar_found=0; 
      if(firstchar_found == 0) {
  if(i >= 254 || i >= nchars-1) done=1;
  i+=n;
  if(i==6) {
          ndatawords=((unsigned int) *outs);
    if(ndatawords!=0) nchars+=ndatawords*2+2;
    bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"OK i==6, outs[i]=%i, nchars=%i",(unsigned int) *outs, nchars);
  }
  outs+=n;
  timeout=0;
      } 
    } else {
      if(timeout==timeoutlim){ // If there is no data after two tries return an.
  if(j==timeoutlim) {// The controller never responded.
    bprintfverb(err,amcinfo->verbose,MC_VERBOSE,"%sComm read_line: The controller did not respond.",amcinfo->motorstr);
    amcinfo->err |= 0x0010;
    return -1;
  } else {
    bprintfverb(err,amcinfo->verbose,MC_VERBOSE,"%sComm read_line: Did not find the appropriate response end character.",amcinfo->motorstr);
          amcinfo->err |= 0x0004;
    return -2; // For some reason the controller never found the end character.
              // which means the response was probably garbage. 
    
  }
      }
      timeout++;
      usleep(2000);
    }
    j++;
  } // while(done==0)

  if(i==0) bprintf(warning,"%sComm read_line: Read 0 characters!!!",amcinfo->motorstr);
  *l = i;
  amcinfo->err &= ~0x0004;
  amcinfo->err &= ~0x0010;
  
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
int getAMCResp(int seq, int *val, int *l, struct MotorInfoStruct* amcinfo)
{
  int n,i;
  unsigned char response[256];
  int rl, rval,rstat;
  //int rseq;
  n=readAMCResp(seq,response,l,amcinfo);
  if(n<0)
    {
      return n;
    }
  //rseq=((int) response[2])/4;
  rval=0;
  rl=((int) response[5]); // Gives the number of words
  bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm getAMCResp: response[2]: %x, [3]: %x, [4]: %x",amcinfo->motorstr,(unsigned char) response[2],(unsigned char) response[3],(unsigned char) response[4]);
  bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm getAMCResp: Number of words: %i",amcinfo->motorstr, rl);
  for(i=0;i<rl;i++)
    {
      rval+=((int)response[8+2*i]) <<(8*(2*i));
      rval+=((int)response[8+2*i+1]) <<(8*(2*i+1));
      bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm getAMCResp: %i: %i-> %i,%i: %i-> %i,",amcinfo->motorstr, 2*i,((int)response[8+2*i]),((int)response[8+2*i]) <<(8*2*i),2*i+1,((int)response[8+2*i+1]),((int)response[8+2*i+1]) <<(8*(2*i+1)));
    }  
  *val=rval;

  rstat=((int) response[3]);

  bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm getAMCResp: Results status: %d , length %d, value %d",amcinfo->motorstr,rstat,*l,*val);
  checkAMCStatus(rstat,amcinfo);


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
int checkAMCResp(int seq, struct MotorInfoStruct* amcinfo)
{
  int n;
  int l=0;
  unsigned char response[256];
  int rstat;
  //int rseq;
  n=readAMCResp(seq,response,&l,amcinfo);
  if(n<0)
    {
      return n;
    }
  //rseq=((int) response[2])/4;

  rstat=((int) response[3]);

  checkAMCStatus(rstat,amcinfo);

  // Returns the status byte
  return rstat;
}

// Only prints to the log if amcinfo->verbose>=2
void checkAMCStatus(int stat, struct MotorInfoStruct* amcinfo)
{
  switch(stat)
    {
    case AMC_COMPLETE:
      bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm checkAMCStatus: Command was completed.",amcinfo->motorstr);
      amcinfo->err=0;
      amcinfo->err_count=0;
      bprintfverb(info,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm checkAMCStatus: amcinfo->err=%i, amcinfo->err_count=%i.",amcinfo->motorstr,amcinfo->err,amcinfo->err_count);
      break;
    case AMC_INCOMPLETE:
      amcinfo->err |= 0x0008;
      bprintfverb(warning,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm checkAMCStatus: Command was not completed.",amcinfo->motorstr);
    case AMC_INVALID:
      amcinfo->err |= 0x0008;
      bprintfverb(warning,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm checkAMCStatus: Invalid Command.",amcinfo->motorstr);
      break;
    case AMC_NOACCESS:
      amcinfo->err |= 0x0008;
      bprintfverb(warning,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm checkAMCStatus: Do not have write access.",amcinfo->motorstr);
      break;
    case AMC_FRAMECRC:
      amcinfo->err |= 0x0008;
      bprintfverb(warning,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm checkAMCStatus: Frame or CRC error.",amcinfo->motorstr);
      break;
    default:
      amcinfo->err |= 0x0004;
      bprintfverb(warning,amcinfo->verbose,MC_EXTRA_VERBOSE,"%sComm checkAMCStatus: Invalid status byte.",amcinfo->motorstr);
      break;
    }
}

void setWriteAccess(struct MotorInfoStruct* amcinfo)
{
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm setWriteAccess: Setting write access",amcinfo->motorstr);  
  int n, count;
  count = send_amccmd(7,0,15,1,cmd,amcinfo);
  n = check_amcready(resp,amcinfo,SELECT_RMUS_OUT);
  if (n < 0)
    {
      berror(err,"%sComm setWriteAccess: Communication error.",amcinfo->motorstr);
      amcinfo->writeset=2;
      return;
    }
  n=checkAMCResp(count,amcinfo);
  if(n>0)
    {
      checkAMCStatus(n,amcinfo);
      if(n!=1) {
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm setWriteAccess: Write access not set",amcinfo->motorstr);  
  amcinfo->writeset=2;
      } else {
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm setWriteAccess: Write access not set",amcinfo->motorstr);  
  amcinfo->writeset=1;
      }
    }
}

int disableAMC(struct MotorInfoStruct* amcinfo)
{
  int count,n;
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm disableAMC: Attempting to disable motor controller.",amcinfo->motorstr);
  count = send_amccmd(1,0,1,1,cmd,amcinfo); // Disable the AMC controller.
  n = check_amcready(resp,amcinfo,SELECT_RMUS_OUT);
  if (n < 0)
    {
      bprintfverb(err,amcinfo->verbose,MC_VERBOSE,"%sComm disableAMC: Communication error.",amcinfo->motorstr);
      return -1;
    }  
  n=checkAMCResp(count,amcinfo);
  if(n==1) 
    {
      amcinfo->disabled=1;
      bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm disableAMC: Controller is now disabled.,",amcinfo->motorstr);
    }
  return n;
}

int enableAMC(struct MotorInfoStruct* amcinfo)
{
  int n;
  int count;
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm enableAMC: Attempting to disable motor controller.",amcinfo->motorstr);
  count = send_amccmd(1,0,0,1,cmd,amcinfo); // Enable the AMC controller.
  n = check_amcready(resp,amcinfo,SELECT_RMUS_OUT);
  if (n < 0)
    {
      bprintfverb(err,amcinfo->verbose,MC_VERBOSE,"%sComm enableAMC: Communication error.",amcinfo->motorstr);
      return -1;
    }  
  n=checkAMCResp(count,amcinfo);
  if(n==1) 
    {
      amcinfo->disabled=0;
      //      bprintf(info,"%sComm enableAMC: AMC controller is enabled.,",amcinfo->motorstr);
    }
  n=1;
  return n;
}

int getAMCResolver(struct MotorInfoStruct* amcinfo)
{
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm getAMCResolver: Querying resolver position.",amcinfo->motorstr);
  return queryAMCInd(18, 0, 2, amcinfo) & 0x3fff; // queryAMCInd returns a 32 bit number
                                                  // but the resolver wraps after 2^14 bits.
}

int checkAMCAccess(struct MotorInfoStruct* amcinfo)
{
  int n;
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"checkAMCAccess: Checking write access.");
  n = queryAMCInd(7, 0, 1, amcinfo);                                                
  bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"checkAMCAccess: Result=%i",n);
  return n;
}

void resetAMC(char *address, struct MotorInfoStruct* amcinfo)
{
  amcinfo->disabled=2;
  amcinfo->init=2;
  amcinfo->writeset=2;
  //  int count = 10;
  close_amc(amcinfo);
  //  while(amcinfo->open==0 && count > 0) {
  open_amc(address,amcinfo);
    //    count--;
    //  }

  if(amcinfo->open==0) {
    bprintfverb(warning,amcinfo->verbose,MC_VERBOSE,"%sComm resetAMC: Failed to open serial port!",amcinfo->motorstr);
    bprintfverb(warning,amcinfo->verbose,MC_VERBOSE,"%sComm resetAMC: Attempt to reset controller failed.",amcinfo->motorstr);
    return;
  }

  //  count = 10;
  //  while(amcinfo->init==0  && count > 0 ) {
    send_amccmd(3,2, 0x0040, 1, cmd, amcinfo); // reset drive status events
    configure_amc(amcinfo);
    //    count--;
    //  }
  if(amcinfo->init!=1) {
    bprintfverb(warning,amcinfo->verbose,MC_VERBOSE,"%sComm resetAMC: Failed to configure the drive!",amcinfo->motorstr);
    bprintfverb(warning,amcinfo->verbose,MC_VERBOSE,"%sComm resetAMC: Attempt to reset controller failed.",amcinfo->motorstr);
    return;
  } else {
    bprintfverb(info,amcinfo->verbose,MC_VERBOSE,"%sComm resetAMC: Controller reset was successful!",amcinfo->motorstr);
    amcinfo->reset=0;
    amcinfo->err_count=0;

  }

}

// Sends a command to restore the RS232 communication parameters.
void restoreAMC(struct MotorInfoStruct* amcinfo)
{
  int count,n;
  amcinfo->init=2;
  amcinfo->disabled=2;

  count = send_amccmd(9,0,0x1cae,1,cmd, amcinfo);
  n = check_amcready(resp,amcinfo,SELECT_RMUS_OUT);
  if (n < 0) {
    bprintf(err,"%sComm restoreAMC: Communication error.",amcinfo->motorstr);
  }
  
  n=checkAMCResp(count,amcinfo);
  if(n==1)  {
    bprintf(info,"%sComm restoreAMC: Restoration was successful,",amcinfo->motorstr);
  }
}
