/* Implementation of motor command routine. 

    Inputs:         Device: device name (i.e. "COM1")
                    motornum: 1 or 2
                    cmdstring: /1 or /2 is pre-pended,  R<CR> are appended

    Return value:   -3 port close error
                    -2 port not available
                    -1 port init error 
                     0 success
                     1 write error
                     2 response timeout
*/

#include "motorcmd.h"

int motorcmd( LPCTSTR Device, int motornum, char *cmdstring ) {
  CSerial com_port;
  FILE *comlog;
  comlog=fopen("comlog.cam", "a+");  /* Open for read/write/append log */
  
  // Make the command string prefix for each motor
  char cmdprefix[3];
  sprintf(cmdprefix,"/%1i",motornum);
  
  // Check port availability
  if( com_port.CheckPort(Device) != 0 ) return -2;  
  
  // Open port and set parameters: 9600,N,8,1, no flow control
  printf("%s \n", "Opened port");
  if( com_port.Open(Device) != ERROR_SUCCESS ) return -1;
  if( com_port.Setup(CSerial::EBaud9600,CSerial::EData8,
		     CSerial::EParNone,CSerial::EStop1) 
      != ERROR_SUCCESS ) return -1;
  

  if( com_port.SetupHandshaking(CSerial::EHandshakeOff) != ERROR_SUCCESS ) 
    return -1;

  // Make reads block the thread until a certain # of bytes have been received
  if( com_port.SetupReadTimeouts(CSerial::EReadTimeoutBlocking) 
      != ERROR_SUCCESS ) return -1;  
    
  // Send the command string
  char thecmd[255];
  sprintf(thecmd,"%s%sR\r",cmdprefix,cmdstring);
  printf("%s \n","I am sending a command");
  if( com_port.Write(thecmd) != ERROR_SUCCESS ) return 1;  
  
  /**log commands to motors*******/
  printf("%s \n","I have sent command\n");
  fprintf(comlog,"%s\n",thecmd);
  fflush(comlog);    
  
  // Initial read to see if motor responded to the command
  char readval[8];
  if( com_port.Read(readval,7,NULL,0,MOTOR_TIMEOUT) == ERROR_TIMEOUT ) {
    com_port.Close();
    return 2;
  }
 printf("%s %s \n","Value read:",readval);  
  fprintf(comlog,"Readval is: %s",readval);
  readval[7] = 0;
    
  // Now go into a query/read loop until motor is ready for new command

  sprintf(thecmd,"%sQR\r",cmdprefix);   // query command
  char status=0;
  while( status == 0 ) {
    com_port.Write(thecmd);
    if( com_port.Read(readval,7,NULL,0,MOTOR_TIMEOUT) == ERROR_TIMEOUT ) {
      com_port.Close();
      return 2;
    }

    status = readval[3];
    status = status & 32;
  }

  // Finish up
  if( com_port.Close() != ERROR_SUCCESS ) return -3;              
  
  fclose(comlog);
  
  return 0;
}

// Send an abort string to a motor. -1=failure, 1=success
// NOTE: This routine never worked properly!

int motorabort( LPCTSTR Device, int motornum ) {
  CSerial com_port;
  
  // Make the command string
  
  // Open port and set parameters: 9600,N,8,1, no flow control
  if( com_port.Open(Device) != ERROR_SUCCESS ) return -1;
  
  if( com_port.Setup(CSerial::EBaud9600,CSerial::EData8,CSerial::EParNone,
		     CSerial::EStop1) != ERROR_SUCCESS ) 
    return -1;
  
    if( com_port.SetupHandshaking(CSerial::EHandshakeOff) != ERROR_SUCCESS ) 
      return -1;

    // Make reads block thread until a certain # of bytes have been received
    if( com_port.SetupReadTimeouts(CSerial::EReadTimeoutBlocking) != 
	ERROR_SUCCESS ) 
      return -1;
    
    // Send the command string
    char cmd[80];
    sprintf(cmd,"/%1iTR\r",motornum);
    if( com_port.Write(cmd) != ERROR_SUCCESS ) return 1;
    else return -1;
}
