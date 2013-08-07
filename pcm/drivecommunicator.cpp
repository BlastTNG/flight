#include <stdio.h>
#include <termios.h>
#include <fcntl.h>   /* File control definitions */
#include <sys/select.h>
#include <unistd.h>  /* UNIX standard function definitions */
#include <math.h>         //for powers and float parsing
#include <limits.h>       //for numerical limits of integer types
#include "drivecommunicator.h"
extern "C" {
#include "blast.h"
}

#define DRIVE_COMM_DEBUG 0
#if DRIVE_COMM_DEBUG
#include <iostream>
#include <iomanip>
#endif

//retry attempts to sychronize
#define DRIVE_COMM_RETRIES 1

/*
DriveCommunicator:
default constructor performs minimal initiation
*/
DriveCommunicator::DriveCommunicator()
{
  dirForward = true;
  derr = DC_UNKNOWN;
  portFD = -1;
  highspeed = false;
}

/*
DriveCommunicator:
perform minimal initialization and then open connection
*/
DriveCommunicator::DriveCommunicator(string deviceName)
{
  dirForward = true;
  derr = DC_UNKNOWN;
  portFD = -1;
  highspeed = false;
  openConnection(deviceName);
}

/*
   ~DriveCommunicator:
   close any active connection
   */
DriveCommunicator::~DriveCommunicator()
{
  closeConnection();
}

/*
openConnection:
open a connection to the device /dev/'deviceName' at drive default settings
if highspeed is true, then use 115200 baud as speed
should always do this now since speed is set in controller startup
*/
void DriveCommunicator::openConnection(string deviceName, 
    bool highspeed/*=true*/)
{
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: opening a connection to the device: " << deviceName << endl;
#endif
  serialDeviceName = deviceName;
  portFD = open(deviceName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (portFD == -1) { //open failed
    //berror(err,"DriveComm: open device %s: ",deviceName.c_str());
    serialDeviceName = "";
    derr = DC_SERIAL_ERROR;
    return;
  }

  //set baud rate, character size, and parity
  struct termios options;
  tcgetattr(portFD, &options);
  if (!highspeed) {
    cfsetispeed(&options, B19200);                 //input speed
    cfsetospeed(&options, B19200);                 //outpur speed
  } else {
    cfsetispeed(&options, B19200);               //input speed
    cfsetospeed(&options, B19200);               //outpur speed
  }

  //set up simple raw 8N2 communications
  options.c_iflag = 0;
  options.c_oflag = 0;
  options.c_cflag &= ~(CSIZE|PARENB);
  options.c_cflag |= CSTOPB;
  options.c_cflag |= CS8;
  options.c_lflag = 0;
  //options.c_lflag |= ICANON;                   //enable canonical (line-based) mode


  tcsetattr(portFD, TCSANOW, &options);     //apply changes

  //try talking to the drive by sending synchronization character
  synchronize();

  //try seeing if controller is running at low speed (shouldn't)
//  if (!highspeed && derr != DC_NO_ERROR) {
//#if DRIVE_COMM_DEBUG
//    cout << "[DriveComm debug]: synchronize failed, trying low speed" << endl;
//#endif
//    openConnection(deviceName,false);
//  }

  this->highspeed = highspeed;
}

/*
closeConnection:
close any active serial connection
*/
void DriveCommunicator::closeConnection()
{
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: closing connection to the device." << endl;
#endif
  if (close(portFD) < 0) {
    derr = DC_SERIAL_ERROR;
    return;
  }
  portFD = -1;
  serialDeviceName = "";	
  derr = DC_NO_ERROR;
}

/*
synchronize:
sends a synchronization character to the drive and awaits a reply
*/
void DriveCommunicator::synchronize()
{
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: SYNCHRONIZING the device." << endl;
#endif
  int n=1;
  char send = 0x0d, receive;   //holders for sent and returned characters
  fd_set input, output;
  struct timeval timeout;

  for(int retryNum = 0; retryNum <= DRIVE_COMM_RETRIES; retryNum++)
  {
    timeout.tv_sec = 2;
#if DRIVE_COMM_DEBUG
    if (retryNum != 0)
      cout << "[DriveComm debug]: ...retry number: " << retryNum << endl;
#endif

    timeout.tv_usec = 0;    //2s timeout
    FD_ZERO(&output);
    FD_SET(portFD, &output);	
#if DRIVE_COMM_DEBUG
    cout << "[DriveComm debug]: ...waiting for write and then sending synchroniztion character" << endl;
#endif
    n = select(portFD+1, NULL, &output, NULL, &timeout);              //wait until a write won't block
    if (n <= 0 || !FD_ISSET(portFD, &output)) {    //error, timeout, or something probably impossible
      derr = DC_SERIAL_ERROR;
#if DRIVE_COMM_DEBUG
      cout << "[DriveComm debug]: ...error: " << derr << endl;
#endif
      continue;
    }
    n = write(portFD, &send, 1);    //write single test character
    if (n < 0) {    //error
      derr = DC_SERIAL_ERROR;
#if DRIVE_COMM_DEBUG
      cout << "[DriveComm debug]: ...error: " << derr << endl;
#endif
      continue;
    }

    timeout.tv_sec = 2;
    timeout.tv_usec = 0;
    FD_ZERO(&input);
    FD_SET(portFD, &input);
#if DRIVE_COMM_DEBUG
    cout << "[DriveComm debug]: ...waiting for read and attempting to read return value" << endl;
#endif
    n = select(portFD+1, &input, NULL, NULL, &timeout);         //wait until read won't block
    if (n <= 0 || !FD_ISSET(portFD, &input)) {
      derr = DC_SERIAL_ERROR;
#if DRIVE_COMM_DEBUG
      cout << "[DriveComm debug]: ...error: " << derr << endl;
#endif
      continue;
    }
    n = read(portFD, &receive, 1);                      //read only single return byte
    if (n < 0) {    //error
      derr = DC_SERIAL_ERROR;
#if DRIVE_COMM_DEBUG
      cout << "[DriveComm debug]: ...error: " << derr << endl;
#endif
      continue;
    }

#if DRIVE_COMM_DEBUG
    cout << "[DriveComm debug]: ...synchronization returned the character: \"" << (int)receive 
      << "\" (sent: \"" << (int)send << "\")" << endl;
#endif
    if (receive == 0x0d) {
      derr = DC_NO_ERROR;        //received echo of synchronization character
      return;
    } else {
      derr = DC_SYNCH_ERROR;
#if DRIVE_COMM_DEBUG
      cout << "[DriveComm debug]: ...error: " << derr << endl;
#endif
      continue;
    }
  }
}

/*
maxCommSpeed:
changes the communication baud rate from the default 9600 to the maximum 115200
now performed by controller in initial program
*/
void DriveCommunicator::maxCommSpeed(unsigned short dest)
{
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: Setting communications speed to maximum:" << endl;
#endif
  if (this->serialDeviceName == "") return;     //must have an open connection to speed it up
  if (highspeed) return;  //already at high speed

  //update the settings on the motor drive
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: ...Changing the settings on the drive." << endl;
#endif
  unsigned short data = 4;
  MotorCommand cmd(dest, 0x0820, &data, 1);  //SCIBR (0x0820) command sets baud rate, data of 4 means 115200
  cmd.buildCommand();
  sendCommand(&cmd);

  //now update the settings on the serial communication channel
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: ...updating serial settings (closes and reopens connection)" << endl;
#endif
  string deviceName = this->serialDeviceName;      //backup since closing will erase member variable
  this->closeConnection();
  this->openConnection(deviceName, true);
}

/*
sendCommand:
sends a binary comand to the motor drive and reads back the confirmation
*/
void DriveCommunicator::sendCommand(MotorCommand *cmd)
{
  lastCommand.setCommand(cmd->getCommand());
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: attempting to SEND the command: ";
  lastCommand.printData(cout);
  cout << endl;
#endif
  int n=1;
  char ret;                  //holder for confirmation character
  fd_set input, output;
  struct timeval timeout;

  timeout.tv_sec = 1;
  timeout.tv_usec = 0;    //1s timeout
  FD_ZERO(&output);
  FD_SET(portFD, &output);
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: ...waiting for chance to write command" << endl;
#endif
  n = select(portFD+1, NULL, &output, NULL, &timeout);              //wait until a write won't block
  if (n <= 0 || !FD_ISSET(portFD, &output)) {    //error, timeout, or something probably impossible
    derr = DC_SERIAL_ERROR;
    return;
  }
  n = write(portFD, lastCommand.getCommand(), (size_t)lastCommand.getCommandLength()); //if no error or timeout, then able to write
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: ...wrote " << n << " bytes, command has a total of: " << lastCommand.getCommandLength() << endl;
#endif
  if (n < 0) {    //error
    derr = DC_SERIAL_ERROR;
    return;
  }

  timeout.tv_sec = 1;
  timeout.tv_usec = 0;
  FD_ZERO(&input);
  FD_SET(portFD, &input);
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: ...waiting for chance to read confirmation" << endl;
#endif
  n = select(portFD+1, &input, NULL, NULL, &timeout);         //wait until read won't block
  if (n <= 0 || !FD_ISSET(portFD, &input)) {
    derr = DC_SERIAL_ERROR;
#if DRIVE_COMM_DEBUG
    cout << "[DriveComm debug]: ...could not read confirmation" << endl;
#endif
    return;
  }
  n = read(portFD, &ret, 1);                      //read only single return byte
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: ...read confirmation character: \"" << ret << "\"" << endl;
#endif
  if (n < 0) {    //error
    derr = DC_SERIAL_ERROR;
    return;
  }

  if (ret == 0x4f) derr = DC_NO_ERROR;        //received correct confirmation character
  else derr = DC_BAD_RETURN;

}

/*
sendQuery:
queries the drive (with axis id 'dest') for a 16 or 32 bt value from a given memory address (dataAddr)
when 'has32bits' is true, will get a 32 bit value, when false will get a 16 it value
the answer command will be stored in the class member returnVal and a pointer to it returned by the function (or NULL on error)
this method assumes a sener address based on using only a single drive over RS-232
*/
MotorCommand* DriveCommunicator::sendQuery(unsigned short dest, unsigned short dataAddr, bool has32bits)
{
  unsigned short sender = dest | 0x0001;  //should be true for RS-232 when with only one driveCLensAdapter
  return sendQuery(dest, dataAddr, sender, has32bits);
}


/*
sendQuery:
queries the drive (with axis id 'dest') for a 16 or 32 bt value from a given memory address (dataAddr)
when 'has32bits' is true, will get a 32 bit value, when false will get a 16 it value
the answer command will be stored in the class member returnVal and a pointer to it returned by the function (or NULL on error)
senderId is the axis ID of the computer sending the command
*/
MotorCommand* DriveCommunicator::sendQuery(unsigned short dest, unsigned short dataAddr, unsigned short senderId, bool has32bits)
{
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: sending a QUERY to the drive" << endl;
#endif

  //construct the command to send
  unsigned short opcode = (has32bits)?0xb005:0xb004;
  unsigned short data[2];
  data[0] = senderId;
  data[1] = dataAddr;
  MotorCommand cmd(dest, opcode, data, 2);
  cmd.buildCommand();
  unsigned short readLength = (has32bits)?14:12;    //number of bytes to be read in the return
  unsigned char buf[15];    //needs rooom for up to a 14 byte response and a confirm byte

  lastCommand.setCommand(cmd.getCommand());
  int n=1;
  fd_set input, output;
  struct timeval timeout;

  timeout.tv_sec = 1;
  timeout.tv_usec = 0;    //1s timeout
  FD_ZERO(&output);
  FD_SET(portFD, &output);
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: ...waiting for chance to write query command" << endl;
#endif
  n = select(portFD+1, NULL, &output, NULL, &timeout);              //wait until a write won't block
  if (n <= 0 || !FD_ISSET(portFD, &output)) {    //error, timeout, or something probably impossible
    derr = DC_SERIAL_ERROR;
    return NULL;
  }
  n = write(portFD, lastCommand.getCommand(), (size_t)lastCommand.getCommandLength()); //if no error or timeout, then able to write
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: ...wrote " << n << " bytes, command has a total of: " << lastCommand.getCommandLength() << endl;
#endif
  if (n < 0) {    //error
    derr = DC_SERIAL_ERROR;
    return NULL;
  }

  //for some reason, can't seem to read the entire answer at once, so try a few times in a loop
  unsigned char answer[15];   //holds total response, each loop will add on to this
  n = 0;
  for (int charsremaining = readLength + 1; charsremaining > 0; charsremaining-=n) {
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    FD_ZERO(&input);
    FD_SET(portFD, &input);
#if DRIVE_COMM_DEBUG
    cout << "[DriveComm debug]: ...waiting for chance to read answer" << endl;
#endif
    n = select(portFD+1, &input, NULL, NULL, &timeout);         //wait until read won't block
    if (n <= 0 || !FD_ISSET(portFD, &input)) {
      derr = DC_SERIAL_ERROR;
#if DRIVE_COMM_DEBUG
      cout << "[DriveComm debug]: ...could not read confirmation" << endl;
#endif
      return NULL;
    }
    n = read(portFD, buf, charsremaining);                      //read answer + return byte
#if DRIVE_COMM_DEBUG
    cout << "[DriveComm debug]: ...read " << n << " bytes out of " << charsremaining << endl;
#endif
    if (n < 0) {    //error
      derr = DC_SERIAL_ERROR;
      return NULL;
    } 
    //transfer result from the read buffer
    for (int i=0; i<n; i++) answer[readLength+1-charsremaining+i] = buf[i];
    //check confirmation character...if it is invalid, there is no sense in continuing to try to read
    if (answer[0] != 0x4f) {
      derr = DC_BAD_RETURN;
      return NULL;
    }
  }
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: ...received answer: " << hex << setfill('0');
  for (int i=0; i<n; i++) cout << setw(2) << (int)answer[i];
  cout << dec << setfill(' ') << endl;
#endif

  returnVal.setCommand(&answer[1]); //don't include confirm character in return command
  if (!returnVal.isValid()) {
    derr = DC_BAD_RETURN;
    return NULL;
  }

  //otherwise, everything is OK
#if DRIVE_COMM_DEBUG
  cout << "[DriveComm debug]: ...successfully parsed answer" << endl;
#endif
  derr = DC_NO_ERROR;        //received well formed return
  return &returnVal;
}


/*
double2Fixed:
converts a double floating-point value to a 32bit fixed point value in the format recognized by the drive
*/
unsigned int DriveCommunicator::double2Fixed(double floatval)
{
  double fintpart, ffracpart;
  short intpart;
  unsigned short fracpart;
  int total;
  ffracpart = modf(floatval, &fintpart);
  if (fintpart >= SHRT_MAX) intpart = SHRT_MAX;
  if (fintpart <= SHRT_MIN) intpart = SHRT_MIN;
  else intpart = (short)fintpart;
  if (ffracpart < 0) ffracpart *= -1;     //remove sign from fractional part
  fracpart = (unsigned short)(ffracpart * pow(2,16));
  total = ((int)intpart)<<16;
  total += fracpart;
  return total;
}

/*
fixed2Double:
converts a 32bit fixed point value (in the drive's format) to a double floating-point value
*/
double DriveCommunicator::fixed2Double(unsigned int fixedval)
{
  double floatval;
  floatval = (double)fixedval;
  return floatval / pow(2,16);
}

/*
sendSpeedCommand:
sends a command to set the CSPD (speed) value and UPD (update)
negative speeds are achieved with a positive CSPD value but a negative CPOS one
the class keeps track of direction to avoid always having to send a CSPD command
*/
void DriveCommunicator::sendSpeedCommand(unsigned short dest, double speed)
{
  int fixedspeed = DriveCommunicator::double2Fixed(fabs(speed));
  MotorCommand cspd(dest, 0x24a0), upd(dest, 0x0108);      //set speed and update commands
  cspd.addData((unsigned short)fixedspeed);
  cspd.addData((unsigned short)(fixedspeed>>16));
  cspd.buildCommand();
  upd.buildCommand();
  this->sendCommand(&cspd);
  if (derr != DC_NO_ERROR) return;  //command failed

  //if speed is different direction than currently, need to change CPOS
  if (!this->dirForward && speed > 0) {       //currently going backward, forward speed command
    //unsigned short pdata[2] = {0xFFFF, 0x7FFF}; //LSB and MSB of largest position command
    unsigned short pdata[2] = {0xca00, 0x3b9a}; //set to 1e9
    MotorCommand cpos(dest, 0x249e, pdata, 2);
    cpos.buildCommand();
    this->sendCommand(&cpos);
    if (derr != DC_NO_ERROR) return;  //command failed
    this->dirForward = true;
  }
  else if (this->dirForward && speed < 0) {      //currently going forward, backward speed command
    //unsigned short pdata[2] = {0x0000, 0x8000}; //LSB and MSB of most negative position
    unsigned short pdata[2] = {0x3600, 0xc425}; //set to -1e9
    MotorCommand cpos(dest, 0x249e, pdata, 2);
    cpos.buildCommand();
    this->sendCommand(&cpos);
    if (derr != DC_NO_ERROR) return;  //command failed
    this->dirForward = false;
  }

  //send update
  this->sendCommand(&upd);
  if (derr != DC_NO_ERROR) return;  //command failed
}

/*
getDriveTemp:
sends a query to the drive with axis ID dest and parses return value sent back to computer (located at senderID)
TODO for this to be useful, need to calibrate a gain and offset for the AD values found.
*/
double DriveCommunicator::getDriveTemp(unsigned short dest, unsigned short senderID)
{
  if (sendQuery(dest, 0x0243, senderID, false) == NULL) return -1;
  unsigned short rawtemp = returnVal.getData(3);
  return (double) rawtemp;
}
