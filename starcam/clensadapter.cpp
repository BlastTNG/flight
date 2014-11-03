#include "clensadapter.h"
#include <sys/select.h>
#include <string>
#include <sstream>
#include <iostream>
#include <cmath>
#include <stdio.h>   /* standard input and output */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <sys/ioctl.h>

#define LENS_DEBUG 0
#define OLD_LENS 1


using namespace std;

const string command_string[] = {"","id","hv","br","bs","rm","xm","sn","vn","vs","lv",
	"mi","mz","fa","mf","pf","fd","sf","dz","in","da","mo","mc","ma","mn","pa","is","df"
}; //two character command strings corresponing to the LENS_COMMAND enum

//indicates if a command needs a value parameter (1) or not (0); corresponds to LENS_COMMAND enum
const short needs_param[] = {0,0,0,1,0,1,0,0,0,0,0,0,0,1,1,0,0,1,0,0,0,0,0,1,1,0,0,0};

const string error_str[] = {"No error", "Could not parse command", "Lens set to manual focus",
	"No lens present", "Distance information not available", "Lens not initialized",
	"Invalid baud rate", "No shutter present", "Insufficient power", "Invalid library",
	"Lens communication error", "Bad Parameter", "Connection error", "Connection Timeout",
	"Non-lens error in autofocus"
}; //string descriptions corresponding to LENS_ERROR enum

/*
	CLensAdapter:
		default constructor. Doesn't establish a connection
*/
CLensAdapter::CLensAdapter()
{
	Init();
}

/*
	CLensAdapter:
		constructor that explicitly specifies the device name
		be sure to check afterwards for an error in establishing a connection
*/
CLensAdapter::CLensAdapter(string deviceName)
{
	Init();
	openConnection(deviceName);
}

/*
	~CLensAdapter:
		default destructor. closes the connection
*/
CLensAdapter::~CLensAdapter()
{
	closeConnection();
}

/*
	Init:
		sets default values of class members
*/
void CLensAdapter::Init(LensAdapterConfigParams params/*=defaultCameraParams.lensParams*/)
{
#if LENS_DEBUG
	cout << "[Lens Debug]: Inside the Init method" << endl;
#endif
	m_eLastCommand = params.lastCommand;
	m_eLastError = params.lastError;
	m_nPortFD = params.portFD;
	m_sSerialDeviceName = params.deviceName;
	m_nFocalRange = params.focalRange;
	m_nFocusTol = params.focusTol;
}
/* original method has been replaced
void CLensAdapter::Init()
{
	m_eLastCommand = LC_NULL;
	m_eLastError = LE_NO_ERROR;
	m_nPortFD = -1;
	m_sSerialDeviceName = "";
}
*/

/*
	openConnection:
		opens a connection to file named deviceName in /dev/
*/
LENS_ERROR CLensAdapter::openConnection(string deviceName)
{
#if LENS_DEBUG
	cout << "[Lens Debug]: Inside the openConnection method" << endl;
#endif
	m_sSerialDeviceName = deviceName;
	m_nPortFD = open(deviceName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
	if (m_nPortFD == -1) { //open failed
		m_sSerialDeviceName = "";
		return (m_eLastError = LE_CONNECTION_ERROR);
	}
	
	//set baud rate, character size, and parity
	struct termios options;
	tcgetattr(m_nPortFD, &options);
	cfsetispeed(&options, B19200);               //input speed
	cfsetospeed(&options, B19200);               //outpur speed
	/* Original options...didn't work right
	options.c_cflag |= (CLOCAL | CREAD);         //set local mode and enable receive
	options.c_cflag &= ~PARENB;                  //no parity
	options.c_cflag &= ~CSTOPB;                  //1 stop bit 
	options.c_cflag &= ~CSIZE;                   //mask character size
	options.c_cflag |= CS8;                      //8-bit characters
	options.c_cflag &= ~CRTSCTS;                 //disable hardware flow control
	options.c_lflag |= ICANON;                   //enable canonical (line-based) mode
	options.c_iflag &= ~(IXON | IXOFF | IXANY);  //disable software flow control
	options.c_iflag |= INLCR;*/
	
	options.c_iflag = 0;
	options.c_iflag = ICRNL;                     //map '\r' to '\n' on input
	options.c_oflag = 0;
	options.c_cflag &= ~(CSTOPB | CSIZE);
	options.c_cflag |= CS8;
	options.c_lflag = 0;
	options.c_lflag |= ICANON;                   //enable canonical (line-based) mode


	tcsetattr(m_nPortFD, TCSANOW, &options);     //apply changes

	//flush the buffer (in case of unclean shutdown)
	if (tcflush(m_nPortFD, TCIOFLUSH) < 0) {
#if LENS_DEBUG
		cout << "[Lens Debug]: failed to flush buffer" << endl;
#endif
	}
	
	return (m_eLastError = LE_NO_ERROR);
}

/*
	closeConnection:
		closes the current connection
*/
LENS_ERROR CLensAdapter::closeConnection()
{
#if LENS_DEBUG
	cout << "[Lens Debug]: Inside the closeConnection method" << endl;
#endif
	if (close(m_nPortFD) < 0)
		return (m_eLastError = LE_CONNECTION_ERROR);
	m_nPortFD = -1;
	m_sSerialDeviceName = "";	
	return (m_eLastError = LE_NO_ERROR);
}

/*

 getErrorString:
 
 returns a string description of the error err
 
*/
string CLensAdapter::getErrorString(LENS_ERROR err)
{
	return error_str[(int) err];
}

/*

 findFocalRange:
 
 sets focus to zero and then infinity to find the range of motor counts while focussing
 sets the member m_nFocalRange when this is found
 
*/
LENS_ERROR CLensAdapter::findFocalRange()
{
	LENS_ERROR err = LE_NO_ERROR;
	string lens_return_str, range_str;
	unsigned int end_pos;
	if (m_nPortFD == -1)  //check if there is an open connection
		return (m_eLastError = LE_CONNECTION_ERROR);
	
	if ((err = this->runCommand(LC_MOVE_FOCUS_ZERO,lens_return_str)) != LE_NO_ERROR) 
		return err;
	cout << "erase me: mz gave: " << lens_return_str << endl;
	if ((err = this->runCommand(LC_MOVE_FOCUS_INFINITY,lens_return_str)) != LE_NO_ERROR) 
		return err;
	cout << "erase me: mi gave: " << lens_return_str << endl;
	
	//if no error occured, lens_return_str should be "OK\nDONExxx,1\n" where xxx == focal range
	end_pos = lens_return_str.find(",",0);
	if (end_pos == string::npos) return (m_eLastError = LE_BAD_PARAM);
	range_str = lens_return_str.substr(4, end_pos-4);
	m_nFocalRange = atoi(range_str.c_str());
	
	return LE_NO_ERROR;
}

/*
	runCommand:
		sends the string cmd and records response in return_value
		should only ever be called by other forms of the runCommand function
		
*/
LENS_ERROR CLensAdapter::runCommand(string cmd, string &return_value)
{
#if LENS_DEBUG
	cout << "[Lens Debug]: Inside the major runCommand method" << endl;
#endif
	int n=1;
	unsigned int end_pos;
	string err_code_str;
	char buf[255];                  //input buffer
	string first, second;
	fd_set input, output;
	struct timeval timeout;
	string cmdNoR = cmd;
	if (cmd.find("\r") == string::npos) cmd += "\r";     //insert carriage return as needed
	
#if LENS_DEBUG
	cout << "[Lens Debug]: performing write of command: " << cmd << endl;
#endif
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	FD_ZERO(&output);
#if LENS_DEBUG
	cout << "[Lens Debug]:   adding fd=" << m_nPortFD << " to output set... " << endl;
#endif
	FD_SET(m_nPortFD, &output);
	n = select(m_nPortFD+1, NULL, &output, NULL, &timeout);
	if (n < 0) return (m_eLastError = LE_COMMUNICATION_ERROR);
	if (n == 0) return (m_eLastError = LE_TIMEOUT);
	if (FD_ISSET(m_nPortFD, &output)) {
#if LENS_DEBUG
		cout << "[Lens Debug]:   calling write method... " << endl;
#endif
		n = write(m_nPortFD, cmd.c_str(), cmd.length()); //if no error or timeout, then able to write
		if (n < 0) return (m_eLastError = LE_COMMUNICATION_ERROR);
	}
	else return (m_eLastError = LE_COMMUNICATION_ERROR);
#if LENS_DEBUG
	cout << "[Lens Debug]: performing read of return value... " << endl;
#endif
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	FD_ZERO(&input);
	FD_SET(m_nPortFD, &input);
	n = select(m_nPortFD+1, &input, NULL, NULL, &timeout);
	if (n < 0) return (m_eLastError = LE_COMMUNICATION_ERROR);
	if (n == 0) return (m_eLastError = LE_TIMEOUT);
	if (FD_ISSET(m_nPortFD, &input)) {
#if LENS_DEBUG
		cout << "[Lens Debug]:   calling read... " << endl;
#endif
		n = read(m_nPortFD, buf, 254);
		if (n < 0) return (m_eLastError = LE_COMMUNICATION_ERROR);
		buf[n] = '\0';
		first = buf;
#if LENS_DEBUG
		cout << "[Lens Debug]:   read line of output: " << first << endl;
#endif
	}
	return_value = first;
	
	//check return value for an error code
	if (first.find("ERR", 0) == 0) {  //an error occured in processing the command
		end_pos = first.find("\n",0);
		if (end_pos == string::npos) return (m_eLastError = LE_COMMUNICATION_ERROR);
		err_code_str = first.substr(3,end_pos-3);
		return (m_eLastError = (LENS_ERROR)atoi(err_code_str.c_str()));
	}
#if OLD_LENS
        if ((first.find(cmdNoR))!=(string::npos)) {
                for (int i=0; i<2; i++) {     //repeat to first read command echo and then "OK"
#if LENS_DEBUG
                        cout << "[Lens Debug]: performing more read (OK, then return value... " << endl;
#endif
                        timeout.tv_sec = 10;
                        timeout.tv_usec = 0;
                        FD_ZERO(&input);
                        FD_SET(m_nPortFD, &input);
                        n = select(m_nPortFD+1, &input, NULL, NULL, &timeout);
                        if (n < 0) return (m_eLastError = LE_COMMUNICATION_ERROR);
                        if (n == 0) return (m_eLastError = LE_TIMEOUT);
                        if (FD_ISSET(m_nPortFD, &input)) {
#if LENS_DEBUG
                                cout << "[Lens Debug]:   calling read... " << endl;
#endif
                                n = read(m_nPortFD, buf, 254);                      //in canonical mode, should read until \r
                                if (n < 0) return (m_eLastError = LE_COMMUNICATION_ERROR);
                                buf[n] = '\0';                //turn buffer into a C-style string
                                second = buf;                  //first line of result (should read "OK")
#if LENS_DEBUG
                                cout << "[Lens Debug]:   read line of output: " << second << endl;
#endif
                        }
                }
                return_value = second;
        }
#endif	
	return (m_eLastError = LE_NO_ERROR);
}

/*
	runCommand:
		runs the command cmd as long as it needs no parameter
		return_value is set to the string returned from the adapter
*/
LENS_ERROR CLensAdapter::runCommand(LENS_COMMAND cmd, string &return_value)
{
#if LENS_DEBUG
	cout << "[Lens Debug]: Inside the minor runCommand method (no parameter)" << endl;
#endif
	if (needs_param[(int)cmd]) return (m_eLastError = LE_BAD_PARAM);
	m_eLastCommand = cmd;
	string str_cmd = command_string[(int)cmd];
	return runCommand(str_cmd, return_value);
}

/*
	runCommand:
		runs the command cmd that requires a paramter value (val)
		return_value is set to the string returned from the adapter
*/
LENS_ERROR CLensAdapter::runCommand(LENS_COMMAND cmd, int val, string &return_value)
{
#if LENS_DEBUG
	cout << "[Lens Debug]: Inside the minor runCommand method (with paramter)" << endl;
#endif
	ostringstream sout;
	if (!needs_param[(int)cmd]) return (m_eLastError = LE_BAD_PARAM);
	m_eLastCommand = cmd;
	sout << command_string[(int)cmd] << val;
	return runCommand(sout.str(), return_value);
}

/*

 preciseMove:
 
 moves the focus incrementally by a precise number of counts
 will perfrm multiple moves when a move misses slightly
 will allow a miss tolerance of tol
 if the lens reaches a stop, indicates unmoved counts in remaining
 when forced is nonzero (TRUE) will ignore stops until not moving
*/
LENS_ERROR CLensAdapter::preciseMove(int counts, int &remaining, int forced/*=0*/)
{
	string return_str, counts_str, stop_str;      //values returned by move function
	unsigned int sep_pos;                         //position of separator "," in return_str
	LENS_ERROR err;
	remaining = counts;
	int moved_by;                                 //the amount moved in each iteration
	int stop_flag = 0;                            //set when a stop is hit
	int no_move_cnt = 0;                          //counts number of times didn't move
	
	while (abs(remaining) > (int)m_nFocusTol) {
		//move by amount remaining
		if ((err = runCommand(LC_MOVE_FOCUS_INC,remaining,return_str)) != LE_NO_ERROR)
			return err;
		sep_pos = return_str.find(",",0);
		if (sep_pos == string::npos) return (m_eLastError = LE_BAD_PARAM);
		counts_str = return_str.substr(4,sep_pos-4);
		stop_str = return_str.substr(sep_pos+1,1);
#if LENS_DEBUG
		cout << "[Lens Debug]: preciseMove moved by " << counts_str << " counts";
		cout << " and " << ((stop_str == "1")?"":"did not ") << "hit a stop." << endl;
#endif
		moved_by = atoi(counts_str.c_str());
		remaining -= moved_by;
		if (stop_str == "1") {            //a stop is hit ("0" otherwise)
			if (forced) {
				//forced case, return when stop hit and move by 0 four times in a row 
				if (moved_by == 0) no_move_cnt++;
				else no_move_cnt = 0;
				if (no_move_cnt > 4) return LE_NO_ERROR;
			} else {
				//unforced case, return when a stop hit twice (avoids spurious stops)
				if (stop_flag)
					return LE_NO_ERROR;       //if a stop is hit twice in a row, return
				else stop_flag = 1;
			}
		} else {      //if a stop wasn't hit, reset both the forced and unforced indicators
			stop_flag = 0;
			no_move_cnt = 0;
		}
	}
	
	return LE_NO_ERROR;
}


