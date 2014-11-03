//this class is no longer being used. instead use CamCommunicator with a localhost socket

#include <string>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/select.h>
#include "fifocommunicator.h"

#define FIFO_DEBUG 0
#if FIFO_DEBUG
#include <iostream>
#endif

#define FIFO_BUF_SIZE 255

/*

 FIFOCommunicator:
 
 Default constructor. Initializes variables
 
*/
FIFOCommunicator::FIFOCommunicator()
{
	Init();
}

/*

 FIFOCommunicator:
 
 Useful constructor. Initializes variables and opens read and write FIFOs
 
*/
FIFOCommunicator::FIFOCommunicator(string fifoInName, string fifoOutName)
{
	Init();
	openFifos(fifoInName, fifoOutName);
}

/*

 Init:
 
 initializes member variables
 
*/
void FIFOCommunicator::Init()
{
#if FIFO_DEBUG
	cout << "[FIFO debug]: in Init method" << endl;
#endif
	m_sFifoInName = "";
	m_sFifoOutName = "";
	m_nFifoInFD = -1;
	m_nFifoOutFD = -1;
	m_nErrorFlag = 0;
}

/*

 openFifos:
 
 opens fifoInName for reading and fifoOutName for writing. 
 returns -1 on error or 0 on success
 
*/
int FIFOCommunicator::openFifos(string fifoInName, string fifoOutName)
{
#if FIFO_DEBUG
	cout << "[FIFO debug]: in openFifos method" << endl;
#endif
	if (closeFifos() < 0)                   //close old connections
		return m_nErrorFlag = -1;
	m_sFifoInName = fifoInName;
	m_sFifoOutName = fifoOutName;
#if FIFO_DEBUG
	cout << "[FIFO debug]: attempting to open RDONLY: " << m_sFifoInName << endl;
#endif
	if ((m_nFifoInFD = open(m_sFifoInName.c_str(), O_RDONLY)) < 0) {
		Init();           //reset member variables
		return m_nErrorFlag = -1;
	}
#if FIFO_DEBUG
	cout << "[FIFO debug]: attempting to open WRONLY: " << m_sFifoOutName << endl;
#endif
	if ((m_nFifoOutFD = open(m_sFifoOutName.c_str(), O_WRONLY)) < 0) {
		close(m_nFifoInFD);      //don't open just one FIFO when other fails
		Init();
		return m_nErrorFlag = -1;
	}
	return m_nErrorFlag = 0;
}

/*

 openOppositeFifos:
 
 for use on "flight computer"
 opens fifoInName for writing and fifoOutName for reading. 
 does this in opposite order of openFifos to prevent both blocking
 returns -1 on error or 0 on success
 
*/
int FIFOCommunicator::openOppositeFifos(string fifoInName, string fifoOutName)
{
#if FIFO_DEBUG
	cout << "[FIFO debug]: in openFifos method" << endl;
#endif
	if (closeFifos() < 0)                   //close old connections
		return m_nErrorFlag = -1;
	m_sFifoInName = fifoInName;
	m_sFifoOutName = fifoOutName;
#if FIFO_DEBUG
	cout << "[FIFO debug]: attempting to open WRONLY: " << m_sFifoOutName << endl;
#endif
	if ((m_nFifoOutFD = open(m_sFifoOutName.c_str(), O_WRONLY)) < 0) {
		Init();           //reset member variables
		return m_nErrorFlag = -1;
	}
#if FIFO_DEBUG
	cout << "[FIFO debug]: attempting to open RDONLY: " << m_sFifoInName << endl;
#endif
	if ((m_nFifoInFD = open(m_sFifoInName.c_str(), O_RDONLY)) < 0) {
		close(m_nFifoOutFD);      //don't open just one FIFO when other fails
		Init();
		return m_nErrorFlag = -1;
	}
	return m_nErrorFlag = 0;
}

/*

 closeFifos:
 
 closes FIFOs if they have been previously opened
 
*/
int FIFOCommunicator::closeFifos()
{
#if FIFO_DEBUG
	cout << "[FIFO debug]: in closeFifos method" << endl;
#endif
	if (m_nFifoInFD != -1 && close(m_nFifoInFD) < 0) {
		return m_nErrorFlag = -1;
	}
	if (m_nFifoOutFD != -1 && close(m_nFifoOutFD) < 0) {
		return m_nErrorFlag = -1;
	}
	Init();        //reset member variables
	return m_nErrorFlag = 0;
}

/*

 ~FIFOCommunicator:
 
 Default destructor. Closes FIFOs
 
*/
FIFOCommunicator::~FIFOCommunicator()
{
	closeFifos();
}
	
/*

 readLoop:
 
 Infinite loop that reads and then interprets commands via interpretFunction
 Designed to run concurrently with pictureLoop and processingLoop (in starcam.cpp)
 If it returns at all then an error has occured
 The interpret function returns a string to be sent back to flight computer
 
*/
void FIFOCommunicator::readLoop(string (*interpretFunction)(string))
{
#if FIFO_DEBUG
	cout << "[FIFO debug]: in readLoop method" << endl;
#endif
	fd_set input;
	char buf[FIFO_BUF_SIZE];
	string line = "";
	string rtnStr;
	int n;
	string::size_type pos;
	if (m_nFifoInFD == -1) return;          //FIFO isn't open
	
	while (1) {
		FD_ZERO(&input);
		FD_SET(m_nFifoInFD, &input);
		if (select(m_nFifoInFD+1, &input, NULL, NULL, NULL) < 0) //won't time out
			return;
		if (!FD_ISSET(m_nFifoInFD, &input)) return;  //should always be false
		if ((n = read(m_nFifoInFD, buf, FIFO_BUF_SIZE-1)) < 0) return;
		buf[n] = '\0';
#if FIFO_DEBUG
	cout << "[FIFO debug]: readloop just read " << n << " bytes: " << buf << endl;
#endif
		line += buf;
		while ((pos = line.find("\n",0)) != string::npos) {
			//interpret the command and send a return value
			if ((rtnStr = (*interpretFunction)(line.substr(0,pos))) != "") //don't send blank return
				if (sendReturnString(rtnStr) < 0) return;
			line = line.substr(pos+1, line.length()-(pos+1)); //set line to text after "\n"
		}
	}
}
	
/*

 looplessRead:
 
 similar to readLoop above but performs only read without looping
 allows user to handle looping and responses, etc.
 
*/
string FIFOCommunicator::looplessRead()
{
	fd_set input;
	char buf[FIFO_BUF_SIZE];
	int n;
	if (m_nFifoInFD == -1) return "";          //FIFO isn't open
	
	FD_ZERO(&input);
	FD_SET(m_nFifoInFD, &input);
	if (select(m_nFifoInFD+1, &input, NULL, NULL, NULL) < 0) //won't time out
		return "";
	if (!FD_ISSET(m_nFifoInFD, &input)) return "";  //should always be false
	if ((n = read(m_nFifoInFD, buf, FIFO_BUF_SIZE-1)) < 0) return "";
	buf[n] = '\0';
#if FIFO_DEBUG
	cout << "[FIFO debug]: read just read " << n << " bytes: " << buf << endl;
#endif
	return (string)buf;
}
	
/*

 sendReturn:
 
 Sends data in rtn back to "flight computer"
 returns number of characters written or -1 on error
 
*/
int FIFOCommunicator::sendReturn(const StarcamReturn* rtn)
{
#if FIFO_DEBUG
	cout << "[FIFO debug]: in sendReturn method" << endl;
#endif
	ostringstream sout;
	sout << rtn->mapmean << " " << rtn->sigma << " " << rtn->exposuretime << " " 
		 << rtn->imagestarttime.tv_sec << " " << rtn->imagestarttime.tv_usec 
		 << " " << rtn->cameraID << " " << rtn->ccdtemperature << " " << rtn->numblobs << " ";
	
	for (int i=0; i<rtn->numblobs; i++)
	{
		sout << rtn->flux[i] << " " << rtn->mean[i] << " " << rtn->snr[i] << " " 
			 << rtn->x[i] << " " << rtn->y[i] << " ";
	}
	
	string output = sout.str();
#if FIFO_DEBUG
	cout << "[FIFO debug]: return value is " << output.length() << " chars:\n" << output << endl;
#endif
	return sendCommand(output);
}

/*

 sendReturnString:
 
 sends a string return value from a command
 value is surrounded by <str></str> tags to identify it as a string
 
*/
int FIFOCommunicator::sendReturnString(string returnStr)
{
	string newstr = (string)"<str>" + returnStr + "</str>";
	return sendCommand(newstr);
}
	
/*

 interpretReturn:
 
 For use on "flight computer". Interprets data sent via sendReturn
 pass pointer to already declared struct to populate
 
*/
StarcamReturn* FIFOCommunicator::interpretReturn(string returnString, StarcamReturn* rtn)
{
#if FIFO_DEBUG
	cout << "[FIFO debug]: in interpretReturn method" << endl;
#endif
	istringstream sin;
	sin.str(returnString);
	
	sin >> rtn->mapmean >> rtn->sigma >> rtn->exposuretime >> rtn->imagestarttime.tv_sec
		>> rtn->imagestarttime.tv_usec >> rtn->cameraID >> rtn->ccdtemperature >> rtn->numblobs;
	
	int top = 15;
	if (rtn->numblobs < 15) top = rtn->numblobs;
	for (int i=0; i<top; i++)
	{
		sin >> rtn->flux[i] >> rtn->mean[i] >> rtn->snr[i] >> rtn->x[i] >> rtn->y[i];
	}
	
	return rtn;
}

/*

 sendCommand:
 
 for use predominantly on "flight computer". sends a command string over fifo
 returns -1 on error, number of characters written otherwise
 
*/
int FIFOCommunicator::sendCommand(string cmd)
{
#if FIFO_DEBUG
	cout << "[FIFO debug]: in sendCommand method with: " << cmd << endl;
#endif
	fd_set output;
	if (m_nFifoOutFD == -1) return -1;          //FIFO isn't open
	if (cmd.find("\n", 0) == string::npos) cmd += "\n";     //terminate command if it isn't already
	
	FD_ZERO(&output);
	FD_SET(m_nFifoOutFD, &output);
	if (select(m_nFifoOutFD+1, NULL, &output, NULL, NULL) < 0) //doesn't time out
		return -1;
	if (!FD_ISSET(m_nFifoOutFD, &output)) return -1;  //should always be false
	return write(m_nFifoOutFD, cmd.c_str(), cmd.length());
}

int FIFOCommunicator::sendCommand(const char* cmd)       //in case flight wants to use C only
{
	string str_cmd = cmd;
	return sendCommand(str_cmd);
}

