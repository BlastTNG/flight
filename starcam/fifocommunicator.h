//class for reading, interpreting, and executing commands received via a FIFO

#ifndef FIFOCOMMUNICATOR_H
#define FIFOCOMMUNICATOR_H

#include <string>
#include <sys/time.h>
#include "camconfig.h"
#include "mycam.h"
#include "blobimage.h"

/**
	@author Steve Benton <steve.benton@utoronto.ca>
	this class is no longer being used. instead use CamCommunicator with a localhost socket
*/
class FIFOCommunicator{
public:
    FIFOCommunicator();
	FIFOCommunicator(string fifoInName, string fifoOutName);
	~FIFOCommunicator();
	void Init();
	
	int openFifos(string fifoInName, string fifoOutName);
	int closeFifos();
	
	//main program of communicator: infinite loop that reads commands and executes on global objects
	void readLoop(string (*interpretFunction)(string));
	string looplessRead();
	
	//send a return value to the flight computer
	int sendReturn(const StarcamReturn* returnStruct);
	int sendReturnString(string returnStr);
	
	//functions for use on "flight computer"
	static StarcamReturn* interpretReturn(string returnString, StarcamReturn* rtn);
	int openOppositeFifos(string fifoInName, string fifoOutName);
	int sendCommand(string cmd);
	int sendCommand(const char* cmd);      //in case flight uses C only

	//accessors
	int getErrorFlag() { return m_nErrorFlag; }
	
private:
	string m_sFifoInName;           //filename of input (from flight) fifo
	string m_sFifoOutName;          //filename of output (to flight) fifo
	int m_nFifoInFD;                //file descriptor for input fifo
	int m_nFifoOutFD;               //file descriptor for output fifo
	int m_nErrorFlag;               //will be -1 in case of error, 0 otherwise. check after constructor
};

#endif
