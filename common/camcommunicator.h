#ifndef CAMCOMMUNICATOR_H
#define CAMCOMMUNICATOR_H

#include <string>
#include "camstruct.h"

#define GOOD_PORT 2671
#define BAD_PORT 2672
#define UGLY_PORT 2673
#define GBU_PORT 2674
#define SC_PORT 2675 
#define SC_PORT_GOOD 2676 
#define SC_PORT_BAD 2677 
#define SC_PORT_UGLY 2678 
#define FOCUS_PORT 2679

/**
	@author Steve Benton <steve.benton@utoronto.ca>
	Class with two modes of operation: host (flight computer) or client (camera computer)
	Relays and partially interprets communications between a host and the client
	Currently implemented using network sockets
*/
class CamCommunicator{
public:
    CamCommunicator();

    ~CamCommunicator();
	
	void Init();
	
	int openHost();
	int openClient(const char* target);
	void closeConnection();
	
	//main program of communicator: infinite loop that reads commands and executes on global objects
	void readLoop(string (*interpretFunction)(string));
	string looplessRead();

	//send a return value to the flight computer
	int sendReturn(const StarcamReturn* returnStruct);
	int sendReturnString(string returnStr);
	//or build a return without actually sending it
	static string buildReturn(const StarcamReturn* returnStruct);
	static string buildReturnString(string returnStr);
	
	//functions for use on "flight computer"
	static StarcamReturn* interpretReturn(string returnString, StarcamReturn* rtn);
	int sendCommand(string cmd);
	int sendCommand(const char* cmd);      //in case flight uses C only

	//accessors
	int getErrorFlag() { return errorFlag; }
	
private:
	const char* target;           //a string specifying where connections will come from/go
	int serverFD;                 //file descriptor for host only: socket server
				      //a value of -2 means that an external server is being used
	int commFD;                   //file descriptor for both: socket communications
	struct addrinfo *p;
	int errorFlag;                //will be -1 in case of error, 0 otherwise

	//allow the new server class to handle some internals
	friend class CamCommServer;

};

#endif
