#include <string.h>
#include <stdio.h>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>      //for gethostbyname
#include <cstdlib>
#include "camcommunicator.h"
extern "C" {
#include "udp.h"
#include "blast.h"
}
#define CAM_COMM_DEBUG 0
#if CAM_COMM_DEBUG
#include <iostream>
#endif

/*

 CamCommunicator:
 
 default constructor, initializes members
 
*/
CamCommunicator::CamCommunicator()
{
	Init();
}


/*

 ~ CamCommunicator:
 
 destructor, closes connection
 
*/
CamCommunicator::~CamCommunicator()
{
	closeConnection();
}


/*

 Init:
 
 initailizes member variables
 
*/
void CamCommunicator::Init()
{
	target = "";
	serverFD = -1;
	commFD = -1;
	errorFlag = 0;
}


/*

 init_sockaddr:
 
 non-member socket utility function lifted from glibc docs
 initializes the sockaddr structure given a hostname and port
 changed to return -1 on error, 0 on success
 
*/
int init_sockaddr(sockaddr_in *name, const char* hostname, uint16_t port)
{
	hostent *hostinfo;
	name->sin_family = AF_INET;
	name->sin_port = htons(port);
	hostinfo = gethostbyname(hostname);
	if (hostinfo == NULL) return -1;
	name->sin_addr = *(in_addr *) hostinfo->h_addr;
	return 0;
}


/*

 openHost:
 
 opens communications from the host's side, returns -1 on failure and 0 otherwise
 for sockets, target should be of the form "addr:port"
 where addr is a hostname or address recognizable by gethostbyname
  it can also have the special value "any" to allow connections from anywhere
 and port is a port number (can be omitted, will use a default value)
 
*/
int CamCommunicator::openHost()
{
	if (commFD >= 0) return -1;  //already an open connection
	struct addrinfo hints, *ai_other, *p;
	char service[20];
	int ret;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET; //use IPv4, AF_UNSPEC allows IPv4 or IPv6
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE | AI_NUMERICSERV;

	/* deal with the weirdness */
	sprintf(service, "%i", SC_PORT);

	if ((ret = getaddrinfo(NULL, service, &hints, &ai_other)) != 0) {
#if CAM_COMM_DEBUG
    cout << "getaddrinfo: " << gai_strerror(ret) << endl;
#endif
	}

	// loop through getaddrinfo results
	for(p = ai_other; p != NULL; p = p->ai_next) {
		if ((commFD = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
			continue;
		}
		if (bind(commFD, p->ai_addr, p->ai_addrlen) == -1) {
			close(commFD);
			continue;
		}
		break;
	}
	if (p == NULL) {
#if CAM_COMM_DEBUG
    cerr << "failed to bind socket\n" << endl;
#endif
  }
	freeaddrinfo(ai_other);
	return 0;
}

/*

 openClient:
 
 opens client communications
 for sockets, target should be of the form "addr:port"
 where addr is a hostname or address recognizable by gethostbyname
 and port is a port number (can be omitted, will use a default value)
 
*/
int CamCommunicator::openClient(const char* target)
{
	if (commFD >= 0) return -1;   //already an open connection
	struct addrinfo hints, *ai_other;
	char service[20];
	int ret;

	memset(&hints, 0, sizeof hints);
	hints.ai_family = AF_INET; //use IPv4, AF_UNSPEC allows IPv4 or IPv6
	hints.ai_socktype = SOCK_DGRAM;
	hints.ai_flags = AI_PASSIVE | AI_NUMERICSERV;

	/* deal with the weirdness */
	sprintf(service, "%i", SC_PORT);

	if ((ret = getaddrinfo(NULL, service, &hints, &ai_other)) != 0) {
#if CAM_COMM_DEBUG
		cout << "getaddrinfo: " << gai_strerror(ret) << endl;
#endif
	}

	// loop through getaddrinfo results
	for(p = ai_other; p != NULL; p = p->ai_next) {
		if ((commFD = socket(p->ai_family, p->ai_socktype, p->ai_protocol)) == -1) {
			continue;
		}
		if (bind(commFD, p->ai_addr, p->ai_addrlen) == -1) {
			close(commFD);
			continue;
		}
		break;
	}
	if (p == NULL) {
#if CAM_COMM_DEBUG
    cerr << "failed to bind socket\n" << endl;
#endif
  }
	freeaddrinfo(ai_other);
	
	this->target = target;
	return 0;
	
}


/*

 closeConnection:
 
 if an open connection exists, close it
 
*/
void CamCommunicator::closeConnection()
{
	if (serverFD >= 0) {
		close(serverFD);
		serverFD = -1;
	}
	if (commFD >= 0) {
		close(commFD);
		commFD = -1;
	}
	if (strcmp(target,"") != 0) target = "";
}



/*

 readLoop:
 
 Infinite loop that reads and then interprets commands via interpretFunction
 Designed to run concurrently with pictureLoop and processingLoop (in starcam.cpp)
 If it returns at all then an error has occured
 The interpret function returns a string to be sent back to flight computer
 
*/
void CamCommunicator::readLoop(string (*interpretFunction)(string))
{
#if CAM_COMM_DEBUG
	cerr << "[Comm debug]: in readLoop method" << endl;
#endif
	char buf[UDP_MAXSIZE];
	string line = "";
	string rtnStr;
	string::size_type pos;
	ssize_t n;

	while (1) {
    		usleep(100000);
		if ((n = recvfrom(commFD, buf, UDP_MAXSIZE, 0, NULL, NULL)) == -1) return;
		if (n == 0) {  //link may be dead...check it out
#if CAM_COMM_DEBUG
			cerr << "[Comm debug]: read empty string, checking if link is dead " << buf << endl;
#endif
			//otherwise, link has been reestablished, continue
		} else { //n > 0
#if CAM_COMM_DEBUG
			cerr << "[Comm debug]: readloop just read " << n << " bytes: " << buf << endl;
#endif
			buf[n] = '\0';
			line += buf;
		}
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
string CamCommunicator::looplessRead()
{
	char buf[UDP_MAXSIZE];
	int n;
	if (commFD == -1) return "";          //communications aren't open
	
	if ((n = recvfrom(commFD, buf, UDP_MAXSIZE, 0, NULL, NULL)) == -1) return "";
	if (n == 0) {  //link may be dead...check it out
#if CAM_COMM_DEBUG
		cerr << "[Comm debug]: read empty string, checking if link is dead " << buf << endl;
#endif
	}
	//otherwise all is well
#if CAM_COMM_DEBUG
	cerr << "[Comm debug]: read just read " << n << " bytes: " << buf << endl;
#endif
	buf[n] = '\0';
	return (string)buf;
}

/*

 sendReturn:
 
 Sends data in rtn back to "flight computer"
 returns number of characters written or -1 on error
 
*/
string CamCommunicator::buildReturn(const StarcamReturn* rtn)
{
	ostringstream sout;
	sout << rtn->frameNum << " " << rtn->mapmean << " " << rtn->sigma << " " << rtn->exposuretime << " " 
			<< rtn->imagestarttime.tv_sec << " " << rtn->imagestarttime.tv_usec 
			<< " " << rtn->camID << " " << rtn->ccdtemperature << " " << rtn->cputemperature << " " << rtn->focusposition << " " << rtn->numblobs << " ";
	
	int top = 15;
	if (rtn->numblobs < 15) top = rtn->numblobs;
	for (int i=0; i<top; i++)
	{
		sout << rtn->flux[i] << " " << rtn->mean[i] << " " << rtn->snr[i] << " " 
				<< rtn->x[i] << " " << rtn->y[i] << " ";
	}
	
	string output = sout.str();
#if CAM_COMM_DEBUG
	cerr << "[Comm debug]: built return of " << output.length() << " chars:\n" << output << endl;
#endif
	return output;
}

int CamCommunicator::sendReturn(const StarcamReturn* rtn)
{
#if CAM_COMM_DEBUG
	cerr << "[Comm debug]: in sendReturn method" << endl;
#endif
	return sendCommand(buildReturn(rtn));
}

/*

 sendReturnString:
 
 sends a string return value from a command
 value is surrounded by <str></str> tags to identify it as a string
 
*/
string CamCommunicator::buildReturnString(string returnStr)
{
	return ((string)"<str>" + returnStr + "</str>");
}

int CamCommunicator::sendReturnString(string returnStr)
{
	return sendCommand(buildReturnString(returnStr));
}
	
/*

 interpretReturn:
 
 For use on "flight computer". Interprets data sent via sendReturn
 pass pointer to already declared struct to populate
 
*/
StarcamReturn* CamCommunicator::interpretReturn(string returnString, StarcamReturn* rtn)
{
#if CAM_COMM_DEBUG
	cerr << "[Comm debug]: in interpretReturn method" << endl;
#endif
	istringstream sin;
	sin.str(returnString);
	
	sin >> rtn->frameNum >> rtn->mapmean >> rtn->sigma >> rtn->exposuretime >> rtn->imagestarttime.tv_sec
			>> rtn->imagestarttime.tv_usec >> rtn->camID >> rtn->ccdtemperature >> rtn->cputemperature >> rtn->focusposition >> rtn->numblobs;
	
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
 
 for use predominantly on "flight computer". sends a command string
 returns -1 on error, number of characters written otherwise
 written for both C++ string and char* strings
 
*/
int CamCommunicator::sendCommand(string cmd)
{
#if CAM_COMM_DEBUG
	cerr << "[Comm debug]: in sendCommand method with: " << cmd << endl;
#endif
	if (commFD == -1) return -1;          //communications aren't open

	//remove all newlines and add a single one at the end
	string sought = "\n";
	string::size_type pos = cmd.find(sought, 0);
	while (pos != string::npos) {
		cmd.replace(pos, sought.size(), "");
		pos = cmd.find(sought, pos - sought.size());
	}
	cmd += "\n";
	
  	if (sendto(commFD, cmd.c_str(), sizeof(cmd), 0, p->ai_addr, p->ai_addrlen) < 0) return -1;
  	return 0;
}

int CamCommunicator::sendCommand(const char* cmd)       //in case flight wants to use C only
{
	string str_cmd = cmd;
	return sendCommand(str_cmd);
}


