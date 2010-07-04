#include <string>
#include <sstream>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>      //for gethostbyname
#include <cstdlib>
#include "sbsccommunicator.h"

#define SBSC_COMM_DEBUG 0
#if SBSC_COMM_DEBUG
#include <iostream>
#endif

#define SBSC_COMM_BUF_SIZE 255
//how long to wait after failed connection attempt to try again (us)
#define CLIENT_RETRY_DELAY 1000000


/*

 SBSCCommunicator:
 
 default constructor, initializes members
 
*/
SBSCCommunicator::SBSCCommunicator()
{
	Init();
}


/*

 ~ SBSCCommunicator:
 
 destructor, closes connection
 
*/
SBSCCommunicator::~SBSCCommunicator()
{
	closeConnection();
}


/*

 Init:
 
 initailizes member variables
 
*/
void SBSCCommunicator::Init()
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
int SBSCCommunicator::openHost(string target)
{
	if (commFD >= 0) return -1;  //already an open connection
	//separate the port and address parts of the target
	string::size_type pos = target.rfind(':', target.size()-1);
	uint16_t portnum = (pos!=string::npos)?atoi(target.substr(pos).c_str()):DEFAULT_CAM_PORT;
	string addrStr = target.substr(0,pos);
#if SBSC_COMM_DEBUG
	cerr << "[Comm debug]: creating server socket bound to host: " << addrStr 
		 << " on port: " << portnum << endl;
#endif	
	
	//create a sockaddr_in structure with this information
	sockaddr_in servaddr;
	int retval;
	if (addrStr == "any") {
		//use a dummy hostname ad then change the address afterwards
		retval = init_sockaddr(&servaddr, "127.0.0.1", portnum);
		servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	}
	else retval = init_sockaddr(&servaddr, addrStr.c_str(), portnum);
	if (retval < 0) return errorFlag = -1;  //something went wrong
	
	//now create a TCP socket, bind it to the target address, and listen on it
	serverFD = socket(PF_INET, SOCK_STREAM, 0);
	if (serverFD < 0) {
	  serverFD = -1;
	  return errorFlag = -1;
	}
	//set an option that will allow existing sockets to be remade (in case of crash)
	int optval = 1;
	setsockopt(serverFD, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
	if (bind(serverFD, (sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
		closeConnection();  //close server
		return errorFlag = -1;
	}
	if (listen(serverFD, 1) < 0) {
		closeConnection();
		return errorFlag = -1;
	}
	
	//wait for client connections and then accept (should block)
#if SBSC_COMM_DEBUG
	cerr << "[Comm debug]: waiting to accept a connection" << endl;
#endif
	sockaddr_in cliaddr;
	socklen_t clilen = sizeof(cliaddr);
	commFD = accept(serverFD, (sockaddr*)&cliaddr, &clilen);
	if (commFD < 0) {
		closeConnection();
		return errorFlag = -1;
	}
#if SBSC_COMM_DEBUG
	cerr << "[Comm debug]: client connection accepted from: " 
		 << inet_ntoa(cliaddr.sin_addr) << endl;
#endif
	this->target = target;
	return 0;
}

/*

 openClient:
 
 opens client communications
 for sockets, target should be of the form "addr:port"
 where addr is a hostname or address recognizable by gethostbyname
 and port is a port number (can be omitted, will use a default value)
 
*/
int SBSCCommunicator::openClient(string target)
{
	if (commFD >= 0) return -1;   //already an open connection
	
	//separate the port and address parts of the target
	string::size_type pos = target.rfind(':', target.size()-1);
	uint16_t portnum = (pos!=string::npos)?atoi(target.substr(pos).c_str()):DEFAULT_CAM_PORT;
	string addrStr = target.substr(0,pos);
#if SBSC_COMM_DEBUG
	cerr << "[Comm debug]: creating client socket connecting to host: " << addrStr 
		 << " on port: " << portnum << endl;
#endif	
	
	//create a sockaddr_in structure with this information
	sockaddr_in servaddr;
	if (init_sockaddr(&servaddr, addrStr.c_str(), portnum) < 0)
		return errorFlag = -1;
	//create the socket and try to connect
	commFD = socket(PF_INET, SOCK_STREAM, 0);
	if (commFD < 0) return errorFlag = -1;
	//set an option that will send keepalive messages (prevent timeout?)
	//I don't think this is necessary--timeout interval is quite long (~hours)
//	int optval = 1;
//	setsockopt(commFD, SOL_SOCKET, SO_KEEPALIVE, &optval, sizeof(optval));
	while ( true ) {
		if ( connect(commFD, (sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
			if (errno == ECONNREFUSED) {  //nobody is listening
#if SBSC_COMM_DEBUG
				cerr << "[Comm debug]: connection refused (nobody listening?) trying again in a bit" << endl;
#endif	
				usleep(CLIENT_RETRY_DELAY);
			} else {  //some other error
				closeConnection();  //close server
				return errorFlag = -1;
			}
		}
		else break;
	}
#if SBSC_COMM_DEBUG
	cerr << "[Comm debug]: connection successful" << endl;
#endif	
	this->target = target;
	return 0;
	
}


/*

 closeConnection:
 
 if an open connection exists, close it
 
*/
void SBSCCommunicator::closeConnection()
{
	if (serverFD >= 0) {
		close(serverFD);
		serverFD = -1;
	}
	if (commFD >= 0) {
		close(commFD);
		commFD = -1;
	}
	if (target != "") target = "";
}


/*

 repairLink:
 
 double checks that link is inactive (read doesn't block, but nothing available)
 if not, will simply return part of string found to caller
 if inactive, closes and tries to reopen the connection
 (can tell if host or not by whether the serverFD is valid)
 in this case will return the string ""
 
*/
string SBSCCommunicator::repairLink()
{
	fd_set test;
	timeval timeout;
	char buf[SBSC_COMM_BUF_SIZE];
	int n;
	FD_ZERO(&test);
	FD_SET(commFD, &test);
	timeout.tv_sec = 2;
	timeout.tv_usec = 0;
	n = select(commFD+1, &test, NULL, NULL, &timeout);
	if (n<0 || !FD_ISSET(commFD,&test)) //latter should never fail
		return "repairfail";  //something is wrong
	if (n==0) return "";  //timeout...link is probably active
	
	//else n>0
	if ((n = read(commFD, buf, SBSC_COMM_BUF_SIZE-1)) < 0) return "";
	if (n == 0) {  //link is probably dead, try to reconnect
#if SBSC_COMM_DEBUG
		cerr << "[Comm debug]: link appears dead, waiting for reconnect" << endl;
#endif
		if (serverFD == -2)     //control is done by external server, let connection die
		  return "repairfail";
		bool isHost = (serverFD >= 0);
		string oldTarget = target;
		closeConnection();
		if (isHost) { 
			if (openHost(oldTarget) < 0) return "repairfail";
		}
		else if (openClient(oldTarget) < 0) return "repairfail";
		else return "";
	}
	//otherwise received a string and link is still active
	buf[n] = '\0';
	return (string)buf;
}

/*

 readLoop:
 
 Infinite loop that reads and then interprets commands via interpretFunction
 Designed to run concurrently with pictureLoop and processingLoop (in starcam.cpp)
 If it returns at all then an error has occured
 The interpret function returns a string to be sent back to flight computer
 
*/
void SBSCCommunicator::readLoop(string (*interpretFunction)(string))
{
#if SBSC_COMM_DEBUG
	cerr << "[Comm debug]: in readLoop method" << endl;
#endif
	fd_set input;
	char buf[SBSC_COMM_BUF_SIZE];
	string line = "";
	string rtnStr;
	int n;
	string::size_type pos;
	if (commFD == -1) return;          //communications aren't open
	
	while (1) {
		FD_ZERO(&input);
		FD_SET(commFD, &input);
		if (select(commFD+1, &input, NULL, NULL, NULL) < 0) //won't time out
			return;
		if (!FD_ISSET(commFD, &input)) return;  //should always be false
		if ((n = read(commFD, buf, SBSC_COMM_BUF_SIZE-1)) < 0) return;
		if (n == 0) {  //link may be dead...check it out
#if SBSC_COMM_DEBUG
			cerr << "[Comm debug]: read empty string, checking if link is dead " << buf << endl;
#endif
			string temp = repairLink();
			if (temp == "repairfail") return; //something bad has happened
			else if (temp != "") line += temp;  //link wasn't dead
			//otherwise, link has been reestablished, continue
		}
		else { //n > 0
#if SBSC_COMM_DEBUG
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
string SBSCCommunicator::looplessRead()
{
	fd_set input;
	char buf[SBSC_COMM_BUF_SIZE];
	int n;
	if (commFD == -1) return "";          //communications aren't open
	
	FD_ZERO(&input);
	FD_SET(commFD, &input);
	if (select(commFD+1, &input, NULL, NULL, NULL) < 0) //won't time out
		return "";
	if (!FD_ISSET(commFD, &input)) return "";  //should always be false
	if ((n = read(commFD, buf, SBSC_COMM_BUF_SIZE-1)) < 0) return "";
	if (n == 0) {  //link may be dead...check it out
#if SBSC_COMM_DEBUG
		cerr << "[Comm debug]: read empty string, checking if link is dead " << buf << endl;
#endif
		string temp = repairLink();
		if (temp == "repairfail") return ""; //something bad has happened
		else if (temp != "") return temp;  //link wasn't dead
		else return "Bad link detected and repaired";
	}
	//otherwise all is well
#if SBSC_COMM_DEBUG
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
string SBSCCommunicator::buildReturn(const SBSCReturn* rtn)
{
	ostringstream sout;
	sout << rtn->frameNum << " " << rtn->mapmean << " " << rtn->sigma << " " << rtn->exposuretime << " " 
			<< rtn->imagestarttime.tv_sec << " " << rtn->imagestarttime.tv_usec 
			<< " " << rtn->camID << " " << rtn->ccdtemperature << " " << rtn->numblobs << " ";
	
	for (int i=0; i<rtn->numblobs; i++)
	{
		sout << rtn->flux[i] << " " << rtn->mean[i] << " " << rtn->snr[i] << " " 
				<< rtn->x[i] << " " << rtn->y[i] << " ";
	}
	
	string output = sout.str();
#if SBSC_COMM_DEBUG
	cerr << "[Comm debug]: built return of " << output.length() << " chars:\n" << output << endl;
#endif
	return output;
}

int SBSCCommunicator::sendReturn(const SBSCReturn* rtn)
{
#if SBSC_COMM_DEBUG
	cerr << "[Comm debug]: in sendReturn method" << endl;
#endif
	return sendCommand(buildReturn(rtn));
}

/*

 sendReturnString:
 
 sends a string return value from a command
 value is surrounded by <str></str> tags to identify it as a string
 
*/
string SBSCCommunicator::buildReturnString(string returnStr)
{
	return ((string)"<str>" + returnStr + "</str>");
}

int SBSCCommunicator::sendReturnString(string returnStr)
{
	return sendCommand(buildReturnString(returnStr));
}
	
/*

 interpretReturn:
 
 For use on "flight computer". Interprets data sent via sendReturn
 pass pointer to already declared struct to populate
 
*/
SBSCReturn* SBSCCommunicator::interpretReturn(string returnString, SBSCReturn* rtn)
{
#if SBSC_COMM_DEBUG
	cerr << "[Comm debug]: in interpretReturn method" << endl;
#endif
	istringstream sin;
	sin.str(returnString);
	
	sin >> rtn->frameNum >> rtn->mapmean >> rtn->sigma >> rtn->exposuretime >> rtn->imagestarttime.tv_sec
			>> rtn->imagestarttime.tv_usec >> rtn->camID >> rtn->ccdtemperature >> rtn->numblobs;
	
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
int SBSCCommunicator::sendCommand(string cmd)
{
#if SBSC_COMM_DEBUG
	cerr << "[Comm debug]: in sendCommand method with: " << cmd << endl;
#endif
	fd_set output;
	if (commFD == -1) return -1;          //communications aren't open

	//remove all newlines and add a single one at the end
	string sought = "\n";
	string::size_type pos = cmd.find(sought, 0);
	while (pos != string::npos) {
		cmd.replace(pos, sought.size(), "");
		pos = cmd.find(sought, pos - sought.size());
	}
	cmd += "\n";
	
	FD_ZERO(&output);
	FD_SET(commFD, &output);
	if (select(commFD+1, NULL, &output, NULL, NULL) < 0) //doesn't time out
		return -1;
	if (!FD_ISSET(commFD, &output)) return -1;  //should always be false
	return write(commFD, cmd.c_str(), cmd.length());
}

int SBSCCommunicator::sendCommand(const char* cmd)       //in case flight wants to use C only
{
	string str_cmd = cmd;
	return sendCommand(str_cmd);
}


