//test program to communicate as the "flight computer" would (can use GUI or terminal)

#include <iostream>
#include <string>
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
}

//these are reversed because the files are named from starcam perspective
const char* defaultCommTarget = "192.168.1.11"; //THEGOOD
//const char* defaultCommTarget = "192.168.1.12"; //THEBAD
//const char* defaultCommTarget = "192.168.1.13"; //THEUGLY

#define SC_TIMEOUT 100 /* milliseconds */
#define USE_COMMAND_GUI 0

#if USE_COMMAND_GUI
#include <qapplication.h>
#include "commandgui.h"

int main(int argc, char *argv[])
{
	const char* commTarget;
	if (argc == 1) {
	  cout << "Connecting to default host: " << defaultCommTarget << endl;
	  commTarget = defaultCommTarget;
	}
	else if (argc == 2) {
	  commTarget = argv[1];
	  cout << "Connecting to host: " << commTarget << endl;
	}
	QApplication a(argc, argv);
	CommandGUI gui(0, "gui", commTarget);
	a.setMainWidget(&gui);
	gui.setGeometry(100, 100, 600, 400);
	gui.show();
	return a.exec();
}

#endif

#if USE_COMMAND_GUI == 0
//#include <unistd.h>
//#include <fcntl.h>
#include <pthread.h>
#include "camcommunicator.h"
#include "camconfig.h"

static string ParseReturn(string rtnStr);
static void* readLoop(void* arg);
static void* sendLoop(void* arg);
const char* SCcmd = "Oconf";;
int SCcmd_flag = 1;

int main(int argc, char *argv[])
		
{
  pthread_t readthread;
  pthread_t sendthread;
  pthread_create(&readthread, NULL, &readLoop, NULL);
  pthread_create(&sendthread, NULL, &sendLoop, NULL);

  pthread_join(readthread, NULL);
  pthread_join(sendthread, NULL);

  return 0;
}

void* sendLoop(void* arg)
{
  int sock;
  string::size_type pos;
  string sought = "\n";

	/* create a socket */
  sock = udp_bind_port(SC_PORT_GOOD, 1);

  if (sock == -1)
    cout << "Unable to bind to port" << endl;

	while (1) {
    while (SCcmd_flag == 0) {
      sleep(1);
    }
    string cmd = string(SCcmd);
    //remove all newlines and add a single one at the end
    pos = cmd.find(sought, 0);
    while (pos != string::npos) {
      cmd.replace(pos, sought.size(), "");
      pos = cmd.find(sought, pos - sought.size());
    }
    cmd += "\n";
    if (udp_bcast(sock, GOOD_PORT, strlen(cmd.c_str()), cmd.c_str(), 0))
      cout << "error broadcasting command" << endl;
//    cout << "sent" << endl;
    SCcmd_flag = 0;
  }
  return NULL;
}

void* readLoop(void* arg)
{
  int sock, port;
  char peer[UDP_MAXHOST];
  char buf[UDP_MAXSIZE];
  string rtn_str = "";
  string line = "";
  string sought = "\n";
  string::size_type pos;
  int recvlen;

  /* create a socket */
  sock = udp_bind_port(SC_PORT, 1);

  if (sock == -1)
    cout << "Unable to bind to port" << endl;

	while (1) {
    rtn_str = "";
		recvlen = udp_recv(sock, SC_TIMEOUT, peer, &port, UDP_MAXSIZE, buf);
//    if (recvlen >= 0) {
//      buf[recvlen] = 0;	/* expect a printable string - terminate it */
//      printf("received message: \"%s\"\n", buf);
//    }
    buf[recvlen] = '\0';
    line += buf;
    while ((pos = line.find("\n",0)) != string::npos) {
      rtn_str = ParseReturn(line.substr(0,pos));
      line = line.substr(pos+1, line.length()-(pos+1));
    }
  }
	return NULL;
}

static string ParseReturn(string rtnStr)
{
  char buf[UDP_MAXSIZE];
  int expInt, expTime;
  int focusRes, moveTol;
  int maxBlobs, grid, threshold, minBlobDist;
    if (rtnStr.substr(0,5) == "<str>") { //response is string
      cout << "Command returned string:   " << rtnStr << endl;
		string Rstr = rtnStr.substr(5, rtnStr.size() - 11);
		if (Rstr[0] == 'E') //it is an errori
        cout << "ERROR: " << Rstr.substr(6, Rstr.size()-6).c_str() << endl;
		else if (Rstr.substr(0,6) == "<conf>") //contains config data
    { 
      cout << "config data" << endl;
			Rstr = Rstr.substr(6, Rstr.size()-6);
        istringstream sin;
        sin.str(Rstr);
        double temp; //value sent for expTime is a double
        sin >> expInt >> temp >> focusRes >> moveTol >> maxBlobs >> grid >> threshold >> minBlobDist;
        expTime = (int)(temp * 1000);
        cout << "--- Exposure Interval = " << expInt << endl
          << "--- Exposure Time = " << expTime << endl
          << "--- Focus Resolution = " << focusRes << endl
          << "--- Move tolerance = " << moveTol << endl
          << "--- Max blobs = " << maxBlobs << endl
          << "--- Grid = " << grid << endl
          << "--- Threshold = " << threshold << endl
          << "--- Mib blob dist = " << minBlobDist << endl;
      }
    } else {        //otherwise it is a return struct
      StarcamReturn rtn;
      CamCommunicator::interpretReturn(rtnStr, &rtn);
      cout << "\n\nObtained a picture return signal from camera #" << rtn.camID << endl
		 	<< "... mapmean =" << rtn.mapmean << endl
		 	<< "... sigma=" << rtn.sigma << endl
		 	<< "... exposuretime=" << rtn.exposuretime << endl
		 	<< "... imagestarttime=" << rtn.imagestarttime.tv_sec << "s " 
			<< rtn.imagestarttime.tv_usec << "us" << endl
			 	<< "... ccdtemperature=" << rtn.ccdtemperature << endl
			 	<< "... focusposition=" << rtn.focusposition << endl
		 	<< "... numblobs=" << rtn.numblobs << endl;
    }
     //reprompt
		 cout << "\nEnter a string to send to the star camera (Ctrl-C quits): ";
		 cin.getline(buf, 256);
		 cout << "Sending command: " << buf << endl;
     SCcmd_flag = 1;
     SCcmd = buf;
    return ""; //doesn't send a response back to camera  
}

#endif         //if USE_COMMAND_GUI == 0
