//test program to communicate as the "flight computer" would (can use GUI or terminal)

//these are reversed because the files are named from starcam perspective
const char* commTarget = "any";

#define USE_COMMAND_GUI 1

#if USE_COMMAND_GUI
#include <qapplication.h>
#include "commandgui.h"

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	CommandGUI gui(0, "gui", commTarget);
	a.setMainWidget(&gui);
	gui.setGeometry(100, 100, 600, 400);
	gui.show();
	return a.exec();
}

#endif

#if USE_COMMAND_GUI == 0
#include <iostream>
#include <string>
//#include <unistd.h>
//#include <fcntl.h>
#include <pthread.h>
#include "camcommunicator.h"
#include "camconfig.h"


//function prototypes
void* readLoop(void* arg);
string displayReturn(string rtn);

int main(int argc, char *argv[])
		
{
	char buf[256];
	//open communication socket as the host
	cout << "Opening communications with target: " << commTarget << " (may block)" << endl;
	CamCommunicator comm;
	if (comm.openHost(commTarget) < 0) 
		cout << "An error occured while opening the communications." << endl;
		
	//start thread that constantly waits to read a return value
	pthread_t thread;
	pthread_create(&thread, NULL, &readLoop, (void*)&comm);
	
	//in this thread, constantly read command strings from keyboard and send to camera
	while (1) {
		cout << "Enter a string to send to the star camera (Ctrl-C quits): ";
		cin.getline(buf, 256);
		cout << "Sending command: " << buf << endl;
		if (comm.sendCommand(buf) < 0) break;
	}
	cout << "Loop has quit unexpectedly...sending of command must have failed." << endl;
			
	//wait for read thread to end (eg. never)
	pthread_join(thread, NULL);
	
	return 0;
}

/*

 readLoop:
 
 simply a wrapper function for calling the CamCommunicator::readLoop member in a pthread
 
*/
void* readLoop(void* arg)
{
	CamCommunicator* comm = (CamCommunicator*)arg;
	comm->readLoop(&displayReturn);
	return NULL;
}

/*

 displayReturn:
 
 interprets and displays the return string obtained from starcam
 
*/
string displayReturn(string rtn_str)
{
	if (rtn_str.substr(0,5) == "<str>") {   //return is a string
		rtn_str = rtn_str.substr(5, rtn_str.length() - 11);
		cout << "Command returned string:\n   " << rtn_str << endl;
	} else {        //otherwise it is a return struct
		StarcamReturn rtn;
		CamCommunicator::interpretReturn(rtn_str, &rtn);
		cout << "\n\nObtained a picture return signal from camera #" << rtn.cameraID << endl
		 	<< "... mapmean =" << rtn.mapmean << endl
		 	<< "... sigma=" << rtn.sigma << endl
		 	<< "... exposuretime=" << rtn.exposuretime << endl
		 	<< "... imagestarttime=" << rtn.imagestarttime.tv_sec << "s " 
			<< rtn.imagestarttime.tv_usec << "us" << endl
			 	<< "... ccdtemperature=" << rtn.ccdtemperature << endl
		 	<< "... numblobs=" << rtn.numblobs << endl;
	}
	
	//reprompt
	cout << "\nEnter a string to send to the star camera (Ctrl-C quits): ";

	
	return "";
}

#endif         //if USE_COMMAND_GUI == 0
