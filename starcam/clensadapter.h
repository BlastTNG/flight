#ifndef CLENSADAPTER_H
#define CLENSADAPTER_H

#include <string>
#include "camconfig.h"
#include "clensadapterdefs.h"

using namespace std;

/**
	@author Steve Benton <steve.benton@utoronto.ca>
*/

class CLensAdapter{
private:
	LENS_COMMAND m_eLastCommand;        //most recently executed command
	LENS_ERROR m_eLastError;            //most recent error to occur
	int m_nPortFD;                      //file descriptor for serial port connection
	string m_sSerialDeviceName;         //name of file in /dev/ folder corresponding to port
	int m_nFocalRange;                  //number of motor counts along entire focal range
	unsigned int m_nFocusTol;           //precision (in counts) for a precise move
	
public:
    CLensAdapter();
	CLensAdapter(string deviceName);
    ~CLensAdapter();
	
	void Init(LensAdapterConfigParams params=defaultCameraParams.lensParams);
	LENS_ERROR openConnection(string deviceName);
	LENS_ERROR closeConnection();
	
	LENS_COMMAND getLastCommand() { return m_eLastCommand; }
	LENS_ERROR getLastError() { return m_eLastError; }
	string getDeviceName() { return m_sSerialDeviceName; }
	static string getErrorString(LENS_ERROR err);
	LENS_ERROR findFocalRange();
	int getFocalRange() {return m_nFocalRange; }
	void setFocusTol(unsigned int tol) { m_nFocusTol = tol; }
	unsigned int getFocusTol() { return m_nFocusTol; }
	
	LENS_ERROR runCommand(LENS_COMMAND cmd, string &return_value);
	LENS_ERROR runCommand(LENS_COMMAND cmd, int val, string &return_value);
	LENS_ERROR runCommand(string cmd, string &return_value);
	
	LENS_ERROR preciseMove(int counts, int &remaining, int force = 0);
};

#endif
