#ifndef DRIVECOMMUNICATOR_H
#define DRIVECOMMUNICATOR_H

/**
	@author Steve Benton

	class implementing serial communications with a Technosoft IDM240 motor drive
*/

#include <string>
#include "motorcommand.h"

using namespace std;


//enumeration of error status
typedef enum {
	DC_NO_ERROR, DC_UNKNOWN, DC_SERIAL_ERROR, 
	DC_SYNCH_ERROR, DC_BAD_RETURN
} DRIVE_ERROR;

class DriveCommunicator{
public:
    DriveCommunicator();
	DriveCommunicator(string deviceName);

    ~DriveCommunicator();
	
	//accessors
	MotorCommand* getLastCommand() { return &this->lastCommand; }
	MotorCommand* getReply() { return &this->returnVal; }
	string getDeviceName() { return this->serialDeviceName; }
	DRIVE_ERROR getError() { return this->derr; }
	bool isGoingForward() { return this->dirForward; }
	bool isOpen() { return portFD > 0; }
	
	
	void openConnection(string deviceName, bool highspeed=false);  //same as above except when highspeed==true use higher speed
	void closeConnection();                    //closes connection, turn of drive power stage if possible
	void synchronize();                        //send synchronization character and check reply
	//maxCommSpeed now done in controller startup
	void maxCommSpeed(unsigned short dest);    //changes from default 9600 baud to 115200
	void sendCommand(MotorCommand *cmd);       //send the command cmd
	MotorCommand* sendQuery(unsigned short dest, unsigned short dataAddr, bool has32bits); //queries for 16 or 32 bit data from an address
	MotorCommand* sendQuery(unsigned short dest, unsigned short dataAddr, unsigned short senderID, bool has32bits);
	
	//more specific commmand-related methods
	static unsigned int double2Fixed(double floatval);
	static double fixed2Double(unsigned int intval);
	void sendSpeedCommand(unsigned short dest, double speed);
	double getDriveTemp(unsigned short dest, unsigned short senderID);     //queries the drive temperature
	
private:
	MotorCommand lastCommand;            //most recently executed commmand
	MotorCommand returnVal;              //for queries
	bool dirForward;                     //true when drive is set to move in "forward" direction
	int portFD;                          //file descriptor for serial port communications
	string serialDeviceName;             //name of file in /dev/ folder corresponding to serial device
	DRIVE_ERROR derr;                     //error status
	bool highspeed;                      //are communications set to high speed

};

#endif
