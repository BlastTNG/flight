#ifndef MOTORCOMMAND_H
#define MOTORCOMMAND_H

/**
	@author Steve Benton
	Simple class for wrapping commands to technosoft IDM240 motor drive
*/

#include <iostream>

class MotorCommand{
public:
	void Init();                //default initialization common to all constructors
	MotorCommand();
	MotorCommand(unsigned char* cmd);
	MotorCommand(unsigned short opcode);                           //use default destid
	MotorCommand(unsigned short destid, unsigned short opcode);
	MotorCommand(unsigned short opcode, unsigned short* data, unsigned short dataLength);     //use default destid
	MotorCommand(unsigned short destid, unsigned short opcode, unsigned short* data, unsigned short dataLength);
	
    ~MotorCommand();
	
	//methods for converting between raw command and separated parts
	void buildCommand();      //creates cmd out of other data
	bool checkCommand();      //ensure that length and checksum bytes of cmd fit
	
	//accessors
	unsigned char* getCommand() { return cmd; }
	unsigned short getCommandLength() { return cmdLength; }
	void setLength(unsigned short length) { this->length = length; }
	void setDestination(unsigned short destid) { this->destid = destid; }
	void setOpCode(unsigned short opcode) { this->opcode = opcode; }
	void addData(unsigned short newdata);      //add one word of data
	unsigned short getData(int index) { return this->data[index]; }
	void setCommand(unsigned char* cmd);
	bool isValid() { return this->validFlag; }
	
	//debugging function prints data (as in all members, not just data member) to given outputstream
	void printData(std::ostream &out);
	
private:
	unsigned char cmd[14];      //the binary command represented as an array of bytes
	unsigned short cmdLength;   //current length of the command array
	unsigned short length;      //the length of the command (number of bytes not counting length of checksum bytes)
	unsigned short destid;      //axis/group id word of the destination
	unsigned short opcode;      //the code for the desired operation
	unsigned short data[4];     //command data consisting of up to 4 16bit words
	unsigned short dataLength;  //current length of the data array
	unsigned char checksum;     //a simple checksum for verifying integrity
	bool validFlag;             //flag indicating whether or not the command is known to be valid
};

#endif
