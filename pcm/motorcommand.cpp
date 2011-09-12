#include "motorcommand.h"
#include <iostream>
#include <iomanip>
using namespace std;

/*
MotorCommand:
 default constructor. only perform zeroing of important data
*/
MotorCommand::MotorCommand()
{
	Init();
}

/*

MotorCommand:
 constructor based off of parsing a command in array-of-bytes form
 user must be very careful since a malformed length byte can probably cause segfaults
 clone the array into data
*/
MotorCommand::MotorCommand(unsigned char* cmd)
{
	Init();
	setCommand(cmd);
}

/*
MotorCommand:
 constructor that only specifies opcode (the default destination is suitable for single drive on default settings)
*/
MotorCommand::MotorCommand(unsigned short opcode)
{
	Init();
	this->opcode = opcode;
}

/*
MotorCommand:
 constructor specifying destination and opcode
*/
MotorCommand::MotorCommand(unsigned short destid, unsigned short opcode)
{
	Init();
	this->destid = destid;
	this->opcode = opcode;
}

/*
MotorCommand:
 constructor specifying opcode and data and will use defualt destination
*/
MotorCommand::MotorCommand(unsigned short opcode, unsigned short* data, unsigned short dataLength)
{
	Init();
	this->opcode = opcode;
	if (dataLength > 4) return;
	this->dataLength = dataLength;
	for (int i=0; i<this->dataLength; i++) this->data[i] = data[i];
}

/*
MotorCommand:
 constructor specifying all fields needed to build a general command
*/
MotorCommand::MotorCommand(unsigned short destid, unsigned short opcode, unsigned short* data, unsigned short dataLength)
{
	Init();
	this->destid = destid;
	this->opcode = opcode;
	if (dataLength > 4) return;
	this->dataLength = dataLength;
	for (int i=0; i<this->dataLength; i++) this->data[i] = data[i];
}

/*
Init:
 perform default initialization of members (mostly just zero them)
*/
void MotorCommand::Init()
{
	this->cmdLength = 0;
	this->length = 0;
	this->destid = 0x0FF0;   //default destid suitable for single drive systems on default settings
	this->opcode= 0;
	this->dataLength = 0;
	this->checksum = 0;
	this->validFlag = false;
}
	
/*
~MotorCommand:
 destructor. no dynamic elements to unallocate
*/
MotorCommand::~MotorCommand()
{
}

/*
setCommand:
 sets the command with raw binary which will be parsed
*/
void MotorCommand::setCommand(unsigned char *cmd)
{
	if (cmd == NULL) return;
	this->validFlag = false;        //consider invalid until confirmed to be good
	this->length = cmd[0];
	this->cmdLength = this->length+2;         //THIS IS A BIG ASSUMPTION!!!! can get segfault if user isn't careful
	for (int i=0; i<this->cmdLength; i++) this->cmd[i] = cmd[i];
	if (this->length < 4 || this->length > 12 || this->length%2 == 1) return;   //length must be even and between 4 and 12 (inclusive)
	this->destid = this->cmd[1]*256 + this->cmd[2];
	this->opcode = this->cmd[3]*256 + this->cmd[4];
	this->dataLength = (this->length-4)/2;
	for (int i=0; i<this->dataLength; i++) {
		data[i] = this->cmd[2*i+5]*256 + this->cmd[2*i+6];
	}
	this->checksum = this->cmd[this->length+1];
	long sum = 0;
	for (int i=0; i<=this->length; i++) sum += this->cmd[i];
	if (this->checksum == sum%256) this->validFlag = true; //otherwise it will remain false (set in Init)
}


/*
buildCommand:
 builds a byte string command to send. requires a valid destination (possibly default), opcode and data
*/
void MotorCommand::buildCommand()
{
	this->length = 4 + this->dataLength*2;
	this->cmdLength = this->length + 2;
	this->cmd[0] = (unsigned char)this->length;
	this->cmd[1] = (unsigned char)(this->destid/256);
	this->cmd[2] = (unsigned char)(this->destid%256);
	this->cmd[3] = (unsigned char)(this->opcode/256);
	this->cmd[4] = (unsigned char)(this->opcode%256);
	for (int i=0; i<this->dataLength; i++) {
		this->cmd[2*i+5] = (unsigned char)(this->data[i]/256);
		this->cmd[2*i+6] = (unsigned char)(this->data[i]%256);
	}
	long sum = 0;
	for (int i=0; i<=this->length; i++) sum += cmd[i];
	this->checksum = (unsigned char)(sum%256);
	this->cmd[this->length+1] = (unsigned char)this->checksum;
	this->validFlag = checkCommand();
}

/*
checkCommand:
 ensures length byte is possible and that checksum adds up right
 must either have called parsing constructor or buildCommand to possibly be correct
*/
bool MotorCommand::checkCommand()
{
	if (this->cmd[0] < 4 || this->cmd[0] > 12 || this->cmd[0]%2 == 1) return false;  //length must be even numbered between 4 and 12
	long sum = 0;
	for (int i=0; i<=this->cmd[0]; i++) sum += this->cmd[i];
	if (cmd[this->cmd[0]+1] != sum%256) return false;   //ensure checksum byte is correct
	return true;
}
	
/*
addData:
 adds one word of data to data array provided there is room for it
*/
void MotorCommand::addData(unsigned short newdata)
{
	if (this->dataLength >= 4) return;                 //can't add more than 4 words of data
	this->data[this->dataLength] = newdata;
	this->dataLength++;
}

void MotorCommand::printData(ostream &out)
{
// 	out << "Command is " << ((this->validFlag)?"valid":"invalid") << endl;
// 	out << "Length: " << (int)this->length << " Destination: " << this->destid << " OpCode: " << opcode << endl << "Data: ";
// 	for (int i=0; i<this->dataLength; i++) out << this->data[i] << " ";
// 	out << endl << "Raw command: ";
	out << hex << setfill('0');
	for (int i=0; i<this->cmdLength; i++) out << setw(2) << (int)this->cmd[i];
	out << dec << setfill(' ');
}
