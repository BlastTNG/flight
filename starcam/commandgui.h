#ifndef COMMANDGUI_H
#define COMMANDGUI_H

#include <qwidget.h>
#include <qthread.h>
#include "camcommunicator.h"

/**
	@author Steve Benton <steve.benton@utoronto.ca>
	GUI interface for starcam commands
	when commands are added to starcam.cpp::interpretCommand, they sould be added here too
*/

class QLabel;
class QListBox;
class QLineEdit;
class QTextEdit;
//class QString;
class CommandGUI;  //advance declaration for use in ReadThread

class ReadThread : public QThread
{
	public:
		ReadThread() { commander = NULL; }
		void setCommander(CommandGUI *cmdr) { commander = cmdr; }
		virtual void run();
	private:
		CommandGUI *commander;
};

class CommandGUI : public QWidget
{
	Q_OBJECT
public:
	CommandGUI(QWidget* parent=0, const char *name = 0, const char* commTarget="aragog.spider");
	~CommandGUI();
	
	void showReturnVal(QString&);
	
	enum Devices { Camera = 0, Image, Lens, Overall };
	enum CameraCmd { TrigExp = 0, TrigFocus, TrigFocusF, SetExpTime, SetExpInt, SetFocRsln, Power };
	enum ImageCmd { SetBadpix = 0, SetMaxBlobs, SetGrid, SetThreshold, SetDisttol};
	enum LensCmd { Move = 0, Force, SetTol, Other };
	enum OverCmd { Conf = 0, showBox };
	
	friend class ReadThread;
	
public slots:
	void deviceSelected(int);
	void commandSelected(int);
	void valueChanged(const QString&);
	void sendCommand();
	
private:
	QLabel *command;        //displays the current command
	QLabel *description;    //displays a description of the current command
	
	QLabel *valDesc;        //description for extra value
	QLineEdit *value;       //for reading extra values needed for command
	QLabel *valLabel;       //displays "Value", needs to hide/shows
	
	QListBox *deviceBox;    //box for selecting the device (cam, img, lens) for the command
	QListBox *camCmds;      //list of camera commands
	QListBox *imgCmds;      //list of image commnads
	QListBox *lensCmds;     //list of commands for the lens
	QListBox *overCmds;     //list of overall/general commands
	QListBox *currentCmds;  //this will be set to whichever of the above boxes is being used

	QTextEdit *returnPane;  //return values are added here
	
	int deviceIndex, commandIndex; //indexes for current selection amongst list boxes
	
	CamCommunicator comm;
	bool keepReading;        //loop condition for read loop
	ReadThread reader;       //loops on reading responses from the flight computer
};


#endif
