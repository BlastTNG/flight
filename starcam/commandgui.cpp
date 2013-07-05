#include "commandgui.h"
#include <string>
#include <sstream>
#include <qlabel.h>
#include <qlistbox.h>
#include <qlineedit.h>
#include <qtextedit.h>
#include <qpushbutton.h>
#include <qlayout.h>
#include "camcommunicator.h"
#include "camconfig.h"

#define COMMAND_DEBUG 0
#if COMMAND_DEBUG
#include <iostream>
using namespace std;
#endif

//definitions of string arrays for use in the GUI
//to add a command, it must appear in interpretCommand in starcam.cpp
//to use it in this GUI, add it to the three lists below, and to the enum in header
//(NB: it must appear in the same location in all lists)
//Finally, the list entry must be added in the constructor
//Yes, I know this is a lot to ask, this system is not perfectly designed

//command strings as recognized by the star camera
const QString CameraCmdStrs[] = { "CtrigExp", "CtrigFocus", "CtrigFocusF", "CsetExpTime",
	"CsetExpInt", "CsetFocRsln", "Cpower" };
const QString ImageCmdStrs[] = { "IsetBadpix", "IsetMaxBlobs", "IsetGrid",
	"IsetThreshold", "IsetDisttol" };
const QString LensCmdStrs[] = { "Lmove", "Lforce", "LsetTol", "L" };
const QString OverCmdStrs[] = { "Oconf", "OshowBox" };
	
//descriptions of star camera commands
const QString CameraCmdDescs[] = { "Triggers camera exposure (if in triggered mode)",
	"Triggers camera autofocus (takes a couple of minutes)",
	"Triggers camera autofocus; forced moves (takes a couple of minutes)",
	"Sets duration of exposures", "Sets interval between exposures",
	"Sets the resolution (step size) for autofocus",
	"Power cycles the star cameras"
};
const QString ImageCmdDescs[] = { "Identifies a bad pixel on CCD, will be set to map mean",
	"Sets maximum number of blobs to identify per image",
	"Sets the grid size for blob finding",
	"Sets the #-sigma threshold for a blob to be considered a star",
	"Sets the closest two stars can be together to be identified as distinct"
};
const QString LensCmdDescs[] = { "Makes a precise move of the lens (proportional feedback)",
	"Similar to move, except ignores lens stops (which may be false when cold)",
	"Sets allowable miss tolerance of precise moves",
	"Execute an arbitrary command recognized by the lens adapter"
};
const QString OverCmdDescs[] = { "Sends a request for configuration state data",
	"Toggle whether boxes are drawn in the image viewer images"
};

//descriptions of the value needed for the command, "" for no value
const QString CameraValDescs[] = { "", "", "", "(double) Exposure duration in ms (default: 100)",
	"(int) interval between exposures in ms, 0 indicates triggered-mode (default: 0)",
	"(int) number of steps in total focal range during autofocus (default: 100)",
	""
};
const QString ImageValDescs[] = { 
	"(space-separated ints) <cam #> <x> <y> (\"0 0\" is top left)",
	"(int) maximum number of blobs (default: 99)",
	"(int) size of grid square in pixels (default: 20)",
	"(double) number of standard deviation above mean flux (default: 5.0)",
	"(int) square of minimum pixel distance between two blobs (default: 400)"
};
const QString LensValDescs[] = { "(int) motor counts to move by (total range ~2100)",
	"(int) motor counts to move by (total range ~2100)",
	"(int) motor counts away from ideal location allowed (default: 1)",
	"(string) any command recognized by the lens adapter"
};
const QString OverValDescs[] = { "",
	"(int/bool) evaluates to 'true' means boxes are on"
};



CommandGUI::CommandGUI(QWidget *parent, const char *name, const char* commTarget /*="aragog.spider"*/)
 : QWidget(parent, name)
{
	command = new QLabel(CameraCmdStrs[TrigExp], this, "command");
	command->setFont(QFont("Arial", 10, QFont::Bold));
	description = new QLabel(CameraCmdDescs[TrigExp], this, "description");
	description->setFont(QFont("Arial", 10, QFont::Bold));
	
	valDesc = new QLabel("", this, "description");
	valDesc->setFont(QFont("Arial", 10, QFont::Bold));
	valDesc->setAlignment(Qt::AlignTop | Qt::AlignLeft | Qt::WordBreak);
	valDesc->setMinimumSize(150,50);	
	value = new QLineEdit(this, "value");
	value->hide();
	value->setMinimumWidth(140);
	connect(value, SIGNAL(textChanged(const QString&)), this, SLOT(valueChanged(const QString&)));
	valLabel = new QLabel("Value:", this, "vallabel");
	valLabel->setFont(QFont("Arial", 12, QFont::Bold));
	valLabel->setMinimumSize(valLabel->sizeHint());
	valLabel->hide();

	deviceBox = new QListBox(this, "deviceBox");
	deviceBox->insertItem("Camera Commands", Camera);
	deviceBox->insertItem("Image Commands", Image);
	deviceBox->insertItem("Lens Commands", Lens);
	deviceBox->insertItem("Overall Commands", Overall);
	deviceBox->setSelected(Camera, TRUE);
	deviceBox->setMinimumSize(150,100);
	connect(deviceBox, SIGNAL(highlighted(int)), this, SLOT(deviceSelected(int)));
	
	camCmds = new QListBox(this, "camCmds");
	camCmds->insertItem(CameraCmdStrs[TrigExp], TrigExp);
	camCmds->insertItem(CameraCmdStrs[TrigFocus], TrigFocus);
	camCmds->insertItem(CameraCmdStrs[TrigFocusF], TrigFocusF);
	camCmds->insertItem(CameraCmdStrs[SetExpTime], SetExpTime);
	camCmds->insertItem(CameraCmdStrs[SetExpInt], SetExpInt);
	camCmds->insertItem(CameraCmdStrs[SetFocRsln], SetFocRsln);
	camCmds->insertItem(CameraCmdStrs[Power], Power);
	camCmds->setSelected(TrigExp, TRUE);
	camCmds->setMinimumSize(150,100);
	connect(camCmds, SIGNAL(highlighted(int)), this, SLOT(commandSelected(int)));

	imgCmds = new QListBox(this, "imgCmds");
	imgCmds->insertItem(ImageCmdStrs[SetBadpix], SetBadpix);
	imgCmds->insertItem(ImageCmdStrs[SetMaxBlobs], SetMaxBlobs);
	imgCmds->insertItem(ImageCmdStrs[SetGrid], SetGrid);
	imgCmds->insertItem(ImageCmdStrs[SetThreshold], SetThreshold);
	imgCmds->insertItem(ImageCmdStrs[SetDisttol], SetDisttol);
	imgCmds->setSelected(SetBadpix, TRUE);
	imgCmds->setMinimumSize(150,100);
	connect(imgCmds, SIGNAL(highlighted(int)), this, SLOT(commandSelected(int)));
	
	lensCmds = new QListBox(this, "lensCmds");
	lensCmds->insertItem(LensCmdStrs[Move], Move);
	lensCmds->insertItem(LensCmdStrs[Force], Force);
	lensCmds->insertItem(LensCmdStrs[SetTol], SetTol);
	lensCmds->insertItem(LensCmdStrs[Other], Other);
	lensCmds->setSelected(Move, TRUE);
	lensCmds->setMinimumSize(150,100);
	connect(lensCmds, SIGNAL(highlighted(int)), this, SLOT(commandSelected(int)));
	
	overCmds = new QListBox(this, "overCmds");
	overCmds->insertItem(OverCmdStrs[Conf], Conf);
	overCmds->insertItem(OverCmdStrs[showBox], showBox);
	overCmds->setSelected(Move, TRUE);
	overCmds->setMinimumSize(150,100);
	connect(overCmds, SIGNAL(highlighted(int)), this, SLOT(commandSelected(int)));
	
	returnPane = new QTextEdit(this, "returnPane");
	returnPane->setTextFormat(LogText);
	returnPane->setMinimumHeight(150);
	
	QVBoxLayout *vl = new QVBoxLayout(this, 0, 1, "vlayout");
	
	QHBoxLayout *hl = new QHBoxLayout(vl, 2, "choicelayout");
	hl->addWidget(deviceBox, 1);	
	QGridLayout *gl = new QGridLayout(hl, 1, 1, -1, "singlegrid");
	gl->addWidget(camCmds, 0, 0);
	gl->addWidget(imgCmds, 0, 0);
	gl->addWidget(lensCmds, 0, 0);
	gl->addWidget(overCmds, 0, 0);
	camCmds->raise();                  //show camera commands by default
	camCmds->setFocus();
	deviceIndex = (int)Camera;
	currentCmds = camCmds;
	commandIndex = (int)TrigExp;
	hl->setStretchFactor(gl, 1);
	QVBoxLayout *vl2 = new QVBoxLayout(hl, 2, "vallayout");
	vl2->addWidget(valLabel,0);
	vl2->addWidget(value,0);
	vl2->addWidget(valDesc,0);
	vl2->addStretch(1);
	hl->setStretchFactor(vl2, 10);
	vl->setStretchFactor(hl, 1);
	
	hl = new QHBoxLayout(vl, 2, "cmdlayout");
	QLabel *label = new QLabel("Command:", this, "cmdlabel");
	label->setFont(QFont("Arial", 12, QFont::Bold));
	hl->addWidget(label, 0);
	hl->addWidget(command, 1, Qt::AlignLeft | Qt::AlignVCenter);
	QPushButton *button = new QPushButton("Send", this, "sendbutton");
	connect(button, SIGNAL(pressed()), this, SLOT(sendCommand()));
	hl->addWidget(button, 0);
	vl->setStretchFactor(hl, 0);
	
	vl->addWidget(description, 0);
	
	vl->addWidget(returnPane, 2);
	
	if (comm.openClient(commTarget) < 0) {
#if COMMAND_DEBUG
		cerr << "[CommandGUI debug]: error opening communications client" << endl;
#endif
	}
	keepReading = TRUE;
	reader.setCommander(this);
	reader.start();
	
}

void CommandGUI::deviceSelected(int device)
{
#if COMMAND_DEBUG
	cerr << "[CommandGUI debug]: selected device number: " << device << endl;
#endif
	if (device == deviceIndex) return;       //only do stuff on change
	deviceIndex = device;
	currentCmds = (device == Camera)?camCmds:(device == Image)?imgCmds:
	  (device == Lens)?lensCmds:overCmds;
	currentCmds->raise();
	currentCmds->setFocus();
	int idx = currentCmds->index(currentCmds->selectedItem());
	if (idx == -1) return;   //this not expected to occur
	commandIndex = -1;   //ensures that changes are made in commandSelected call	
	commandSelected(idx);
}

void CommandGUI::commandSelected(int cmd)
{
#if COMMAND_DEBUG
	cerr << "[CommandGUI debug]: selected command number: " << cmd << endl;
#endif
	if (cmd == commandIndex) return;
	commandIndex = cmd;
	command->setText(currentCmds->selectedItem()->text());
	const QString* cmdDescArray = (deviceIndex == Camera)?CameraCmdDescs:
			(deviceIndex == Image)?ImageCmdDescs:
			(deviceIndex == Lens)?LensCmdDescs:OverCmdDescs;
	description->setText(cmdDescArray[cmd]);
	const QString* valDescArray = (deviceIndex == Camera)?CameraValDescs:
			(deviceIndex == Image)?ImageValDescs:
			(deviceIndex == Lens)?LensValDescs:OverValDescs;
	if (valDescArray[cmd][0] == '\0') {  //val with no description means no value
		valDesc->hide();
		value->hide();
		valLabel->hide();
	} else {
		valDesc->show();
		valDesc->setText(valDescArray[cmd]);
		value->show();
		value->setText("");
		valLabel->show();
	}
}

void CommandGUI::valueChanged(const QString&)
{
#if COMMAND_DEBUG
	cerr << "[CommandGUI debug]: value changed" << endl;
#endif
	QString str = currentCmds->selectedItem()->text();
	str += "=";
	str += value->text();
	command->setText(str);
}

void CommandGUI::sendCommand()
{
#if COMMAND_DEBUG
	cerr << "[CommandGUI debug]: sending a command" << endl;
#endif
	QString str = "Sending the command: ";
	str += command->text();
	returnPane->append(str);
	
	comm.sendCommand(command->text().ascii());
}

//appends a return value to returnPane, str should be exactly as read
void CommandGUI::showReturnVal(QString &str)
{
	if (str.find("<str>") == 0) {
		str = str.remove("<str>");
		str = str.remove("</str>");
		returnPane->append("Obtained return string: " + str);
	} else {
		StarcamReturn rtn;
		CamCommunicator::interpretReturn((string)str.ascii(), &rtn);
		ostringstream sout;
		sout << "\n\nObtained a picture return signal from camera #" << rtn.camID << "\n"
			 << "... mapmean =" << rtn.mapmean << "\n"
			 << "... sigma=" << rtn.sigma << "\n"
			 << "... exposuretime=" << rtn.exposuretime << "\n"
			 << "... imagestarttime=" << rtn.imagestarttime.tv_sec << "s " 
				 << rtn.imagestarttime.tv_usec << "us\n"
			 << "... ccdtemperature=" << rtn.ccdtemperature << "\n"
			 << "... focusposition=" << rtn.focusposition << "\n"
			 << "... numblobs=" << rtn.numblobs;
		returnPane->append(sout.str().c_str());
	}
}

//destructor: causes read thread to end
CommandGUI::~CommandGUI()
{
	keepReading = FALSE;
	reader.terminate();
}

//"infinite" loop to run in another thread and read responses
void ReadThread::run()
{
	if (commander == NULL) return;
	string line = "";
	QString qline;
	string::size_type pos;
	while (commander->keepReading) {
		msleep(500);
		line += commander->comm.looplessRead();
		while ((pos = line.find("\n",0)) != string::npos) {
			qline = line.substr(0,pos).c_str();
			commander->showReturnVal(qline);
			line = line.substr(pos+1, line.length()-(pos+1)); //set line to text after "\n"
		}

	}
}

