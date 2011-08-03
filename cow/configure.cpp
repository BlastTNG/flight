#include <iostream>
#include <string>
using namespace std;

int main() {
	cerr<<"#THIS FILE IS GENERATED. RUN ./configure TO CONFIGURE AND EDIT configTool.cpp TO DEVELOP"<<endl<<endl;

	cout<<"Where do you you want cow to install to? (default:/usr/local): ";
	char INSTALL_PATH_[256];
	cin.getline(INSTALL_PATH_,256);
	string INSTALL_PATH(INSTALL_PATH_);
	cerr<<"target.path=\\\"\\\\\\\""<<(INSTALL_PATH.size()?INSTALL_PATH:"/usr/local")<<"/bin\\\\\\\"\\\""<<endl;

	cout<<endl<<"What host should cow connect to by default? (default:widow): ";
	char BLASTCMD_HOST_[256];
	cin.getline(BLASTCMD_HOST_,256);
	string BLASTCMD_HOST(BLASTCMD_HOST_);
	cerr<<"DEFINES += BLASTCMD_HOST=\\\"\\\\\\\""<<(BLASTCMD_HOST.size()?BLASTCMD_HOST:"widow")<<"\\\\\\\"\\\""<<endl;

	cout<<endl<<"What should the default DIRFILE be? (default:/data/etc): ";
	char CUR_DIR_[256];
	cin.getline(CUR_DIR_,256);
	string CUR_DIR(CUR_DIR_);
	cerr<<"DEFINES += CUR_DIR=\\\"\\\\\\\""<<(CUR_DIR.size()?CUR_DIR:"/data/etc")<<"\\\\\\\"\\\""<<endl;

	cout<<endl<<"Where should cow store its data? (default:/data/etc/cow): ";
	char DATA_ETC_NARSIL_DIR_[256];
	cin.getline(DATA_ETC_NARSIL_DIR_,256);
	string DATA_ETC_NARSIL_DIR(DATA_ETC_NARSIL_DIR_);
	cerr<<"DEFINES += DATA_ETC_NARSIL_DIR=\\\"\\\\\\\""<<(DATA_ETC_NARSIL_DIR.size()?DATA_ETC_NARSIL_DIR:"/data/etc/cow")<<"\\\\\\\"\\\""<<endl;
	cerr<<"log.path = "<<(DATA_ETC_NARSIL_DIR.size()?DATA_ETC_NARSIL_DIR:"/data/etc/cow")<<endl;
	
	cout<<endl;
	cerr<<"\
log.files = data/prev_status data/log.txt \n\
\n\
QT += gui \n\
QMAKE_INSTALL_FILE = install -m 666 -p \n\
INSTALLS += target log \n\
\n\
HEADERS += \\ \n\
src/cow.h \\ \n\
src/share/netcmd.h \\ \n\
src/widgets.h \n\
\
SOURCES += \\ \n\
src/netcmd.c \\ \n\
src/cow.cpp \n\
\n\
LIBS += -lpthread -lgetdata++ \n\
\n\
RESOURCES += \\ \n\
icons/icons.qrc \n\
"<<endl;

}
