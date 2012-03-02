target.path=\"\\\"/usr/local/bin\\\"\"
DEFINES += DATA_ETC_NARSIL_DIR=\"\\\"/data/etc/cow\\\"\"
log.path = /data/etc/cow
log.files = data/prev_status data/log.txt

######################################################

QT += gui 
QMAKE_INSTALL_FILE = install -m 666 -p 
INSTALLS += target log 
INCPATH += ../common/

HEADERS += \ 
src/cow.h \ 
../common/netcmd.h \ 
src/widgets.h 
SOURCES += \ 
../common/netcmd.c \ 
src/cow.cpp 

LIBS += -lpthread -lgetdata++ 

RESOURCES += \ 
icons/icons.qrc 

