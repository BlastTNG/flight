target.path=\"\\\"/usr/local/bin\\\"\"
DEFINES += DATA_ETC_NARSIL_DIR=\"\\\"/data/etc/cow\\\"\"
log.path = /data/etc/cow
log.files = data/prev_status data/log.txt

######################################################

QT += gui 
QMAKE_INSTALL_FILE = install -m 666 -p 
INSTALLS += target log 

HEADERS += \ 
src/cow.h \ 
src/share/netcmd.h \ 
src/widgets.h 
SOURCES += \ 
src/netcmd.c \ 
src/cow.cpp 

LIBS += -lpthread -lgetdata++ 

RESOURCES += \ 
icons/icons.qrc 

