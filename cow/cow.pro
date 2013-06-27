unix:!mac {
  target.path=\"\\\"/usr/local/bin\\\"\"
}
mac {
  target.path=\"/Applications\"
  # I don't understand why qmake doesn't do this automatically
  QMAKE_CLEAN += '-r cow.app'
  ICON = cow.icns
}
DEFINES += DATA_ETC_NARSIL_DIR=\"\\\"/data/etc/cow\\\"\"
log.path = /data/etc/cow
log.files = data/prev_status data/log.txt

######################################################

QT += gui 
QMAKE_INSTALL_FILE = install -m 666 -p 
INSTALLS += target log 
INCLUDEPATH += ../common/

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

