unix:!mac {
  target.path=\"\\\"/usr/local/bin\\\"\"
}
mac {
  target.path=\"/Applications\"
  # I don't understand why qmake doesn't do this automatically
  QMAKE_CLEAN += '-r cow.app'
  ICON = cow.icns
}
DEFINES += DATA_ETC_COW_DIR=\\\"/data/etc/cow\\\"
DEFINES += COW_SVN_REVISION=\"\\\"${SVN_REVISION}\\\"\"
log.path = /data/etc/cow
log.files = data/prev_status data/log.txt

######################################################

QT += gui 
QMAKE_INSTALL_FILE = install -m 666 -p 
INSTALLS += target log 
INCLUDEPATH += ../common/include/ /usr/local/include

HEADERS += \ 
src/cow.h \ 
../common/include/netcmd.h \ 
src/widgets.h 
SOURCES += \ 
../common/netcmd.c \ 
src/cow.cpp 

LIBS += -lpthread -lgetdata++ 

RESOURCES += \ 
icons/icons.qrc 

