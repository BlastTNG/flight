TARGET   = cow
TEMPLATE = app

unix:!mac {
  target.path=/usr/local/bin
  desktopfile.path = /usr/share/applications
  desktopfile.files += cow.desktop
  INSTALLS += desktopfile
  iconfile.path = /usr/share/icons/hicolor/scalable/apps
  iconfile.files = cow.svg
  INSTALLS += iconfile
}
mac {
  target.path=\"/Applications\"
  # I don't understand why qmake doesn't do this automatically
  QMAKE_CLEAN += '-r cow.app'
  ICON = cow.icns
}
DEFINES += DATA_ETC_COW_DIR=\\\"/data/etc/cow\\\"
DEFINES += COW_SVN_REVISION=\"\\\"${SVN_REVISION}\\\"\"
DEFINES += PRAGMAPACK
log.path = /data/etc/cow
# not using normal files property, since using custom install command
log.thefiles = data/prev_status data/log.txt data/*.herd
log.commands = install -m 666 -p $$log.thefiles $$log.path

######################################################

QT += gui 
QT += widgets

INSTALLS += target log 
INCLUDEPATH += /usr/local/lib/

CONFIG += qt
HEADERS += \ 
src/cow.h \ 
src/widgets.h 
SOURCES += \ 
src/cow.cpp 

LIBS += -lpthread -lgetdata++ -lnetcmd

RESOURCES += \ 
icons/icons.qrc 

