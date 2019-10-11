#-------------------------------------------------
#
# Project created by QtCreator 2017-06-27T13:14:04
#
#-------------------------------------------------

QT       += core gui concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QT += testlib

QMAKE_CXXFLAGS += -std=c++0x

TARGET = guaca
TEMPLATE = app

unix:!mac {
  target.path = /usr/local/bin
  desktopfile.path = /usr/share/applications
  desktopfile.files += guaca.desktop
  INSTALLS += desktopfile
  iconfile.path = /usr/share/icons/hicolor/scalable/apps
  iconfile.files = images/guaca.svg
  INSTALLS += iconfile
}

INSTALLS += target

SOURCEPATH += ../common/ \
              ../external_libs/linklist/ \

INCLUDEPATH += ../common/include/ \
               ../external_libs/linklist/ \

SOURCES += main.cpp\
        mainwindow.cpp \
    ../liblinklist/linklist.c \
    ../liblinklist/linklist_compress.c \
    ../liblinklist/linklist_writer.c \
    ../liblinklist/linklist_connect.c \
    options.cpp \
    logscroll.cpp

HEADERS  += mainwindow.h \
    ../liblinklist/linklist.h \
    ../liblinklist/linklist_compress.h \
    ../liblinklist/linklist_writer.h \
    ../liblinklist/linklist_connect.h \
    options.h \
    logscroll.h

LIBS += -lssl -lcrypto


FORMS    += mainwindow.ui \
    options.ui \
    logscroll.ui

RESOURCES += \
    guacapics.qrc
