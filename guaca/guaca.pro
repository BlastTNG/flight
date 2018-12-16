#-------------------------------------------------
#
# Project created by QtCreator 2017-06-27T13:14:04
#
#-------------------------------------------------

QT       += core gui concurrent

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

QT += testlib

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
    ../external_libs/linklist/linklist.c \
    ../external_libs/linklist/linklist_compress.c \
    ../external_libs/linklist/linklist_writer.c \
    ../external_libs/linklist/linklist_connect.c \
    ../common/CRC_func.c \

HEADERS  += mainwindow.h \
    ../external_libs/linklist/linklist.h \
    ../external_libs/linklist/linklist_compress.h \
    ../external_libs/linklist/linklist_writer.h \
    ../external_libs/linklist/linklist_connect.h \
    ../common/include/CRC_func.h \

LIBS += -lssl -lcrypto


FORMS    += mainwindow.ui

RESOURCES += \
    guacapics.qrc
