QT      += core gui xml widgets
#QT      += websockets network

TARGET   = owl
TEMPLATE = app

unix:!mac {
  target.path = /usr/local/bin
  LIBS += -lpython2.7
  desktopfile.path = /usr/share/applications
  desktopfile.files += owl.desktop
  INSTALLS += desktopfile
  iconfile.path = /usr/share/icons/hicolor/scalable/apps
  iconfile.files = owl.svg
  INSTALLS += iconfile
}

mac {
  target.path = /Applications
  # I don't understand why qmake doesn't do this automatically
  QMAKE_CLEAN += '-r owl.app'
  ICON = owl.icns
  LIBS += -lpython2.6
}
INSTALLS += target
CONFIG += link_pkgconfig

owlfiles.path = /data/etc/owl
owlfiles.files += owl-files/spider.owl
owlfiles.files += owl-files/bit2016.owl
INSTALLS += owlfiles

contains(QT_MAJOR_VERSION, 5) {
  greaterThan(QT_MINOR_VERSION, 2) {
    QT      += websockets network
    SOURCES += PWebServer.cpp
    HEADERS  += PWebServer.h
  }
}

#QT      += websockets network

SOURCES += main.cpp\
        PMainWindow.cpp \
    PObject.cpp \
    PAbstractDataItem.cpp \
    PBitMultiDataItem.cpp \
    PMultiDataItem.cpp \
    PBox.cpp \
    PMdiArea.cpp \
    PExtrema.cpp \
    PMap.cpp \
    PNumberDataItem.cpp \
    POwlAnimation.cpp \
    PDotOwl.cpp \
    PTimeDataItem.cpp \
    PDirfileDataItem.cpp \
    PStyleChooser.cpp \
    PBoxTitle.cpp
    #PWebServer.cpp

HEADERS  += PMainWindow.h \
    PObject.h \
    PAbstractDataItem.h \
    PBitMultiDataItem.h \
    PExtremaDataItem.h \
    PMultiDataItem.h \
    PBox.h \
    PMdiArea.h \
    PNumberDataItem.h \
    PExtrema.h \
    PMap.h \
    POwlAnimation.h \
    PTimeDataItem.h \
    PDirfileDataItem.h \
    PStyle.h \
    PStyleChooser.h \
    PBoxTitle.h
    #PWebServer.h

FORMS    += PMainWindow.ui \
    PStyleChooser.ui

LIBS     += -lgetdata++
LIBS     += -lqjson

RESOURCES += \
    icons.qrc \
    client.qrc

QMAKE_CXXFLAGS = -O2

OTHER_FILES += \
    PClient.html \
    README.txt
