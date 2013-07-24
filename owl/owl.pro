QT      += core gui xml

TARGET   = owl
TEMPLATE = app

unix:!mac {
  target.path = /usr/local/bin
  LIBS += -lpython2.7
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
INSTALLS += owlfiles

SOURCES += main.cpp\
        PMainWindow.cpp \
    PObject.cpp \
    PAbstractDataItem.cpp \
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
    PDotPal.cpp \
    PStyleChooser.cpp \
    PBoxTitle.cpp \
    PServer.cpp \
    PWebServerInfo.cpp

HEADERS  += PMainWindow.h \
    PObject.h \
    PAbstractDataItem.h \
    PMultiDataItem.h \
    PBox.h \
    PMdiArea.h \
    PNumberDataItem.h \
    PExtrema.h \
    PMap.h \
    POwlAnimation.h \
    PTimeDataItem.h \
    PDirfileDataItem.h \
    PDotPal.h \
    PStyle.h \
    PStyleChooser.h \
    PBoxTitle.h \
    PServer.h \
    PWebServerInfo.h

FORMS    += PMainWindow.ui \
    PStyleChooser.ui \
    PWebServerInfo.ui

LIBS     += -lgetdata++ -lqjson

RESOURCES += \
    icons.qrc \
    client.qrc

QMAKE_CXXFLAGS = -O0

OTHER_FILES += \
    PClient.html \
    README.txt

