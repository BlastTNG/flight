QT      += core gui xml

TARGET   = owl
TEMPLATE = app

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

LIBS     += -lgetdata++

RESOURCES += \
    icons.qrc \
    client.qrc

QMAKE_CXXFLAGS = -O2

OTHER_FILES += \
    PClient.html \
    README.txt
