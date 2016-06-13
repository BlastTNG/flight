TEMPLATE = app
TARGET = QuickDiag
#CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++11 #-stdlib=libc++
DEPENDPATH += .
INCLUDEPATH += . /usr/local/include
LIBS += -L/usr/local/lib -lgetdata++

# Input
HEADERS += MainWindow.h SetupView.h DiagnosticsView.h LeafNode.h NodeGrid.h DetailsView.h
SOURCES += main.cpp MainWindow.cpp SetupView.cpp DiagnosticsView.cpp LeafNode.cpp NodeGrid.cpp DetailsView.cpp
