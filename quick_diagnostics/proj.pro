TEMPLATE = app
TARGET = QuickDiag
#CONFIG += c++11
QMAKE_CXXFLAGS += -std=c++11 #-stdlib=libc++
DEPENDPATH += .
INCLUDEPATH += . /usr/local/include
LIBS += -L/usr/local/lib -lgetdata++

# Input
HEADERS += MainWindow.h SetupView.h DiagnosticsView.h StatusNode.h  ParentNode.h LeafNode.h PathLabel.h NodeGrid.h DetailsView.h
SOURCES += main.cpp MainWindow.cpp SetupView.cpp DiagnosticsView.cpp ParentNode.cpp LeafNode.cpp PathLabel.cpp NodeGrid.cpp DetailsView.cpp
