TEMPLATE = app
TARGET = proj
QMAKE_CXXFLAGS += -std=c++11
DEPENDPATH += .
INCLUDEPATH += . /usr/local/include
LIBS += -L/usr/local/lib -lgetdata++

# Input
HEADERS += MainWindow.h SetupView.h DiagnosticsView.h ParentNode.h LeafNode.h
SOURCES += main.cpp MainWindow.cpp SetupView.cpp DiagnosticsView.cpp ParentNode.cpp LeafNode.cpp
