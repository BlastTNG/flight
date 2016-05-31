#include <QtGui>
#include "MainWindow.h"

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  MainWindow* window = new MainWindow();
  window->resize(600, 600);
  window->setWindowTitle("Quick Diagnostics");
  window->show();
  return app.exec();
}
