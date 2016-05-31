#include "MainWindow.h"

// TODO: don't make the Diag view until later
MainWindow::MainWindow() : QWidget() {
  setupView = new SetupView();
  diagView = new DiagnosticsView();

  // Store the views in a stacked layout, allowing an easy switch between the two
  viewStack = new QStackedLayout();
  viewStack->addWidget(setupView);
  viewStack->addWidget(diagView);
  viewStack->setCurrentWidget(setupView); // start on the setup view
  this->setLayout(viewStack);
}

void MainWindow::generateDiagnostics(GetData::Dirfile* dirfile, json config) {
        
}
