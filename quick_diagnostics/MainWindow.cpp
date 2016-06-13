#include "MainWindow.h"
#include "LeafNode.h"
#include "NodeGrid.h"
#include "LeafGroup.h"
#include <string>

using namespace std;

MainWindow::MainWindow() : QWidget() {
  setupView = new SetupView();
  diagView = NULL;

  // Store the views in a stacked layout, allowing an easy switch between the two
  viewStack = new QStackedLayout();
  viewStack->addWidget(setupView);
  viewStack->setCurrentWidget(setupView); // start on the setup view
  this->setLayout(viewStack);

  // When the user is done selecting the necessary files, generate the diagnostics view
  QObject::connect(setupView, SIGNAL(doneSetup(GetData::Dirfile*, json)), SLOT(generateDiagnostics(GetData::Dirfile*, json)));
}

/**
  Use the given dirfile and configuration file to generate a view that provides diagnostics for all of the sensors.
*/
void MainWindow::generateDiagnostics(GetData::Dirfile* dirfile, json config) {

  diagView = new DiagnosticsView(dirfile, config);
  if (diagView->errorList->empty()) {
    viewStack->addWidget(diagView);
    viewStack->setCurrentWidget(diagView);
  } else {
    // If errors are encountered, notify the user and let him/her reselect the files
    QMessageBox msgBox;
    QString txt;
    for (int i = 0; i < diagView->errorList->size(); ++i) {
      txt.append(QString::number(i));
      txt.append(") " + *diagView->errorList->at(i) + "\n");
    }
    msgBox.setText("There is an error in the .json file");
    msgBox.setInformativeText(txt);
    msgBox.exec();
  }
}
