#include "MainWindow.h"
#include "StatusNode.h"
#include "ParentNode.h"
#include "LeafNode.h"
#include "NodeGrid.h"
#include <string>

using namespace std;

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

  // When the user is done selecting the necessary files, generate the diagnostics view
  QObject::connect(setupView, SIGNAL(doneSetup(GetData::Dirfile*, json)), SLOT(generateDiagnostics(GetData::Dirfile*, json)));
}

/*
  Return whether or not the given json obj is a leaf or not.
  The node is a leaf if all fields are primitive (that is, no json object is contained within this json object).
  http://nlohmann.github.io/json/classnlohmann_1_1basic__json_a7c774ef0eceff6d06095f617e2dbd488.html#a7c774ef0eceff6d06095f617e2dbd488
*/
bool isLeaf(json obj) {
  for (json::iterator it = obj.begin(); it != obj.end(); ++it) {
    json element = *it;
    if (!element.is_primitive()) return false;
  }
  return true;
}

/**
  Get a list of leaf nodes, one for every fieldCode with the given prefix in the given dirfile
*/
QList<StatusNode*>* getLeavesForPrefix(DiagnosticsView* diagView, GetData::Dirfile* dirfile, string prefix, double lo, double hi, QLinkedList<const char*>* unusedFields) {
  QList<StatusNode*>* list = new QList<StatusNode*>();
  
  // Get Null-terminated list of fields
  const char** fieldList = dirfile->FieldList();
  for (int i = 0; fieldList[i] != NULL; i++) {
    const char* fieldCode = fieldList[i];  
  
    // If the fieldCode has the given prefix:
    string strField(fieldCode);
    if (strField.substr(0, prefix.size()).compare(prefix) == 0) {

      // Create a LeafNode for this field
      LeafNode* leaf = new LeafNode(dirfile, fieldCode, lo, hi);
      list->append(leaf);
      diagView->configureLeafNode(leaf);

      // Remove this field code from the list of unused fields
      unusedFields->removeAll(fieldCode);
    }
  }
  return list;
}

/*
  Generate the view tree

  unusedFields is a ref to a list of all of the fields that have not yet been used in any node 
*/
NodeGrid* generateTree(DiagnosticsView* diagView, GetData::Dirfile* dirfile, json config, QLinkedList<const char*>* unusedFields) {
    
  // Generate a list of widgets by iterating through the json object.
  // These widgets will be the cells of a GridLayout
  QList<StatusNode*> gridCells; 
  for(json::iterator it = config.begin(); it != config.end(); ++it) {
    json element = *it;

    // Create many leaf nodes
    if (isLeaf(element)) {
      // Get the prefix field. TODO: catch exceptions
      string prefix = element["prefix"].get<string>();
      double lo = element["lo"].get<double>();
      double hi = element["hi"].get<double>();

      // For every field with the prefix, add a leaf-node to the grid for that fieldcode
      QList<StatusNode*>* leaves = getLeavesForPrefix(diagView, dirfile, prefix, lo, hi, unusedFields);
      gridCells.append(*leaves);
      delete leaves; // delete the list (doesn't delete the elements in the list)
    } 

    // Recursively generate the parent node. TODO: switch from recursion to iteration, don't bust the stack
    else {
      NodeGrid* childView = generateTree(diagView, dirfile, it.value(), unusedFields);
      QString name = QString::fromStdString(it.key());
      ParentNode* parent = new ParentNode(name, childView);
      diagView->configureParentNode(parent); // when the parent is double-clicked, transition to its childView
      gridCells.append(parent);
    }
  }

  // Add all of the gridCells to a grid
  NodeGrid* nodeGrid = new NodeGrid();
  int numCells = gridCells.size();
  int numCols = sqrt(numCells); // arrange the grid cells in a square formation
  for (int i = 0; i < numCells; ++i) {
    StatusNode* n = gridCells.at(i);
    nodeGrid->addChildNode(n, i / numCols, i % numCols);
  }
  return nodeGrid;
}

/**
  Use the given dirfile and configuration file to generate a view that provides diagnostics for all of the sensors.
*/
void MainWindow::generateDiagnostics(GetData::Dirfile* dirfile, json config) {

  // TODO: generate the Diag-View within the diag-view class, then 
  // decide to display it based on what it returns
  // will be a lot cleaner than passing diaView around here, and configuring all nodes

  // Create a linked list of all of the field codes
  QLinkedList<const char*>* unusedFields = new QLinkedList<const char*>();
  const char** fieldCodes = dirfile->FieldList();
  for (int i = 0; fieldCodes[i] != NULL; ++i) {
    const char* fC = fieldCodes[i];
    unusedFields->push_back(fC);
  }

  DiagnosticsView* diagView = new DiagnosticsView();
  NodeGrid* rootView = generateTree(diagView, dirfile, config, unusedFields);

  // If the configuration doesn't define at least one leaf for each sensor, issue a warning
  if (!unusedFields->empty()) {

    // Make a QString list of all of the unused fields, deleting them as we go
    QString detailsStr("Unused Fields:\n\n");
    while (!unusedFields->empty()) {
      const char* fC = unusedFields->takeFirst();
      detailsStr.append(fC);
      detailsStr.append("\n");
      delete fC;
    }

    QMessageBox msgBox;
    msgBox.setText("The selected .json file doesn't account for all field codes");
    msgBox.setInformativeText("Press Show Details to list the unused fields, Cancel to reselect .json, Ok to disregard this msg");
    msgBox.setDetailedText(detailsStr);
    msgBox.setStandardButtons(QMessageBox::Cancel | QMessageBox::Ok);
    int ret = msgBox.exec();

    // If the user presses cancel, let him/her reselect the files
    if (ret == QMessageBox::Cancel) {
      return;  
    }
  }

  ParentNode* rootParent = new ParentNode("Root", rootView);
  diagView->setRoot(rootParent); // sets root view to the first diagnostics view
  viewStack->addWidget(diagView);
  viewStack->setCurrentWidget(diagView);
}
