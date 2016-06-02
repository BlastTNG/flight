#include "MainWindow.h"
#include "ParentNode.h"
#include "LeafNode.h"
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
QList<QWidget*>* getLeavesForPrefix(GetData::Dirfile* dirfile, string prefix) {
  QList<QWidget*>* list = new QList<QWidget*>();
  
  // Get Null-terminated list of fields
  const char** fieldList = dirfile->FieldList();
  for (int i = 0; fieldList[i] != NULL; i++) {
    const char* fieldCode = fieldList[i];  
  
    // If the fieldCode has the given prefix, create a LeafNode for it
    string strField(fieldCode);
    if (strField.substr(0, prefix.size()).compare(prefix) == 0) {
      LeafNode* leaf = new LeafNode(dirfile, fieldCode);
      list->append(leaf);
    }
  }
  return list;
}

QWidget* generateTree(DiagnosticsView* diagView, GetData::Dirfile* dirfile, json config) {
    
  // Generate a list of widgets by iterating through the json object.
  // These widgets will be the cells of a GridLayout
  QList<QWidget*> gridCells; 
  for(json::iterator it = config.begin(); it != config.end(); ++it) {
    json element = *it;

    // Create many leaf nodes
    if (isLeaf(element)) {
      // Get the prefix field
      string prefix = element["prefix"].get<string>();

      // For every field with the prefix, add a leaf-node to the grid for that fieldcode
      QList<QWidget*>* leaves = getLeavesForPrefix(dirfile, prefix);
      gridCells.append(*leaves);
      delete leaves; // delete the list (doesn't delete the elements in the list)
    } 
    // Recursively generate the parent node. TODO: switch from recursion to iteration, don't bust the stack
    else {
      QWidget* childView = generateTree(diagView, dirfile, it.value());
      QString name = QString::fromStdString(it.key());
      ParentNode* parent = new ParentNode(name, childView);
      diagView->configureParentNode(parent); // when the parent is double-clicked, transition to its childView
      gridCells.append(parent);
    }
  }

  // Add all of the gridCells to a grid
  QGridLayout* grid = new QGridLayout();
  int numCells = gridCells.size();
  int numCols = sqrt(numCells); // arrange the grid cells in a square formation
  for (int i = 0; i < numCells; ++i) {
    QWidget* w = gridCells.at(i);
    grid->addWidget(w, i / numCols, i % numCols);
  }

  // Return the grid as a widget
  QWidget* widget = new QWidget();
  widget->setLayout(grid);
  return widget;
}

/**
  Use the given dirfile and configuration file to generate a view that provides diagnostics for all of the sensors.
*/
void MainWindow::generateDiagnostics(GetData::Dirfile* dirfile, json config) {
  DiagnosticsView* diagView = new DiagnosticsView();
  QWidget* rootView = generateTree(diagView, dirfile, config);
  ParentNode* rootParent = new ParentNode("Root", rootView);
  diagView->setRoot(rootParent); // sets root view to the first diagnostics view
  viewStack->addWidget(diagView);
  viewStack->setCurrentWidget(diagView);
}
