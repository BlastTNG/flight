#include "DiagnosticsView.h"

DiagnosticsView::DiagnosticsView(GetData::Dirfile* dirfile, json config) : QWidget(), dirfile(*dirfile) {

  setContentsMargins(0, 0, 0, 0);

  // Init 
  selectedNode = NULL;
  lastNumFrames = -1; // so that displayed nodes always update on first go
  
  // The main part of the DiagnosticsView is a widget with a stack layout, where each widget in the 
  // stack is a view of different sensors. Put the main view in a scroll area.
  stackLayout = new QStackedLayout();
  QWidget* mainView = new QWidget();
  mainView->setLayout(stackLayout);

  detailsView = new DetailsView();
  updateClock = new UpdateClock();

  // Generate the view map
  errorList = new QList<QString*>();
  viewMap = new QMap<QString, NodeGrid*>();
  generateViewMap(dirfile, config);

  if (viewMap->empty()) {
    errorList->append(new QString("The .json file doesn't define any valid views"));
    return;
  }

  // Create a combo-box of the views in the view map
  QComboBox* comboBox = new QComboBox();
  QMapIterator<QString, NodeGrid*> i(*viewMap);
  while (i.hasNext()) {
    i.next();
    comboBox->addItem(i.key());
  }

  // When the user selects a new view via the combo box, update the main view area
  QObject::connect(comboBox, SIGNAL(currentIndexChanged(const QString&)), this, SLOT(switchView(const QString&)));

  QGridLayout* topPanel = new QGridLayout();
  topPanel->addWidget(comboBox, 0, 0);
  topPanel->addWidget(updateClock, 1, 0);
  topPanel->addWidget(detailsView, 0, 1, 2, 1);


  // Arrange the elements vertically
  QVBoxLayout* vBox = new QVBoxLayout();
  vBox->setContentsMargins(0, 0, 0, 0);
  vBox->setSpacing(3);
  vBox->addLayout(topPanel);
  vBox->addWidget(mainView); 
  this->setLayout(vBox);

  // Set the current view to the first view in the combo box
  currentGrid = NULL;
  if (!viewMap->empty()) {
    QString viewName = comboBox->itemText(0);
    NodeGrid* view = viewMap->value(viewName);
    pushView(view);
  }
    
  // Update the status of the diagnostics view frequently
  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(updateDisplayedNodes()));
  timer->start(1000); // TODO: put to 500ms
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
QList<LeafNode*>* DiagnosticsView::getLeavesForPrefix(GetData::Dirfile* dirfile, string prefix, double lo, double hi) {
  QList<LeafNode*>* list = new QList<LeafNode*>();
  
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

      // When a leaf-node is clicked, show its details in the detail label
      QObject::connect(leaf, SIGNAL(clicked(LeafNode*)), this, SLOT(updateDetailLabel(LeafNode*)));
    }
  }
  return list;
}

NodeGrid* DiagnosticsView::generateWithManualLayout(GetData::Dirfile* dirfile, json config) {

  string viewName;
  QList<PositionedLeaf*>* leaves = new QList<PositionedLeaf*>();
  try {

    // Iterate through the json object, parsing the data into leaves
    for (json::iterator it = config.begin(); it != config.end(); ++it) {
      json element = *it;

      // Skip primitive elements (specifically, "layout")
      if (element.is_primitive()) {
        continue;
      }

      string* fieldCode = new string(it.key());
      double lo = element["lo"].get<double>();
      double hi = element["hi"].get<double>();
      LeafNode* n = new LeafNode(dirfile, fieldCode->c_str(), lo, hi);

      // When a leaf-node is clicked, show its details in the detail label
      QObject::connect(n, SIGNAL(clicked(LeafNode*)), this, SLOT(updateDetailLabel(LeafNode*)));

      int x = element["x"].get<int>();
      int y = element["y"].get<int>();
      int w = element["w"].get<int>();
      int h = element["h"].get<int>();
      PositionedLeaf* pl = new PositionedLeaf(n, x, y, w, h);
      leaves->append(pl);
    }

  } catch(const std::exception& e) {
    QString* err = new QString(e.what());
    errorList->append(err);  
  }
  return new NodeGrid(leaves);
}

/*
  Generate a node grid

  unusedFields is a ref to a list of all of the fields that have not yet been used in any node 
*/
NodeGrid* DiagnosticsView::generateWithAutoLayout(GetData::Dirfile* dirfile, json config) {
    
  // Generate a list of widgets by iterating through the json object.
  // These widgets will be the cells of a GridLayout
  QList<LeafGroup*>* leafGroups = new QList<LeafGroup*>();
  for(json::iterator it = config.begin(); it != config.end(); ++it) {
    json element = *it;

    // Views cannot be nested in other views
    if (!isLeaf(element)) {
      errorList->append(new QString("Views cannot be nested inside of other views"));
      break;
    }

    // Skip primitive elements (specifically, "layout")
    if (element.is_primitive()) {
      continue;
    }

    // In case an exception is thrown, prepare a string
    QString* expStr = new QString("Exception thrown for \"");
    expStr->append(QString::fromStdString(it.key()));
    expStr->append("\": ");
    try {
      // Read the .json file (may throw exceptions)
      string prefix = element["prefix"].get<string>();
      double lo = element["lo"].get<double>();
      double hi = element["hi"].get<double>();

      // For every field with the prefix, add a leaf-node to the grid for that fieldcode
      QList<LeafNode*>* leaves = getLeavesForPrefix(dirfile, prefix, lo, hi);
      string groupName = it.key();
      LeafGroup* group = new LeafGroup(groupName, leaves);
      leafGroups->append(group);

      delete expStr;
    } catch(std::domain_error& e) {
      expStr->append(e.what());
      errorList->append(expStr);  
    } catch(std::out_of_range& e) {
      expStr->append(e.what());
      errorList->append(expStr);
    }
  }  
  NodeGrid* nodeGrid = new NodeGrid(leafGroups);
  return nodeGrid;
}

string DiagnosticsView::getViewLayout(json view) {
  string layout = "";
  try {
    layout = view["layout"].get<string>();
  } catch(std::domain_error& e) {
    layout = "error";
  } catch(std::out_of_range& e) {
    layout = "error";
  }
  return layout;
}

void DiagnosticsView::generateViewMap(GetData::Dirfile* dirfile, json config) {

  // Iterate through all of the views defined in the json file, generating them and adding them to the viewMap as we go
  for(json::iterator it = config.begin(); it != config.end(); ++it) {
    json element = *it;
    string key = it.key();
    QString viewName = QString::fromStdString(key);
    
    // Generate the layout based on its desired layout
    NodeGrid* view;
    string layout = getViewLayout(it.value());
    if (layout.compare("manual") == 0) {
      view = generateWithManualLayout(dirfile, it.value());  
      viewMap->insert(viewName, view);
    } 
    else if (layout.compare("auto") == 0) {
      view = generateWithAutoLayout(dirfile, it.value());
      viewMap->insert(viewName, view);
    }
    else {
      QString* s = new QString("Invalid format for view ");
      s->append(viewName);
      errorList->append(s); 
    }
  }
}

void DiagnosticsView::updateDetailLabel(LeafNode* leaf) {

  // Get the clicked leaf-node, and select it
  if (selectedNode != NULL) {
    selectedNode->unselect();
  }
  leaf->select();
  selectedNode = leaf;
}

void DiagnosticsView::updateDisplayedNodes() {

  // Get the number of frames
  int numFrames = dirfile.NFrames();

  // Report any dirfile errors
  if (dirfile.Error() != GD_E_OK) {
    QMessageBox box;
    QString msg("Dirfile error when reading number of frames: ");
    msg += dirfile.ErrorString();
    box.setText(msg);
    box.exec();
    return;
  }

  // Read dirfile only if new data was received, or this is the first update of a new view
  if (lastNumFrames < numFrames || newView) {
    lastNumFrames = numFrames; 
    updateClock->updateNumFrames(numFrames);
    newView = false;

    // Update all of the currently displayed nodes
    if (currentGrid != NULL) {
      currentGrid->updateChildren(numFrames); 
    }
  }
  
  // Update the detail label for the selected node
  if (selectedNode != NULL) {
    detailsView->updateDetails(selectedNode);
  }
}

// Switch to the view with the given name
void DiagnosticsView::switchView(const QString& viewName) {
  NodeGrid* nextView = viewMap->value(viewName, currentGrid); // if the name is not found, default to the current grid (no change)
  pushView(nextView);
}

// Push and display the parent's child view
void DiagnosticsView::pushView(NodeGrid* nextView) {

  // Push the view on to this widget's stack, and display it
  stackLayout->addWidget(nextView);
  stackLayout->setCurrentWidget(nextView);

  // Update ref
  currentGrid = nextView;

  // Set to true so that we update the newly displayed nodes
  newView = true;
}
