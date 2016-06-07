#include "DiagnosticsView.h"
#include "ParentNode.h"
#include "LeafNode.h"

DiagnosticsView::DiagnosticsView() : QWidget() {
  pathStack = new QStack<QWidget*>();  
  
  // The main part of the DiagnosticsView is a widget with a stack layout, where each widget in the 
  // stack is a view of different sensors. Put the main view in a scroll area.
  stackLayout = new QStackedLayout();
  QWidget* mainView = new QWidget();
  mainView->setLayout(stackLayout);

  // Configure the path label / navigator.
  // When the user requests a view via the navigator, switch to that view
  pathLabel = new PathLabel();
  QObject::connect(pathLabel, SIGNAL(viewRequested(QWidget*)), stackLayout, SLOT(setCurrentWidget(QWidget*)));

  detailLabel = new QLabel("Click a leaf-node to show its details here");
 
  // Arrange the elements vertically
  QVBoxLayout* vBox = new QVBoxLayout();
  vBox->addWidget(pathLabel);
  vBox->addWidget(detailLabel);
  vBox->addWidget(mainView); 
  this->setLayout(vBox);

  // Init the selected leaf
  selectedNode = NULL;
}

void DiagnosticsView::setRoot(ParentNode* root) {
  pushView(root);
}

void DiagnosticsView::configureParentNode(ParentNode* parent) {
  // When the parent node is double-clicked, transition to its child view
  QObject::connect(parent, SIGNAL(selected(ParentNode*)), this, SLOT(pushView(ParentNode*)));
}

void DiagnosticsView::configureLeafNode(LeafNode* leaf) {
  // When a leaf-node is clicked, show its details in the detail label
  QObject::connect(leaf, SIGNAL(clicked(const char*)), this, SLOT(updateDetailLabel(const char*)));
}

void DiagnosticsView::updateDetailLabel(const char* fieldCode) {
  // Use the field-code to get and display more details about the field
  detailLabel->setText(fieldCode); 

  // Get the clicked leaf-node, and select it
  LeafNode* leaf = qobject_cast<LeafNode*>(QObject::sender());  
  if (selectedNode != NULL) {
    selectedNode->unselect();
  }
  leaf->select();
  selectedNode = leaf;
}

// Push and display the parent's child view
void DiagnosticsView::pushView(ParentNode* clickedParent) {

  NodeGrid* nextView = clickedParent->getChildView();

  // Update the path navigator
  pathLabel->pushWidget(clickedParent->text(), nextView);

  // Push the view on to this widget's stack, and display it
  stackLayout->addWidget(nextView);
  stackLayout->setCurrentWidget(nextView);
}
