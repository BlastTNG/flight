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

  detailsView = new DetailsView();
 
  // Arrange the elements vertically
  QVBoxLayout* vBox = new QVBoxLayout();
  vBox->setSpacing(3);
  vBox->addWidget(detailsView);
  vBox->addWidget(pathLabel);
  vBox->addWidget(mainView); 
  this->setLayout(vBox);

  // Init pointers to null
  selectedNode = NULL;
  currentGrid = NULL;

  // Update the status of the diagnostics view frequently
  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(updateDisplayedNodes()));
  timer->start(1000); // TODO: put to 500ms
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
  QObject::connect(leaf, SIGNAL(clicked(LeafNode*)), this, SLOT(updateDetailLabel(LeafNode*)));
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
  // Update all of the currently displayed nodes
  if (currentGrid != NULL) {
    currentGrid->updateChildren(); 
  }

  // Update the detail label for the selected node
  if (selectedNode != NULL) {
    detailsView->updateDetails(selectedNode);
  }
}

// Push and display the parent's child view
void DiagnosticsView::pushView(ParentNode* clickedParent) {

  // Get ref to child view
  NodeGrid* nextView = clickedParent->getChildView();

  // Update the path navigator
  pathLabel->pushWidget(clickedParent->text(), nextView);

  // Push the view on to this widget's stack, and display it
  stackLayout->addWidget(nextView);
  stackLayout->setCurrentWidget(nextView);

  // Update ref
  currentGrid = nextView;
}
