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
 
  // Arrange the elements vertically
  QVBoxLayout* vBox = new QVBoxLayout();
  vBox->addWidget(pathLabel);
  vBox->addWidget(mainView); 
  this->setLayout(vBox);
}

void DiagnosticsView::setRoot(ParentNode* root) {
  pushView(root);
}

void DiagnosticsView::configureParentNode(ParentNode* parent) {
  // When the parent node is double-clicked, transition to its child view
  QObject::connect(parent, SIGNAL(selected(ParentNode*)), this, SLOT(pushView(ParentNode*)));
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
