#include "PathLabel.h"
#include "ParentNode.h"

PathLabel::PathLabel() : QWidget() {
  labelStack = new QStack<ParentNode*>();
  layout = new QHBoxLayout();  
  layout->setAlignment(Qt::AlignLeft);
  layout->addWidget(new QLabel("Navigator:"));
  setLayout(layout);
}

void PathLabel::popWidgets(ParentNode* clickedLabel) {

  // Pop the given path, until the rightmost label (the one double-clicked), describes the requested widget
  // Invariant: the labelStack will always have at least one element (the root)
  ParentNode* p = labelStack->top();
  while (p != clickedLabel) {
    labelStack->pop();
    p->close(); // remove from the PathLabel
    delete p;
    p = labelStack->top();
  }

  // Emit the requested view, which the child view of the clicked label
  QWidget* childView = clickedLabel->getChildView();
  emit viewRequested(childView);
}

void PathLabel::pushWidget(QString name, QWidget* widget) {
  // Add a parent-node to the path, and emit the label when it is double-clicked
  // so that the Diagnostics view and navigator can update
  ParentNode* label = new ParentNode(name, widget);
  layout->addWidget(label);
  QObject::connect(label, SIGNAL(selected(ParentNode*)), this, SLOT(popWidgets(ParentNode*)));

  // Store a ref to the newly added label by adding to the stack
  labelStack->push(label);
}

