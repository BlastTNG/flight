#include "ParentNode.h"

ParentNode::ParentNode(QString name, NodeGrid* childView) : StatusNode(name), childView(childView) {
  setMargin(3);
}

void ParentNode::updateStatus() {
  // TODO: it's not really necessary to have the parent node have a color, too
  // Could at some point make a cumulative color though

  // A parent's status is good if all of its children's statuses are good, otherwise bad
  /*const QList<StatusNode*>& children = childView->getChildNodes();*/
  //bool allGood = true;
  //for (int i = 0; i < children.size(); i++) {
    //StatusNode* n = children.at(i);
    //if (n->getStatus() != GOOD) {
      //allGood = false;  
      //break;
    //}
  //}
  //Status s = allGood ? GOOD : BAD;
  //setStatus(s);
  /*updateStyle();*/
}

NodeGrid* ParentNode::getChildView() {
  return childView;
}

// When the label is double-clicked, emit this label's child view
// NB: evt is not used
void ParentNode::mouseDoubleClickEvent(QMouseEvent* /*evt*/) {
  emit selected(this);
}
