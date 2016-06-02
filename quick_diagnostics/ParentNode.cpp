#include "ParentNode.h"

ParentNode::ParentNode(QString name, QWidget* childView) : StatusNode(name), childView(childView) {
  setStyleSheet("border: 2px solid black");
}

QWidget* ParentNode::getChildView() {
  return childView;
}

// When the label is double-clicked, emit this label's child view
void ParentNode::mouseDoubleClickEvent(QMouseEvent* evt) {
  emit selected(this);
}
