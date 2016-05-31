#include "ParentNode.h"

ParentNode::ParentNode(QString name, QWidget* childView) : QLabel(name), childView(childView) {}

// When the label is double-clicked, emit this label's child view
void ParentNode::mouseDoubleClickEvent(QMouseEvent* evt) {
  emit selected(childView);
}
