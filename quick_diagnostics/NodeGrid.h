#include <QtGui>
#include "StatusNode.h"
#include "LeafNode.h"

#ifndef NODEGRID_H
#define NODEGRID_H

/*
  A grid layout of Status Nodes, that additionally stores a reference to each 
  of its children Status Nodes.
*/
class NodeGrid : public QWidget {
  Q_OBJECT
public:
  NodeGrid() : QWidget() {
    childNodes = new QList<StatusNode*>();
    gridLayout = new QGridLayout();
    gridLayout->setSpacing(1);
    setLayout(gridLayout);
  };
  ~NodeGrid() {
    delete childNodes; 
  }
  void addChildNode(StatusNode* node, int row, int col) {
    gridLayout->addWidget(node, row, col);
    childNodes->append(node);
  }
  const QList<StatusNode*>& getChildNodes() {
    return *childNodes;
  }
  // Update all of the children of this grid
  void updateChildren() {
    for (int i = 0; i < childNodes->size(); ++i) {
      StatusNode* n = childNodes->at(i);
      n->updateStatus();
    }
  }
private:
  QList<StatusNode*>* childNodes; // List of all of this grid's status nodes
  QGridLayout* gridLayout; // ref to this widget's underlying grid layout
};

#endif
