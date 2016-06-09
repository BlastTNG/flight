#ifndef NODEGRID_H
#define NODEGRID_H

#include <QtGui>
#include "LeafGroup.h"
#include "LeafNode.h"

/*
  A grid layout of Status Nodes, that additionally stores a reference to each 
  of its children Status Nodes.
*/
class NodeGrid : public QWidget {
  Q_OBJECT
public:
  // TODO: make the constructor take in a group of parent nodes, and a 
  // group of groups of leave nodes
  NodeGrid(QList<QWidget*>* parents, QList<LeafGroup*>* leafGroups);
  ~NodeGrid();
  //const QList<StatusNode*>& getChildNodes();
  void updateChildren(); // Update all of the children of this grid
private:
  QList<LeafNode*>* childLeaves; 
};

#endif
