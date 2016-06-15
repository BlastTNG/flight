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
  NodeGrid(QList<LeafGroup*>* leafGroups);
  ~NodeGrid();
  void updateChildren(int frameNum); // Update all of the children of this grid, to the given frame number
private:
  QList<LeafNode*>* childLeaves; 
};

#endif
