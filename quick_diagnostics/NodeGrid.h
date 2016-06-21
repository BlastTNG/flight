#ifndef NODEGRID_H
#define NODEGRID_H

#include <QtGui>
#include "LeafGroup.h"
#include "LeafNode.h"
#include "PositionedLeaf.h"

/*
  A grid layout of Status Nodes, that additionally stores a reference to each 
  of its children Status Nodes.
*/
class NodeGrid : public QWidget {
  Q_OBJECT
public:
  NodeGrid(QList<PositionedLeaf*>* leaves); // for new json format
  NodeGrid(QList<LeafGroup*>* leafGroups); // for traditional json format
  ~NodeGrid();
  void updateChildren(int frameNum); // Update all of the children of this grid, to the given frame number

private:
  QList<LeafNode*>* childLeaves; 
};

#endif
