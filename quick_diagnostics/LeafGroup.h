#ifndef LEAFGROUP_H
#define LEAFGROUP_H

#include <QtGui>
#include "LeafNode.h"

using namespace std;

// Convenience class used for organizing groups of leaves in a NodeGrid
class LeafGroup {

public:
  LeafGroup(string groupName, QList<LeafNode*>* leaves) : groupName(groupName), leaves(leaves) {};
  ~LeafGroup() {};

  string groupName;
  QList<LeafNode*>* leaves;
};

#endif
