#ifndef POSITIONEDLEAF_H
#define POSITIONEDLEAF_H

#include "LeafNode.h"

class PositionedLeaf {
public:
  PositionedLeaf(LeafNode* leaf, int x, int y, int w, int h) : leaf(leaf), x(x), y(y), w(w), h(h) {}
  LeafNode* leaf;
  int x, y, w, h;
};

#endif
