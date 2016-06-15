#include "NodeGrid.h"

NodeGrid::NodeGrid(QList<LeafGroup*>* leafGroups) : QWidget() {
  
  QVBoxLayout* layout = new QVBoxLayout();
  layout->setSpacing(5);
  
  QFont* font = new QFont();
  font->setPixelSize(20);

  // Make a grid for each group of leaves
  childLeaves = new QList<LeafNode*>();
  for (int i = 0; i < leafGroups->size(); ++i) {
    LeafGroup* group = leafGroups->at(i);

    // Give the group a label
    QString name(group->groupName.c_str());
    QLabel* nameLabel = new QLabel(name);
    nameLabel->setFont(*font);
    nameLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    layout->addWidget(nameLabel);

    // Lay out the leaves in a roughly square grid
    // TODO: let the user specify the layout of the nodes
    QGridLayout* grid = new QGridLayout();
    grid->setSpacing(1);
    QList<LeafNode*>* leaves = group->leaves;
    int numLeaves = leaves->size();
    int numCols = (int) sqrt(numLeaves); 
    for (int j = 0; j < numLeaves; ++j) {
      LeafNode* leaf = leaves->at(j);
      childLeaves->append(leaf);
      grid->addWidget(leaf, j / numCols, j % numCols);
    }
    layout->addLayout(grid);
  }
  setLayout(layout);
}

NodeGrid::~NodeGrid() {
  delete childLeaves;
}

// Update all of the children of this grid
void NodeGrid::updateChildren(int frameNum) {
  for (int i = 0; i < childLeaves->size(); ++i) {
    LeafNode* n = childLeaves->at(i);
    n->updateStatus(frameNum);
  }
}
