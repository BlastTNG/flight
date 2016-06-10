#include "NodeGrid.h"

NodeGrid::NodeGrid(QList<QWidget*>* parents, QList<LeafGroup*>* leafGroups) : QWidget() {
  childLeaves = new QList<LeafNode*>();
  
  QVBoxLayout* layout = new QVBoxLayout();
  layout->setSpacing(5);
  
  QFont* font = new QFont();
  font->setPixelSize(20);

  // Put all of the parents in a row along the top
  if (!parents->empty()) {
    QLabel* l = new QLabel("Change view to:");
    l->setFont(*font);
    layout->addWidget(l);
    QHBoxLayout* hBox = new QHBoxLayout();
    for (int i = 0; i < parents->size(); ++i) {
      hBox->addWidget(parents->at(i));
    }
    QWidget* parentBox = new QWidget();
    parentBox->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    parentBox->setLayout(hBox);
    layout->addWidget(parentBox);
  }

  // Make a grid for each group of leaves
  for (int i = 0; i < leafGroups->size(); ++i) {
    LeafGroup* group = leafGroups->at(i);

    // Give the group a label
    QString name(group->groupName.c_str());
    QLabel* nameLabel = new QLabel(name);
    nameLabel->setFont(*font);
    nameLabel->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    layout->addWidget(nameLabel);

    // Lay out the leaves in a roughly square grid
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

/*
const QList<StatusNode*>& NodeGrid::getChildNodes() {
  return *childNodes;
} */

// Update all of the children of this grid
void NodeGrid::updateChildren() {
  for (int i = 0; i < childLeaves->size(); ++i) {
    StatusNode* n = childLeaves->at(i);
    n->updateStatus();
  }
}
