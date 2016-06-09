#ifndef DETAILSVIEW_H
#define DETAILSVIEW_H

#include <QtGui>
#include "LeafNode.h"

class DetailsView : public QWidget {
  Q_OBJECT
public:
  DetailsView();
  ~DetailsView() {};
  void updateDetails(LeafNode* leaf); // set this view to show the details of the emitted leaf
private:
  void updateGrid(QString key, QString value); // update the grid value for the given key
  QMap<QString, int>* indexMap; // map from value name to row number in the grid
  QMap<QString, QLabel*>* valueMap; // map from field name to label for value of that field
  QGridLayout* layout;
};

#endif
