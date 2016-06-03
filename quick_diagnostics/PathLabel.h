#include <QtGui>
#include "ParentNode.h"

#ifndef PATHLABEL_H
#define PATHLABEL_H

class PathLabel : public QWidget {
  Q_OBJECT
public:
  PathLabel();
  ~PathLabel() {};
  QStack<ParentNode*>* labelStack;
private:
  QHBoxLayout* layout;
public slots:
  void pushWidget(QString name, NodeGrid* widget);
private slots:
  void popWidgets(ParentNode* clickedLabel); // pop until the rightmost view is the child view of the clicked label
signals:
  // emit the requested view (when the user clicks a label describing that view)
  void viewRequested(QWidget* widget); 
};

#endif
