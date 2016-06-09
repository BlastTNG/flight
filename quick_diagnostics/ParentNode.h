#ifndef PARENTNODE_H
#define PARENTNODE_H

#include <QtGui>
#include "StatusNode.h"
#include "NodeGrid.h"

class ParentNode : public StatusNode {
  Q_OBJECT
public:
  ParentNode(QString name, NodeGrid* childView);
  ~ParentNode() {};
  NodeGrid* getChildView();
  void updateStatus();
private:
  NodeGrid* childView;
signals:
  // Emit itself when selected
  void selected(ParentNode* thisNode);
protected:
  void mouseDoubleClickEvent(QMouseEvent* evt); // override from widget
};

#endif
