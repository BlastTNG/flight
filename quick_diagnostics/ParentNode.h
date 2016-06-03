#include <QtGui>
#include "StatusNode.h"
#include "NodeGrid.h"

#ifndef PARENTNODE_H
#define PARENTNODE_H

class ParentNode : public StatusNode {
  Q_OBJECT
public:
  ParentNode(QString name, NodeGrid* childView);
  ~ParentNode() {};
  NodeGrid* getChildView();
private:
  NodeGrid* childView;
public slots:
  void updateStatus();
signals:
  // Emit itself when selected
  void selected(ParentNode* thisNode);
protected:
  void mouseDoubleClickEvent(QMouseEvent* evt); // override from widget
};

#endif
