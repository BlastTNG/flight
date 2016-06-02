#include <QtGui>
#include "StatusNode.h"

#ifndef PARENTNODE_H
#define PARENTNODE_H

class ParentNode : public StatusNode {
  Q_OBJECT
public:
  ParentNode(QString name, QWidget* childView);
  ~ParentNode() {};
  QWidget* getChildView();
private:
  QWidget* childView;
signals:
  // Emit itself when selected
  void selected(ParentNode* thisNode);
protected:
  void mouseDoubleClickEvent(QMouseEvent* evt); // override from widget
};

#endif
