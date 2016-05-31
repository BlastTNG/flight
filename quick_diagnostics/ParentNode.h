#include <QtGui>

#ifndef PARENTNODE_H
#define PARENTNODE_H

class ParentNode : public QLabel {
  Q_OBJECT
public:
  ParentNode(QString name, QWidget* childView);
  ~ParentNode() {};
private:
  QWidget* childView;
signals:
  void selected(QWidget* view);
protected:
  void mouseDoubleClickEvent(QMouseEvent* evt); // override from widget
};

#endif
