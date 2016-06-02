#include <QtGui>

#ifndef STATUSNODE_H
#define STATUSNODE_H

/**
  Simple abstract class for FieldNode and Parentnode.
*/
class StatusNode : public QLabel {
  Q_OBJECT
public:
  StatusNode(QString name) : QLabel(name) {
    setStyleSheet("QLabel {background-color: green}");
    setMinimumSize(5, 5);
  }
  virtual ~StatusNode() {};
};

#endif
