#ifndef STATUSNODE_H
#define STATUSNODE_H

#include <QtGui>

/**
  Simple abstract class for FieldNode and Parentnode.
*/
class StatusNode : public QLabel {
  Q_OBJECT
public:
  StatusNode(QString name) : QLabel(name) {

    // Init this node's palette 
    QPalette palette = this->palette();
    palette.setColor(QPalette::Background, Qt::gray);
    setAutoFillBackground(true);
    setPalette(palette);
  }

  virtual ~StatusNode() {};
  virtual void updateStatus() = 0; // update the node's status
};

#endif
