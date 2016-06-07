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
    setMinimumSize(5, 5); // all the node to be very small

    // Init this node's palette 
    QPalette palette = this->palette();
    palette.setColor(QPalette::Background, Qt::gray);
    setAutoFillBackground(true);
    setPalette(palette);

    // Update the status of the node every 500ms
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateStatus()));
    timer->start(500);
  }

  virtual ~StatusNode() {};
public slots:
  virtual void updateStatus() = 0; // update the node's status
};

#endif
