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
    status = READ_ERROR; // TODO: is this a reasonable default?
    updateStyle();

    // Update the status of the node every 500ms
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateStatus()));
    timer->start(500);
  }

  virtual ~StatusNode() {};

  // Possible detector status' 
  enum Status {
    GOOD, // detector reading is in expected range
    BAD, // dectector reading is outside expected range
    READ_ERROR, // error reading the dirfile for this detector
  };

  Status getStatus() { return status; }
  void setStatus(Status s) { status = s; }

  // Update the node's color to reflect its current status
  void updateStyle() {
    QString color;
    switch (status) {
      case GOOD:
        color = "green";
        break;
      case BAD:
        color = "red";
        break;
      case READ_ERROR:
        color = "gray";
        break;
    }
    setStyleSheet("QLabel {background-color:" + color + "}");
  }
private:
  Status status;
public slots:
  virtual void updateStatus() = 0; // update the node's status
};

#endif
