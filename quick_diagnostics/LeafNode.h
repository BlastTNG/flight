#ifndef LEAFNODE_H
#define LEAFNODE_H

#include <QtGui>
#include <getdata/dirfile.h>

class LeafNode : public QLabel {
  Q_OBJECT
public:
  LeafNode(GetData::Dirfile* dirfile, const char* fieldCode, double lo, double hi);
  ~LeafNode() {};

  // Uses the most recent value in the dirfle to update the status of this node
  void updateStatus();

  // Accessors
  QString getFieldCode();
  double getCurrentValue();
  double getLoValue();
  double getHiValue();
  const QList<QString>& getErrorList();
public slots:
  void select();
  void unselect();
private:
  GetData::Dirfile* dirfile; // reference to the dirfile
  const char* fieldCode; // the name of the field in the dirfile to which this leaf-node corresponds

  // list of all of the errors encountered on the most recent read of this node
  QList<QString>* errorList; 

  double currentValue; // the value most recently read from the dirfile
  const double lo, hi; // the lo and hi value for this channel
  static const QColor loColor, hiColor, errorColor;
  QColor statusColor; // this node's current color (reflecting its status)
  bool isSelected; // true if the user has selected this node, false ow
signals:
  void clicked(LeafNode* thisLeaf); 
protected:
  // All overrided from QWidget
  void mousePressEvent(QMouseEvent* evt);
  void mouseMoveEvent(QMouseEvent* evt);
  void leaveEvent(QEvent* evt); 
  void paintEvent(QPaintEvent* event);
};

#endif
