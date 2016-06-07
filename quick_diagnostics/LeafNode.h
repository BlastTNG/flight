#include <QtGui>
#include <getdata/dirfile.h>
#include "StatusNode.h"

#ifndef LEAFNODE_H
#define LEAFNODE_H

class LeafNode : public StatusNode {
  Q_OBJECT
public:
  LeafNode(GetData::Dirfile* dirfile, const char* fieldCode, double lo, double hi);
  ~LeafNode() {};
  void updateStatus();
public slots:
  void select();
  void unselect();
  QString getDetails(); // get the details about this node and its dirfile field
private:
  GetData::Dirfile* dirfile; // reference to the dirfile
  const char* fieldCode; // the name of the field in the dirfile to which this leaf-node corresponds

  bool isSelected; // true if the user has selected this node, false ow
  bool isDirfileError; // true if there is/was an error while reading this field from dirfile, false ow
  QString dirfileError; // get the error msg encountered while reading thsi file from dirfile, if applicable

  const double lo, hi; // the lo and hi value for this channel
  static const QColor hiColor, errorColor;
  static const QColor loColor;
  QColor statusColor; // this node's current color (reflecting its status)
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
