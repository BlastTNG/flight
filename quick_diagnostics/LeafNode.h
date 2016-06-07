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
public slots:
  void updateStatus();
  void select();
  void unselect();
private:
  const GetData::Dirfile* dirfile; // reference to the dirfile
  const char* fieldCode; // the name of the field in the dirfile to which this leaf-node corresponds
  bool isSelected;
  const double lo, hi; // the lo and hi value for this channel
  static const QColor hiColor;
  static const QColor loColor;
  QColor statusColor; // this node's current color (reflecting its status)
signals:
  void clicked(const char* fieldCode);
protected:
  // All overrided from QWidget
  void mousePressEvent(QMouseEvent* evt);
  void mouseMoveEvent(QMouseEvent* evt);
  void leaveEvent(QEvent* evt); 
  void paintEvent(QPaintEvent* event);
};

#endif
