#include <QtGui>
#include <getdata/dirfile.h>
#include "StatusNode.h"

#ifndef LEAFNODE_H
#define LEAFNODE_H

class LeafNode : public StatusNode {
  Q_OBJECT
public:
  LeafNode(GetData::Dirfile* dirfile, const char* fieldCode);
  ~LeafNode() {};
private:
  const GetData::Dirfile* dirfile; // reference to the dirfile
  const char* fieldCode; // the name of the field in the dirfile to which this leaf-node corresponds
signals:
  void clicked(const char* fieldCode);
protected:
  void mousePressEvent(QMouseEvent* evt); // override from QWidget
};

#endif
