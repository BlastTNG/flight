#include "DetailsView.h"

DetailsView::DetailsView() : QWidget() {

  // Configure the index map
  indexMap = new QMap<QString, int>();
  indexMap->insert("Field Code", 0);
  indexMap->insert("Current Value", 1);
  indexMap->insert("Lo", 2);
  indexMap->insert("Hi", 3);
  indexMap->insert("Dirfile Error", 4);

  // Create the grid of values from the index map
  layout = new QGridLayout();
  QMapIterator<QString, int> i(*indexMap);
  valueMap = new QMap<QString, QLabel*>();
  while (i.hasNext()) {
    i.next();
    layout->addWidget(new QLabel(i.key()), 0, i.value());
    QLabel* l = new QLabel("");
    valueMap->insert(i.key(), l);
    layout->addWidget(l, 1, i.value());
  }
  setLayout(layout);
}

void DetailsView::updateGrid(QString key, QString value) {
  valueMap->value(key)->setText(value);
}

QString asString(double d) {
  /*char buf[32];*/
  //sprintf(buf, "%10.5f", d);
  /*return buf;*/
  return QString::number(d);
}

void DetailsView::updateDetails(LeafNode* leaf) {
  updateGrid("Field Code", leaf->getFieldCode()); 
  updateGrid("Current Value", asString(leaf->getCurrentValue()));
  updateGrid("Lo", asString(leaf->getLoValue()));
  updateGrid("Hi", asString(leaf->getHiValue()));
  updateGrid("Dirfile Error", leaf->getDirfileError());
}
