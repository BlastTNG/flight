#include "DetailsView.h"

DetailsView::DetailsView() : QWidget() {

  // Configure the index map
  indexMap = new QMap<QString, int>();
  indexMap->insert("Field Code", 0);
  indexMap->insert("Current Value", 1);
  indexMap->insert("Lo", 2);
  indexMap->insert("Hi", 3);
  indexMap->insert("Dirfile Errors", 4);

  setStyleSheet("background-color: lightGray");

  // Create the grid of values from the index map
  layout = new QGridLayout();
  layout->setSpacing(1);
  layout->setColumnStretch(0, 1);
  layout->setColumnStretch(1, 3);
  layout->setRowMinimumHeight(4, 40); // give the errors row more space
  QMapIterator<QString, int> i(*indexMap);
  valueMap = new QMap<QString, QLabel*>();
  QFont font;
  font.setPixelSize(10);
  while (i.hasNext()) {
    i.next();
    QLabel* keyLabel = new QLabel(i.key());
    keyLabel->setFont(font);
    keyLabel->setMargin(3);
    layout->addWidget(keyLabel, i.value(), 0);

    QLabel* valueLabel = new QLabel("");
    valueLabel->setFont(font);
    valueLabel->setMargin(3);
    valueMap->insert(i.key(), valueLabel);
    layout->addWidget(valueLabel, i.value(), 1);
  }

  QVBoxLayout* vBox = new QVBoxLayout();
  vBox->setSpacing(1);
  QLabel* nameLabel = new QLabel("Details for Selected Node");
  nameLabel->setFont(font);
  nameLabel->setMargin(3);
  nameLabel->setAlignment(Qt::AlignCenter);
  vBox->addWidget(nameLabel);
  vBox->addLayout(layout);
  setLayout(vBox);
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

  const QList<QString>& errorList = leaf->getErrorList();
  QString errors;
  for (int i = 0; i < errorList.size(); ++i) {
    errors += QString::number(i);
    errors += ") ";
    errors += errorList.at(i);
    errors += "<br>";
  }
  updateGrid("Dirfile Errors", errors);
}
