#ifndef DOUBLEE_H
#define DOUBLEE_H

#include <qlineedit.h>

class DoubleEntry : public QLineEdit {
  Q_OBJECT
public:
  DoubleEntry(double min, double max, QWidget *parent, const char *name);
  void SetMinMax(double mi, double ma);
  void SetDoubleValue(double val);
  void SetParentField(int com, int par);
  double value();
  void RecordDefaults();
private:
  int command, param;
  double Min, Max;
};

#endif
