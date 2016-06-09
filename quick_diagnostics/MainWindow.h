#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QtGui>
#include "SetupView.h"
#include "DiagnosticsView.h"
#include <getdata/dirfile.h>
#include "json.hpp"
using json = nlohmann::json;

class MainWindow : public QWidget {
  Q_OBJECT
public:
  MainWindow();
  ~MainWindow() {};
private:
  QStackedLayout* viewStack;
  SetupView* setupView;
  DiagnosticsView* diagView;
private slots:
  void generateDiagnostics(GetData::Dirfile* dirfile, json config);
};

#endif
