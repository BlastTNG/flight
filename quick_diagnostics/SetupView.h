#include <QtGui>
#include <getdata/dirfile.h>
#include "json.hpp"
using json = nlohmann::json;

#ifndef SETUPVIEW_H
#define SETUPVIEW_H

class SetupView : public QWidget {
  Q_OBJECT
public:
  SetupView();
  ~SetupView() {};
private:
  QString dirfilePath;
  QString configPath;
signals:
  void doneSetup(GetData::Dirfile* dirfile, json config);
private slots:
  void updateDirfilePath(const QString& path);
  void updateConfigPath(const QString& path);
  void checkFiles(); 
};

#endif
