#include <QtGui>
#include "ParentNode.h"
#include "PathLabel.h"

#ifndef DIAGNOSTICSVIEW_H
#define DIAGNOSTICSVIEW_H

class DiagnosticsView : public QWidget {
  Q_OBJECT
public:
  DiagnosticsView();
  ~DiagnosticsView() {};
  void addView(QWidget* view); 
  void setRoot(ParentNode* rootView);
  void configureParentNode(ParentNode* parent);
private:
  QStackedLayout* stackLayout;
  QStack<QWidget*>* pathStack; // stack of the widgets in the stackLayout
  PathLabel* pathLabel; // describes the path in the diagnostics tree, provides navigation
public slots:
  void pushView(ParentNode* parent); // push this parent's child view  to the stackLayout and pathStack
};

#endif