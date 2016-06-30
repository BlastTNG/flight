#ifndef DIAGNOSTICSVIEW_H
#define DIAGNOSTICSVIEW_H

#include <QtGui>
#include "DetailsView.h"
#include "LeafNode.h"
#include "NodeGrid.h"
#include "UpdateClock.h"
#include "json.hpp"

using json = nlohmann::json;

class DiagnosticsView : public QWidget {
  Q_OBJECT
public:
  DiagnosticsView(GetData::Dirfile* dirfile, json config);
  ~DiagnosticsView() {};
  void addView(QWidget* view); 
  QList<QString*>* errorList; // list of errors (non-empty if hasErrors is true)
private:
  QMap<QString, NodeGrid*>* viewMap; // map from view name to view
  QStackedLayout* stackLayout;
  DetailsView* detailsView; // when a leaf-node is clicked, display more detailed information about it here
  UpdateClock* updateClock;
  LeafNode* selectedNode; // the currently selected node
  NodeGrid* currentGrid; // the currently displayed view
  bool newView; // true if we have switched views and not yet updated the nodes

  GetData::Dirfile& dirfile; // ref to the dirfile
  int lastNumFrames; // the number of frames that were in the dirfile in the last update

  NodeGrid* generateWithManualLayout(GetData::Dirfile* dirfile, json config);
  NodeGrid* generateWithAutoLayout(GetData::Dirfile* dirfile, json config);
  QList<LeafNode*>* getLeavesForPrefix(GetData::Dirfile* dirfile, string prefix, double lo, double hi);
  string getViewLayout(json view);
  void generateViewMap(GetData::Dirfile* dirfile, json config);
public slots:
  void pushView(NodeGrid* nextView); // push this view to the stackLayout
  void updateDetailLabel(LeafNode* leaf);
private slots:
  void updateDisplayedNodes(); // use GetData to update the status of all currently displayed nodes
  void switchView(const QString& viewName); // change what view is displayed
};

#endif
