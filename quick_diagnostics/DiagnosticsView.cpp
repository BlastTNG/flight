#include "DiagnosticsView.h"
#include "ParentNode.h"
#include "LeafNode.h"

DiagnosticsView::DiagnosticsView() : QWidget() {
 /*
    Path Label
    Detail Label (Later)
    Stack view
    Btn for reselecting files, maybe show the name of the current files
    */
  
  GetData::Dirfile* dirfile = new GetData::Dirfile("/Users/matthewriley/Documents/BLAST/sample.dirfile", GD_RDONLY);
  if (dirfile->Error() != GD_E_OK) {
    // Show a popup explaining the error that occurred while opening the file
    QMessageBox msgBox;
    QString errorMsg("Error while opening dirfile: ");
    msgBox.setText(errorMsg + dirfile->ErrorString());
    msgBox.exec();
  }

  // TODO: For now, just test that the transitions work
  LeafNode* labelA = new LeafNode(dirfile, "roach2_kid0025_i");
  LeafNode* labelB = new LeafNode(dirfile, "roach2_kid0028_q");

  ParentNode* a = new ParentNode("parent A", labelA);
  a->setStyleSheet("QLabel { background-color: green; }");
  ParentNode* b = new ParentNode("parent B", labelB);
  b->setStyleSheet("QLabel { background-color: green; }");
  QGridLayout* grid = new QGridLayout();
  grid->addWidget(a, 0, 0);
  grid->addWidget(b, 0, 1);

  QWidget* w = new QWidget();
  w->setLayout(grid);

  QStackedLayout* stack = new QStackedLayout();
  stack->addWidget(w);
  stack->addWidget(labelA);
  stack->addWidget(labelB);
  this->setLayout(stack);

  QObject::connect(a, SIGNAL(selected(QWidget*)), stack, SLOT(setCurrentWidget(QWidget*)));
  QObject::connect(b, SIGNAL(selected(QWidget*)), stack, SLOT(setCurrentWidget(QWidget*)));
}
