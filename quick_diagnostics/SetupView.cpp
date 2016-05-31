#include "SetupView.h"
using json = nlohmann::json;

QHBoxLayout* generateSelector(SetupView* parent, QString fileExt) {
  QFileDialog* dialog = new QFileDialog(parent, "Select " + fileExt, "", "(*." + fileExt + ")");
  QPushButton* btn = new QPushButton("Select " + fileExt);
  QLabel* pathName = new QLabel("No " + fileExt + " selected");

  // Open the dialog when the btn is pressed
  QObject::connect(btn, SIGNAL(clicked()), dialog, SLOT(open()));

  // When a file is selected, display the path of the selected file, and save the path
  QObject::connect(dialog, SIGNAL(fileSelected(const QString&)), pathName, SLOT(setText(const QString&)));
  if (fileExt.toStdString().compare("dirfile") == 0) {
    QObject::connect(dialog, SIGNAL(fileSelected(const QString&)), parent, SLOT(updateDirfilePath(const QString&)));
  } else {
    QObject::connect(dialog, SIGNAL(fileSelected(const QString&)), parent, SLOT(updateConfigPath(const QString&)));
  }

  QHBoxLayout* layout = new QHBoxLayout();
  layout->addWidget(btn);
  layout->addWidget(pathName);
  return layout;
}

SetupView::SetupView() : QWidget() {

  // Layout the widget's components
  QVBoxLayout* vBox = new QVBoxLayout();
  vBox->addWidget(new QLabel("Please select dirfile and json config file:"));
  vBox->addLayout(generateSelector(this, "dirfile"));
  vBox->addLayout(generateSelector(this, "json"));
  QPushButton* btn = new QPushButton("Done");
  vBox->addWidget(btn);
  this->setLayout(vBox);

  // When the user presses done, emit the two entered file paths
  QObject::connect(btn, SIGNAL(clicked()), this, SLOT(checkFiles()));
}

void SetupView::checkFiles() {

  // Attempt to open the selected dirfile, report errors
  GetData::Dirfile* dirfile = new GetData::Dirfile(dirfilePath.toStdString().c_str(), GD_RDONLY);
  if (dirfile->Error() != GD_E_OK) {
    QMessageBox msgBox;
    msgBox.setText("Invalid dirfile, please reselect");
    msgBox.setInformativeText(dirfile->ErrorString());
    msgBox.exec();
    return; 
  }

  // Attempt to open the selected json file, report errors
  QFile configFile(configPath);
  if (!configFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
    QMessageBox msgBox;
    msgBox.setText("Could not open the given .json file, please reselect");
    msgBox.exec();
    return;
  }

  // Parse the config file text into a json object
  // TODO: figure out how to allocate on heap, instead
  QTextStream in(&configFile);
  QString txt = in.readAll();
  json j = json::parse(txt.toStdString());
  if (!j.is_structured() || j.is_null()) {
    QMessageBox msgBox;
    msgBox.setText("Error parsing .json file, Check for syntax errors.");
    msgBox.exec();
    return;
  }

  emit doneSetup(dirfile, j);
}

void SetupView::updateDirfilePath(const QString& path) {
  dirfilePath = path;  
}

void SetupView::updateConfigPath(const QString& path) {
  configPath = path;
}
