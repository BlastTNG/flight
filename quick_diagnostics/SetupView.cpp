#include "SetupView.h"
#include <stdexcept>
#include "DiagnosticsView.h"
using json = nlohmann::json;

void generateSelector(QString defaultPath, SetupView* parent, QString fileExt, int row, QGridLayout* layout) {
  QFileDialog* dialog = new QFileDialog(parent, "Select " + fileExt, "", "(*." + fileExt + ")");
  if (fileExt.toStdString().compare("dirfile") == 0) {
    dialog->setFileMode(QFileDialog::Directory);
  }

  QPushButton* btn = new QPushButton("Select " + fileExt);
  QLabel* pathName = new QLabel(defaultPath);

  // Open the dialog when the btn is pressed
  QObject::connect(btn, SIGNAL(clicked()), dialog, SLOT(open()));

  // When a file is selected, display the path of the selected file, and save the path
  QObject::connect(dialog, SIGNAL(fileSelected(const QString&)), pathName, SLOT(setText(const QString&)));
  if (fileExt.toStdString().compare("dirfile") == 0) {
    QObject::connect(dialog, SIGNAL(fileSelected(const QString&)), parent, SLOT(updateDirfilePath(const QString&)));
  } else {
    QObject::connect(dialog, SIGNAL(fileSelected(const QString&)), parent, SLOT(updateConfigPath(const QString&)));
  }

  layout->addWidget(btn, row, 0);
  layout->addWidget(pathName, row, 1);
}

SetupView::SetupView() : QWidget() {

  // Attempt to load the paths from settings
  settings = new QSettings("Quick Diagnostics", "BLAST");
  dirfilePath = settings->value("dirfile_path", "No dirfile selected").toString(); 
  configPath = settings->value("config_path", "No .json config selected").toString();

  // Layout the widget's components
  QGridLayout* layout = new QGridLayout();
  layout->setSpacing(20);
  layout->setColumnStretch(0, 1);
  layout->setColumnStretch(1, 5);
  layout->setAlignment(Qt::AlignCenter);
  layout->addWidget(new QLabel("Please select dirfile and json config file:"), 0, 0, 1, 2);
  generateSelector(dirfilePath, this, "dirfile", 1, layout);
  generateSelector(configPath,this,  "json", 2, layout);
  QPushButton* btn = new QPushButton("Done");
  layout->addWidget(btn, 3, 0, 1, 2);
  this->setLayout(layout);

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

  // Parse the config file text into a json object, report errors
  // TODO: figure out how to allocate the json object on heap, rather than on stack
  QTextStream in(&configFile);
  QString txt = in.readAll();
  json j;
  try {
    j = json::parse(txt.toStdString());
  } catch(std::invalid_argument& e) {
    QMessageBox msgBox;
    msgBox.setText("Invalid .json file, please reselect");
    msgBox.setInformativeText(e.what());
    msgBox.exec();
    return;     
  }

  emit doneSetup(dirfile, j);
}

void SetupView::updateDirfilePath(const QString& path) {
  dirfilePath = path;  
  settings->setValue("dirfile_path", path);
}

void SetupView::updateConfigPath(const QString& path) {
  configPath = path;
  settings->setValue("config_path", path);
}
